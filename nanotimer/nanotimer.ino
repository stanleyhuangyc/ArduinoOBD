/*************************************************************************
* OBD-II based performance timer and logger
* Distributed under GPL v2.0
* Copyright (c) 2014 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <OBD.h>
#include <SD.h>
#include <MicroLCD.h>
#include "config.h"
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_SLEEPING 0x20

static uint32_t lastFileSize = 0;
static int lastSpeed = -1;
static uint32_t lastSpeedTime = 0;
static int speed = 0;
static uint32_t distance = 0;
static uint16_t fileIndex = 0;
static uint32_t startTime = 0;

#define STAGE_IDLE 0
#define STAGE_WAIT_START 1
#define STAGE_MEASURING 2

static byte stage = STAGE_IDLE;

#define SPEED_THRESHOLD_1 60 /* kph */
#define SPEED_THRESHOLD_2 100 /* kph */
#define SPEED_THRESHOLD_3 200 /* kph */
#define DISTANCE_THRESHOLD 400 /* meters */

static uint16_t times[4] = {0};

#if ENABLE_DATA_LOG
class COBDLogger : public COBD, public CDataLogger
#else
class COBDLogger : public COBD
#endif
{
public:
    COBDLogger():state(0) {}
    void setup()
    {
#if ENABLE_DATA_LOG
        if (!(state & STATE_SD_READY)) {
            if (checkSD()) {
                state |= STATE_SD_READY;
                showStates();
            }
        }
#endif

        do {
            showStates();
        } while (!init());

        state |= STATE_OBD_READY;

        showStates();

#if ENABLE_DATA_LOG
        uint16_t index = openFile(LOG_TYPE_DEFAULT);
        lcd.print("File ID:");
        if (index) {
            lcd.setFlags(FLAG_PAD_ZERO);
            lcd.printInt(index, 5);
            lcd.setFlags(0);
        } else {
            lcd.print("N/A");
        }
#endif

        initTimerScreen();
    }
    void loop()
    {
        static byte index = 0;
        static byte index2 = 0;
        static byte index3 = 0;

        timerLoop();

#if ENABLE_DATA_LOG
        // flush SD data every 1KB
        if (dataSize - lastFileSize >= 1024 && stage != STAGE_MEASURING) {
            flushFile();
            lastFileSize = dataSize;
            // display logged data size
        }
#endif

        if (errors >= 2) {
            reconnect();
        }
    }
#if ENABLE_DATA_LOG
    bool checkSD()
    {
        Sd2Card card;
        SdVolume volume;
        lcd.setCursor(0, 0);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        state &= ~STATE_SD_READY;
        pinMode(SS, OUTPUT);
        if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
            const char* type;
            char buf[20];

            switch(card.type()) {
            case SD_CARD_TYPE_SD1:
                type = "SD1";
                break;
            case SD_CARD_TYPE_SD2:
                type = "SD2";
                break;
            case SD_CARD_TYPE_SDHC:
                type = "SDHC";
                break;
            default:
                type = "SDx";
            }

            lcd.clear();
            lcd.print(type);
            lcd.write(' ');
            if (!volume.init(card)) {
                lcd.print("No FAT!");
                return false;
            }

            uint32_t volumesize = volume.blocksPerCluster();
            volumesize >>= 1; // 512 bytes per block
            volumesize *= volume.clusterCount();
            volumesize >>= 10;

            sprintf(buf, "%dGB", (int)((volumesize + 511) / 1000));
            lcd.print(buf);
        } else {
            lcd.clear();
            lcd.print("SD");
            lcd.draw(cross, 32, 0, 16, 16);
            return false;
        }

        if (!SD.begin(SD_CS_PIN)) {
            lcd.setCursor(48, 0);
            lcd.print("Bad SD");
            return false;
        }

        state |= STATE_SD_READY;
        return true;
    }
#endif
private:
    void dataIdleLoop()
    {
        if (state & STATE_SLEEPING) return;

        if (getState() == OBD_CONNECTED)
            return;

        // called while initializing
        char buf[10];
        unsigned int t = (millis() - startTime) / 1000;
        sprintf(buf, "%02u:%02u", t / 60, t % 60);
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setCursor(97, 7);
        lcd.print(buf);
    }
    void timerLoop()
    {
        uint32_t elapsed = millis() - startTime;
        uint16_t n;
#if !ENABLE_DATA_LOG
        uint32_t dataTime;
#endif
        int speed;
        if (!read(PID_SPEED, speed))
            return;

        dataTime = millis();
#if ENABLE_DATA_LOG
        logData(0x100 | PID_SPEED, speed);

        int rpm = 0;
        if (read(PID_RPM, rpm)) {
            dataTime = millis();
            logData(0x100 | PID_RPM, rpm);
        }
#endif

        lcd.setFontSize(FONT_SIZE_XLARGE);
        // estimate distance
        distance += (uint32_t)(speed + lastSpeed) * (dataTime - lastSpeedTime) / 2 / 3600;

        if (lastSpeed != speed) {
            lcd.setCursor(0, 4);
            lcd.printInt((unsigned int)speed % 1000, 3);
            lastSpeed = speed;
        }

        lastSpeedTime = dataTime;

        if (stage == STAGE_WAIT_START) {
            if (speed > 0) {
                stage = STAGE_MEASURING;
                startTime = lastSpeedTime;

#if ENABLE_DATA_LOG
                uint32_t t = dataTime;
                dataTime = lastSpeedTime;
                logData(0x100 | PID_SPEED, lastSpeed);
                dataTime = t;
                logData(0x100 | PID_SPEED, speed);
#endif

                lastSpeed = 0;
                distance = 0;

                memset(times, 0, sizeof(times));

                initTimerScreen();
            }
        } else if (stage == STAGE_MEASURING) {
            // display elapsed time (mm:ss:mm)
            n = elapsed / 1000;
            if (n < 100) {
                lcd.setCursor(0, 0);
                lcd.printInt(n, 2);
                n = (elapsed % 1000) / 100;
                lcd.setFontSize(FONT_SIZE_MEDIUM);
                lcd.setCursor(32, 1);
                lcd.write('.');
                lcd.write('0' + n);
            }
            if (times[2] == 0 && speed >= SPEED_THRESHOLD_3) {
                times[2] = elapsed / 100;
                stage = STAGE_IDLE;
                lcd.clear(0, 0, 128, 24);
                showTimerResults();
                lcd.setFontSize(FONT_SIZE_MEDIUM);
                lcd.setCursor(0, 0);
                lcd.print("DONE!");
            } else if (times[1] == 0 && speed >= SPEED_THRESHOLD_2) {
                times[1] = elapsed / 100;
                showTimerResults();
            } else if (times[0] == 0 && speed >= SPEED_THRESHOLD_1) {
                times[0] = elapsed / 100;
                showTimerResults();
            } else if (speed == 0) {
                // speed go back to 0
                stage = STAGE_IDLE;
            }
            if (distance > 0) {
                lcd.setFontSize(FONT_SIZE_SMALL);
                lcd.setCursor(62, 6);
                if (distance >= 400) {
                    lcd.printInt(400, 3);
                    if (!times[3]) {
                        times[3] = elapsed / 100;
                        showTimerResults();
                    }
                } else {
                    lcd.printInt(distance, 3);
                }
            }
#if ENABLE_DATA_LOG
            // log speed data
            logData(0x100 | PID_SPEED, speed);
            // log additional data
            int rpm;
            if (read(PID_RPM, rpm)) {
                dataTime = millis();
                logData(0x100 | PID_RPM, rpm);
            }
#endif
        } else {
            if (speed == 0) {
                stage = STAGE_WAIT_START;
                initTimerScreen();
                lcd.setFontSize(FONT_SIZE_MEDIUM);
                lcd.setCursor(0, 0);
                lcd.println(" GET");
                lcd.println("READY");
                delay(500);
            }
        }
    }
    void reconnect()
    {
#if ENABLE_DATA_LOG
        closeFile();
#endif
        lcd.clear();
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.print("Reconnecting");
        startTime = millis();
        state &= ~STATE_OBD_READY;
        state |= STATE_SLEEPING;
        //digitalWrite(SD_CS_PIN, LOW);
        for (int i = 0; !init(); i++) {
            if (i == 10) lcd.clear();
        }
        state &= ~STATE_SLEEPING;
        fileIndex++;
        setup();
    }
    byte state;

    // screen layout related stuff
    void showStates()
    {
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setCursor(0, 3);
        if (state & STATE_OBD_READY) {
            lcd.println("OBD connected!   ");
        } else {
            lcd.println("Connecting OBD...");
        }
    }
    void showTimerResults()
    {
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setCursor(56, 0);
        lcd.print(" 0~60:  --");
        lcd.setCursor(56, 2);
        lcd.print("0~100:  --");
        lcd.setCursor(56, 4);
        lcd.print("0~200:  --");
        lcd.setCursor(56, 6);
        lcd.print(" 400m:  --");
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        char buf[8];
        if (times[0]) {
            sprintf(buf, "%2d.%1d", times[0] / 10, times[0] % 10);
            Serial.println(times[0]);
            lcd.setCursor(92, 0);
            lcd.print(buf);
        }
        if (times[1]) {
            sprintf(buf, "%2d.%1d", times[1] / 10, times[1] % 10);
            Serial.println(buf);
            lcd.setCursor(92, 2);
            lcd.print(buf);
        }
        if (times[2]) {
            sprintf(buf, "%2d.%1d", times[2] / 10, times[2] % 10);
            Serial.println(buf);
            lcd.setCursor(92, 4);
            lcd.print(buf);
        }
        if (times[3]) {
            sprintf(buf, "%2d.%1d", times[3] / 10, times[3] % 10);
            Serial.println(buf);
            lcd.setCursor(92, 6);
            lcd.print(buf);
        }
    }
    void initTimerScreen()
    {
        lcd.clear();
        showTimerResults();
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setCursor(24, 7);
        lcd.print("km/h");
    }
};

static COBDLogger logger;

void setup()
{
#ifdef DEBUG
    DEBUG.begin(DEBUG_BAUDRATE);
#endif

    lcd.begin();
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.println("PerformanceBox");

    logger.begin();
#if ENABLE_DATA_LOG
    logger.initSender();
    logger.checkSD();
#endif
    logger.setup();
}

void loop()
{
    logger.loop();
}
