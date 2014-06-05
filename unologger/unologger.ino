/*************************************************************************
* Arduino OBD-II/G-Force Data Logger
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <OBD.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <MPU6050.h>
#include "MultiLCD.h"
#include "images.h"
#include "config.h"
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "datalogger.h"

#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_ACC_READY 0x10
#define STATE_SLEEPING 0x20

#define OBD_MODEL_UART 0
#define OBD_MODEL_I2C 1

static uint32_t lastFileSize = 0;
static int speed = 0;
static uint32_t distance = 0;
static uint16_t fileIndex = 0;
static uint32_t startTime = 0;
static uint16_t elapsed = 0;
static uint8_t lastPid = 0;
static int lastValue = 0;

static byte pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE, PID_INTAKE_MAP};
static byte pidTier2[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_FUEL_LEVEL, PID_DISTANCE};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)

#if OBD_MODEL == OBD_MODEL_UART
class COBDLogger : public COBD, public CDataLogger
#else
class COBDLogger : public COBDI2C, public CDataLogger
#endif
{
public:
    COBDLogger():state(0) {}
    void setup()
    {
        showStates();

#if USE_MPU6050
        Wire.begin();
        if (MPU6050_init() == 0) {
            state |= STATE_ACC_READY;
            showStates();
        }
#endif

        do {
            showStates();
        } while (!init(OBD_PROTOCOL));

        state |= STATE_OBD_READY;

        showStates();

        lcd.clear();
        benchmark();
        delay(5000);

#if ENABLE_DATA_LOG
        // open file for logging
        if (!(state & STATE_SD_READY)) {
            lcd.setFontSize(FONT_SIZE_MEDIUM);
            lcd.setCursor(0, 10);
            if (checkSD()) {
                state |= STATE_SD_READY;
                uint16_t index = openFile();
                lcd.setCursor(0, 12);
                if (index) {
                    lcd.print("File ID: ");
                    lcd.print(index);
                } else {
                    lcd.print("No File");
                }
                delay(1000);
            }
        }
#endif

        showECUCap();
        delay(3000);

        initScreen();
    }
    void benchmark()
    {
        lcd.setFontSize(FONT_SIZE_MEDIUM);

        char buf[OBD_RECV_BUF_SIZE];
        uint8_t count = 0;
        startTime = millis();
        for (uint8_t n = 0; n < TIER_NUM1; n++) {
            sendQuery(pidTier1[n]);
            lcd.write('[');
            lcd.print(millis());
            lcd.write(']');
            lcd.println(pidTier1[n], HEX);
            memset(buf, 0, sizeof(buf));
            if (receive(buf) > 0) {
                lcd.println(buf);
                count++;
            }
        }
        lcd.setCursor(0, 28);
        if (count) {
            lcd.print("OBD Time: ");
            lcd.printInt((millis() - startTime) / count);
            lcd.print("ms");
        } else {
            lcd.print("No PID!");
        }
    }
    void loop()
    {
        static byte index = 0;
        static byte index2 = 0;
        static byte index3 = 0;

        logOBDData(pidTier1[index++]);
        if (index == TIER_NUM1) {
            index = 0;
            if (index2 == TIER_NUM2) {
                index2 = 0;
            } else {
                logOBDData(pidTier2[index2++]);
            }
        }

#if USE_MPU6050
        if (state & STATE_ACC_READY) {
            processAccelerometer();
        }
#endif
        if (errors >= 5) {
            reconnect();
        }
    }
#if ENABLE_DATA_LOG
    bool checkSD()
    {
        Sd2Card card;
        SdVolume volume;
        state &= ~STATE_SD_READY;
        pinMode(SD_CS_PIN, OUTPUT);
        if (card.init(SPI_FULL_SPEED, SD_CS_PIN)) {
            const char* type;

            lcd.print("SD");
            switch(card.type()) {
            case SD_CARD_TYPE_SD1:
                type = "1";
                break;
            case SD_CARD_TYPE_SD2:
                type = "2";
                break;
            case SD_CARD_TYPE_SDHC:
                type = "HC";
                break;
            default:
                type = "x";
            }

            lcd.print(type);
            lcd.write(' ');
            if (!volume.init(card)) {
                lcd.print("No FAT");
                return false;
            }

            uint32_t volumesize = volume.blocksPerCluster();
            volumesize >>= 1; // 512 bytes per block
            volumesize *= volume.clusterCount();
            volumesize >>= 10;

            lcd.print((int)((volumesize + 511) / 1000));
            lcd.print("GB");
        } else {
            lcd.print("SD  ");
            showTickCross(false);
            digitalWrite(SD_CS_PIN, HIGH);
            return false;
        }

        if (!SD.begin(SD_CS_PIN)) {
            lcd.print("Bad");
            return false;
        }

        state |= STATE_SD_READY;
        return true;
    }
#endif
    void initScreen()
    {
        initLoggerScreen();
    }
private:
    void dataIdleLoop()
    {
        if (lastPid) {
            showData(lastPid, lastValue);
            lastPid = 0;
        }
        uint16_t t = (millis() - startTime) >> 10;
        if (t != elapsed) {
            lcd.setFontSize(FONT_SIZE_MEDIUM);
            lcd.setCursor(260, 8);
            lcd.printInt(elapsed / 60, 2);
            lcd.write(':');
            lcd.setFlags(FLAG_PAD_ZERO);
            lcd.printInt(elapsed % 60, 2);
            lcd.setFlags(0);
            elapsed = t;
        }

#if ENABLE_DATA_LOG
        // flush SD data every 1KB
        if (dataSize - lastFileSize >= 1024) {
            flushFile();
            lastFileSize = dataSize;
            // display logged data size
            lcd.setCursor(260, 11);
            lcd.printInt(dataSize >> 10, 4);
        }
#endif
    }
#if USE_MPU6050
    void processAccelerometer()
    {
        accel_t_gyro_union data;
        MPU6050_readout(&data);
        dataTime = millis();
        // log x/y/z of accelerometer
        logData(PID_ACC, data.value.x_accel, data.value.y_accel, data.value.z_accel);
        // log x/y/z of gyro meter
        logData(PID_GYRO, data.value.x_gyro, data.value.y_gyro, data.value.z_gyro);
    }
#endif
    void logOBDData(byte pid)
    {
        int value;
        // send a query to OBD adapter for specified OBD-II pid

        // receive and parse the response
        if (read(pid, value)) {
            dataTime = millis();
            // log data to SD card
            logData(0x100 | pid, value);
            lastValue = value;
            lastPid = pid;
            errors = 0;
        } else {
            errors++;
            return;
        }
    }
    void showECUCap()
    {
        lcd.clear();
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        byte pid = 0;
        uint16_t col;
        uint8_t row;
        for (byte i = 0; ; pid++) {
            if (pid % 0x20 == 0) continue;
            col = (i / 15) * 52;
            row = (i % 15) << 1;
            i++;
            if (col > 280) break;
            lcd.setCursor(col, row);
            lcd.print(0x100 | pid, HEX);
            showTickCross(isValidPID(pid));
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
        state &= ~(STATE_OBD_READY | STATE_ACC_READY);
        state |= STATE_SLEEPING;
        //digitalWrite(SD_CS_PIN, LOW);
        for (uint16_t i = 0; ; i++) {
            if (i == 5) {
                lcd.backlight(false);
                lcd.clear();
            }
            if (init()) {
                int value;
                if (read(PID_RPM, value) && value > 0)
                    break;
            }
        }
        state &= ~STATE_SLEEPING;
        fileIndex++;
        lcd.backlight(true);
        setup();
    }
    void showTickCross(bool yes)
    {
        lcd.setColor(yes ? RGB16_GREEN : RGB16_RED);
        lcd.draw(yes ? tick : cross, 16, 16);
        lcd.setColor(RGB16_WHITE);
    }
    // screen layout related stuff
    void showStates()
    {
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(0, 6);
        lcd.print("OBD ");
        showTickCross(state & STATE_OBD_READY);
#if USE_MPU6050
        lcd.setCursor(0, 8);
        lcd.print("ACC ");
        showTickCross(state & STATE_ACC_READY);
#endif
    }
    void showData(byte pid, int value)
    {
        switch (pid) {
        case PID_RPM:
            lcd.setCursor(0, 2);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt((unsigned int)value % 10000, 4);
            showChart(value);
            break;
        case PID_SPEED:
            lcd.setCursor(90, 2);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt((unsigned int)value % 1000, 3);
            break;
        case PID_ENGINE_LOAD:
            lcd.setCursor(164, 2);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt(value % 100, 3);
            break;
        case PID_INTAKE_TEMP:
            if ((uint16_t)value < 1000) {
                lcd.setCursor(248, 2);
                lcd.setFontSize(FONT_SIZE_XLARGE);
                lcd.printInt(value, 3);
            }
            break;
        case PID_INTAKE_MAP:
            lcd.setCursor(164, 9);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt((uint16_t)value % 1000, 3);
            break;
        case PID_COOLANT_TEMP:
            lcd.setCursor(8, 9);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt((uint16_t)value % 1000, 3);
            break;
        case PID_DISTANCE:
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.setCursor(90, 9);
            lcd.printInt((uint16_t)value % 1000, 3);
            break;
        }
    }
    void showChart(int value)
    {
        static uint16_t pos = 0;
        if (value >= 500) {
            byte n = (value - 600) / 30;
            if (n > 130) n = 130;
            lcd.fill(pos, pos, 239 - n, 239, RGB16_CYAN);
        }
        pos = (pos + 1) % 320;
        lcd.fill(pos, pos, 110, 239);
    }
    void initLoggerScreen()
    {
        lcd.clear();
        lcd.backlight(true);
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setColor(RGB16_CYAN);
        lcd.setCursor(4, 0);
        lcd.print("ENGINE RPM");
        lcd.setCursor(104, 0);
        lcd.print("SPEED");
        lcd.setCursor(164, 0);
        lcd.print("ENGINE LOAD");
        lcd.setCursor(248, 0);
        lcd.print("INTAKE TEMP");

        lcd.setCursor(4, 7);
        lcd.print("COOLANT TEMP");
        lcd.setCursor(104, 7);
        lcd.print("DISTANCE");
        lcd.setCursor(164, 7);
        lcd.print("INTAKE MAP");

        lcd.setCursor(260, 7);
        lcd.print("ELAPSED");
        lcd.setCursor(260, 10);
        lcd.print("LOG SIZE");

        lcd.setColor(RGB16_YELLOW);
        lcd.setCursor(24, 5);
        lcd.print("rpm");
        lcd.setCursor(110, 5);
        lcd.print("km/h");
        lcd.setCursor(216, 4);
        lcd.print("%");
        lcd.setCursor(304, 4);
        lcd.print("C");
        lcd.setCursor(64, 11);
        lcd.print("C");
        lcd.setCursor(110, 12);
        lcd.print("km");
        lcd.setCursor(200, 12);
        lcd.print("kpa");
        lcd.setCursor(296, 12);
        lcd.print("KB");

        lcd.setColor(RGB16_WHITE);


        //lcd.setCursor(0, 5);
        //lcd.print("THR:   %");
        //lcd.setCursor(80, 5);
        //lcd.print("AIR:   C");
    }
    byte state;
};

static COBDLogger logger;

void setup()
{
    lcd.begin();
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setColor(RGB16_YELLOW);
    lcd.println("UNOLOGGER");
    lcd.setColor(RGB16_WHITE);

    logger.begin();
    logger.initSender();

    logger.setup();
}

void loop()
{
    logger.loop();
}
