/*************************************************************************
* Arduino GPS/OBD-II/G-Force Data Logger
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <OBD.h>
#include <SPI.h>
#include <SD.h>
#include <MPU6050.h>
#include "MicroLCD.h"
#include "images.h"
#include "config.h"
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_ACC_READY 0x10
#define STATE_SLEEPING 0x20

#define MODE_LOGGER 0
#define MODE_TIMER 1

#if USE_GPS
// GPS logging can only be enabled when there is additional hardware serial UART
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define GPSUART Serial2
#elif defined(__AVR_ATmega644P__)
#define GPSUART Serial
#endif

#ifdef GPSUART

#include <TinyGPS.h>

#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_BAUDRATE "$PMTK251,115200*1F"

TinyGPS gps;

#endif // GPSUART
#endif

static uint32_t lastFileSize = 0;
//static uint32_t lastDataTime;
static uint32_t lastGPSDataTime = 0;
static uint16_t lastSpeed = -1;
static uint32_t lastSpeedTime = 0;
static uint16_t speed = 0;
static uint16_t speedGPS = 0;
static int startDistance = 0;
static uint16_t fileIndex = 0;
static uint32_t startTime = 0;
static long lastLat = 0;
static long lastLon = 0;
static long curLat = 0;
static long curLon = 0;
static uint16_t distance = 0;

#define STAGE_IDLE 0
#define STAGE_WAIT_START 1
#define STAGE_MEASURING 2

static byte mode = MODE_DEFAULT;
static byte stage = STAGE_IDLE;

#define SPEED_THRESHOLD_1 60 /* kph */
#define SPEED_THRESHOLD_2 100 /* kph */
#define SPEED_THRESHOLD_3 200 /* kph */
#define DISTANCE_THRESHOLD 400 /* meters */

static uint16_t times[4] = {0};

static byte pollPattern[]= {
    PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_RPM, PID_SPEED, PID_THROTTLE, PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_RPM, PID_SPEED, PID_THROTTLE,
};

static byte pollPattern2[] = {
    PID_DISTANCE, PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_FUEL_LEVEL,
};

#define PATTERN_NUM sizeof(pollPattern)
#define PATTERN_NUM2 sizeof(pollPattern2)
#define POLL_INTERVAL 10000 /* ms */

class COBDLogger : public COBD, public CDataLogger
{
public:
    COBDLogger():state(0) {}
    void setup()
    {
        lastGPSDataTime = 0;

        showStates();

#if USE_MPU6050
        if (MPU6050_init() == 0) state |= STATE_ACC_READY;
        showStates();
#endif

#ifdef GPSUART
        unsigned long t = millis();
        do {
            if (GPSUART.available()) {
                char c = GPSUART.read();
                if (c == '\r') {
                    state |= STATE_GPS_FOUND;
                    break;
                }
            }
        } while (millis() - t <= 5000);
#endif

        do {
            showStates();
        } while (!init());

        state |= STATE_OBD_READY;

        showStates();

        uint16_t flags = FLAG_CAR | FLAG_OBD;
        if (state & STATE_GPS_FOUND) flags |= FLAG_GPS;
        if (state & STATE_ACC_READY) flags |= FLAG_ACC;
#if ENABLE_DATA_LOG
        uint16_t index = openFile(LOG_TYPE_DEFAULT, flags);
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(86, 0);
        if (index) {
            lcd.write('[');
            lcd.setFlags(FLAG_PAD_ZERO);
            lcd.printInt(index, 5);
            lcd.setFlags(0);
            lcd.write(']');
        } else {
            lcd.print("NO LOG");
        }
        delay(100);
#endif

#ifndef MEMORY_SAVING
        //showECUCap();
        //delay(1000);
#endif

        readSensor(PID_DISTANCE, startDistance);

#if ENABLE_DATA_LOG
        // open file for logging
        if (!(state & STATE_SD_READY)) {
            if (checkSD()) {
                state |= STATE_SD_READY;
                showStates();
            }
        }
#endif

        initScreen();
    }
    void loop()
    {
        static byte index = 0;
        static byte index2 = 0;
        static uint32_t lastTime = 0;

#ifdef GPSUART
        if (millis() - lastGPSDataTime > GPS_DATA_TIMEOUT || gps.satellites() < 3) {
            // GPS not ready
            state &= ~STATE_GPS_READY;
        } else {
            // GPS ready
            state |= STATE_GPS_READY;
        }
#endif


        if (mode == MODE_TIMER) {
            timerLoop();
        } else {
            logOBDData(pollPattern[index]);
            index = (index + 1) % PATTERN_NUM;

            if (index == 0) {
                if (isValidPID(PID_INTAKE_MAP))
                    logOBDData(PID_INTAKE_MAP);
                else if (isValidPID(PID_MAF_FLOW))
                    logOBDData(PID_MAF_FLOW);
            }

            if (millis() - lastTime > POLL_INTERVAL) {
                byte pid = pollPattern2[index2];
                if (isValidPID(pid)) {
                    lastTime = millis();
                    logOBDData(pid);
                    index2 = (index2 + 1) % PATTERN_NUM2;
                }
            }

#if USE_MPU6050
            if (state & STATE_ACC_READY) {
                processAccelerometer();
            }
#endif
        }

#if ENABLE_DATA_LOG
        // flush SD data every 1KB
        if (dataSize - lastFileSize >= 1024 && stage != STAGE_MEASURING) {
            flushFile();
            lastFileSize = dataSize;
            // display logged data size
            if (mode == MODE_LOGGER) {
                char buf[7];
                sprintf(buf, "%4uKB", (int)(dataSize >> 10));
                lcd.setFont(FONT_SIZE_SMALL);
                lcd.setCursor(92, 7);
                lcd.print(buf);
            }
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
        lcd.setFont(FONT_SIZE_MEDIUM);
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
    void initScreen()
    {
        if (mode == MODE_LOGGER)
            initLoggerScreen();
        else
            initTimerScreen();
    }
private:
    void dataIdleLoop()
    {
        if (state & STATE_SLEEPING) return;

#ifdef GPSUART
        // detect GPS signal
        if (GPSUART.available())
            processGPS();

        if (lastGPSDataTime) {
            state |= STATE_GPS_READY;
        }
#endif

        if (getState() == OBD_CONNECTED)
            return;

        // called while initializing
        char buf[10];
        unsigned int t = (millis() - startTime) / 1000;
        sprintf(buf, "%02u:%02u", t / 60, t % 60);
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(97, 7);
        lcd.print(buf);
#if USE_MPU6050
        if (state & STATE_ACC_READY) {
            accel_t_gyro_union data;
            MPU6050_readout(&data);
            char buf[8];
            lcd.setFont(FONT_SIZE_SMALL);
            int temp = (data.value.temperature + 12412) / 340;
            sprintf(buf, "TEMP%3dC", temp);
            lcd.setCursor(80, 2);
            lcd.print(buf);

            sprintf(buf, "AX%3d", data.value.x_accel / 160);
            lcd.setCursor(98, 3);
            lcd.print(buf);
            sprintf(buf, "AY%3d", data.value.y_accel / 160);
            lcd.setCursor(98, 4);
            lcd.print(buf);
            sprintf(buf, "AZ%3d", data.value.z_accel / 160);
            lcd.setCursor(98, 5);
            lcd.print(buf);

            sprintf(buf, "GX%3d", data.value.x_gyro / 256);
            lcd.setCursor(64, 3);
            lcd.print(buf);
            sprintf(buf, "GY%3d", data.value.y_gyro / 256);
            lcd.setCursor(64, 4);
            lcd.print(buf);
            sprintf(buf, "GZ%3d", data.value.z_gyro / 256);
            lcd.setCursor(64, 5);
            lcd.print(buf);
            delay(50);
        }
#endif
    }
#ifdef GPSUART
    void processGPS()
    {
        // process GPS data
        char c = GPSUART.read();
        if (!gps.encode(c))
            return;

        // parsed GPS data is ready
        static uint32_t lastAltTime = 0;

        dataTime = millis();

        speedGPS = (uint16_t)(gps.speed() * 1852 / 100);
        if (speedGPS > 1000) speedGPS = 0;
        logData(PID_GPS_SPEED, speedGPS);

        // no need to log GPS data when vehicle has not been moving
        // that's when previous speed is zero and current speed is also zero
        byte sat = gps.satellites();
        if (sat >= 3 && sat < 100) {
            // lastSpeed will be updated
            //ShowSensorData(PID_SPEED, speed);

            gps.get_position(&curLat, &curLon, 0);

            // calclate distance
            if (lastLat) {
                int16_t latDiff = curLat - lastLat;
                int16_t lonDiff = curLon - lastLon;
                distance = sqrt(latDiff * latDiff + lonDiff * lonDiff);
            }

            logData(PID_GPS_COORDINATES, (float)curLat / 100000, (float)curLon / 100000);

            if (dataTime - lastAltTime > 10000) {
                uint32_t time;
                uint32_t date;
                gps.get_datetime(&date, &time, 0);
                logData(PID_GPS_TIME, time, date);
                logData(PID_GPS_ALTITUDE, (float)gps.altitude());
            }

            if (mode == MODE_LOGGER) {
                lcd.setFont(FONT_SIZE_SMALL);
                if (((unsigned int)dataTime >> 11) & 1) {
                    char buf[16];
                    sprintf(buf, "LAT:%d.%05ld  ", (int)(curLat / 100000), curLat % 100000);
                    lcd.setCursor(0, 6);
                    lcd.print(buf);
                    sprintf(buf, "LON:%d.%05ld  ", (int)(curLon / 100000), curLon % 100000);
                    lcd.setCursor(0, 7);
                    lcd.print(buf);
                } else {
                    char buf[16];
                    lcd.setCursor(0, 6);
                    sprintf(buf, "SAT:%u  ", (unsigned int)sat);
                    lcd.print(buf);
                    lcd.setCursor(0, 7);
                    uint32_t time;
                    gps.get_datetime(0, &time, 0);
                    sprintf(buf, "TIME:%08ld ", time);
                    lcd.print(buf);
                }
            }
        }
        lastGPSDataTime = dataTime;
    }
#endif
#if USE_MPU6050
    void processAccelerometer()
    {
        accel_t_gyro_union data;
        MPU6050_readout(&data);
        dataTime = millis();
        // log x/y/z of accelerometer
        logData(PID_ACC, data.value.x_accel, data.value.y_accel, data.value.z_accel);
        //showGForce(data.value.y_accel);
        // log x/y/z of gyro meter
        logData(PID_GYRO, data.value.x_gyro, data.value.y_gyro, data.value.z_gyro);
    }
#endif
    void logOBDData(byte pid)
    {
        int value;
#ifdef OBD_MIN_INTERVAL
        uint32_t start = millis();
#endif

        // send a query to OBD adapter for specified OBD-II pid
        if (readSensor(pid, value)) {
            dataTime = millis();
            showLoggerData(pid, value);
            // log data to SD card
            logData(0x100 | pid, value);
        }

        // if OBD response is very fast, go on processing other data for a while
#ifdef OBD_MIN_INTERVAL
        while (millis() - start < OBD_MIN_INTERVAL) {
            dataIdleLoop();
        }
#endif
    }
    void timerLoop()
    {
        uint32_t elapsed = millis() - startTime;
        uint16_t n;

        int speed;
        if (!readSensor(PID_SPEED, speed))
            return;

        dataTime = millis();

        lcd.setFont(FONT_SIZE_XLARGE);
        if (lastSpeed != speed) {
            lcd.setCursor(0, 4);
            lcd.printInt((unsigned int)speed % 1000, 3);
            lastSpeed = speed;
        }

        if (!(state & STATE_GPS_READY)) {
            // estimate distance
           distance += (uint32_t)(speed + lastSpeed) * (dataTime - lastSpeedTime) / 2 / 3600;
        }
        lastSpeedTime = dataTime;

        if (stage == STAGE_WAIT_START) {
            if (speed > 0) {
                stage = STAGE_MEASURING;
                startTime = lastSpeedTime;

                uint32_t t = dataTime;
                dataTime = lastSpeedTime;
                logData(0x100 | PID_SPEED, lastSpeed);
                dataTime = t;
                logData(0x100 | PID_SPEED, speed);

                lastLat = curLat;
                lastLon = curLon;
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
                lcd.setFont(FONT_SIZE_MEDIUM);
                lcd.setCursor(32, 1);
                lcd.write('.');
                lcd.write('0' + n);
            }
            if (times[2] == 0 && (speedGPS >= SPEED_THRESHOLD_3 || speed >= SPEED_THRESHOLD_3)) {
                times[2] = elapsed / 100;
                stage = STAGE_IDLE;
                lcd.clearLine(0);
                lcd.clearLine(1);
                lcd.clearLine(2);
                showTimerResults();
                lcd.setFont(FONT_SIZE_MEDIUM);
                lcd.setCursor(0, 0);
                lcd.print("DONE!");
            } else if (times[1] == 0 && (speedGPS >= SPEED_THRESHOLD_2 || speed >= SPEED_THRESHOLD_2)) {
                times[1] = elapsed / 100;
                showTimerResults();
            } else if (times[0] == 0 && (speedGPS >= SPEED_THRESHOLD_1 || speed >= SPEED_THRESHOLD_1)) {
                times[0] = elapsed / 100;
                showTimerResults();
            } else if (speed == 0) {
                // speed go back to 0
                stage = STAGE_IDLE;
            }
            if (distance > 0) {
                lcd.setFont(FONT_SIZE_SMALL);
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
            // log speed data
            logData(0x100 | PID_SPEED, speed);
            // log additional data
            int rpm;
            if (readSensor(PID_RPM, rpm)) {
                dataTime = millis();
                logData(0x100 | PID_RPM, rpm);
            }
        } else {
            if (speed == 0) {
                stage = STAGE_WAIT_START;
                initTimerScreen();
                lcd.setFont(FONT_SIZE_MEDIUM);
                lcd.setCursor(0, 0);
                lcd.println(" GET");
                lcd.println("READY");
                delay(500);
            }
        }
    }
#ifndef MEMORY_SAVING
    void showECUCap()
    {
        char buffer[24];
        byte pidlist[] = {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD, PID_ABS_ENGINE_LOAD, PID_MAF_FLOW, PID_INTAKE_MAP, PID_FUEL_LEVEL, PID_FUEL_PRESSURE, PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_TIMING_ADVANCE, PID_BAROMETRIC};
        const char* namelist[] = {"RPM", "SPEED", "THROTTLE", "ENG.LOAD1", "ENG.LOAD2", "MAF", "MAP", "FUEL LV.", "FUEL PRE.", "COOLANT", "INTAKE","AMBIENT", "IGNITION", "BARO"};
        byte i = 0;
        lcd.clear();
        lcd.setFont(FONT_SIZE_SMALL);
        for (; i < sizeof(pidlist) / sizeof(pidlist[0]) / 2; i++) {
            lcd.setCursor(0, i);
            sprintf(buffer, "%s:%c", namelist[i], isValidPID(pidlist[i]) ? 'Y' : 'N');
            lcd.print(buffer);
        }
        for (byte row = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++, row++) {
            lcd.setCursor(64, row);
            sprintf(buffer, "%s:%c", namelist[i], isValidPID(pidlist[i]) ? 'Y' : 'N');
            lcd.print(buffer);
        }
    }
#endif
    void reconnect()
    {
#if ENABLE_DATA_LOG
        closeFile();
#endif
        lcd.clear();
        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.print("Reconnecting");
        startTime = millis();
        state &= ~(STATE_OBD_READY | STATE_ACC_READY);
        state |= STATE_SLEEPING;
        //digitalWrite(SD_CS_PIN, LOW);
        for (int i = 0; !init(); i++) {
            if (i == 10) lcd.clear();
        }
        state &= ~STATE_SLEEPING;
        fileIndex++;
        write('\r');
        setup();
    }
    byte state;

    // screen layout related stuff
    void showStates()
    {
        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.setCursor(0, 2);
        lcd.print("OBD");
        lcd.draw((state & STATE_OBD_READY) ? tick : cross, 32, 16, 16, 16);
        lcd.setCursor(0, 4);
        lcd.print("ACC");
        lcd.draw((state & STATE_ACC_READY) ? tick : cross, 32, 32, 16, 16);
        lcd.setCursor(0, 6);
        if (!(state & STATE_GPS_READY)) {
            lcd.print("GPS");
            lcd.draw((state & STATE_GPS_FOUND) ? tick : cross, 32, 48, 16, 16);
        }
    }
    void showLoggerData(byte pid, int value)
    {
        char buf[8];
        switch (pid) {
        case PID_RPM:
            lcd.setCursor(64, 0);
            lcd.setFont(FONT_SIZE_XLARGE);
            lcd.printInt((unsigned int)value % 10000, 4);
            break;
        case PID_SPEED:
            if (lastSpeed != value) {
                lcd.setCursor(0, 0);
                lcd.setFont(FONT_SIZE_XLARGE);
                lcd.printInt((unsigned int)value % 1000, 3);
                lastSpeed = value;
            }
            break;
        case PID_THROTTLE:
            lcd.setCursor(24, 5);
            lcd.setFont(FONT_SIZE_SMALL);
            lcd.printInt(value % 100, 3);
            break;
        case PID_INTAKE_TEMP:
            if (value < 1000) {
                lcd.setCursor(102, 5);
                lcd.setFont(FONT_SIZE_SMALL);
                lcd.printInt(value, 3);
            }
            break;
        case PID_DISTANCE:
            if ((unsigned int)value >= startDistance) {
                sprintf(buf, "%4ukm", ((unsigned int)value - startDistance) % 1000);
                lcd.setFont(FONT_SIZE_SMALL);
                lcd.setCursor(92, 6);
                lcd.print(buf);
            }
            break;
        }
    }
    void showGForce(int g)
    {
        byte n;
        /* 0~1.5g -> 0~8 */
        g /= 85 * 25;
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(0, 3);
        if (g == 0) {
            lcd.clearLine(1);
        } else if (g < 0 && g >= -10) {
            for (n = 0; n < 10 + g; n++) {
                lcd.write(' ');
            }
            for (; n < 10; n++) {
                lcd.write('<');
            }
            lcd.print("        ");
        } else if (g > 0 && g < 10) {
            lcd.print("        ");
            for (n = 0; n < g; n++) {
                lcd.write('>');
            }
            for (; n < 10; n++) {
                lcd.write(' ');
            }
        }
    }
    void initLoggerScreen()
    {
        lcd.clear();
        lcd.backlight(true);
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(24, 3);
        lcd.print("km/h");
        lcd.setCursor(110, 3);
        lcd.print("rpm");
        lcd.setCursor(0, 5);
        lcd.print("THR:   %");
        lcd.setCursor(80, 5);
        lcd.print("AIR:   C");
    }
    void showTimerResults()
    {
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(56, 0);
        lcd.print(" 0~60:  --");
        lcd.setCursor(56, 2);
        lcd.print("0~100:  --");
        lcd.setCursor(56, 4);
        lcd.print("0~200:  --");
        lcd.setCursor(56, 6);
        lcd.print(" 400m:  --");
        lcd.setFont(FONT_SIZE_MEDIUM);
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
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(24, 7);
        lcd.print("km/h");
    }
};

static COBDLogger logger;

void setup()
{
#ifdef DEBUG
    DEBUG.begin(38400);
#endif

    lcd.begin();
    lcd.backlight(true);
    lcd.setFont(FONT_SIZE_MEDIUM);
    lcd.println("OBD Logger");
    lcd.println("Initializing");

    logger.begin();
    logger.initSender();

#ifdef GPSUART
#ifdef GPS_OPEN_BAUDRATE
    GPSUART.begin(GPS_OPEN_BAUDRATE);
    delay(10);
    GPSUART.println(PMTK_SET_BAUDRATE);
    GPSUART.end();
#endif
    GPSUART.begin(GPS_BAUDRATE);
    // switching to 10Hz mode, effective only for MTK3329
    //GPSUART.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    GPSUART.println(PMTK_SET_NMEA_UPDATE_10HZ);
#endif

#ifdef MODE_SWITCH_PIN
    pinMode(MODE_SWITCH_PIN, INPUT);
#endif

#if ENABLE_DATA_LOG
    logger.checkSD();
#else
    lcd.clear();
#endif
    logger.setup();

#ifdef MODE_SWITCH_PIN
    if (digitalRead(MODE_SWITCH_PIN) == 0) {
        delay(500);
        if (digitalRead(MODE_SWITCH_PIN) == 0) {
            mode = 1 - mode;
            logger.initScreen();
            while (digitalRead(MODE_SWITCH_PIN) == 0);
        }
    }
#endif
}

void loop()
{
    logger.loop();
}
