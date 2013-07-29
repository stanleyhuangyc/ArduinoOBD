/*************************************************************************
* Arduino GPS/OBD-II/G-Force Data Logger
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <OBD.h>
#include <SD.h>
#include <MultiLCD.h>
#include <TinyGPS.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include "images.h"
#include "datalogger.h"

/**************************************
* Choose SD pin here
**************************************/
//#define SD_CS_PIN 4 // ethernet shield
//#define SD_CS_PIN 7 // microduino
//#define SD_CS_PIN 10 // SD breakout
#define SD_CS_PIN SS

/**************************************
* Config GPS here
**************************************/
#define USE_GPS
#define GPS_BAUDRATE 38400 /* bps */
//#define GPS_OPEN_BAUDRATE 4800 /* bps */

/**************************************
* Other options
**************************************/
//#define OBD_MIN_INTERVAL 200 /* ms */
#define GPS_DATA_TIMEOUT 2000 /* ms */

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_CONNECTED 0x4
#define STATE_GPS_READY 0x8
#define STATE_ACC_READY 0x10
#define STATE_DATE_SAVED 0x20

// additional PIDs (non-OBD)
#define PID_GPS_DATETIME 0xF0
#define PID_GPS_SPEED 0xF01
#define PID_GPS_COORDINATE 0xF2
#define PID_GPS_ALTITUDE 0xF3
#define PID_ACC 0xF8
#define PID_GYRO 0xF9

#ifdef USE_GPS
// GPS logging can only be enabled when there is additional hardware serial UART
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define GPSUART Serial2
#elif defined(__AVR_ATmega644P__)
#define GPSUART Serial1
#endif

#ifdef GPSUART

#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_BAUDRATE "$PMTK251,115200*1F"

TinyGPS gps;

#endif // GPSUART
#endif

LCD_ILI9325D lcd; /* for 2.8" TFT shield */
#define LCD_LINES 24
#define CHAR_WIDTH 9

#define RGB16(r,g,b) (((uint16_t)(r >> 3) << 11) | ((uint16_t)(g >> 2) << 5) | (b >> 2))
#define RGB16_RED 0xF800
#define RGB16_GREEN 0x7E0
#define RGB16_BLUE 0x1F
#define RGB16_WHITE 0xFFFF

static uint32_t lastFileSize = 0;
static uint32_t lastDataTime = 0;
static uint32_t lastGPSDataTime = 0;
static uint16_t lastRefreshTime = 0;
static uint16_t lastSpeed = -1;
static int startDistance = 0;
static uint16_t fileIndex = 0;
static uint32_t startTime = 0;

// cached data to be displayed
static byte lastPID = 0;
static int lastData;

class COBDLogger : public COBD, public CDataLogger
{
public:
    COBDLogger():state(0) {}
    void setup()
    {
        lastGPSDataTime = 0;

        showStates();

        if (MPU6050_init() == 0) state |= STATE_ACC_READY;
        showStates();

#ifdef GPSUART
        unsigned long t = millis();
        do {
            if (GPSUART.available() && GPSUART.read() == '\r') {
                state |= STATE_GPS_CONNECTED;
                break;
            }
        } while (millis() - t <= 2000);
#endif

        do {
            showStates();
        } while (!init());

        state |= STATE_OBD_READY;

        showStates();

        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.setCursor(0, 14);
        lcd.print("VIN: XXXXXXXX");

        showECUCap();
        delay(3000);

        readSensor(PID_DISTANCE, startDistance);

        // open file for logging
        if (!(state & STATE_SD_READY)) {
            if (checkSD()) {
                state |= STATE_SD_READY;
                showStates();
            }
        }

        openFile(fileIndex);

        initScreen();

        lastDataTime = millis();
        lastRefreshTime = lastDataTime >> 10;
    }
    void loop()
    {
        static byte count = 0;
        byte dataCount;
        uint32_t t = millis();

        logOBDData(PID_RPM);
        logOBDData(PID_SPEED);
        logOBDData(PID_THROTTLE);
        dataCount = 3;

        switch (count++) {
        case 0:
        case 128:
            logOBDData(PID_DISTANCE);
            dataCount++;
            break;
        case 32:
            logOBDData(PID_COOLANT_TEMP);
            dataCount++;
            break;
        case 64:
            logOBDData(PID_INTAKE_TEMP);
            dataCount++;
            break;
        case 160:
            if (isValidPID(PID_AMBIENT_TEMP)) {
                logOBDData(PID_AMBIENT_TEMP);
                dataCount++;
            }
            break;
        case 192:
            if (isValidPID(PID_BAROMETRIC)) {
                logOBDData(PID_BAROMETRIC);
                dataCount++;
            }
            break;
        }
        if ((count & 1) == 0) {
            logOBDData(PID_ENGINE_LOAD);
            dataCount++;
        } else {
            if (isValidPID(PID_INTAKE_MAP)) {
                logOBDData(PID_INTAKE_MAP);
                dataCount++;
            } else if (isValidPID(PID_MAF_FLOW)) {
                logOBDData(PID_MAF_FLOW);
                dataCount++;
            }
        }

        if (dataTime >> 10 != lastRefreshTime) {
            char buf[10];
            unsigned int sec = (dataTime - startTime) / 1000;
            sprintf(buf, "%02u:%02u", sec / 60, sec % 60);
            lcd.setFont(FONT_SIZE_MEDIUM);
            lcd.setCursor(250, 2);
            lcd.print(buf);
            lcd.setCursor(250, 11);
            lcd.printInt((uint16_t)(dataTime - t) / dataCount);
            lcd.setFont(FONT_SIZE_SMALL);
            lcd.print(" ms");

            dataCount = 0;
            lastRefreshTime = dataTime >> 10;
        }

        if (errors >= 3) {
            reconnect();
            count = 0;
        }

#ifdef GPSUART
        if (millis() - lastGPSDataTime > GPS_DATA_TIMEOUT || gps.satellites() < 3) {
            // GPS not ready
            state &= ~STATE_GPS_READY;
        } else {
            // GPS ready
            state |= STATE_GPS_READY;
        }
#endif
        if (state & STATE_ACC_READY) {
            processAccelerometer();
        }
    }
    bool checkSD()
    {
        Sd2Card card;
        SdVolume volume;
        state &= ~STATE_SD_READY;
        pinMode(SS, OUTPUT);
        lcd.setCursor(0, 4);

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

            sprintf(buf, "%dMB", (int)volumesize);
            lcd.print(buf);
        } else {
            lcd.print("No SD Card      ");
            return false;
        }

        lcd.setCursor(0, 6);
        if (!SD.begin(SD_CS_PIN)) {
            lcd.print("Bad SD");
            return false;
        }

        char filename[13];
        // now determine log file name
        for (fileIndex = 1; fileIndex; fileIndex++) {
            sprintf(filename, FILE_NAME_FORMAT, fileIndex);
            if (!SD.exists(filename)) {
                break;
            }
        }
        if (!fileIndex) {
            lcd.print("Bad File");
            return false;
        }

        lcd.print("File:");
        lcd.print(filename);
        state |= STATE_SD_READY;
        return true;
    }
private:
    void initIdleLoop()
    {
        // called while initializing
        char buf[10];
        unsigned int t = (millis() - startTime) / 1000;
        sprintf(buf, "%02u:%02u", t / 60, t % 60);
        lcd.setCursor(0, 28);
        lcd.print(buf);
#ifdef GPSUART
        // detect GPS signal
        if (GPSUART.available()) {
            char c = GPSUART.read();
            if (gps.encode(c)) {
                state |= STATE_GPS_READY;
                lastGPSDataTime = millis();

                unsigned long date, time;
                gps.get_datetime(&date, &time, 0);
                long lat, lon;
                gps.get_position(&lat, &lon, 0);
                lcd.setCursor(0, 14);
                lcd.print("Time:");
                lcd.print(time);
                lcd.setCursor(0, 16);
                lcd.print("LAT: ");
                lcd.print(lat);
                lcd.setCursor(0, 18);
                lcd.print("LON: ");
                lcd.println(lon);
            }
        }
#endif
    }
#ifdef GPSUART
    void dataIdleLoop()
    {
        if (lastDataTime && GPSUART.available())
            processGPS();
    }
    void processGPS()
    {
        // process GPS data
        char c = GPSUART.read();
        if (!gps.encode(c))
            return;

        // parsed GPS data is ready
        static uint32_t lastAltTime = 0;
        uint32_t time;
        uint32_t date;

        dataTime = millis();

        gps.get_datetime(&date, &time, 0);
        logData(PID_GPS_TIME, time, date);

        uint32_t speed = gps.speed() * 1852 / 100;
        logData(PID_GPS_SPEED, speed);

        // no need to log GPS data when vehicle has not been moving
        // that's when previous speed is zero and current speed is also zero
        byte sat = gps.satellites();
        if (sat >= 3 && sat < 100) {
            // lastSpeed will be updated
            //ShowSensorData(PID_SPEED, speed);

            long lat, lon;
            gps.get_position(&lat, &lon, 0);
            logData(PID_GPS_COORDINATES, (float)lat / 100000, (float)lon / 100000);

            if (dataTime - lastAltTime > 10000) {
                logData(PID_GPS_ALTITUDE, (float)gps.altitude());
            }

            lcd.setFont(FONT_SIZE_MEDIUM);
            char buf[16];
            sprintf(buf, "%d.%ld", (int)(lat / 100000), lat % 100000);
            lcd.setCursor(50, 18);
            lcd.print(buf);
            sprintf(buf, "%d.%ld", (int)(lon / 100000), lon % 100000);
            lcd.setCursor(50, 21);
            lcd.print(buf);
            int32_t alt = gps.altitude();
            if (alt < 1000000) {
                sprintf(buf, "%dm ", (int)(alt / 100));
                lcd.setCursor(50, 24);
                lcd.print(buf);
            }
            lcd.setCursor(50, 27);
            lcd.printInt(gps.satellites());

        }
        lastGPSDataTime = dataTime;
    }
#endif
    void processAccelerometer()
    {
        char buf[20];
        accel_t_gyro_union data;
        MPU6050_readout(&data);
        dataTime = millis();

        lcd.setFont(FONT_SIZE_SMALL);

        sprintf(buf, "%3d", data.value.x_accel / 160);
        lcd.setCursor(197, 21);
        lcd.print(buf);
        sprintf(buf, "%3d", data.value.y_accel / 160);
        lcd.setCursor(239, 21);
        lcd.print(buf);
        sprintf(buf, "%3d", data.value.z_accel / 160);
        lcd.setCursor(281, 21);
        lcd.print(buf);

        sprintf(buf, "%3d", data.value.x_gyro / 256);
        lcd.setCursor(197, 26);
        lcd.print(buf);
        sprintf(buf, "%3d", data.value.y_gyro / 256);
        lcd.setCursor(239, 26);
        lcd.print(buf);
        sprintf(buf, "%3d", data.value.z_gyro / 256);
        lcd.setCursor(281, 26);
        lcd.print(buf);

        // log x/y/z of accelerometer
        logData(PID_ACC, data.value.x_accel, data.value.y_accel, data.value.z_accel);
        // log x/y/z of gyro meter
        logData(PID_GYRO, data.value.x_gyro, data.value.y_gyro, data.value.z_gyro);
    }
    void logOBDData(byte pid)
    {
        char buffer[OBD_RECV_BUF_SIZE];
        int value;
        uint32_t start = millis();

        // send a query to OBD adapter for specified OBD-II pid
        sendQuery(pid);
        // wait for reponse
        bool hasData;
        do {
            dataIdleLoop();
        } while (!(hasData = available()) && millis() - start < OBD_TIMEOUT_SHORT);
        // no need to continue if no data available
        if (!hasData) {
            errors++;
            return;
        }

        // display data while waiting for OBD response
        showSensorData(lastPID, lastData);

        // get response from OBD adapter
        pid = 0;
        char* data = getResponse(pid, buffer);
        if (!data) {
            // try recover next time
            write('\r');
            return;
        }
        // keep data timestamp of returned data as soon as possible
        dataTime = millis();

        // convert raw data to normal value
        value = normalizeData(pid, data);

        // log data to SD card
        logData(0x100 | pid, value);

        lastPID = pid;
        lastData = value;

        // flush SD data every 1KB
        if (dataSize - lastFileSize >= 1024) {
            flushFile();
            // display logged data size
            lcd.setFont(FONT_SIZE_MEDIUM);
            lcd.setCursor(250, 8);
            lcd.printInt((unsigned int)(dataSize >> 10));
            lcd.setFont(FONT_SIZE_SMALL);
            lcd.print(" KB");
            lastFileSize = dataSize;
        }

        // if OBD response is very fast, go on processing other data for a while
#ifdef OBD_MIN_INTERVAL
        while (millis() - start < OBD_MIN_INTERVAL) {
            dataIdleLoop();
        }
#endif
    }
    void showECUCap()
    {
        byte pidlist[] = {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD, PID_MAF_FLOW, PID_INTAKE_MAP, PID_FUEL_LEVEL, PID_FUEL_PRESSURE, PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_TIMING_ADVANCE, PID_BAROMETRIC};
        const char* namelist[] = {"ENGINE RPM", "SPEED", "THROTTLE", "ENGINE LOAD", "MAF", "MAP", "FUEL LEVEL", "FUEL PRESSURE", "COOLANT TEMP", "INTAKE TEMP","AMBIENT TEMP", "IGNITION TIMING", "BAROMETER"};

        lcd.setFont(FONT_SIZE_MEDIUM);
        for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++) {
            lcd.setCursor(160, i * 2 + 4);
            lcd.print(namelist[i]);
        }
        for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++) {
            bool valid = isValidPID(pidlist[i]);
            lcd.setColor(valid ? RGB16_GREEN : RGB16_RED);
            lcd.draw(valid ? tick : cross, 304, i * 16 + 32, 16, 16);
        }
        lcd.setColor(RGB16_WHITE);
    }
    void reconnect()
    {
        closeFile();
        lcd.clear();
        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.print("Reconnecting...");
        state &= ~(STATE_OBD_READY | STATE_ACC_READY | STATE_DATE_SAVED);
        //digitalWrite(SD_CS_PIN, LOW);
        for (int i = 0; !init(); i++) {
            if (i == 10) lcd.clear();
        }
        fileIndex++;
        write('\r');
        setup();
    }
    byte state;

    // screen layout related stuff
    void showStates()
    {
        lcd.setFont(FONT_SIZE_MEDIUM);

        lcd.setCursor(0, 8);
        lcd.print("OBD");
        lcd.setColor((state & STATE_OBD_READY) ? RGB16_GREEN : RGB16_RED);
        lcd.draw((state & STATE_OBD_READY) ? tick : cross, 36, 64, 16, 16);
        lcd.setColor(RGB16_WHITE);

        lcd.setCursor(0, 10);
        lcd.print("ACC");
        lcd.setColor((state & STATE_ACC_READY) ? RGB16_GREEN : RGB16_RED);
        lcd.draw((state & STATE_ACC_READY) ? tick : cross, 36, 80, 16, 16);
        lcd.setColor(RGB16_WHITE);

        lcd.setCursor(0, 12);
        lcd.print("GPS");
        if (state & STATE_GPS_READY) {
            lcd.setColor(RGB16_GREEN);
            lcd.draw(tick, 36, 96, 16, 16);
        } else if (state & STATE_GPS_CONNECTED)
            lcd.print(" --");
        else {
            lcd.setColor(RGB16_RED);
            lcd.draw(cross, 36, 96, 16, 16);
        }
        lcd.setColor(RGB16_WHITE);
    }
    void showSensorData(byte pid, int value)
    {
        char buf[8];
        switch (pid) {
        case PID_RPM:
            lcd.setFont(FONT_SIZE_XLARGE);
            lcd.setCursor(34, 7);
            lcd.printInt((unsigned int)value % 10000, 4);
            break;
        case PID_SPEED:
            if (lastSpeed != value) {
                lcd.setFont(FONT_SIZE_XLARGE);
                lcd.setCursor(50, 2);
                lcd.printInt((unsigned int)value % 1000, 3);
                lastSpeed = value;
            }
            break;
        case PID_THROTTLE:
            lcd.setFont(FONT_SIZE_SMALL);
            lcd.setCursor(39, 12);
            lcd.printInt(value % 100, 3);
            break;
        case PID_INTAKE_TEMP:
            lcd.setFont(FONT_SIZE_SMALL);
            lcd.setCursor(102, 12);
            lcd.printInt(value % 1000, 3);
            break;
        case PID_DISTANCE:
            if ((unsigned int)value >= startDistance) {
                lcd.setFont(FONT_SIZE_MEDIUM);
                lcd.setCursor(250, 5);
                lcd.printInt(((unsigned int)value - startDistance) % 1000);
                lcd.setFont(FONT_SIZE_SMALL);
                lcd.print(" km");
            }
            break;
        }
    }
    void initScreen()
    {
        lcd.clear();
        lcd.draw2x(frame0[0], 0, 0, 78, 58);
        lcd.draw2x(frame0[0], 164, 0, 78, 58);
        lcd.draw2x(frame0[0], 0, 124, 78, 58);
        lcd.draw2x(frame0[0], 164, 124, 78, 58);

        //lcd.setColor(RGB16(0x7f, 0x7f, 0x7f));
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(110, 4);
        lcd.print("km/h");
        lcd.setCursor(110, 8);
        lcd.print("rpm");
        lcd.setCursor(15, 12);
        lcd.print("THR:    %");
        lcd.setCursor(78, 12);
        lcd.print("AIR:    C");

        //lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.setCursor(185, 2);
        lcd.print("ELAPSED:");
        lcd.setCursor(185, 5);
        lcd.print("DISTANCE:");
        lcd.setCursor(185, 8);
        lcd.print("LOG SIZE:");
        lcd.setCursor(185, 11);
        lcd.print("OBD TIME:");

        lcd.setCursor(20, 18);
        lcd.print("LAT:");
        lcd.setCursor(20, 21);
        lcd.print("LON:");
        lcd.setCursor(20, 24);
        lcd.print("ALT:");
        lcd.setCursor(20, 27);
        lcd.print("SAT:");

        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.setCursor(185, 18);
        lcd.print("Accelerometer");
        lcd.setCursor(200, 23);
        lcd.print("Gyroscope");
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(185, 21);
        lcd.print("X:     Y:     Z:");
        lcd.setCursor(185, 26);
        lcd.print("X:     Y:     Z:");

        //lcd.setColor(0xFFFF);
        /*
        lcd.setCursor(32, 4);
        lcd.print("%");
        lcd.setCursor(68, 5);
        lcd.print("Intake Air");
        lcd.setCursor(112, 4);
        lcd.print("C");
        */
    }
};

static COBDLogger logger;

void setup()
{
    delay(100);
    lcd.begin();

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
    //GPSUART.println(PMTK_SET_NMEA_UPDATE_5HZ);
#endif
    Wire.begin();

    //lcd.clear();
    lcd.setLineHeight(8);
    lcd.setFont(FONT_SIZE_MEDIUM);
    lcd.backlight(true);
    lcd.setColor(0xFFE0);
    lcd.print("MEGA LOGGER - OBD-II/GPS/G-FORCE");
    lcd.setColor(0xFFFF);
    logger.checkSD();
    logger.setup();
}

void loop()
{
    logger.loop();
}
