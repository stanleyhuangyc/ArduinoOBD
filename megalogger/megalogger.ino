/*************************************************************************
* Arduino GPS/OBD-II/G-Force Data Logger
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "OBD.h"
#include "SD.h"
#include "MultiLCD.h"
#include "TinyGPS.h"
#include "MPU6050.h"
#include "images.h"

/**************************************
* Choose SD pin here
**************************************/
//#define SD_CS_PIN 4 // ethernet shield
//#define SD_CS_PIN 7 // microduino
#define SD_CS_PIN 10 // SD breakout

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
//#define CONSOLE Serial

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

#define FILE_NAME_FORMAT "OBD%05d.CSV"

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

// SD card
Sd2Card card;
SdVolume volume;
File sdfile;

LCD_ILI9325D lcd; /* for 2.8" TFT shield */
#define LCD_LINES 24
#define CHAR_WIDTH 9

static uint32_t fileSize = 0;
static uint32_t lastFileSize = 0;
static uint32_t lastDataTime = 0;
static uint32_t lastGPSDataTime = 0;
static uint16_t lastSpeed = -1;
static int startDistance = 0;
static uint16_t fileIndex = 0;
static uint32_t startTime = 0;

// cached data to be displayed
static byte lastPID = 0;
static int lastData;

class CLogger : public COBD
{
public:
    CLogger():state(0) {}
    void Setup()
    {
        ShowStates();

        if (MPU6050_init() == 0) state |= STATE_ACC_READY;
        ShowStates();

#ifdef GPSUART
        unsigned long t = millis();
        do {
            if (GPSUART.available() && GPSUART.read() == '\r') {
                state |= STATE_GPS_CONNECTED;
                break;
            }
        } while (millis() - t <= 1500);
#endif

        do {
            ShowStates();
        } while (!Init());

        state |= STATE_OBD_READY;

        ShowStates();

        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.setCursor(0, 14);
        lcd.print("VIN: XXXXXXXX");

        ShowECUCap();
        delay(3000);

        ReadSensor(PID_DISTANCE, startDistance);

        // open file for logging
        if (!(state & STATE_SD_READY)) {
            if (CheckSD()) {
                state |= STATE_SD_READY;
                ShowStates();
            }
        }

        fileSize = 0;
        char filename[13];
        sprintf(filename, FILE_NAME_FORMAT, fileIndex);
        sdfile = SD.open(filename, FILE_WRITE);
        if (!sdfile) {
        }

        InitScreen();
        lastDataTime = millis();
    }
    void Loop()
    {
        static byte loopCount = 0;
        static uint32_t lastTime = millis();
        static byte dataCount = 0;

        dataCount += 3;
        LogData(PID_RPM);
#ifdef GPSUART
        if (millis() - lastGPSDataTime > GPS_DATA_TIMEOUT || gps.satellites() < 3) {
            // GPS not ready
            state &= ~STATE_GPS_READY;
        } else {
            // GPS ready
            state |= STATE_GPS_READY;
        }
        LogData(PID_SPEED);
#else
        LogData(PID_SPEED);
#endif
        LogData(PID_THROTTLE);
        if (state & STATE_ACC_READY) {
            ProcessAccelerometer();
        }

        switch (loopCount++) {
        case 0:
        case 64:
        case 128:
        case 192:
            LogData(PID_DISTANCE);
            dataCount++;
            break;
        case 4:
            LogData(PID_COOLANT_TEMP);
            dataCount++;
            break;
        case 20:
            LogData(PID_INTAKE_TEMP);
            dataCount++;
            break;
        }

        uint32_t t = millis();
        if (t >> 10 != lastTime >> 10) {
            char buf[10];
            unsigned int sec = (t - startTime) / 1000;
            sprintf(buf, "%02u:%02u", sec / 60, sec % 60);
            lcd.setFont(FONT_SIZE_MEDIUM);
            lcd.setCursor(265, 2);
            lcd.print(buf);
            sprintf(buf, "%ums ", (uint16_t)(t - lastTime) / dataCount);
            lcd.setCursor(265, 11);
            lcd.print(buf);

            dataCount = 0;
            lastTime = t;
        }

        if (errors >= 5) {
            Reconnect();
            loopCount = 0;
        }
    }
    bool CheckSD()
    {
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

            sprintf(buf, "%dGB", (int)((volumesize + 511) / 1000));
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
    void InitIdleLoop()
    {
        // called while initializing
        char buf[10];
        unsigned int t = (millis() - startTime) / 1000;
        sprintf(buf, "%02u:%02u", t / 60, t % 60);
        lcd.setCursor(0, 22);
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
                lcd.setCursor(0, 8);
                lcd.print("Time:");
                lcd.print(time);
                lcd.setCursor(0, 9);
                lcd.print("LAT: ");
                lcd.print(lat);
                lcd.setCursor(0, 10);
                lcd.print("LON: ");
                lcd.println(lon);
            }
        }
#endif
    }
#ifdef GPSUART
    void DataIdleLoop()
    {
        if (lastDataTime && GPSUART.available())
            ProcessGPS();
    }
    void ProcessGPS()
    {
        // process GPS data
        char c = GPSUART.read();
        if (!gps.encode(c))
            return;

        // parsed GPS data is ready
        uint32_t dataTime = millis();
        uint16_t elapsed = (uint16_t)(dataTime - lastDataTime);
        int len;
        char buf[32];
        unsigned long date, time;
        gps.get_datetime(&date, &time, 0);
        len = sprintf(buf, "%u,F0,%ld %ld\n", elapsed, date, time);
        sdfile.write((uint8_t*)buf, len);

        unsigned int speed = (unsigned int)(gps.speed() * 1852 / 100 / 1000);

        // no need to log GPS data when vehicle has not been moving
        // that's when previous speed is zero and current speed is also zero
        if (!(speed == 0 && lastSpeed == 0) && gps.satellites() >= 3) {
            // lastSpeed will be updated
            //ShowSensorData(PID_SPEED, speed);
            len = sprintf(buf, "%u,F1,%u\n", elapsed, speed);
            sdfile.write((uint8_t*)buf, len);

            long lat, lon, alt;
            gps.get_position(&lat, &lon, 0);
            len = sprintf(buf, "%u,F2,%ld %ld\n", elapsed, lat, lon);
            sdfile.write((uint8_t*)buf, len);

            alt = gps.altitude();
            len = sprintf(buf, "%u,F3,%ld\n", elapsed, alt);
            sdfile.write((uint8_t*)buf, len);

            lcd.setFont(FONT_SIZE_MEDIUM);
            sprintf(buf, "%d.%ld", (int)(lat / 100000), lat % 100000);
            lcd.setCursor(60, 17);
            lcd.print(buf);
            sprintf(buf, "%d.%ld", (int)(lon / 100000), lon % 100000);
            lcd.setCursor(60, 20);
            lcd.print(buf);
            sprintf(buf, "%d.%02um ", (int)(alt / 100), (unsigned int)alt % 100);
            lcd.setCursor(60, 23);
            lcd.print(buf);
            lcd.setCursor(51, 26);
            lcd.printInt(gps.satellites(), 2);
        }
        lastDataTime = dataTime;
        lastGPSDataTime = dataTime;
    }
#endif
    void ProcessAccelerometer()
    {
        char buf[20];
        accel_t_gyro_union data;
        MPU6050_readout(&data);
        uint32_t dataTime = millis();

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

        int len;
        uint16_t elapsed = (uint16_t)(dataTime - lastDataTime);
        // log x/y/z of accelerometer
        len = sprintf(buf, "%u,F10,%d %d %d\n", data.value.x_accel, data.value.y_accel, data.value.z_accel);
        sdfile.write((uint8_t*)buf, len);
        // log x/y/z of gyro meter
        len = sprintf(buf, "%u,F11,%d %d %d\n", data.value.x_gyro, data.value.y_gyro, data.value.z_gyro);
        sdfile.write((uint8_t*)buf, len);
        lastDataTime = dataTime;
    }
    void LogData(byte pid)
    {
        char buffer[OBD_RECV_BUF_SIZE];
        int value;
        uint32_t start = millis();

        // send a query to OBD adapter for specified OBD-II pid
        Query(pid);
        // wait for reponse
        bool hasData;
        do {
            DataIdleLoop();
        } while (!(hasData = available()) && millis() - start < OBD_TIMEOUT_SHORT);
        // no need to continue if no data available
        if (!hasData) {
            errors++;
            return;
        }

        // display data while waiting for OBD response
        ShowSensorData(lastPID, lastData);

        // get response from OBD adapter
        pid = 0;
        char* data = GetResponse(pid, buffer);
        if (!data) {
            // try recover next time
            write('\r');
            return;
        }
        // keep data timestamp of returned data as soon as possible
        uint32_t dataTime = millis();

        // convert raw data to normal value
        value = GetConvertedValue(pid, data);

        lastPID = pid;
        lastData = value;

        // log data to SD card
        char buf[32];
        uint16_t elapsed = (uint16_t)(dataTime - lastDataTime);
        byte len = sprintf(buf, "%u,%X,%d\n", elapsed, pid, value);
        // log OBD data
        sdfile.write((uint8_t*)buf, len);
        fileSize += len;
        lastDataTime = dataTime;

        // flush SD data every 1KB
        if (fileSize - lastFileSize >= 1024) {
            sdfile.flush();
            // display logged data size
            char buf[7];
            sprintf(buf, "%uK", (int)(fileSize >> 10));
            lcd.setFont(FONT_SIZE_MEDIUM);
            lcd.setCursor(265, 8);
            lcd.print(buf);
            lastFileSize = fileSize;
        }

        // if OBD response is very fast, go on processing other data for a while
#ifdef OBD_MIN_INTERVAL
        while (millis() - start < OBD_MIN_INTERVAL) {
            DataIdleLoop();
        }
#endif
    }
    void ShowECUCap()
    {
        char buffer[24];
        byte pidlist[] = {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD, PID_MAF_FLOW, PID_INTAKE_MAP, PID_FUEL_LEVEL, PID_FUEL_PRESSURE, PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_TIMING_ADVANCE, PID_BAROMETRIC};
        const char* namelist[] = {"ENGINE RPM", "SPEED", "THROTTLE", "ENGINE LOAD", "MAF", "MAP", "FUEL LEVEL", "FUEL PRESSURE", "COOLANT TEMP", "INTAKE TEMP","AMBIENT TEMP", "IGNITION TIMING", "BAROMETER"};

        lcd.setCursor(194, 6);
        lcd.print("PID:");
        lcd.setFont(FONT_SIZE_SMALL);
        for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++) {
            lcd.setCursor(194, i + 8);
            sprintf(buffer, "%s: %s", namelist[i], IsValidPID(PID_RPM) ? "Yes" : "No");
            lcd.print(buffer);
        }
    }
    void Reconnect()
    {
        sdfile.close();
        lcd.clear();
        lcd.print("Reconnecting...");
        state &= ~(STATE_OBD_READY | STATE_ACC_READY | STATE_DATE_SAVED);
        //digitalWrite(SD_CS_PIN, LOW);
        for (int i = 0; !Init(); i++) {
            if (i == 10) lcd.clear();
        }
        fileIndex++;
        Setup();
    }
    byte state;

    // screen layout related stuff
    void ShowStates()
    {
        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.setCursor(0, 8);
        lcd.print("GPS:");
        if (state & STATE_GPS_READY)
            lcd.print("Yes");
        else if (state & STATE_GPS_CONNECTED)
            lcd.print("--");
        else
            lcd.print("No");

        lcd.setCursor(0, 10);
        lcd.print("ACC:");
        lcd.print((state & STATE_ACC_READY) ? "Yes" : "No");
        lcd.setCursor(0, 12);
        lcd.print("OBD:");
        lcd.print((state & STATE_OBD_READY) ? "Yes" : "No");
    }
    virtual void ShowSensorData(byte pid, int value)
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
                sprintf(buf, "%ukm", ((unsigned int)value - startDistance) % 1000);
                lcd.setCursor(265, 5);
                lcd.print(buf);
            }
            break;
        }
    }
    virtual void InitScreen()
    {
        //lcd.clear();
        lcd.setLineHeight(8);
        lcd.backlight(true);

        lcd.clear(156, 0, 8, 240);
        lcd.clear(0, 116, 320, 8);

        lcd.draw2x(frame0[0], 0, 0, 78, 58);
        lcd.draw2x(frame0[0], 164, 0, 78, 58);
        lcd.draw2x(frame0[0], 0, 124, 78, 58);
        lcd.draw2x(frame0[0], 164, 124, 78, 58);

        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(110, 5);
        lcd.print("km/h");
        lcd.setCursor(110, 9);
        lcd.print("rpm");
        lcd.setCursor(15, 12);
        lcd.print("THR:    %");
        lcd.setCursor(78, 12);
        lcd.print("AIR:    C");

        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.setCursor(175, 2);
        lcd.print("Elapsed:");
        lcd.setCursor(175, 5);
        lcd.print("Distance:");
        lcd.setCursor(175, 8);
        lcd.print("Log Data:");
        lcd.setCursor(175, 11);
        lcd.print("OBD Time:");

        lcd.setCursor(15, 17);
        lcd.print("LAT:");
        lcd.setCursor(15, 20);
        lcd.print("LON:");
        lcd.setCursor(15, 23);
        lcd.print("ALT:");
        lcd.setCursor(15, 26);
        lcd.print("SAT:");

        lcd.setCursor(185, 18);
        lcd.print("Accelerometer");
        lcd.setCursor(200, 23);
        lcd.print("Gyroscope");
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(185, 21);
        lcd.print("X:     Y:     Z:");
        lcd.setCursor(185, 26);
        lcd.print("X:     Y:     Z:");

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

static CLogger logger;

void setup()
{
    lcd.begin();
    //lcd.clear();
    lcd.setLineHeight(10);
    lcd.setFont(FONT_SIZE_MEDIUM);
    lcd.backlight(true);
    lcd.print("MEGA LOGGER - OBD-II/GPS/G-FORCE");
    lcd.setCursor(0, 2);
    lcd.print("Initializing...");

#ifdef CONSOLE
    CONSOLE.begin(115200);
#endif
    // start serial communication at the adapter defined baudrate
    OBDUART.begin(OBD_SERIAL_BAUDRATE);
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

    delay(500);

    //lcd.setColor(0x7FF);
    lcd.print("OK");
    logger.CheckSD();
    logger.Setup();
}

void loop()
{
    logger.Loop();
}
