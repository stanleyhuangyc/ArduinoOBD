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
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include "MicroLCD.h"
#include "images.h"
#include "datalogger.h"

/**************************************
* Choose SD pin here
**************************************/
//#define SD_CS_PIN SS // generic
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
* Choose LCD model here
**************************************/
LCD_SSD1306 lcd;
//LCD_ZTOLED lcd;

/**************************************
* Other options
**************************************/
#define USE_MPU6050 0
#define OBD_MIN_INTERVAL 50 /* ms */
#define GPS_DATA_TIMEOUT 2000 /* ms */

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_CONNECTED 0x4
#define STATE_GPS_READY 0x8
#define STATE_ACC_READY 0x10
#define STATE_SLEEPING 0x20

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

#if USE_MPU6050
        if (MPU6050_init() == 0) state |= STATE_ACC_READY;
        showStates();
#endif

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

        uint16_t flags = FLAG_CAR | FLAG_OBD;
        if (state & STATE_GPS_CONNECTED) flags |= FLAG_GPS;
        if (state & STATE_ACC_READY) flags |= FLAG_ACC;
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
        delay(1000);

#ifndef MEMORY_SAVING
        showECUCap();
        delay(3000);
#endif

        readSensor(PID_DISTANCE, startDistance);

        // open file for logging
        if (!(state & STATE_SD_READY)) {
            if (checkSD()) {
                state |= STATE_SD_READY;
                showStates();
            }
        }


        initScreen();
    }
    void loop()
    {
        static byte count = 0;

#ifdef GPSUART
        if (millis() - lastGPSDataTime > GPS_DATA_TIMEOUT || gps.satellites() < 3) {
            // GPS not ready
            state &= ~STATE_GPS_READY;
        } else {
            // GPS ready
            state |= STATE_GPS_READY;
        }
#endif

        logOBDData(PID_RPM);
        logOBDData(PID_SPEED);

#if USE_MPU6050
        if (state & STATE_ACC_READY) {
            processAccelerometer();
        }
#endif

        switch (count++) {
        case 0:
        case 128:
            logOBDData(PID_DISTANCE);
            break;
        case 32:
            logOBDData(PID_COOLANT_TEMP);
            break;
        case 64:
            logOBDData(PID_INTAKE_TEMP);
            break;
        case 160:
            if (isValidPID(PID_AMBIENT_TEMP))
                logOBDData(PID_AMBIENT_TEMP);
            break;
        case 192:
            if (isValidPID(PID_BAROMETRIC))
                logOBDData(PID_BAROMETRIC);
            break;
        default:
            logOBDData(PID_THROTTLE);
        }

        if ((count & 1) == 0) {
            logOBDData(PID_ENGINE_LOAD);
        } else {
            if (isValidPID(PID_INTAKE_MAP))
                logOBDData(PID_INTAKE_MAP);
            else if (isValidPID(PID_MAF_FLOW))
                logOBDData(PID_MAF_FLOW);
        }

        if (errors >= 3) {
            reconnect();
            count = 0;
        }
    }
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
private:
    void initIdleLoop()
    {
        if (state & STATE_SLEEPING) return;

        // called while initializing
        char buf[10];
        unsigned int t = (millis() - startTime) / 1000;
        sprintf(buf, "%02u:%02u", t / 60, t % 60);
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(97, 7);
        lcd.print(buf);
#ifdef GPSUART
        // detect GPS signal
        if (GPSUART.available())
            processGPS();

        if (lastGPSDataTime) {
            state |= STATE_GPS_READY;
        }
#endif
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
    void dataIdleLoop()
    {
        if (GPSUART.available())
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

        dataTime = millis();

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
                uint32_t time;
                uint32_t date;
                gps.get_datetime(&date, &time, 0);
                logData(PID_GPS_TIME, time, date);
                logData(PID_GPS_ALTITUDE, (float)gps.altitude());
            }

            lcd.setFont(FONT_SIZE_SMALL);
            if (((unsigned int)dataTime >> 11) & 1) {
                char buf[16];
                sprintf(buf, "LAT:%d.%05ld  ", (int)(lat / 100000), lat % 100000);
                lcd.setCursor(0, 6);
                lcd.print(buf);
                sprintf(buf, "LON:%d.%05ld  ", (int)(lon / 100000), lon % 100000);
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
            char buf[7];
            sprintf(buf, "%4uKB", (int)(dataSize >> 10));
            lcd.setFont(FONT_SIZE_SMALL);
            lcd.setCursor(92, 7);
            lcd.print(buf);
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
    void reconnect()
    {
        closeFile();
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
            lcd.draw((state & STATE_GPS_CONNECTED) ? tick : cross, 32, 48, 16, 16);
        }
    }
    virtual void showSensorData(byte pid, int value)
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
    virtual void showGForce(int g)
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
    virtual void initScreen()
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
};

static COBDLogger logger;

void setup()
{
    lcd.begin();
    lcd.backlight(true);
    lcd.setFont(FONT_SIZE_MEDIUM);
    lcd.println("OBD/GPS Logger");
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

    delay(500);

    //lcd.setColor(0x7FF);
    lcd.setCursor(0, 2);
    logger.checkSD();
    logger.setup();
}

void loop()
{
    logger.loop();
}
