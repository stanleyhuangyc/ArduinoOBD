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
#include "Multilcd.h"
#include "TinyGPS.h"
#include "MPU6050.h"

//#define SD_CS_PIN 4 // ethernet shield with SD
#define SD_CS_PIN 7 // microduino
//#define SD_CS_PIN 10 // SD breakout

#define GPS_BAUDRATE 38400 /* bps */

#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_CONNECTED 0x4
#define STATE_GPS_READY 0x8
#define STATE_ACC_READY 0x10

// additional PIDs (non-OBD)
#define PID_GPS_DATETIME 0xF01
#define PID_GPS_COORDINATE 0xF02
#define PID_GPS_ALTITUDE 0xF03
#define PID_GPS_SPEED 0xF04
#define PID_ACC 0xF10
#define PID_GYRO 0xF11

#define OBD_MIN_INTERVAL 100 /* ms */
#define GPS_DATA_TIMEOUT 3000 /* ms */
#define FILE_NAME_FORMAT "OBD%05d.CSV"

// GPS logging can only be enabled when there is additional hardware serial UART
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define GPSUART Serial3
#elif defined(__AVR_ATmega644P__)
#define GPSUART Serial1
#endif

#ifdef GPSUART
TinyGPS gps;
#endif // GPSUART

// SD card
Sd2Card card;
SdVolume volume;
File sdfile;

// enable one LCD
LCD_OLED lcd; /* for I2C OLED module */
//LCD_PCD8544 lcd; /* for LCD4884 shield or Nokia 5100 screen module */
//LCD_1602 lcd; /* for LCD1602 shield */

static uint32_t fileSize = 0;
static uint32_t lastFileSize = 0;
static uint32_t lastDataTime;
static uint32_t lastGPSDataTime = 0;
static int startDistance = 0;
static uint16_t fileIndex = 0;

class CLogger : public COBD
{
public:
    CLogger():state(0) {}
    void InitIdleLoop()
    {
        // called while initializing
#ifdef GPSUART
        // detect GPS signal
        if (GPSUART.available()) {
            if (gps.encode(GPSUART.read())) {
                state |= STATE_SD_READY;
                lastGPSDataTime = millis();
            }
        }
        if (state & STATE_ACC_READY) {
            accel_t_gyro_union data;
            MPU6050_readout(&data);
            char buf[8];
            sprintf(buf, "X:%4d", data.value.x_accel / 190);
            lcd.setCursor(9, 1);
            lcd.print(buf);
            sprintf(buf, "Y:%4d", data.value.y_accel / 190);
            lcd.setCursor(9, 2);
            lcd.print(buf);
            sprintf(buf, "Z:%4d", data.value.z_accel / 190);
            lcd.setCursor(9, 3);
            lcd.print(buf);
            delay(100);
        }
#endif
    }
    void DataIdleLoop()
    {
        // called while waiting for OBD-II response
#ifdef GPSUART
        if (GPSUART.available())
            ProcessGPS();
#endif
    }
    void Setup()
    {
        ShowStates();

        if (MPU6050_init() == 0) state |= STATE_ACC_READY;
        ShowStates();

#ifdef GPSUART
        unsigned long t = millis();
        do {
            if (GPSUART.available()) {
                state |= STATE_GPS_CONNECTED;
                break;
            }
        } while (millis() - t <= 1000);
#endif

        do {
            ShowStates();
        } while (!Init());

        state |= STATE_OBD_READY;

        ShowStates();
        delay(1000);

        ReadSensor(PID_DISTANCE, startDistance);

        // open file for logging
        if (!(state & STATE_SD_READY)) {
            do {
                delay(1000);
            } while (!CheckSD());
            state |= STATE_SD_READY;
            ShowStates();
        }

        fileSize = 0;
        char filename[13];
        sprintf(filename, FILE_NAME_FORMAT, fileIndex);
        sdfile = SD.open(filename, FILE_WRITE);
        if (!sdfile) {
            lcd.setCursor(0, 0);
            lcd.print("File Error");
            delay(3000);
        }

        InitScreen();
        lastDataTime = millis();
    }
    void Loop()
    {
        static byte count = 0;

        LogData(PID_RPM);

        if (millis() - lastGPSDataTime > GPS_DATA_TIMEOUT) {
            // GPS not ready
            state &= ~STATE_GPS_READY;
            LogData(PID_SPEED);
        } else {
            // GPS ready
            state |= STATE_GPS_READY;
        }

        LogData(PID_THROTTLE);

        if (state & STATE_ACC_READY) {
            ProcessAccelerometer();
        }

        switch (count++) {
        case 0:
        case 64:
        case 128:
        case 192:
            LogData(PID_DISTANCE);
            break;
        case 4:
            LogData(PID_COOLANT_TEMP);
            break;
        case 20:
            LogData(PID_INTAKE_TEMP);
            break;
        }

        if (errors >= 5) {
            Reconnect();
            count = 0;
        }
    }
    bool CheckSD()
    {
        lcd.setCursor(0, 0);
        state &= ~STATE_SD_READY;
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

            sprintf(buf, "%dGB", (int)(volumesize + 511) >> 10);
            lcd.print(buf);
        } else {
            lcd.print("No SD Card      ");
            return false;
        }

        if (!SD.begin(SD_CS_PIN)) {
            lcd.setCursor(8, 0);
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
            lcd.setCursor(8, 8);
            lcd.print("Bad File");
            return false;
        }

        filename[2] = '[';
        filename[8] = ']';
        filename[9] = 0;
        lcd.setCursor(9, 0);
        lcd.print(filename + 2);
        state |= STATE_SD_READY;
        return true;
    }
private:
    void ProcessGPS()
    {
        char buf[32];
        // process GPS data
        char c = GPSUART.read();
        if (!gps.encode(c))
            return;

        // parsed GPS data is ready
        uint32_t dataTime = millis();
        lastGPSDataTime = dataTime;

        uint16_t elapsed = (uint16_t)(dataTime - lastDataTime);
        unsigned long fix_age;
        int len;
        unsigned long date, time;
        gps.get_datetime(&date, &time, &fix_age);
        len = sprintf(buf, "%u,F01,%ld %ld\n", elapsed, date, time);
        sdfile.write((uint8_t*)buf, len);

        long lat, lon;
        gps.get_position(&lat, &lon, &fix_age);
        len = sprintf(buf, "%u,F02,%ld %ld\n", elapsed, lat, lon);
        sdfile.write((uint8_t*)buf, len);

        // display LAT/LON if screen is big enough
        if (((unsigned int)dataTime / 1000) & 1)
            sprintf(buf, "%d.%ld  ", (int)(lat / 100000), lat % 100000);
        else
            sprintf(buf, "%d.%ld  ", (int)(lon / 100000), lon % 100000);
        lcd.setCursor(0, 3);
        lcd.print(buf);

        unsigned int speed = (unsigned int)(gps.speed() * 1852 / 100 / 1000);
        ShowSensorData(PID_SPEED, speed);
        len = sprintf(buf, "%u,F03,%ld\n", elapsed, speed);
        sdfile.write((uint8_t*)buf, len);

        len = sprintf(buf, "%u,F04,%ld\n", elapsed, gps.altitude());
        sdfile.write((uint8_t*)buf, len);
        lastDataTime = dataTime;
    }
    void ProcessAccelerometer()
    {
        accel_t_gyro_union data;
        MPU6050_readout(&data);
        uint32_t dataTime = millis();

        ShowGForce(data.value.y_accel);

        int len;
        uint16_t elapsed = (uint16_t)(dataTime - lastDataTime);
        char buf[20];
        // log x/y/z of accelerometer
        len = sprintf(buf, "%u,F10,%d %d %d", data.value.x_accel, data.value.y_accel, data.value.z_accel);
        sdfile.write((uint8_t*)buf, len);
        // log x/y/z of gyro meter
        len = sprintf(buf, "%u,F11,%d %d %d", data.value.x_gyro, data.value.y_gyro, data.value.z_gyro);
        sdfile.write((uint8_t*)buf, len);
    }
    void LogData(byte pid)
    {
        uint32_t start = millis();

        // send a query to OBD adapter for specified OBD-II pid
        int value;
        if (ReadSensor(pid, value)) {
            // save data timestamp at first time
            uint32_t dataTime = millis();

            // display data on screen
            ShowSensorData(pid, value);

            // log data to SD card
            char buf[32];
            uint16_t elapsed = (uint16_t)(dataTime - lastDataTime);
            byte len = sprintf(buf, "%u,%X,%d\n", elapsed, pid, value);
            // log OBD data
            sdfile.write((uint8_t*)buf, len);
            fileSize += len;
            lastDataTime = dataTime;
        }

        // flush SD data every 1KB
        if (fileSize - lastFileSize >= 1024) {
            sdfile.flush();
            // display logged data size
            char buf[7];
            sprintf(buf, "%4uKB", (int)(fileSize >> 10));
            lcd.setCursor(10, 3);
            lcd.print(buf);
            lastFileSize = fileSize;
        }

        // if OBD response is very fast, go on processing GPS data for a while
        while (millis() - start < OBD_MIN_INTERVAL) {
#ifdef GPSUART
            if (GPSUART.available())
                ProcessGPS();
#endif
        }
    }
    void Reconnect()
    {
        sdfile.close();
        lcd.clear();
        lcd.print("Reconnecting...");
        state &= ~(STATE_OBD_READY | STATE_ACC_READY);
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
        lcd.setCursor(0, 1);
        lcd.print("OBD:");
        lcd.print((state & STATE_OBD_READY) ? "Yes" : "No");
        lcd.setCursor(0, 2);
        lcd.print("GPS:");
        if (state & STATE_GPS_READY)
            lcd.print("Yes");
        else if (state & STATE_GPS_CONNECTED)
            lcd.print("--");
        else
            lcd.print("No");

        lcd.setCursor(0, 3);
        lcd.print("ACC:");
        lcd.print((state & STATE_ACC_READY) ? "Yes" : "No");
    }
    virtual void ShowSensorData(byte pid, int value)
    {
        char buf[8];
        switch (pid) {
        case PID_RPM:
            sprintf(buf, "%4u", (unsigned int)value % 10000);
            lcd.setCursor(4, 2);
            lcd.print(buf);
            break;
        case PID_SPEED:
            sprintf(buf, "%3u", (unsigned int)value % 1000);
            lcd.setCursor(0, 0);
            lcd.printLarge(buf);
            break;
        case PID_THROTTLE:
            sprintf(buf, "%2d", value % 100);
            lcd.setCursor(13, 2);
            lcd.print(buf);
            break;
        case PID_DISTANCE:
            if ((unsigned int)value >= startDistance) {
                sprintf(buf, "%4ukm", ((unsigned int)value - startDistance) % 1000);
                lcd.setCursor(10, 0);
                lcd.print(buf);
            }
            break;
        }
    }
    virtual void ShowGForce(int g)
    {
        byte n;
        /* 0~2g -> 0~8 */
        g /= 190 * 25;
        lcd.setCursor(0, 1);
        if (g == 0) {
            lcd.clearLine(1);
        } else if (g < 0 && g >= -8) {
            for (n = 0; n < 8 + g; n++) {
                lcd.write(' ');
            }
            for (; n < 8; n++) {
                lcd.write('<');
            }
            lcd.print("        ");
        } else if (g > 0 && g < 8) {
            lcd.print("        ");
            for (n = 0; n < g; n++) {
                lcd.write('>');
            }
            for (; n < 8; n++) {
                lcd.write(' ');
            }
        }
    }
    virtual void InitScreen()
    {
        lcd.clear();
        lcd.backlight(true);
        lcd.setCursor(lcd.getLines() <= 2 ? 4 : 7, 0);
        lcd.print("kph");
        lcd.setCursor(0, 2);
        lcd.print("RPM:");
        lcd.setCursor(9, 2);
        lcd.print("THR:  %");
    }
};

static CLogger logger;

void setup()
{
    lcd.begin();
    lcd.clear();
    lcd.backlight(true);
    lcd.print("OBD/GPS Logger");
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");

    Wire.begin();

    // start serial communication at the adapter defined baudrate
    OBDUART.begin(OBD_SERIAL_BAUDRATE);
#ifdef GPSUART
    GPSUART.begin(GPS_BAUDRATE);
#endif

    pinMode(SS, OUTPUT);

    delay(1000);

    lcd.clear();
    logger.CheckSD();
    logger.Setup();
}

void loop()
{
    logger.Loop();
}
