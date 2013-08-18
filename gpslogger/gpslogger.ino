/*************************************************************************
* Arduino GPS Data Logger / Speed Meter / Odometer
* Distributed under GPL v2.0
* Written by Stanley Huang
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include "MicroLCD.h"
#include "datalogger.h"
#include "images.h"

#if defined(__AVR_ATmega644P__)
#define DISPLAY_MODES 2
#else
#define DISPLAY_MODES 1
#endif

#define USE_MPU6050 0

#define SD_CS_PIN SS
//#define SD_CS_PIN 7 // for microduino
//#define SD_CS_PIN 4 // for ethernet shield

LCD_SSD1306 lcd;

uint32_t start;
uint32_t distance = 0;
uint16_t speed = 0;
byte sat = 0;
uint16_t records = 0;
char heading[2];
bool acc = false;
byte displayMode = 0;

#if USE_MPU6050
#include <MPU6050.h>
#endif

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define GPSUART Serial2
#elif defined(__AVR_ATmega644P__)
#define GPSUART Serial1
#else
#define GPSUART Serial
#endif
#define GPS_BAUDRATE 38400

#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

TinyGPS gps;

// SD card
File sdfile;

CDataLogger logger;

void initScreen();

#if USE_MPU6050
bool initACC()
{
    if (MPU6050_init() != 0)
        return false;
    return true;

}
void processACC()
{
    accel_t_gyro_union data;
    MPU6050_readout(&data);
    logger.dataTime = millis();
    // log x/y/z of accelerometer
    logger.logData(PID_ACC, data.value.x_accel, data.value.y_accel, data.value.z_accel);
    // log x/y/z of gyro meter
    logger.logData(PID_GYRO, data.value.x_gyro, data.value.y_gyro, data.value.z_gyro);
}
#endif

void processGPS()
{
    static long lastLat = 0;
    static long lastLon = 0;
    static uint32_t lastTime = 0;
    uint32_t time;

    // parsed GPS data is ready
    logger.dataTime = millis();

    uint32_t date;
    gps.get_datetime(&date, &time, 0);
    logger.logData(PID_GPS_TIME, time, date);

    speed = (uint16_t)(gps.speed() * 1852 / 100);
    if (speed < 1000) speed = 0;
    logger.logData(PID_GPS_SPEED, speed, 0);

    if (sat >= 3 && gps.satellites() < 3) {
        initScreen();
    }
    sat = gps.satellites();

    long lat, lon;
    gps.get_position(&lat, &lon, 0);
    logger.logData(PID_GPS_COORDINATES, (float)lat / 100000, (float)lon / 100000);

    if (logger.dataTime - lastTime >= 3000 && speed > 0) {
        if (lastLat == 0) lastLat = lat;
        if (lastLon == 0) lastLon = lon;

        int16_t latDiff = lat - lastLat;
        int16_t lonDiff = lon - lastLon;

        uint16_t d = latDiff * latDiff + lonDiff * lonDiff;
        if (d >= 100) {
            distance += sqrt(d);
            lastLat = lat;
            lastLon = lon;

            if (latDiff > 0) {
                heading[0] = 'N';
            } else if (latDiff < 0) {
                heading[0] = 'S';
            } else {
                heading[0] = ' ';
            }
            if (lonDiff > 0) {
                heading[1] = 'E';
            } else if (lonDiff < 0) {
                heading[1] = 'W';
            } else {
                heading[1] = ' ';
            }
        }
        lastTime = logger.dataTime;

        // flush file every several seconds
        logger.flushFile();
    }

    long alt = gps.altitude();
    logger.logData(PID_GPS_ALTITUDE, (float)alt);

    records++;
}

bool CheckSD()
{
    Sd2Card card;
    SdVolume volume;

    lcd.setCursor(0, 0);
    lcd.setFont(FONT_SIZE_MEDIUM);
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
        lcd.setCursor(0, 0);
        lcd.print("Bad SD");
        return false;
    }
    return true;
}

void initScreen()
{
    lcd.clear();
    switch (displayMode) {
    case 0:
        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.setCursor(48, 1);
        lcd.write('.');
        lcd.setCursor(48, 6);
        lcd.write('.');
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(110, 0);
        lcd.write(':');
        lcd.setCursor(110, 3);
        lcd.print("kph");
        lcd.setCursor(64, 3);
        lcd.print("km/h");
        lcd.setCursor(76, 7);
        lcd.print("km");
        lcd.setCursor(104, 6);
        lcd.print("S:");
        break;
    default:
        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.setCursor(48, 0);
        lcd.write(':');
        lcd.setCursor(48, 3);
        lcd.write('.');
        lcd.setCursor(48, 6);
        lcd.write('.');
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(88, 1);
        lcd.write('.');
        lcd.setCursor(76, 4);
        lcd.print("kph");
        lcd.setCursor(76, 7);
        lcd.print("km");
    }
    lcd.setCursor(104, 6);
    lcd.print("S:");
}

#if USE_MPU6050
void displayMPU6050()
{
    accel_t_gyro_union data;
    char buf[8];
    MPU6050_readout(&data);

    int temp = (data.value.temperature + 12412) / 340;
    sprintf(buf, "TEMP%3dC", temp);
    lcd.setFont(FONT_SIZE_SMALL);
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
}
#endif

void setup()
{
    Wire.begin();
    lcd.begin();
    lcd.setFont(FONT_SIZE_MEDIUM);

    // init button
    pinMode(8, INPUT);

    CheckSD();

    lcd.setCursor(0, 2);
    lcd.print("ACC");
    lcd.draw(acc ? tick : cross, 32, 16, 16, 16);

    lcd.setCursor(0, 4);
    lcd.print("GPS");
    lcd.draw(cross, 32, 32, 16, 16);

    GPSUART.begin(GPS_BAUDRATE);
    logger.initSender();

#if USE_MPU6050
    acc = initACC();
#endif

    byte n = 0xff;
    uint32_t tm = 0;
    start = millis();
    char progress[] = {'-', '/', '|', '\\'};
    do {
        if (!GPSUART.available()) continue;
        if (n == 0xff) {
            // draw a tick (once)
            lcd.draw(tick, 32, 32, 16, 16);
            lcd.setCursor(0, 6);
            lcd.print("SAT ");
            n = 0;
        }
        char c = GPSUART.read();

        if (sat == 0 && millis() - tm > 100) {
            lcd.setCursor(32, 6);
            lcd.write(progress[n]);
            n = (n + 1) % 4;
            tm = millis();
        }

        if (!gps.encode(c)) continue;

#if USE_MPU6050
        if (acc) {
            displayMPU6050();
        }
#endif

        sat = gps.satellites();
        if (sat < 100) {
            lcd.setCursor(32, 6);
            lcd.printInt(sat);
        }
    } while (sat < 3 && millis() - start < 30000);

    //GPSUART.println(PMTK_SET_NMEA_UPDATE_10HZ);

    logger.openFile(LOG_TYPE_ROUTE, FLAG_CYCLING | FLAG_GPS | (acc ? FLAG_ACC : 0));

    initScreen();

	start = millis();
}

void displaySpeedDistance()
{
    uint16_t elapsed = (millis() - start) / 1000;
    uint16_t n;

    // display elapsed time (mm:ss)
    lcd.setFlags(FLAG_PAD_ZERO);
    lcd.setFont(FONT_SIZE_SMALL);
    lcd.setCursor(98, 0);
    lcd.printInt(n = elapsed / 60, 2);
    elapsed -= n * 60;
    lcd.setCursor(116, 0);
    lcd.printInt(elapsed, 2);
    lcd.setFlags(0);

    if (sat < 3) return;

    // display speed
    lcd.setFont(FONT_SIZE_XLARGE);
    n = speed / 1000;
    if (n >= 100) {
        lcd.setCursor(0, 0);
        lcd.printInt(n, 3);
    } else {
        lcd.setCursor(16, 0);
        lcd.printInt(n, 2);
    }
    lcd.setCursor(56, 0);
    n = speed - n * 1000;
    lcd.printInt(n / 100);

    // display distance
    lcd.setCursor(0, 5);
    lcd.printInt(n = distance / 1000, 3);
    lcd.setCursor(56, 5);
    n = distance - n * 1000;
    lcd.printInt(n / 100);

    // display average speed
    if (elapsed > 60) {
        uint16_t avgSpeed = distance * 36 / elapsed / 10;
        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.setCursor(102, 1);
        lcd.printInt(avgSpeed, 3);
    }
}

#if DISPLAY_MODES > 1
void displayTimeSpeedDistance()
{
    uint32_t elapsed = millis() - start;
    uint16_t n;

    // display elapsed time (mm:ss:mm)
    n = elapsed / 60000;
    if (n >= 100) {
        lcd.setCursor(0, 0);
        lcd.printInt(n, 3);
    } else {
        lcd.setCursor(16, 0);
        lcd.printInt(n, 2);
    }
    elapsed -= n * 60000;
    lcd.setFlags(FLAG_PAD_ZERO);
    lcd.setCursor(56, 0);
    lcd.printInt(n = elapsed / 1000, 2);
    elapsed -= n * 1000;
    lcd.setCursor(96, 1);
    lcd.setFont(FONT_SIZE_SMALL);
    lcd.printInt(n = elapsed / 10, 2);
    lcd.setFlags(0);

    if (sat < 3) return;

    // display speed
    lcd.setFont(FONT_SIZE_LARGE);
    n = speed / 1000;
    if (n >= 100) {
        lcd.setCursor(0, 3);
        lcd.printInt(n, 3);
    } else {
        lcd.setCursor(16, 3);
        lcd.printInt(n, 2);
    }
    lcd.setCursor(56, 3);
    n = speed - n * 1000;
    lcd.printInt(n / 100);

    // display distance
    lcd.setCursor(0, 6);
    lcd.printInt(n = distance / 1000, 3);
    lcd.setCursor(56, 6);
    n = distance - n * 1000;
    lcd.printInt(n / 100);
}
#endif

void displayMinorInfo()
{
    lcd.setFont(FONT_SIZE_SMALL);
    lcd.setCursor(0, 0);
    lcd.write(heading[0]);
    lcd.write(heading[1]);

    lcd.setFlags(0);
    lcd.setCursor(98, 7);
    lcd.printInt(records, 5);
    if (sat < 100) {
        lcd.setCursor(116, 6);
        lcd.printInt((uint16_t)sat, 2);
    }
}

void loop()
{
    if (GPSUART.available()) {
        char c = GPSUART.read();
        if (gps.encode(c)) {
            processGPS();
        } else {
            return;
        }
    }

#if USE_MPU6050
    processACC();
#endif

#if DISPLAY_MODES > 1
    switch (displayMode) {
    case 0:
        displaySpeedDistance();
        break;
    default:
        displayTimeSpeedDistance();
    }

    if (digitalRead(8) == 0) {
        delay(50);
        if (digitalRead(8) == 0) {
            displayMode = (displayMode + 1) % DISPLAY_MODES;
            while (digitalRead(8) == 0);
            initScreen();
        }
    }
#else
    displaySpeedDistance();
#endif

    displayMinorInfo();
}
