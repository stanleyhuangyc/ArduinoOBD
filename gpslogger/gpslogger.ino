/*************************************************************************
* Arduino GPS Data Logger / Speed Meter / Odometer
* Distributed under GPL v2.0
* Written by Stanley Huang
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <MultiLCD.h>
#include <SD.h>
#include <TinyGPS.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include "datalogger.h"
#include "images.h"

#define DISPLAY_MODES 2

//#define SD_CS_PIN 4 // ethernet shield
#define SD_CS_PIN 7 // microduino
//#define SD_CS_PIN 10 // SD breakout

LCD_SSD1306 lcd;
//LCD_ZTOLED lcd;

uint32_t start;
uint32_t distance = 0;
uint16_t speed = 0;
byte sat = 0;
uint16_t records = 0;
char heading[2];
bool acc;
byte displayMode = 0;

#if defined(__AVR_ATmega644P__)
#define GPSUART Serial1
#else
#define GPSUART Serial
#endif
#define GPS_BAUDRATE 38400

#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

TinyGPS gps;

// SD card
Sd2Card card;
SdVolume volume;
File sdfile;

CDataLogger logger;

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

void processGPS()
{
    static long lastLat = 0;
    static long lastLon = 0;
    static uint32_t lastTime = 0;
    uint32_t time;

    // parsed GPS data is ready
    logger.dataTime = millis();

    {
        uint32_t date;
        gps.get_datetime(&date, &time, 0);
        logger.logData(PID_GPS_TIME, time, date);
    }

    speed = (uint16_t)(gps.speed() * 1852 / 100);
    if (speed < 1000) speed = 0;
    logger.logData(PID_GPS_SPEED, speed, 0);

    sat = gps.satellites();

    {
        long lat, lon;
        gps.get_position(&lat, &lon, 0);
        logger.logData(PID_GPS_COORDINATES, (float)lat / 100000, (float)lon / 100000);

        if (logger.dataTime - lastTime >= 3000 && speed > 0) {
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
        }
    }

    {
        long alt = gps.altitude();
        logger.logData(PID_GPS_ALTITUDE, (float)alt);
    }
    records++;
}

bool CheckSD()
{
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
        lcd.setCursor(110, 1);
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

void setup()
{
    Wire.begin();
    lcd.begin();
    lcd.setFont(FONT_SIZE_MEDIUM);

    // init button
    pinMode(8, INPUT);

    CheckSD();

    unsigned long t = millis();

    lcd.setCursor(0, 2);
    lcd.print("ACC");
    lcd.draw(acc ? tick : cross, 32, 16, 16, 16);

    lcd.setCursor(0, 4);
    lcd.print("GPS");
    lcd.draw(cross, 32, 32, 16, 16);

    Serial.begin(57600);
    GPSUART.begin(GPS_BAUDRATE);
    logger.initSender();

    acc = initACC();

    byte n = 0xff;
    uint32_t tm = 0;
    start = millis();
    char progress[] = {'-', '/', '|', '\\'};
    do {
        if (!GPSUART.available()) continue;
        if (n == 0xff) {
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

        if (acc) {
            displayMPU6050();
        }

        sat = gps.satellites();
        if (sat < 100) {
            lcd.setCursor(32, 6);
            lcd.printInt(sat);
        }
    } while (sat < 3 || millis() - start < 3000);
    //} while (0);

    GPSUART.println(PMTK_SET_NMEA_UPDATE_10HZ);

    logger.openFile(LOG_TYPE_ROUTE, FLAG_CYCLING | FLAG_GPS | (acc ? FLAG_ACC : 0));

    initScreen();

	Serial.begin(9600);

	start = millis();
}

void displaySpeedDistance()
{
    // display speed
    lcd.setFont(FONT_SIZE_XLARGE);
    uint16_t n = speed / 1000;
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
    uint16_t elapsed = (millis() - start) / 1000;
    byte avgSpeed = distance * 36 / elapsed / 10;
    lcd.setFont(FONT_SIZE_SMALL);
    lcd.setCursor(92, 1);
    lcd.printInt(avgSpeed, 3);

    // display elapsed time (mm:ss)
    lcd.setFlags(FLAG_PAD_ZERO);
    lcd.setCursor(98, 0);
    lcd.printInt(n = elapsed / 60, 2);
    elapsed -= n * 60;
    lcd.setCursor(116, 0);
    lcd.printInt(elapsed, 2);

    lcd.setFlags(0);
}

void displayTimeSpeedDistance()
{
    uint32_t elapsed = millis() - start;
    uint16_t n;

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
}

void displayMinorInfo()
{
    lcd.setCursor(0, 0);
    lcd.write(heading[0]);
    lcd.write(heading[1]);

    lcd.setFlags(0);
    lcd.setFont(FONT_SIZE_SMALL);
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

    processACC();

    switch (displayMode) {
    case 0:
        displaySpeedDistance();
        break;
    default:
        displayTimeSpeedDistance();
    }

    displayMinorInfo();

    if (digitalRead(8) == 0) {
        delay(50);
        if (digitalRead(8) == 0) {
            displayMode = (displayMode + 1) % DISPLAY_MODES;
            while (digitalRead(8) == 0);
            initScreen();
        }
    }
}
