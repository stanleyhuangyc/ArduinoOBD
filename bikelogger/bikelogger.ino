/*************************************************************************
* A prototype for cycling data logger (based on GPS and MEMS)
* Distributed under GPL v2.0
* Written by Stanley Huang
*************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "MultiLCD.h"
#include "config.h"
#include "datalogger.h"

#define DISPLAY_MODES 1

uint32_t start;
uint32_t distance = 0;
int speed = 0;
byte sat = 0;
uint32_t records = 0;
char heading[2];
bool acc = false;

float curLat = 0;
float curLon = 0;
int16_t alt = 0;
int16_t startAlt = 32767;
long lastLat = 0;
long lastLon = 0;
uint32_t time;


#if USE_MPU6050
#include <Wire.h>
#include <MPU6050.h>
#endif

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define GPSUART Serial2
#elif defined(__AVR_ATmega644P__)
#define GPSUART Serial1
#else
#define GPSUART Serial
#endif

#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

TinyGPS gps;

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
    static uint32_t lastTime = 0;

    // parsed GPS data is ready
    logger.dataTime = millis();

    uint32_t date;
    gps.get_datetime(&date, &time, 0);
    logger.logData(PID_GPS_TIME, time, date);

    speed = (int)(gps.speed() * 1852 / 100);
    if (speed < 1000) speed = 0;
    logger.logData(PID_GPS_SPEED, speed);

    /*
    if (sat >= 3 && gps.satellites() < 3) {
        initScreen();
    }
    */

    sat = gps.satellites();
    if (sat > 100) sat = 0;

    long lat, lon;
    gps.get_position(&lat, &lon, 0);
    curLat = (float)lat / 100000;
    curLon = (float)lon / 100000;
    logger.logData(PID_GPS_COORDINATES, curLat, curLon);

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

#if ENABLE_DATA_LOG
        // flush file every several seconds
        logger.flushFile();
#endif
    }

    alt = gps.altitude() / 100;
    if (alt > -10000 && alt < 10000) {
        logger.logData(PID_GPS_ALTITUDE, alt);
        if (sat > 5 && startAlt == 32767) {
            // save start altitude
            startAlt = alt;
        }
    } else {
        alt = 0;
    }

    records++;
}

bool CheckSD()
{
    Sd2Card card;
    SdVolume volume;

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

        lcd.print(type);
        lcd.write(' ');
        if (!volume.init(card)) {
            lcd.println("No FAT!");
            return false;
        }

        uint32_t volumesize = volume.blocksPerCluster();
        volumesize >>= 1; // 512 bytes per block
        volumesize *= volume.clusterCount();
        volumesize >>= 10;

        lcd.print((int)((volumesize + 511) / 1000));
        lcd.println("GB");
    } else {
        lcd.println("SD Error");
        return false;
    }

    if (!SD.begin(SD_CS_PIN)) {
        lcd.println("Bad SD");
        return false;
    }
    return true;
}

void initScreen()
{
    lcd.clear();
    lcd.setColor(RGB16_CYAN);
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(32, 0);
    lcd.print("Speed");
    lcd.setCursor(120, 0);
    lcd.print("Distance");
    lcd.setCursor(32, 8);
    lcd.print("Watts");
    lcd.setCursor(120, 8);
    lcd.print("Calories");
    lcd.setCursor(40, 16);
    lcd.print("Time");
    lcd.setCursor(150, 16);
    lcd.print("RPM");
    lcd.setCursor(20, 24);
    lcd.print("Altitude");
    lcd.setCursor(120, 24);
    lcd.print("Alt Gained");

    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(76, 5);
    lcd.print("km/h");
    lcd.setCursor(180, 5);
    lcd.print("km");
    lcd.setCursor(72, 28);
    lcd.print('m');
    lcd.setCursor(192, 28);
    lcd.print('m');

    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(240, 0);
    lcd.println("UTC:");
    lcd.setCursor(240, 2);
    lcd.println("LAT:");
    lcd.setCursor(240, 4);
    lcd.println("LON:");
    lcd.setCursor(240, 6);
    lcd.println("ALT:");
    lcd.setCursor(240, 8);
    lcd.println("SAT:");
    lcd.setCursor(240, 10);
    lcd.println("PTS:");
    lcd.setCursor(240, 12);
    lcd.println("KBs:");
}

#if USE_MPU6050
void displayMPU6050()
{
    accel_t_gyro_union data;
    char buf[8];
    MPU6050_readout(&data);

    int temp = (data.value.temperature + 12412) / 340;
    sprintf(buf, "TEMP%3dC", temp);
    lcd.setFontSize(FONT_SIZE_MEDIUM);
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
    lcd.begin();
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setColor(RGB16_YELLOW);
    lcd.println("Freematics GPS Logger/Meter");
    lcd.setColor(RGB16_WHITE);
    lcd.println("\r\nInitializing...");

#if ENABLE_DATA_LOG
    CheckSD();

    int index = logger.openFile();
    lcd.print("File: ");
    lcd.println(index);
#endif // ENABLE_DATA_LOG

#if USE_MPU6050
    Wire.begin();
    acc = initACC();

    lcd.print("ACC:");
    lcd.println(acc ? "YES" : "NO");
#endif

    logger.initSender();

    GPSUART.begin(GPS_BAUDRATE);
    delay(1000);

    if (GPSUART.available()) {
        // init GPS
        lcd.println("GPS:");
        byte n = 0xff;
        uint32_t tm = 0;
        start = millis();
        char progress[] = {'-', '/', '|', '\\'};
        do {
            if (!GPSUART.available()) continue;
            if (n == 0xff) {
                lcd.print("YES");
                lcd.setCursor(0, 10);
                lcd.print("SAT ");
                n = 0;
            }
            char c = GPSUART.read();

            if (sat == 0 && millis() - tm > 100) {
                lcd.setCursor(32, 12);
                lcd.write(progress[n]);
                n = (n + 1) % 4;
                tm = millis();
            }

            if (!gps.encode(c)) continue;
            sat = gps.satellites();
            if (sat > 100) sat = 0;
            lcd.setCursor(32, 12);
            lcd.printInt(sat);
        } while (sat < 3 && millis() - start < 30000);
    } else {
        lcd.println("No GPS");
    }
    delay(1000);

    //GPSUART.println(PMTK_SET_NMEA_UPDATE_10HZ);


    initScreen();

	start = millis();
}

void displayTimeSpeedDistance()
{
    uint32_t elapsed = millis() - start;
    uint16_t n;

    // display elapsed time (mm:ss:mm)
    lcd.setColor(RGB16_WHITE);
    lcd.setFontSize(FONT_SIZE_XLARGE);
    n = elapsed / 60000;
    lcd.setCursor(0, 18);
    lcd.printInt(n, 2);
    lcd.write(':');
    elapsed -= n * 60000;
    lcd.setFlags(FLAG_PAD_ZERO);
    lcd.printInt(n = elapsed / 1000, 2);
    lcd.write('.');
    elapsed -= n * 1000;
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.write(elapsed / 100 + '0');
    //if (sat < 3) return;


    lcd.setFlags(0);
    lcd.setColor(RGB16_WHITE);
    lcd.setFontSize(FONT_SIZE_XLARGE);

    // display RPM
    lcd.setCursor(120, 18);
    lcd.printInt(120, 4);

    // display speed
    n = speed / 1000;
    lcd.setCursor(0, 3);
    lcd.printInt(n, 3);
    n = speed - n * 1000;
    lcd.write('.');
    lcd.write(n / 100 + '0');

    // display distance
    lcd.setCursor(100, 3);
    lcd.printInt(n = distance / 1000, 3);
    n = distance - n * 1000;
    lcd.write('.');
    lcd.write(n / 100 + '0');

    // display watts
    lcd.setCursor(0, 10);
    lcd.printInt(123, 4);
    lcd.setCursor(120, 10);
    lcd.printInt(234, 4);

    // display altitude
    lcd.setCursor(0, 26);
    if (alt >= 0) {
        lcd.printInt(alt, 4);
    } else {
        lcd.print(" -");
        lcd.printInt(-alt, 3);
    }

    lcd.setCursor(120, 26);
    if (startAlt != 32767) {
        if (alt >= startAlt) {
            lcd.printInt(alt - startAlt, 4);
        } else {
            lcd.print(" -");
            lcd.printInt(startAlt - alt, 3);
        }
    } else {
        lcd.printInt(0, 4);
    }
}

void displayExtraInfo()
{
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setColor(RGB16_WHITE);
    lcd.setCursor(0, 0);
    lcd.write(heading[0]);
    lcd.write(heading[1]);

    lcd.setFlags(0);
    lcd.setCursor(266, 0);
    lcd.print(time);
    lcd.setCursor(266, 2);
    lcd.print(curLat, 5);
    lcd.setCursor(266, 4);
    lcd.print(curLon, 5);
    lcd.setCursor(266, 6);
    lcd.print(alt);
    lcd.print(' ');
    lcd.setCursor(266, 8);
    lcd.print(sat);
    lcd.print(' ');

    lcd.setCursor(266, 10);
    lcd.printLong(records);

    lcd.setCursor(266, 12);
    lcd.printInt(logger.dataSize >> 10);
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

    displayTimeSpeedDistance();
    displayExtraInfo();
}
