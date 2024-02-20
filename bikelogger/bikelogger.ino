/*************************************************************************
* Reference code for cycling data logger (based on GPS and MEMS)
* Works with Freematics OBD-II Telematics Advanced Kit 
* Visit http://freematics.com for more information
* Distributed under GPL v2.0
* Written by Stanley Huang
*************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <SD.h>
#include <TinyGPS.h>
#include <MultiLCD.h>
#include "config.h"
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "datalogger.h"


#define EARTH_RADIUS_KM 6371.0
#define DEG_TO_RAD (M_PI / 180.0)
#define MIN_DISTANCE_KM 0.01


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

// Constants for GPS calculations
const int KM_PER_HOUR_CONVERSION = 1852;
const int MIN_SPEED = 1000;
const int MAX_SATELLITES = 100;
const int LAT_LON_SCALE_FACTOR = 100000;
const int MIN_ALTITUDE = -10000;
const int MAX_ALTITUDE = 10000;
const uint32_t LOG_INTERVAL = 3000;
const int MIN_DISTANCE_SQUARED = 100;


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
    return MPU6050_init() == 0;
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

// Convert GPS speed to km/h
int convertSpeedToKmPerHour() {
    return (int)(gps.speed() * KM_PER_HOUR_CONVERSION / 100);
}

// Update speed and log
void updateAndLogSpeed() {
    speed = convertSpeedToKmPerHour();
    if (speed < MIN_SPEED) speed = 0;
    logger.logData(PID_GPS_SPEED, speed);
}

// Update satellite count and ensure it's within valid range
void updateSatelliteCount() {
    sat = gps.satellites();
    if (sat > MAX_SATELLITES) sat = 0;
}

// Update and log position
void updateAndLogPosition() {
    long lat, lon;
    gps.get_position(&lat, &lon, 0);
    curLat = (float)lat / LAT_LON_SCALE_FACTOR;
    curLon = (float)lon / LAT_LON_SCALE_FACTOR;
    logger.logData(PID_GPS_LATITUDE, lat);
    logger.logData(PID_GPS_LONGITUDE, lon);
}

// Update and log altitude
void updateAndLogAltitude() {
    alt = gps.altitude() / 100;
    if (alt > MIN_ALTITUDE && alt < MAX_ALTITUDE) {
        logger.logData(PID_GPS_ALTITUDE, alt);
        if (sat > 5 && startAlt == 32767) {
            startAlt = alt;
        }
    } else {
        alt = 0;
    }
}

void processGPS() {
    static uint32_t lastTime = 0;

    logger.dataTime = millis();

    uint32_t date;
    gps.get_datetime(&date, &time, 0);
    logger.logData(PID_GPS_TIME, (int32_t)time);

    updateAndLogSpeed();
    updateSatelliteCount();
    updateAndLogPosition();

    if (logger.dataTime - lastTime >= LOG_INTERVAL && speed > 0) {
        processDistanceAndHeading();

        lastTime = logger.dataTime;

#if ENABLE_DATA_LOG
        logger.flushFile();
#endif
    }

    updateAndLogAltitude();

    records++;
}

void processDistanceAndHeading() {
    // This function uses the Haversine formula to accurately calculate the distance traveled on a spherical surface, such as the Earth.
    // If computational expense is an issue, revert this to the simpler calculation of differences

    if (lastLat == 0) lastLat = curLat;
    if (lastLon == 0) lastLon = curLon;

    double latDiffRad = (curLat - lastLat) * DEG_TO_RAD;
    double lonDiffRad = (curLon - lastLon) * DEG_TO_RAD;

    double a = sin(latDiffRad / 2) * sin(latDiffRad / 2) +
                cos(lastLat * DEG_TO_RAD) * cos(curLat * DEG_TO_RAD) *
                sin(lonDiffRad / 2) * sin(lonDiffRad / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double d = EARTH_RADIUS_KM * c;

    auto getHeading = [](double diffRad) {
        return diffRad > 0 ? 'N' : diffRad < 0 ? 'S' : ' ';
    };

    if (d >= MIN_DISTANCE_KM) {
        distance += d;
        lastLat = curLat;
        lastLon = curLon;

        heading[0] = getHeading(latDiffRad);
        heading[1] = getHeading(lonDiffRad);
    }
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
    if (!checkSD()) {
        lcd.println("SD Error");
        while (true)
            ;
    }

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
    char c = GPSUART.available() ? GPSUART.read() : '\0';
    gps.encode(c) ? processGPS() : return;


#if USE_MPU6050
    processACC();
#endif

    displayTimeSpeedDistance();
    displayExtraInfo();
	
#if ENABLE_DATA_LOG
    logger.flushFile();
#endif
}
