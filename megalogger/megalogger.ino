/*************************************************************************
* Arduino GPS/OBD-II/G-Force Data Logger
* Distributed under GPL v2.0
* Copyright (c) 2013-2014 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
* Visit http://freematics.com for more information
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <OBD.h>
#include <SPI.h>
#include <SD.h>
#include <MultiLCD.h>
#include <TinyGPS.h>
#include <MPU6050.h>
#include "Narcoleptic.h"
#include "config.h"
#include "images.h"
#if ENABLE_DATA_OUT && USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_CONNECTED 0x4
#define STATE_GPS_READY 0x8
#define STATE_ACC_READY 0x10
#define STATE_GUI_ON 0x20

// adapter type
#define OBD_ADAPTER_I2C 0
#define OBD_ADAPTER_UART 1

#if USE_GPS
// GPS logging can only be enabled when there is additional hardware serial UART
#define GPSUART Serial2

#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_BAUDRATE "$PMTK251,115200*1F"

TinyGPS gps;

#endif

void doIdleTasks();

static uint8_t lastFileSize = 0;
static uint32_t lastGPSDataTime = 0;
static uint32_t lastACCDataTime = 0;
static uint32_t lastRefreshTime = 0;
static uint32_t distance = 0;
static uint32_t startTime = 0;
static uint16_t lastSpeed = 0;
static uint32_t lastSpeedTime = 0;
static int gpsSpeed = -1;
static uint16_t gpsDate = 0;

static byte pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static byte pidTier2[] = {PID_INTAKE_MAP, PID_MAF_FLOW, PID_TIMING_ADVANCE};
static byte pidTier3[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_ENGINE_FUEL_RATE};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)
#define TIER_NUM3 sizeof(pidTier3)

byte state = 0;

CDataLogger logger;

class CMyOBD : public COBD
{
    void dataIdleLoop()
    {
        doIdleTasks();
    }
};

class CMyOBDI2C : public COBDI2C
{
    void dataIdleLoop()
    {
        doIdleTasks();
    }
};

#if OBD_ADAPTER_MODEL == OBD_MODEL_I2C
CMyOBDI2C obd;
#else
CMyOBD obd;
#endif

void setColorByValue(int value, int threshold1, int threshold2, int threshold3)
{
    if (value < 0) value = -value;
    if (value < threshold1) {
      lcd.setColor(RGB16_WHITE);
    } else if (value < threshold2) {
      byte n = (uint32_t)(threshold2 - value) * 255 / (threshold2 - threshold1);
      lcd.setColor(255, 255, n);
    } else if (value < threshold3) {
      byte n = (uint32_t)(threshold3 - value) * 255 / (threshold3 - threshold2);
      lcd.setColor(255, n, 0);
    } else {
      lcd.setColor(255, 0, 0);
    }
}

void showPIDData(byte pid, int value)
{
    char buf[8];
    switch (pid) {
    case PID_RPM:
        lcd.setFontSize(FONT_SIZE_XLARGE);
        lcd.setCursor(32, 6);
        if (value >= 10000) break;
        setColorByValue(value, 2500, 3500, 5000);
        lcd.printInt(value, 4);
        break;
    case PID_SPEED:
        if (value < 1000) {
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.setCursor(50, 2);
            setColorByValue(value, 60, 100, 160);
            lcd.printInt(value, 3);

            if (gpsSpeed != -1) {
                lcd.setFontSize(FONT_SIZE_SMALL);
                lcd.setCursor(110, 2);
                lcd.setColor(RGB16_YELLOW);
                int diff = gpsSpeed - value;
                if (diff >= 0) {
                    lcd.write('+');
                    lcd.printInt(diff);
                } else {
                    lcd.write('-');
                    lcd.printInt(-diff);
                }
                lcd.write(' ');
            }
        }
        break;
    case PID_ENGINE_LOAD:
        lcd.setFontSize(FONT_SIZE_XLARGE);
        lcd.setCursor(50, 10);
        if (value >= 100) value = 99;
        setColorByValue(value, 75, 80, 100);
        lcd.printInt(value, 3);
        break;
    case PID_THROTTLE:
        lcd.setFontSize(FONT_SIZE_LARGE);
        lcd.setCursor(80, 21);
        if (value >= 100) value = 99;
        setColorByValue(value, 50, 75, 100);
        lcd.printInt(value, 2);
        break;
    case PID_ENGINE_FUEL_RATE:
        if (value < 100) {
            lcd.setFontSize(FONT_SIZE_LARGE);
            lcd.setCursor(80, 24);
            lcd.printInt(value, 2);
        }
        break;
    case PID_INTAKE_TEMP:
        if (value < 100) {
            lcd.setFontSize(FONT_SIZE_LARGE);
            lcd.setCursor(80, 27);
            lcd.printInt(value, 2);
        }
        break;
    case PID_VOLTAGE:
        lcd.setFontSize(FONT_SIZE_LARGE);
        lcd.setCursor(80, 18);
        lcd.printInt(value / 10, 2);
        lcd.write('.');
        lcd.printInt(value % 10);
        break;
    }
    lcd.setColor(RGB16_WHITE);
}

void initScreen()
{
    // fade out backlight
    for (int n = 254; n >= 0; n--) {
        lcd.setBackLight(n);
        delay(5);
    }
    lcd.clear();
    lcd.draw4bpp(frame0[0], 78, 58);
    lcd.setXY(164, 0);
    lcd.draw4bpp(frame0[0], 78, 58);
    lcd.setXY(0, 124);
    lcd.draw4bpp(frame0[0], 78, 58);
    lcd.setXY(164, 124);
    lcd.draw4bpp(frame0[0], 78, 58);

    lcd.setColor(RGB16_CYAN);
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(110, 4);
    lcd.print("km/h");
    lcd.setCursor(110, 8);
    lcd.print("RPM");
    lcd.setCursor(110, 11);
    lcd.print("ENGINE");
    lcd.setCursor(110, 12);
    lcd.print("LOAD %");

    //lcd.setFont(FONT_SIZE_MEDIUM);
    lcd.setColor(RGB16_CYAN);
    lcd.setCursor(184, 2);
    lcd.print("ELAPSED:");
    lcd.setCursor(184, 5);
    lcd.print("DISTANCE:        km");
    lcd.setCursor(184, 8);
    lcd.print("AVG SPEED:       kph");
    lcd.setCursor(184, 11);
    lcd.print("ALTITUDE:        m");

    lcd.setCursor(18, 19);
    lcd.print("BATTERY:            V");
    lcd.setCursor(18, 22);
    lcd.print("THROTTLE:        %");
    lcd.setCursor(18, 25);
    lcd.print("FUEL RATE:       L/h");
    lcd.setCursor(18, 28);
    lcd.print("INTAKE:          C");

    lcd.setCursor(184, 18);
    lcd.print("UTC:");
    lcd.setCursor(184, 19);
    lcd.print("LAT:");
    lcd.setCursor(280, 19);
    lcd.print("SAT:");
    lcd.setCursor(184, 20);
    lcd.print("LON:");

    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(184, 22);
    lcd.print("ACC:");
    lcd.setCursor(184, 23);
    lcd.print("GYR:");

    lcd.setCursor(184, 25);
    lcd.print("OBD FREQ:");

    lcd.setCursor(184, 26);
    lcd.print("GPS FREQ:");

    lcd.setCursor(184, 27);
    lcd.print("LOG SIZE:");

    //lcd.setColor(0xFFFF);
    /*
    lcd.setCursor(32, 4);
    lcd.print("%");
    lcd.setCursor(68, 5);
    lcd.print("Intake Air");
    lcd.setCursor(112, 4);
    lcd.print("C");
    */

    state |= STATE_GUI_ON;

    // fade in backlight
    for (int n = 1; n <= 255; n++) {
        lcd.setBackLight(n);
        delay(10);
    }
}

bool connectOBD()
{
    obd.begin();
    if (obd.init(OBD_PROTOCOL))
        return true;

#if 0
    for (byte proto = (byte)PROTO_ISO_9141_2; proto <= PROTO_CAN_11B_250K; proto++) {
        lcd.setCursor(60, 8);
        lcd.print("Protocol ");
        lcd.print(proto);
        if (obd.init((OBD_PROTOCOLS)proto))
            return true;
    }
#endif

    obd.end();
    return false;
}

bool checkSD()
{
#if ENABLE_DATA_LOG
    Sd2Card card;
    SdVolume volume;
    state &= ~STATE_SD_READY;
    pinMode(SS, OUTPUT);
    lcd.setCursor(0, 4);

    lcd.setFontSize(FONT_SIZE_MEDIUM);
    if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
        const char* type;
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

        lcd.print((int)volumesize);
        lcd.print("MB");
    } else {
        lcd.print("No SD Card");
        return false;
    }

    lcd.setCursor(0, 6);
    if (!SD.begin(SD_CS_PIN)) {
        lcd.print("Bad SD");
        return false;
    }

    state |= STATE_SD_READY;
    return true;
#else
    return false;
#endif
}

#if USE_GPS
void processGPS()
{
    // process GPS data
    char c = GPSUART.read();
    if (!gps.encode(c))
        return;

    // parsed GPS data is ready
    uint32_t time;
    uint32_t date;

    logger.dataTime = millis();

    gps.get_datetime(&date, &time, 0);
    if (date != gpsDate) {
        // log date only if it's changed
        logger.logData(PID_GPS_DATE, (int32_t)time);
        gpsDate = date;
    }
    logger.logData(PID_GPS_TIME, (int32_t)time);

    int32_t lat, lon;
    gps.get_position(&lat, &lon, 0);

    byte sat = gps.satellites();

    // show GPS data interval
    lcd.setFontSize(FONT_SIZE_SMALL);
    if (lastGPSDataTime) {
        lcd.setCursor(242, 26);
        lcd.printInt((uint16_t)logger.dataTime - lastGPSDataTime);
        lcd.print("ms");
        lcd.printSpace(2);
    }

    // keep current data time as last GPS time
    lastGPSDataTime = logger.dataTime;

    // display UTC date/time
    lcd.write(' ');
    lcd.setCursor(214, 18);
    lcd.setFlags(FLAG_PAD_ZERO);
    lcd.printLong(date, 6);
    lcd.write(' ');
    lcd.printLong(time, 8);

    // display latitude
    lcd.setCursor(214, 19);
    lcd.print(lat / 100000);
    lcd.write('.');
    lcd.printLong(abs(lat) % 100000, 5);
    // display longitude
    lcd.setCursor(214, 20);
    lcd.print(lon / 100000);
    lcd.write('.');
    lcd.printLong(abs(lon) % 100000, 5);
    // log latitude/longitude
    logger.logData(PID_GPS_LATITUDE, lat);
    logger.logData(PID_GPS_LONGITUDE, lon);

    // display number of satellites
    if (sat < 100) {
        lcd.setCursor(280, 20);
        lcd.printInt(sat);
    }
    // display altitude
    int32_t alt = gps.altitude();
    lcd.setFlags(0);
    if (alt > -1000000 && alt < 1000000) {
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(250, 11);
        lcd.print(alt / 100);
        lcd.write(' ');
        // log altitude
        logger.logData(PID_GPS_ALTITUDE, (int)(alt / 100));
    }

    // only log these data when satellite status is good
    if (sat >= 3) {
        gpsSpeed = gps.speed() * 1852 / 100000;
        logger.logData(PID_GPS_SPEED, gpsSpeed);
    }
}
#endif

void processAccelerometer()
{
#if USE_MPU6050
    logger.dataTime = millis();

    if (logger.dataTime - lastACCDataTime < ACC_DATA_INTERVAL) {
        return;
    }

    char buf[8];
    accel_t_gyro_union data;
    MPU6050_readout(&data);

    lcd.setFontSize(FONT_SIZE_SMALL);

    data.value.x_accel /= ACC_DATA_RATIO;
    data.value.y_accel /= ACC_DATA_RATIO;
    data.value.z_accel /= ACC_DATA_RATIO;
    data.value.x_gyro /= GYRO_DATA_RATIO;
    data.value.y_gyro /= GYRO_DATA_RATIO;
    data.value.z_gyro /= GYRO_DATA_RATIO;

    // display acc data
    lcd.setCursor(214, 22);
    setColorByValue(data.value.x_accel, 50, 100, 200);
    lcd.print(data.value.x_accel);
    setColorByValue(data.value.y_accel, 50, 100, 200);
    lcd.write('/');
    lcd.print(data.value.y_accel);
    setColorByValue(data.value.z_accel, 50, 100, 200);
    lcd.write('/');
    lcd.print(data.value.z_accel);
    lcd.printSpace(8);

    // display gyro data
    lcd.setCursor(214, 23);
    lcd.setColor(RGB16_WHITE);
    lcd.print(data.value.x_gyro);
    lcd.write('/');
    lcd.print(data.value.y_gyro);
    lcd.write('/');
    lcd.print(data.value.z_gyro);
    lcd.printSpace(8);

    // log x/y/z of accelerometer
    logger.logData(PID_ACC, data.value.x_accel, data.value.y_accel, data.value.z_accel);
    // log x/y/z of gyro meter
    logger.logData(PID_GYRO, data.value.x_gyro, data.value.y_gyro, data.value.z_gyro);

    lastACCDataTime = logger.dataTime;
#endif
}

void logOBDData(byte pid)
{
    char buffer[OBD_RECV_BUF_SIZE];
    uint32_t start = millis();
    int value;

    // send query for OBD-II PID
    obd.sendQuery(pid);
    // let PID parsed from response
    pid = 0;
    // read responded PID and data
    if (!obd.getResult(pid, value)) {
        return;
    }

    logger.dataTime = millis();
    // display data
    showPIDData(pid, value);

    // log data to SD card
    logger.logData(0x100 | pid, value);

    if (pid == PID_SPEED) {
        // estimate distance travelled since last speed update
        distance += (uint32_t)(value + lastSpeed) * (logger.dataTime - lastSpeedTime) / 6000;
        // display speed
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(250, 5);
        lcd.printInt(distance / 1000);
        lcd.write('.');
        lcd.printInt(((uint16_t)distance % 1000) / 100);
        // calculate and display average speed
        int avgSpeed = (unsigned long)distance * 3600 / (millis() - startTime);
        lcd.setCursor(250, 8);
        lcd.printInt(avgSpeed);

        lastSpeed = value;
        lastSpeedTime = logger.dataTime;
    }
#if ENABLE_DATA_LOG
    // flush SD data every 1KB
    if ((logger.dataSize >> 10) != lastFileSize) {
        logger.flushFile();
        // display logged data size
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setCursor(242, 27);
        lcd.print((unsigned int)(logger.dataSize >> 10));
        lcd.print("KB");
        lastFileSize = logger.dataSize >> 10;
    }
#endif

    // if OBD response is very fast, go on processing other data for a while
#ifdef OBD_MIN_INTERVAL
    while (millis() - start < OBD_MIN_INTERVAL) {
        doIdleTasks();
    }
#endif
}

void showECUCap()
{
    byte pidlist[] = {PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_FUEL_PRESSURE, PID_INTAKE_MAP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP, PID_MAF_FLOW, PID_THROTTLE, PID_AUX_INPUT,
        PID_EGR_ERROR, PID_COMMANDED_EVAPORATIVE_PURGE, PID_FUEL_LEVEL, PID_CONTROL_MODULE_VOLTAGE, PID_ABSOLUTE_ENGINE_LOAD, PID_AMBIENT_TEMP, PID_COMMANDED_THROTTLE_ACTUATOR, PID_ETHANOL_FUEL,
        PID_FUEL_RAIL_PRESSURE, PID_HYBRID_BATTERY_PERCENTAGE, PID_ENGINE_OIL_TEMP, PID_FUEL_INJECTION_TIMING, PID_ENGINE_FUEL_RATE, PID_ENGINE_TORQUE_DEMANDED, PID_ENGINE_TORQUE_PERCENTAGE};

    lcd.setFontSize(FONT_SIZE_MEDIUM);
    for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i += 2) {
        lcd.setCursor(184, i + 4);
        for (byte j = 0; j < 2; j++) {
            lcd.printSpace(2);
            lcd.print((int)pidlist[i + j] | 0x100, HEX);
            bool valid = obd.isValidPID(pidlist[i + j]);
            if (valid) {
                lcd.setColor(RGB16_GREEN);
                lcd.draw(tick, 16, 16);
                lcd.setColor(RGB16_WHITE);
            } else {
                lcd.printSpace(2);
            }
        }
    }
}

void reconnect()
{
    // fade out backlight
    for (int n = 254; n >= 0; n--) {
        lcd.setBackLight(n);
        delay(20);
    }
#if ENABLE_DATA_LOG
    logger.closeFile();
#endif
    lcd.clear();
    state &= ~(STATE_OBD_READY | STATE_GUI_ON);
    //digitalWrite(SD_CS_PIN, LOW);
    for (;;) {
        if (!obd.init())
            continue;

        int value;
        if (obd.read(PID_RPM, value) && value > 0)
            break;

        Narcoleptic.delay(1000);
    }
    // re-initialize
    state |= STATE_OBD_READY;
    startTime = millis();
    lastSpeedTime = startTime;
    lastSpeed = 0;
    distance = 0;
#if ENABLE_DATA_LOG
    logger.openFile();
#endif
    initScreen();
    // fade in backlight
    for (int n = 1; n <= 255; n++) {
        lcd.setBackLight(n);
        delay(10);
    }
}

// screen layout related stuff
void showStates()
{
    lcd.setFontSize(FONT_SIZE_MEDIUM);

    lcd.setCursor(0, 8);
    lcd.print("OBD ");
    lcd.setColor((state & STATE_OBD_READY) ? RGB16_GREEN : RGB16_RED);
    lcd.draw((state & STATE_OBD_READY) ? tick : cross, 16, 16);
    lcd.setColor(RGB16_WHITE);

    lcd.setCursor(0, 10);
    lcd.print("ACC ");
    lcd.setColor((state & STATE_ACC_READY) ? RGB16_GREEN : RGB16_RED);
    lcd.draw((state & STATE_ACC_READY) ? tick : cross, 16, 16);
    lcd.setColor(RGB16_WHITE);

    lcd.setCursor(0, 12);
    lcd.print("GPS ");
    if (state & STATE_GPS_CONNECTED) {
        lcd.setColor(RGB16_GREEN);
        lcd.draw(tick, 16, 16);
    } else {
        lcd.setColor(RGB16_RED);
        lcd.draw(cross, 16, 16);
    }
    lcd.setColor(RGB16_WHITE);
}

void doIdleTasks()
{
    if (!(state & STATE_GUI_ON))
        return;

    if (state & STATE_ACC_READY) {
        processAccelerometer();
    }
#if USE_GPS
    uint32_t t = millis();
    while (GPSUART.available() && millis() - t < MAX_GPS_PROCESS_TIME) {
        processGPS();
    }
#endif
}

void setup()
{
    lcd.begin();
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setColor(0xFFE0);
    lcd.print("MEGA LOGGER - OBD-II/GPS/MEMS");
    lcd.setColor(RGB16_WHITE);

    Wire.begin();

#if USE_GPS
#ifdef GPS_OPEN_BAUDRATE
    GPSUART.begin(GPS_OPEN_BAUDRATE);
    delay(10);
    GPSUART.println(PMTK_SET_BAUDRATE);
    GPSUART.end();
#endif
    GPSUART.begin(GPS_BAUDRATE);
    // switching to 10Hz mode, effective only for MTK3329
    //GPSUART.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    //GPSUART.println(PMTK_SET_NMEA_UPDATE_10HZ);
    lastGPSDataTime = 0;
#endif

    logger.initSender();

    checkSD();

#if USE_MPU6050
    if (MPU6050_init() == 0) state |= STATE_ACC_READY;
#endif
    showStates();

#if USE_GPS
    unsigned long t = millis();
    do {
        if (GPSUART.available() && GPSUART.read() == '\r') {
            state |= STATE_GPS_CONNECTED;
            break;
        }
    } while (millis() - t <= 2000);
    showStates();
#endif

    delay(1000);

    while (!connectOBD());
    state |= STATE_OBD_READY;

    showStates();

    //lcd.setFont(FONT_SIZE_MEDIUM);
    //lcd.setCursor(0, 14);
    //lcd.print("VIN: XXXXXXXX");

    // open file for logging
    if (!(state & STATE_SD_READY)) {
        if (checkSD()) {
            state |= STATE_SD_READY;
            showStates();
        }
    }

#if ENABLE_DATA_LOG
    uint16_t index = logger.openFile();
    lcd.setCursor(0, 16);
    lcd.print("File ID:");
    lcd.println(index);
#endif

    showECUCap();
    delay(2000);

    initScreen();

    startTime = millis();
    lastSpeedTime = startTime;
    lastRefreshTime = millis();
}

void loop()
{
    static byte index = 0;
    static byte index2 = 0;
    static byte index3 = 0;
    uint32_t t = millis();

    logOBDData(pidTier1[index++]);
    t = millis() - t;

    if (index == TIER_NUM1) {
        index = 0;
        if (index2 == TIER_NUM2) {
            index2 = 0;
            if (obd.isValidPID(pidTier3[index3])) {
                logOBDData(pidTier3[index3]);
            }
            index3 = (index3 + 1) % TIER_NUM3;
            if (index3 == 0) {
                int v = obd.getVoltage();
                showPIDData(PID_VOLTAGE, v);
                logger.logData(PID_VOLTAGE, v);
            }
        } else {
            if (obd.isValidPID(pidTier2[index2])) {
                logOBDData(pidTier2[index2]);
            }
            index2++;
        }
    }

    if (logger.dataTime -  lastRefreshTime >= 1000) {
        char buf[12];
        // display elapsed time
        unsigned int sec = (logger.dataTime - startTime) / 1000;
        sprintf(buf, "%02u:%02u", sec / 60, sec % 60);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(250, 2);
        lcd.print(buf);
        // display OBD time
        if (t < 10000) {
            lcd.setFontSize(FONT_SIZE_SMALL);
            lcd.setCursor(242, 25);
            lcd.printInt((uint16_t)t);
            lcd.print("ms");
            lcd.printSpace(2);
        }
        lastRefreshTime = logger.dataTime;
    }

    if (obd.errors >= 3) {
        reconnect();
    }

#if USE_GPS
    if (millis() - lastGPSDataTime > GPS_DATA_TIMEOUT || gps.satellites() < 3) {
        // GPS not ready
        state &= ~STATE_GPS_READY;
    } else {
        // GPS ready
        state |= STATE_GPS_READY;
    }
#endif
}
