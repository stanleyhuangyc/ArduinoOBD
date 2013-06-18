/*************************************************************************
* Arduino GPS/OBD-II/G-Force Data Logger
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
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
#define SD_CS_PIN 7 // microduino
//#define SD_CS_PIN 10 // SD breakout

/**************************************
* Config GPS here
**************************************/
#define USE_GPS
#define GPS_BAUDRATE 4800 /* bps */
//#define GPS_OPEN_BAUDRATE 4800 /* bps */

/**************************************
* Choose LCD model here
**************************************/
#define USE_SSD1306
//#define USE_ZTOLED
//#define USE_LCD1602
//#define USE_LCD4884

/**************************************
* Other options
**************************************/
#define USE_MPU6050
#define OBD_MIN_INTERVAL 50 /* ms */
#define GPS_DATA_TIMEOUT 2000 /* ms */
#define ENABLE_DATA_OUT

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
#define GPSUART Serial3
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

#ifdef ENABLE_DATA_OUT
///////////////////////////////////////////////////////////////////////////
//LOG_DATA data type
typedef struct {
    uint32_t /*DWORD*/ time;
    uint16_t /*WORD*/ pid;
    uint8_t /*BYTE*/ flags;
    uint8_t /*BYTE*/ checksum;
    union {
        int16_t i16[2];
        int32_t i32;
        float f;
    } data;
    /*
    union {
        int16_t i16[6];
        int32_t i32[3];
        float f[3];
    } data;*/
} LOG_DATA;
///////////////////////////////////////////////////////////////////////////
SoftwareSerial softSerial(9, 10);
#endif

// SD card
Sd2Card card;
SdVolume volume;
File sdfile;

// enable one LCD
#if defined(USE_ZTOLED)
LCD_ZTOLED lcd; /* for ZT OLED module */
#define LCD_LINES 4
#define CHAR_WIDTH 6
#elif defined(USE_SSD1306)
LCD_SSD1306 lcd; /* for SSD1306 OLED module */
#define LCD_LINES 8
#define CHAR_WIDTH 6
#elif defined(USE_LCD1602)
LCD_1602 lcd; /* for LCD1602 shield */
#define LCD_LINES 2
#define CHAR_WIDTH 1
#elif defined(USE_LCD4884)
LCD_PCD8544 lcd; /* for LCD4884 shield or Nokia 5100 screen module */
#define LCD_LINES 5
#define CHAR_WIDTH 6
#endif

static uint32_t fileSize = 0;
static uint32_t lastFileSize = 0;
static uint32_t lastDataTime;
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
        lastGPSDataTime = 0;

        ShowStates();

#ifdef USE_MPU6050
        if (MPU6050_init() == 0) state |= STATE_ACC_READY;
        ShowStates();
#endif

#ifdef GPSUART
        unsigned long t = millis();
        do {
            if (GPSUART.available()) {
                state |= STATE_GPS_CONNECTED;
                break;
            }
        } while (millis() - t <= 2000);
#endif

        do {
            ShowStates();
        } while (!Init());

        state |= STATE_OBD_READY;

        ShowStates();

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
        static byte count = 0;

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
            lcd.setCursor(8 * CHAR_WIDTH, 0);
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
            lcd.setCursor(8 * CHAR_WIDTH, 8);
            lcd.print("Bad File");
            return false;
        }

        filename[2] = '[';
        filename[8] = ']';
        filename[9] = 0;
        lcd.setCursor(127 - 7 * CHAR_WIDTH, 0);
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.print(filename + 2);
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
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(97, 7);
        lcd.print(buf);
#ifdef GPSUART
        // detect GPS signal
        ProcessGPS();
        if (lastGPSDataTime) {
            state |= STATE_GPS_READY;
        }
#endif
#ifdef USE_MPU6050
        if (state & STATE_ACC_READY) {
            accel_t_gyro_union data;
            MPU6050_readout(&data);
            char buf[8];
            lcd.setFont(FONT_SIZE_SMALL);
#if LCD_LINES > 2
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

#else
            sprintf(buf, "X:%3d", data.value.x_accel / 160);
            lcd.setFont(FONT_SIZE_SMALL);
            lcd.setCursor(10 * CHAR_WIDTH, 1);
            lcd.print(buf);
#endif
            delay(50);
        }
#endif
    }
#ifdef GPSUART
    void DataIdleLoop()
    {
        if (GPSUART.available())
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
        len = sprintf(buf, "%u,F0,%06ld %08ld\n", elapsed, date, time);
        sdfile.write((uint8_t*)buf, len);

        unsigned int speed = (unsigned int)(gps.speed() * 1852 / 100 / 1000);
#ifdef ENABLE_DATA_OUT
        SendData(dataTime, 0xF00D, (float)speed);
#endif

        // no need to log GPS data when vehicle has not been moving
        // that's when previous speed is zero and current speed is also zero
        if (!(speed == 0 && lastSpeed == 0) && gps.satellites() >= 3) {
            // lastSpeed will be updated
            //ShowSensorData(PID_SPEED, speed);
            len = sprintf(buf, "%u,F1,%u\n", elapsed, speed);
            sdfile.write((uint8_t*)buf, len);

            long lat, lon;
            gps.get_position(&lat, &lon, 0);

            len = sprintf(buf, "%u,F2,%ld %ld\n", elapsed, lat, lon);
            sdfile.write((uint8_t*)buf, len);

            len = sprintf(buf, "%u,F3,%ld\n", elapsed, gps.altitude());
            sdfile.write((uint8_t*)buf, len);

#ifdef ENABLE_DATA_OUT
            delay(10);
            SendData(dataTime, 0xF00C, (float)gps.altitude());
#endif

            lcd.setFont(FONT_SIZE_SMALL);
#if LCD_LINES > 2
            if (((unsigned int)dataTime / 3000) & 1) {
                sprintf(buf, "LAT:%d.%05ld  ", (int)(lat / 100000), lat % 100000);
                lcd.setCursor(0, 6);
                lcd.print(buf);
                sprintf(buf, "LON:%d.%05ld  ", (int)(lon / 100000), lon % 100000);
                lcd.setCursor(0, 7);
                lcd.print(buf);
            } else {
                lcd.setCursor(0, 6);
                sprintf(buf, "ALT:%um SAT:%u", (unsigned int)(gps.altitude() / 100), (unsigned int)gps.satellites());
                lcd.print(buf);
                lcd.setCursor(0, 7);
                sprintf(buf, "TIME:%08ld ", time);
                lcd.print(buf);
            }
#else
            if (((unsigned int)dataTime / 1000) & 1)
                sprintf(buf, "%d.%ld  ", (int)(lat / 100000), lat % 100000);
            else
                sprintf(buf, "%d.%ld  ", (int)(lon / 100000), lon % 100000);
            lcd.setCursor(0, 1);
            lcd.print(buf);
#endif

#ifdef ENABLE_DATA_OUT
            SendData(dataTime, 0xF00A, (float)lat / 100000);
            delay(20);
            SendData(dataTime, 0xF00B, (float)lon / 100000);
#endif
        }
        lastDataTime = dataTime;
        lastGPSDataTime = dataTime;
    }
#endif
    void ProcessAccelerometer()
    {
#ifdef USE_MPU6050
        accel_t_gyro_union data;
        MPU6050_readout(&data);
        uint32_t dataTime = millis();

#if LCD_LINES > 2
        ShowGForce(data.value.y_accel);
#endif

        int len;
        uint16_t elapsed = (uint16_t)(dataTime - lastDataTime);
        char buf[20];
        // log x/y/z of accelerometer
        len = sprintf(buf, "%u,F10,%d %d %d\n", data.value.x_accel, data.value.y_accel, data.value.z_accel);
        sdfile.write((uint8_t*)buf, len);
        // log x/y/z of gyro meter
        len = sprintf(buf, "%u,F11,%d %d %d\n", data.value.x_gyro, data.value.y_gyro, data.value.z_gyro);
        sdfile.write((uint8_t*)buf, len);
        lastDataTime = dataTime;
#endif
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

#ifdef ENABLE_DATA_OUT
        SendData(dataTime, 0x0100 | pid, (float)value);
#endif // ENABLE_DATA_OUT

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
            sprintf(buf, "%4uKB", (int)(fileSize >> 10));
            lcd.setFont(FONT_SIZE_SMALL);
            lcd.setCursor(92, 7);
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
    void SendData(uint32_t time, uint16_t pid, float value)
    {
        LOG_DATA ld = {time, pid, 0, 1};
        ld.data.f = value;
        uint8_t checksum = 0;
        for (int i = 0; i < sizeof(ld); i++) {
            checksum ^= *((char*)&ld + i);
        }
        ld.checksum = checksum;
        softSerial.write((uint8_t*)&ld, sizeof(LOG_DATA));
    }
    void ShowECUCap()
    {
        char buffer[24];
        byte pidlist[] = {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD, PID_ABS_ENGINE_LOAD, PID_MAF_FLOW, PID_INTAKE_MAP, PID_FUEL_LEVEL, PID_FUEL_PRESSURE, PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_TIMING_ADVANCE, PID_BAROMETRIC};
        const char* namelist[] = {"RPM", "SPEED", "THROTTLE", "ENG.LOAD1", "ENG.LOAD2", "MAF", "MAP", "FUEL LV.", "FUEL PRE.", "COOLANT", "INTAKE","AMBIENT", "IGNITION", "BARO"};
        byte i = 0;
        lcd.clear();
        lcd.setFont(FONT_SIZE_SMALL);
        for (; i < sizeof(pidlist) / sizeof(pidlist[0]) / 2; i++) {
            lcd.setCursor(0, i);
            sprintf(buffer, "%s:%c", namelist[i], IsValidPID(pidlist[i]) ? 'Y' : 'N');
            lcd.print(buffer);
        }
        for (byte row = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++, row++) {
            lcd.setCursor(64, row);
            sprintf(buffer, "%s:%c", namelist[i], IsValidPID(pidlist[i]) ? 'Y' : 'N');
            lcd.print(buffer);
        }
    }
    void Reconnect()
    {
        sdfile.close();
        lcd.clear();
        lcd.setFont(FONT_SIZE_MEDIUM);
        lcd.print("Reconnecting");
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
        lcd.setCursor(0, 2);
        lcd.print("OBD");
        lcd.draw((state & STATE_OBD_READY) ? tick : cross, 32, 16, 16, 16);
#if LCD_LINES > 2
        lcd.setCursor(0, 4);
        lcd.print("ACC");
        lcd.draw((state & STATE_ACC_READY) ? tick : cross, 32, 32, 16, 16);
        lcd.setCursor(0, 6);
        if (!(state & STATE_GPS_READY)) {
            lcd.print("GPS");
            lcd.draw((state & STATE_GPS_CONNECTED) ? tick : cross, 32, 48, 16, 16);
        }
#endif
    }
    virtual void ShowSensorData(byte pid, int value)
    {
        char buf[8];
        switch (pid) {
        case PID_RPM:
#if LCD_LINES <= 2
            lcd.setCursor(8, 0);
#else
            lcd.setCursor(64, 0);
#endif
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
#if LCD_LINES > 2
        case PID_THROTTLE:
            lcd.setCursor(24, 4);
            lcd.setFont(FONT_SIZE_MEDIUM);
            lcd.printInt(value % 100, 3);
            break;
        case PID_INTAKE_TEMP:
            lcd.setCursor(96, 4);
            lcd.setFont(FONT_SIZE_MEDIUM);
            lcd.printInt(value, 3);
            break;
        case PID_DISTANCE:
            if ((unsigned int)value >= startDistance) {
                sprintf(buf, "%4ukm", ((unsigned int)value - startDistance) % 1000);
                lcd.setFont(FONT_SIZE_SMALL);
                lcd.setCursor(92, 6);
                lcd.print(buf);
            }
            break;
#endif
        }
    }
    virtual void ShowGForce(int g)
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
    virtual void InitScreen()
    {
        lcd.clear();
        lcd.backlight(true);
        lcd.setFont(FONT_SIZE_SMALL);
#if LCD_LINES <= 2
        lcd.setCursor(4, 0);
        lcd.print("kph");
#else
        lcd.setCursor(24, 2);
        lcd.print("km/h");
#endif
#if LCD_LINES <= 2
        lcd.setCursor(13, 0);
        lcd.print("rpm");
#else
        lcd.setCursor(110, 2);
        lcd.print("rpm");
        lcd.setCursor(0, 5);
        lcd.setCursor(0, 4);
        lcd.print("THR:    %");
        lcd.setCursor(73, 4);
        lcd.print("AIR:    C");
#endif
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

#ifdef ENABLE_DATA_OUT
    softSerial.begin(9600);
#endif // ENABLE_DATA_OUT

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
    lcd.setCursor(0, 2);
    logger.CheckSD();
    logger.Setup();
}

void loop()
{
    logger.Loop();
}
