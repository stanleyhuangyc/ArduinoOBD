/*************************************************************************
* Tester sketch for Freematics OBD-II Adapter for Arduino
* Distributed under GPL v2.0
* Written by Stanley Huang <stanleyhuangyc@gmail.com>
* Visit freematics.com for product information
*************************************************************************/

#include <Arduino.h>
#include <OBD.h>
#include <SPI.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU9150.h>
#include "MultiLCD.h"
#include "config.h"
#if ENABLE_DATA_LOG
#include <SD.h>
#endif
#include "datalogger.h"

#define OBD_MODEL_UART 0
#define OBD_MODEL_I2C 1

#define STATE_MEMS_READY 1
#define STATE_INIT_DONE 2

typedef struct {
  uint16_t left;
  uint16_t right;
  uint16_t bottom;
  uint16_t height;
  uint16_t pos;
} CHART_DATA;

CHART_DATA chartRPM = {24, 319, 239, 100, 24};
void chartUpdate(CHART_DATA* chart, int value);

void(* resetFunc) (void) = 0; //declare reset function at address 0

static uint32_t lastFileSize = 0;
static int speed = 0;
static uint32_t distance = 0;
static uint16_t fileIndex = 0;
static uint32_t startTime = 0;
static uint16_t elapsed = 0;
static uint8_t lastPid = 0;
static int lastValue = 0;

#if USE_MPU6050 || USE_MPU9150
MPU6050 accelgyro;
#endif

void chartUpdate(CHART_DATA* chart, int value)
{
  if (value > chart->height) value = chart->height;
  for (uint16_t n = 0; n < value; n++) {
    byte b = n * 255 / chart->height;
    lcd.setPixel(chart->pos, chart->bottom - n, RGB16(0, 0, b));
  }
  if (chart->pos++ == chart->right) {
    chart->pos = chart->left;
  }
  lcd.fill(chart->pos, chart->pos, 239 - chart->height, chart->bottom);
}

#if OBD_MODEL == OBD_MODEL_UART
class COBDDevice : public COBD, public CDataLogger
#else
class COBDDevice : public COBDI2C, public CDataLogger
#endif
{
public:
    COBDDevice():state(0) {}
    void setup()
    {
#if ENABLE_DATA_LOG
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setColor(RGB16_WHITE);
        lcd.setCursor(0, 3);
        checkSD();
#endif

#if USE_MPU6050 || USE_MPU9150
        Wire.begin();
        accelgyro.initialize();
        if (accelgyro.testConnection()) state |= STATE_MEMS_READY;
#endif

        testOut();

        while (!init(OBD_PROTOCOL));
        
        showVIN();
        
        //showECUCap();
        //delay(3000);

        initScreen();

        state |= STATE_INIT_DONE;
    }
#if ENABLE_DATA_LOG
bool checkSD()
{
    Sd2Card card;
    SdVolume volume;
    pinMode(SS, OUTPUT);

    if (card.init(SPI_FULL_SPEED, SD_CS_PIN)) {
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
            return false;
        }

        uint32_t volumesize = volume.blocksPerCluster();
        volumesize >>= 1; // 512 bytes per block
        volumesize *= volume.clusterCount();
        volumesize >>= 10;

        lcd.print((int)volumesize);
        lcd.print("MB");
    } else {
        return false;
    }

    if (!SD.begin(SD_CS_PIN)) {
        return false;
    }

    return true;
}
#endif
    void testOut()
    {
        static const char PROGMEM cmds[][6] = {"ATZ\r", "ATL1\r", "ATRV\r", "0100\r", "010C\r", "010D\r", "0902\r"};
        char buf[128];
        
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setCursor(0, 4);
    
        // recover from possible previous incomplete communication
        recover();
        for (byte i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
            char cmd[6];
            memcpy_P(cmd, cmds[i], sizeof(cmd));
            lcd.setColor(RGB16_WHITE);
            lcd.print("Sending ");
            lcd.println(cmd);
            lcd.setColor(RGB16_CYAN);
            if (sendCommand(cmd, buf, sizeof(buf))) {
                char *p = strstr(buf, cmd);
                if (p)
                    p += strlen(cmd);
                else
                    p = buf;
                while (*p == '\r') p++;
                while (*p) {
                    lcd.write(*p);
                    if (*p == '\r' && *(p + 1) != '\r')
                        lcd.write('\n');
                    p++;
                }
            } else {
                lcd.println("Timeout");
            }
            delay(1000);
        }
    }
    void showVIN()
    {
      char buf[255];
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      if (getVIN(buf, sizeof(buf))) {
          lcd.setColor(RGB16_WHITE);
          lcd.print("\nVIN:");
          lcd.setColor(RGB16_YELLOW);
          lcd.println(buf);
      }
    }
    void loop()
    {
        static byte index2 = 0;
        const byte pids[]= {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD};
        int values[sizeof(pids)];
        // read multiple OBD-II PIDs
        if (read(pids, sizeof(pids), values) == sizeof(pids)) {
          dataTime = millis();
          for (byte n = 0; n < sizeof(pids); n++) {
            logData((uint16_t)pids[n] | 0x100, values[n]);
            showData(pids[n], values[n]);
          }
        }
        static byte lastSec = 0;
        const byte pids2[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_DISTANCE};
        byte sec = (uint8_t)(millis() >> 10);
        if (sec != lastSec) {
          // goes in every other second
          int value;
          byte pid = pids2[index2 = (index2 + 1) % (sizeof(pids2))];
          // read single OBD-II PID
          if (isValidPID(pid) && read(pid, value)) {
            dataTime = millis();
            logData((uint16_t)pid | 0x100, value);
            lastSec = sec;
          }
        }
        if (errors >= 5) {
            reconnect();
        }
    }
#if USE_MPU6050 || USE_MPU9150
    void logMEMSData()
    {
      int16_t ax, ay, az;
      int16_t gx, gy, gz;
#if USE_MPU9150
      int16_t mx, my, mz;
      accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
#else
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
#endif
      dataTime = millis();
  
      lcd.setFontSize(FONT_SIZE_SMALL);
      lcd.setCursor(24, 14);
      lcd.print(ax >> 4);
      lcd.print('/');
      lcd.print(ay >> 4);
      lcd.print('/');
      lcd.print(az >> 4);
      lcd.print(' ');
  
      lcd.setCursor(152, 14);
      lcd.print(gx >> 4);
      lcd.print('/');
      lcd.print(gy >> 4);
      lcd.print('/');
      lcd.print(gz >> 4);
      lcd.print(' ');
  
#if USE_MPU9150
      lcd.setCursor(252, 14);
      lcd.print(mx >> 4);
      lcd.print('/');
      lcd.print(my >> 4);
      lcd.print('/');
      lcd.print(mz >> 4);
      lcd.print(' ');
#endif
      lcd.setFontSize(FONT_SIZE_MEDIUM);

      // log x/y/z of accelerometer
      logData(PID_ACC, ax, ay, az);
      // log x/y/z of gyro meter
      logData(PID_GYRO, gx, gy, gz);
#if USE_MPU9150
      // log x/y/z of compass
      logData(PID_COMPASS, mx, my, mz);
#endif
    }
#endif
    void reconnect()
    {
        lcd.clear();
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.print("Reconnecting");
        startTime = millis();
        //digitalWrite(SD_CS_PIN, LOW);
        for (uint16_t i = 0; ; i++) {
            if (i == 5) {
                lcd.setBackLight(0);
                lcd.clear();
            }
            if (init()) {
              lcd.setBackLight(255);
              lcd.clear();
              lcd.print("Reseting...");
              // reset Arduino
              resetFunc();        
            }
        }
    }
    // screen layout related stuff
    void showData(byte pid, int value)
    {
        switch (pid) {
        case PID_RPM:
            lcd.setCursor(0, 2);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt((unsigned int)value % 10000, 4);
            showChart(value);
            break;
        case PID_SPEED:
            lcd.setCursor(90, 2);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt((unsigned int)value % 1000, 3);
            break;
        case PID_ENGINE_LOAD:
            lcd.setCursor(164, 2);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt(value % 100, 3);
            break;
        case PID_INTAKE_TEMP:
            if (value < 0) value = 0;
            lcd.setCursor(248, 2);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt(value, 3);
            break;
        case PID_INTAKE_MAP:
            lcd.setCursor(164, 9);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt((uint16_t)value % 1000, 3);
            break;
        case PID_COOLANT_TEMP:
            lcd.setCursor(8, 9);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt((uint16_t)value % 1000, 3);
            break;
        case PID_DISTANCE:
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.setCursor(90, 9);
            lcd.printInt((uint16_t)value % 1000, 3);
            break;
        }
    }
    void ShowVoltage(float v)
    {
        lcd.setFontSize(FONT_SIZE_LARGE);
        lcd.setCursor(260, 10);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.print(v);
    }
    void showChart(int value)
    {
        uint16_t height;
        if (value >= 560) {
          height = (value - 500) / 60;
        } else {
          height = 1;
        }
        chartUpdate(&chartRPM, height);
    }
    void initScreen()
    {
        lcd.clear();
        lcd.setBackLight(255);
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setColor(RGB16_CYAN);
        lcd.setCursor(4, 0);
        lcd.print("ENGINE RPM");
        lcd.setCursor(104, 0);
        lcd.print("SPEED");
        lcd.setCursor(164, 0);
        lcd.print("ENGINE LOAD");
        lcd.setCursor(248, 0);
        lcd.print("INTAKE TEMP");

        lcd.setCursor(4, 7);
        lcd.print("COOLANT TEMP");
        lcd.setCursor(104, 7);
        lcd.print("DISTANCE");
        lcd.setCursor(164, 7);
        lcd.print("INTAKE MAP");

        lcd.setCursor(260, 9);
        lcd.print("BATTERY");

        lcd.setCursor(0, 14);
        lcd.print("ACC");
        lcd.setCursor(122, 14);
        lcd.print("GYRO");
        lcd.setCursor(230, 14);
        lcd.print("MAG");

        lcd.setColor(RGB16_YELLOW);
        lcd.setCursor(24, 5);
        lcd.print("rpm");
        lcd.setCursor(110, 5);
        lcd.print("km/h");
        lcd.setCursor(216, 4);
        lcd.print("%");
        lcd.setCursor(304, 4);
        lcd.print("C");
        lcd.setCursor(64, 11);
        lcd.print("C");
        lcd.setCursor(110, 12);
        lcd.print("km");
        lcd.setCursor(200, 12);
        lcd.print("kpa");
        lcd.setCursor(296, 12);
        lcd.print("V");
        
        lcd.setColor(RGB16_CYAN);
        lcd.setXY(0, 140);
        lcd.print("6500");
        lcd.setXY(0, 186);
        lcd.print("3500");
        lcd.setXY(0, 232);
        lcd.print("500");

        lcd.setColor(RGB16_WHITE);
    }
#if USE_MPU6050 || USE_MPU9150
    void dataIdleLoop()
    {
      // while waiting for response from OBD-II adapter, let's do something here
      if (state == (STATE_MEMS_READY | STATE_INIT_DONE)) {
        logMEMSData();
      }
    }
#endif
    byte state;
};

COBDDevice myOBD;

void setup()
{
    lcd.begin();
    lcd.clear();
    lcd.setColor(RGB16_YELLOW);
    lcd.println("Freematics OBD-II Adapter Tester");

    myOBD.begin();
    myOBD.initSender();
    myOBD.setup();
}

void loop()
{
    myOBD.loop();
}
