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
#include "datalogger.h"

#define OBD_MODEL_UART 0
#define OBD_MODEL_I2C 1

#define STATE_MEMS_READY 1
#define STATE_INIT_DONE 2

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

static const PROGMEM uint8_t tick[16 * 16 / 8] =
{0x00,0x80,0xC0,0xE0,0xC0,0x80,0x00,0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0x78,0x30,0x00,0x00,0x01,0x03,0x07,0x0F,0x1F,0x1F,0x1F,0x0F,0x07,0x03,0x01,0x00,0x00,0x00,0x00};

static const byte PROGMEM pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static const byte PROGMEM pidTier2[] = {PID_INTAKE_MAP, PID_MAF_FLOW, PID_TIMING_ADVANCE};
static const byte PROGMEM pidTier3[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_ENGINE_FUEL_RATE, PID_DISTANCE};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)
#define TIER_NUM3 sizeof(pidTier3)

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
#if USE_MPU6050 || USE_MPU9150
        Wire.begin();
        accelgyro.initialize();
        if (accelgyro.testConnection()) state |= STATE_MEMS_READY;
#endif

        testOut();

        while (!init(OBD_PROTOCOL));
        
        showVIN();
        
        showECUCap();
        delay(3000);

        benchmark();
        delay(5000);

        initScreen();

        state |= STATE_INIT_DONE;
    }
    void testOut()
    {
        static const char PROGMEM cmds[][6] = {"ATZ\r", "ATL1\r", "ATRV\r", "0100\r", "010C\r", "010D\r", "0902\r"};
        char buf[OBD_RECV_BUF_SIZE];
        
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
            if (sendCommand(cmd, buf)) {
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
      char buf[OBD_RECV_BUF_SIZE];
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      if (getVIN(buf)) {
          lcd.setColor(RGB16_WHITE);
          lcd.print("\nVIN:");
          lcd.setColor(RGB16_YELLOW);
          lcd.println(buf);
      } else {
          lcd.println("error");
      }
    }
    void showECUCap()
    {
        static const byte PROGMEM pidlist[] = {PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_FUEL_PRESSURE, PID_INTAKE_MAP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP, PID_MAF_FLOW, PID_THROTTLE, PID_AUX_INPUT,
            PID_EGR_ERROR, PID_COMMANDED_EVAPORATIVE_PURGE, PID_FUEL_LEVEL, PID_CONTROL_MODULE_VOLTAGE, PID_ABSOLUTE_ENGINE_LOAD, PID_AMBIENT_TEMP, PID_COMMANDED_THROTTLE_ACTUATOR, PID_ETHANOL_FUEL,
            PID_FUEL_RAIL_PRESSURE, PID_HYBRID_BATTERY_PERCENTAGE, PID_ENGINE_OIL_TEMP, PID_FUEL_INJECTION_TIMING, PID_ENGINE_FUEL_RATE, PID_ENGINE_TORQUE_DEMANDED, PID_ENGINE_TORQUE_PERCENTAGE};
    
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i += 2) {
            for (byte j = 0; j < 2; j++) {
                byte pid = pgm_read_byte(pidlist + i + j);
                lcd.setCursor(216 + j * 56 , i + 4);
                lcd.print((int)pid | 0x100, HEX);
                bool valid = isValidPID(pid);
                if (valid) {
                    lcd.setColor(RGB16_GREEN);
                    lcd.draw(tick, 16, 16);
                    lcd.setColor(RGB16_WHITE);
                }
            }
        }
    }
    void benchmark()
    {
        lcd.clear();
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setColor(RGB16_YELLOW);
        lcd.println("Benchmarking OBD-II...");
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_SMALL);

        uint32_t elapsed;
        char buf[OBD_RECV_BUF_SIZE];
        uint16_t count = 0;
        for (elapsed = 0; elapsed < 10000; ) {
          lcd.setCursor(0, 4);
          for (byte n = 0; n < TIER_NUM1; n++) {
              byte pid = pgm_read_byte(pidTier1 + n);
              char cmd[6];
              sprintf(cmd, "01%02X\r", pid);
              lcd.setColor(RGB16_CYAN);
              lcd.print('[');
              lcd.print(elapsed);
              lcd.print("] ");
              lcd.setColor(RGB16_WHITE);
              lcd.println(cmd);
              startTime = millis();
              if (sendCommand(cmd, buf)) {
                elapsed += (millis() - startTime);
                count++;
                lcd.setColor(RGB16_GREEN);
                lcd.println(buf);
              } else {
                lcd.setColor(RGB16_RED);
                lcd.println("Timeout!");
              }
          }
        }
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setColor(RGB16_WHITE);
        if (count) {
            lcd.print("\nOBD-II PID Access Time: ");
            lcd.print(elapsed / count);
            lcd.println("ms");
        } else {
            lcd.println("\nNo Data!");
        }

#if USE_MPU6050 || USE_MPU9150
        if (!(state & STATE_MEMS_READY)) return;
        lcd.setColor(RGB16_YELLOW);
        lcd.println("\nBenchmarking MEMS...");
        startTime = millis();
        for (count = 0, elapsed = 0; elapsed < 3000; elapsed = millis() - startTime, count++) {
          int16_t ax, ay, az;
          int16_t gx, gy, gz;
#if USE_MPU9150
          int16_t mx, my, mz;
          accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
#else
          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
#endif
        }
        lcd.setColor(RGB16_WHITE);
        lcd.println();
#if USE_MPU9150
        lcd.print('9');
#else
        lcd.print('6');
#endif
        lcd.print("-Axis Data Access Time: ");
        lcd.print(elapsed / count);
        lcd.println("ms");
#endif
    }
    void logOBDData(byte pid)
    {
        int value;
        if (read(pid, value)) {
          logData(pid, value);
          showData(pid, value);
        }
    }
    void loop()
    {
        static byte index = 0;
        static byte index2 = 0;
        static byte index3 = 0;
        byte pid = pgm_read_byte(pidTier1 + index++);
        logOBDData(pid);
        if (index == TIER_NUM1) {
            index = 0;
            if (index2 == TIER_NUM2) {
                index2 = 0;
                pid = pgm_read_byte(pidTier3 + index3);
                if (isValidPID(pid)) {
                  logOBDData(pid);
                }
                index3 = (index3 + 1) % TIER_NUM3;
                if (index3 == 0) {
                    float v = getVoltage();
                    ShowVoltage(v);
                }
            } else {
                pid = pgm_read_byte(pidTier2 + index2);
                if (isValidPID(pid)) {
                  logOBDData(pid);
                }
                index2++;
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
        #define CHART_HEIGHT 100
        static uint16_t pos = 0;
        uint16_t height;
        if (value >= 600) {
          height = (value - 600) / 64;
          if (height > CHART_HEIGHT) height = CHART_HEIGHT;
        } else {
          height = 1;
        }
        lcd.fill(pos, pos, 239 - height, 239, RGB16_CYAN);
        pos = (pos + 1) % 320;
        lcd.fill(pos, pos, 239 - CHART_HEIGHT, 239);
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

        lcd.setCursor(260, 7);
        lcd.print("ELAPSED");
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
        lcd.setColor(RGB16_WHITE);
    }
    void dataIdleLoop()
    {
      // while waiting for response from OBD-II adapter, let's do something here
      if (state == (STATE_MEMS_READY | STATE_INIT_DONE)) {
        logMEMSData();
      }
    }
    byte state;
};

static COBDDevice myOBD;

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
