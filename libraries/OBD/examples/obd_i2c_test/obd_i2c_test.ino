/*************************************************************************
* Sample sketch for Freematics OBD-II I2C Adapter
* Reads and prints several OBD-II PIDs value and MEMS sensor data
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2012-2015 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <OBD.h>
#include <I2Cdev.h>
#include <MPU9150.h>

COBDI2C obd;
MPU6050 accelgyro;

void testOut()
{
    static const char cmds[][6] = {"ATZ\r", "ATL1\r", "ATH0\r", "ATRV\r", "0100\r", "010C\r", "0902\r"};
    char buf[128];

    for (byte i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        const char *cmd = cmds[i];
        Serial.print("Sending ");
        Serial.println(cmd);
        if (obd.sendCommand(cmd, buf, sizeof(buf))) {
            char *p = strstr(buf, cmd);
            if (p)
                p += strlen(cmd);
            else
                p = buf;
            while (*p == '\r') p++;
            while (*p) {
                Serial.write(*p);
                if (*p == '\r' && *(p + 1) != '\r')
                    Serial.write('\n');
                p++;
            }
        } else {
            Serial.println("Timeout");
        }
        delay(1000);
    }
    Serial.println();
}

void readMEMS()
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int temp;

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    temp = accelgyro.getTemperature();

    Serial.print('[');
    Serial.print(millis());
    Serial.print(']');

    Serial.print("ACC=");
    Serial.print(ax);
    Serial.print('/');
    Serial.print(ay);
    Serial.print('/');
    Serial.print(az);

    Serial.print(" GYRO=");
    Serial.print(gx);
    Serial.print('/');
    Serial.print(gy);
    Serial.print('/');
    Serial.println(gz);
}

void readPIDs()
{
    static const byte pidlist[] = {PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP, PID_THROTTLE, PID_FUEL_LEVEL};
    Serial.print('[');
    Serial.print(millis());
    Serial.print(']');
    for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++) {
        byte pid = pidlist[i];
        bool valid = obd.isValidPID(pid);
        Serial.print((int)pid | 0x100, HEX);
        Serial.print('=');
        if (valid) {
            int value;
            if (obd.readPID(pid, value)) {
              Serial.print(value);
            }
        }
        Serial.print(' ');
     }
     Serial.println();
}

void readBatteryVoltage()
{
  Serial.print('[');
  Serial.print(millis());
  Serial.print(']');
  Serial.print("Battery:");
  Serial.print(obd.getVoltage(), 1);
  Serial.println('V');
}

void setup() {
  Serial.begin(115200);
  delay(500);
  obd.begin();
  accelgyro.initialize();

  // send some commands for testing and show response for debugging purpose
  //testOut();
 
  // initialize OBD-II adapter
  do {
    Serial.println("Init...");
  } while (!obd.init());

  char buf[64];
  if (obd.getVIN(buf, sizeof(buf))) {
      Serial.print("VIN:");
      Serial.println(buf);
  }
  
  unsigned int codes[6];
  byte dtcCount = obd.readDTC(codes, 6);
  if (dtcCount == 0) {
    Serial.println("No DTC"); 
  } else {
    Serial.print(dtcCount); 
    Serial.print(" DTC:");
    for (byte n = 0; n < dtcCount; n++) {
      Serial.print(' ');
      Serial.print(codes[n], HEX);
    }
    Serial.println();
  }
  delay(3000);
}

void loop() {
  readPIDs();
  readBatteryVoltage();
  readMEMS();
}
 
