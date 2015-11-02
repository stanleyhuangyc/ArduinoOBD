/*************************************************************************
* Sample sketch for Freematics OBD-II I2C Adapter
* Reads and prints several OBD-II PIDs value and MEMS sensor data
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2012-2015 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <OBD.h>
#include <I2Cdev.h>
#include <MPU9150.h>

COBDI2C obd;
MPU6050 accelgyro;

void testOut()
{
    static const char PROGMEM cmds[][6] = {"ATZ\r", "ATL1\r", "ATRV\r", "0100\r", "010C\r", "0902\r"};
    char buf[OBD_RECV_BUF_SIZE];

    for (byte i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        char cmd[6];
        memcpy_P(cmd, cmds[i], sizeof(cmd));
        Serial.print("Sending ");
        Serial.println(cmd);
        if (obd.sendCommand(cmd, buf)) {
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
        delay(500);
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

    // display MEMS data
    Serial.print("ACC=");
    Serial.print(ax);
    Serial.write('/');
    Serial.print(ay);
    Serial.write('/');
    Serial.println(az);

    Serial.print("GYRO=");
    Serial.print(gx);
    Serial.write('/');
    Serial.print(gy);
    Serial.write('/');
    Serial.println(gz);
}

void showECUCap()
{
    static const byte PROGMEM pidlist[] = {PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP, PID_THROTTLE, PID_FUEL_LEVEL};
    for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++) {
        byte pid = pgm_read_byte(pidlist + i);
        bool valid = obd.isValidPID(pid);
        Serial.print('0');
        Serial.print((int)pid | 0x100, HEX);
        Serial.print('=');
        if (valid) {
            int value;
            if (obd.read(pid, value)) {
              byte n = Serial.println(value);
            }
        } else {
          Serial.println('X'); 
        }
     }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin();
  accelgyro.initialize();
  testOut();
  readMEMS();
  Serial.println("Init...");
  while (!obd.init());  
}

void loop() {
  showECUCap();
  readMEMS();
}
