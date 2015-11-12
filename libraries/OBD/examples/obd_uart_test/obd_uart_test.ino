/*************************************************************************
* Sample sketch for Freematics OBD-II UART Adapter
* Reads and prints several OBD-II PIDs value and MEMS sensor data
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2012-2015 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <OBD.h>

SoftwareSerial mySerial(A2, A3);
COBD obd;

void testOut()
{
    static const char PROGMEM cmds[][6] = {"ATZ\r", "ATL1\r", "ATH0\r", "ATRV\r", "0100\r", "010C\r", "0902\r"};
    char buf[OBD_RECV_BUF_SIZE];

    for (byte i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        char cmd[6];
        memcpy_P(cmd, cmds[i], sizeof(cmd));
        mySerial.print("Sending ");
        mySerial.println(cmd);
        if (obd.sendCommand(cmd, buf)) {
            char *p = strstr(buf, cmd);
            if (p)
                p += strlen(cmd);
            else
                p = buf;
            while (*p == '\r') p++;
            while (*p) {
                mySerial.write(*p);
                if (*p == '\r' && *(p + 1) != '\r')
                    mySerial.write('\n');
                p++;
            }
        } else {
            mySerial.println("Timeout");
        }
        delay(1000);
    }
    mySerial.println();
}

void readPID()
{
    static const byte PROGMEM pidlist[] = {PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP, PID_THROTTLE, PID_FUEL_LEVEL};
    for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++) {
        byte pid = pgm_read_byte(pidlist + i);
        bool valid = obd.isValidPID(pid);
        mySerial.print('0');
        mySerial.print((int)pid | 0x100, HEX);
        mySerial.print('=');
        if (valid) {
            int value;
            if (obd.read(pid, value)) {
              byte n = mySerial.println(value);
            }
        } else {
          mySerial.println('X'); 
        }
     }
}

void setup() {
  delay(500);
  mySerial.begin(9600);
  obd.begin();

  do {
    testOut();
    mySerial.println("Init...");
  } while (!obd.init());  

  char buf[OBD_RECV_BUF_SIZE];
  if (obd.getVIN(buf)) {
      mySerial.print("VIN:");
      mySerial.println(buf);
  }
  delay(1000);
}

void loop() {
  readPID();
  delay(500);
}
