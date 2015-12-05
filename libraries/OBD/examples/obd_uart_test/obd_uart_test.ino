/*************************************************************************
* Testing sketch for Freematics OBD-II UART Adapter
* Reads and prints several OBD-II PIDs value
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* Written by Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <OBD.h>

// On Arduino Leonardo, Micro, MEGA or DUE, hardware serial can be used for output
// as OBD-II adapter should connect to Serial1, otherwise we use software serial
//SoftwareSerial mySerial(A2, A3);
#define mySerial Serial

COBD obd;

void testOut()
{
    static const char cmds[][6] = {"ATZ\r", "ATL1\r", "ATH0\r", "ATRV\r", "0100\r", "010C\r", "0902\r"};
    char buf[128];

    for (byte i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        const char *cmd = cmds[i];
        mySerial.print("Sending ");
        mySerial.println(cmd);
        if (obd.sendCommand(cmd, buf, sizeof(buf))) {
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
    static const byte pidlist[] = {PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP, PID_THROTTLE, PID_FUEL_LEVEL};
    mySerial.print('[');
    mySerial.print(millis());
    mySerial.print(']');
    for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++) {
        byte pid = pidlist[i];
        bool valid = obd.isValidPID(pid);
        mySerial.print((int)pid | 0x100, HEX);
        mySerial.print('=');
        if (valid) {
            int value;
            if (obd.read(pid, value)) {
              mySerial.print(value);
            }
        }
        mySerial.print(' ');
     }
     mySerial.println();
}

void setup()
{
  delay(500);
  mySerial.begin(115200);
  // this will begin serial
  obd.begin();

  // send some commands for testing and show response
  testOut();
  
  // initialize OBD-II adapter
  do {
    mySerial.println("Init...");
  } while (!obd.init());  

  char buf[64];
  if (obd.getVIN(buf, sizeof(buf))) {
      mySerial.print("VIN:");
      mySerial.println(buf);
  }
  delay(1000);
}

void loop()
{
  readPID();
}
