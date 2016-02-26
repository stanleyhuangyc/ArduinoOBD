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
// as OBD-II UART adapter connects to Serial1, otherwise we use software serial
SoftwareSerial mySerial(A2, A3);
//#define mySerial Serial

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

void readPIDSingle()
{
    int value;
    mySerial.print('[');
    mySerial.print(millis());
    mySerial.print(']');
    mySerial.print("RPM=");
    if (obd.read(PID_RPM, value)) {
      mySerial.print(value);
    }
    mySerial.println();
}

void readPIDMultiple()
{
    static const byte pids[] = {PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE, PID_COOLANT_TEMP, PID_INTAKE_TEMP};
    int values[sizeof(pids)];
    if (obd.read(pids, sizeof(pids), values) == sizeof(pids)) {
      for (byte i = 0; i < sizeof(pids) ; i++) {
        mySerial.print('[');
        mySerial.print(millis());
        mySerial.print(']');
        mySerial.print((int)pids[i] | 0x100, HEX);
        mySerial.print('=');
        mySerial.println(values[i]);
       }
    }
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
  readPIDSingle();
  readPIDMultiple();
}
