/*************************************************************************
* Testing sketch for Freematics OBD-II UART Adapter
* Reads and prints several OBD-II PIDs value
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* Written by Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <OBD.h>

// On Arduino Leonardo, Micro, MEGA or DUE, hardware serial can be used for output
// as OBD-II UART adapter uses to Serial1
// On Arduino UNO and those have no Serial1, we use software serial for output
// as OBD-II UART adapter uses to Serial
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
    if (obd.readPID(PID_RPM, value)) {
      mySerial.print(value);
    }
    mySerial.println();
}

void readPIDMultiple()
{
    static const byte pids[] = {PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE, PID_COOLANT_TEMP, PID_INTAKE_TEMP};
    int values[sizeof(pids)];
    if (obd.readPID(pids, sizeof(pids), values) == sizeof(pids)) {
      mySerial.print('[');
      mySerial.print(millis());
      mySerial.print(']');
      for (byte i = 0; i < sizeof(pids) ; i++) {
        mySerial.print((int)pids[i] | 0x100, HEX);
        mySerial.print('=');
        mySerial.print(values[i]);
        mySerial.print(' ');
       }
       mySerial.println();
    }
}

void readBatteryVoltage()
{
  mySerial.print('[');
  mySerial.print(millis());
  mySerial.print(']');
  mySerial.print("Battery:");
  mySerial.print(obd.getVoltage(), 1);
  mySerial.println('V');
}

void readMEMS()
{
  int x, y, z;
  mySerial.print('[');
  mySerial.print(millis());
  mySerial.print(']');
  if (obd.readAccel(x, y, z)) {
    mySerial.print("ACC:");
    mySerial.print(x);
    mySerial.print('/');
    mySerial.print(y);
    mySerial.print('/');
    mySerial.print(z);
    mySerial.print(' ');
  }
  if (obd.readGyro(x, y, z)) {
    mySerial.print("GYRO:");
    mySerial.print(x);
    mySerial.print('/');
    mySerial.print(y);
    mySerial.print('/');
    mySerial.print(z);
    mySerial.print(' ');
  }
  mySerial.println();
}

void setup()
{
  mySerial.begin(115200);
  while (!mySerial);
  
  // this will begin serial
  obd.begin();

  mySerial.print("Adapter version: ");
  mySerial.println(obd.version);
  delay(1000);

  // send some commands for testing and show response for debugging purpose
  //testOut();
 
  // initialize OBD-II adapter
  do {
    mySerial.println("Init...");
  } while (!obd.init());

  char buf[64];
  if (obd.getVIN(buf, sizeof(buf))) {
      mySerial.print("VIN:");
      mySerial.println(buf);
  }
  
  unsigned int codes[6];
  byte dtcCount = obd.readDTC(codes, 6);
  if (dtcCount == 0) {
    mySerial.println("No DTC"); 
  } else {
    mySerial.print(dtcCount); 
    mySerial.print(" DTC:");
    for (byte n = 0; n < dtcCount; n++) {
      mySerial.print(' ');
      mySerial.print(codes[n], HEX);
    }
    mySerial.println();
  }
  delay(3000);
}


void loop()
{
  readPIDSingle();
  readPIDMultiple();
  readBatteryVoltage();
  if (obd.version > 10) {
    readMEMS();
  }
}
