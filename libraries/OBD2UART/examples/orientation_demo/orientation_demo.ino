/*************************************************************************
* Testing sketch for Freematics OBD-II UART Adapter V2.1
* Reads and prints motion sensor data and computed orientation data
* Distributed under BSD
* Visit https://freematics.com/products for more product information
* Written by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <OBD2UART.h>

// On Arduino Leonardo, Micro, MEGA or DUE, hardware serial can be used for output as the adapter occupies Serial1
// On Arduino UNO and those have no Serial1, we use software serial for output as the adapter uses Serial
#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
SoftwareSerial mySerial(A2, A3);
#else
#define mySerial Serial
#endif

#if defined(ESP32) && !defined(Serial1)
HardwareSerial Serial1(1);
#endif

COBD obd;

void setup()
{
  mySerial.begin(115200);
  while (!mySerial);
  
  // this will begin serial

  for (;;) {
    delay(1000);
    byte version = obd.begin();
    mySerial.print("Freematics OBD-II Adapter ");
    if (version > 0) {
      mySerial.println("detected");
      break;
    } else {
      mySerial.println("not detected");
    }
  }
  
  // initialize MEMS with sensor fusion enabled
  bool hasMEMS = obd.memsInit(true);
  mySerial.print("Motion sensor is ");
  mySerial.println(hasMEMS ? "present" : "not present");
  if (!hasMEMS) {
    for (;;) delay(1000);
  }
}


void loop()
{
  int16_t acc[3] = {0};
  int16_t gyro[3] = {0};
  int16_t mag[3] = {0};

  if (!obd.memsRead(acc, gyro, mag)) return;
  
  mySerial.print("ACC:");
  mySerial.print(acc[0]);
  mySerial.print('/');
  mySerial.print(acc[1]);
  mySerial.print('/');
  mySerial.print(acc[2]);

  mySerial.print(" GYRO:");
  mySerial.print(gyro[0]);
  mySerial.print('/');
  mySerial.print(gyro[1]);
  mySerial.print('/');
  mySerial.print(gyro[2]);

  mySerial.print(" MAG:");
  mySerial.print(mag[0]);
  mySerial.print('/');
  mySerial.print(mag[1]);
  mySerial.print('/');
  mySerial.print(mag[2]);

  mySerial.println();

  float yaw, pitch, roll;
  if (obd.memsOrientation(yaw, pitch, roll)) {
    mySerial.print("Orientation: ");
    mySerial.print(yaw, 2);
    mySerial.print(' ');
    mySerial.print(pitch, 2);
    mySerial.print(' ');
    mySerial.println(roll, 2);
  }

  delay(100);
}
