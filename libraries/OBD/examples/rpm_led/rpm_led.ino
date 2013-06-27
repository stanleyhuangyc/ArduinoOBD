/*************************************************************************
* Sample sketch based on OBD-II library for Arduino
* Distributed under GPL v2.0
* Copyright (c) 2012-2013 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include "OBD.h"

COBD obd;

void setup()
{
  // we'll use the debug LED as output
  pinMode(13, OUTPUT);  
  // start serial communication at the adapter defined baudrate
  obd.begin();
  // initiate OBD-II connection until success
  while (!obd.init());  
}

void loop()
{
  int value;
  if (obd.readSensor(PID_RPM, value)) {
    // RPM is read and stored in 'value'
    // light on LED when RPM exceeds 5000
    digitalWrite(13, value > 5000 ? HIGH : LOW);
  }
}
