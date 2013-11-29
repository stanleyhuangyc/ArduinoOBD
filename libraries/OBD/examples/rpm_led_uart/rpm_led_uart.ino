/*************************************************************************
* Sample sketch based on OBD-II library for Arduino
* Distributed under GPL v2.0
* Copyright (c) 2012-2013 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <OBD.h>

COBD obd;

void setup()
{
  // we'll use the debug LED as output
  pinMode(13, OUTPUT);  
  // start communication with OBD-II UART adapter
  obd.begin();
  // initiate OBD-II connection until success
  while (!obd.init());  
}

void loop()
{
  int value;
  if (obd.read(PID_RPM, value)) {
    // RPM is successfully read and its value stored in variable 'value'
    // light on LED when RPM exceeds 3000
    digitalWrite(13, value > 3000 ? HIGH : LOW);
  }
}
