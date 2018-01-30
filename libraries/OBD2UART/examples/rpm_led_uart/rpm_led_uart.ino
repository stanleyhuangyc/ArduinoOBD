/*************************************************************************
* Testing sketch for Freematics OBD-II UART Adapter
* Reads engine RPM data from OBD and triggers Arduino onboard LED
* Distributed under BSD
* Visit https://freematics.com/products for more product information
* Written by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <OBD2UART.h>

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
  if (obd.readPID(PID_RPM, value)) {
    // RPM is successfully read and its value stored in variable 'value'
    // light on LED when RPM exceeds 3000
    digitalWrite(13, value > 3000 ? HIGH : LOW);
  }
}
