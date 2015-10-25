/*************************************************************************
HelloOBD
A simple example for interfacing with the Freematics Arduino OBD-II 
Adapter (Model A).
Copyright (C) 2015 Raghavendra Rao <raghavendra624@gmail.com>
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <U8glib.h>
#include <OBD.h>

class CMyOBD : public COBD
{
public:
  CMyOBD() {
    u8g = U8GLIB_SH1106_128X64(U8G_I2C_OPT_NONE);
    pidRPM = 0;
    pidThrottle = 0;
    pidSpeed = 0;
    pidMAFFlow = 0;
    pidIntakeTemp = 0;
    bReady = false;
  }
  
  void loop()
  {
    // read
    read(PID_RPM, pidRPM);
    read(PID_THROTTLE, pidThrottle);
    read(PID_SPEED, pidSpeed);
    read(PID_MAF_FLOW, pidMAFFlow);
    read(PID_INTAKE_TEMP, pidIntakeTemp);
                  
    if (errors >= 2) {
      setup();
    }
  }

private:
  U8GLIB_SH1106_128X64 u8g;
  int pidRPM, pidThrottle, pidSpeed;
  int pidMAFFlow, pidIntakeTemp;
  char szStr[50];
  bool bReady;
  
  void dataIdleLoop()
  {
    // picture loop  
    u8g.firstPage();  
    do {
      draw();
    } while( u8g.nextPage() );    
  }

  void checkState()
  {
    if (errors > 0)
    {
      bReady = false;
      sprintf(szStr, "Connecting");
      for (int i = 0; i <= (errors % 10); i++)
      {
        szStr[i+10] = '.';
      }  
      u8g.drawStr(0, 0, szStr);
    }
    else
    {
      bReady = true;
      sprintf(szStr, "Connected");
      u8g.drawStr(0, 0, szStr);
    }
  }
  
  void draw(void) {
    u8g_prepare();
    // clear string
    memset(szStr, 0, sizeof(szStr));
    // Check & show connection state
    checkState();
    // Draw stats
    if (bReady)
    {
      // PID_SPEED
      sprintf(szStr, "Speed: %i", pidSpeed);
      u8g.drawStr(0, 12, szStr);
      // PID_RPM
      sprintf(szStr, "RPM: %i", pidRPM);
      u8g.drawStr(70, 12, szStr);
      // PID_THROTTLE
      sprintf(szStr, "Throttle: %i", pidThrottle);
      u8g.drawStr(0, 24, szStr);
      // PID_MAF_FLOW
      sprintf(szStr, "MAF Flow: %i", pidMAFFlow);
      u8g.drawStr(0, 36, szStr);    
      // PID_INTAKE_TEMP
      sprintf(szStr, "Intake Temp: %i", pidIntakeTemp);
      u8g.drawStr(0, 48, szStr);    
    }
  }  

  void u8g_prepare(void) {
    u8g.setFont(u8g_font_6x10);
    u8g.setFontRefHeightExtendedText();
    u8g.setDefaultForegroundColor();
    u8g.setFontPosTop();
  }
};

static CMyOBD myobd;

void setup(void) {
  // start communication with OBD-II UART adapter
  myobd.begin();
}

void loop(void) {
  myobd.loop();
}
