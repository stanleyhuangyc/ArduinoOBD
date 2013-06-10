/*
  ZtLib.cpp - ZT module Drive Library for Wiring & Arduino
  Copyright (c) 2012 Alvin Li(Kozig/www.kozig.com).  All right reserved.
  This library is free software;
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  Version:V1.1
*/

#ifndef __ZTLIB_H__
#define __ZTLIB_H__

#include <inttypes.h>

//////////////////
#define ZTSEG8B4A036A_DADDR 0x51
#define SET_ADDR       0x61
#define WRITE_CODE     0xAA
#define WRITE_CMD      0x55

#define DOT_NONE       (0)
#define DOT_BIT1       (1<<0)
#define DOT_BIT2       (1<<1)
#define DOT_BIT3       (1<<2)
#define DOT_BIT4       (1<<3)

#define DISP_0BIT      (0)
#define DISP_1BIT      (1)
#define DISP_2BIT      (2)
#define DISP_3BIT      (3)
#define DISP_4BIT      (4)
#define DISP_AUTO      (5)
///////////////////////////////////

#define ZTSCI2CMX_DADDRESS    0x51
// Ä£¿é¼Ä´æÆ÷µØÖ·
#define REG_CMD          0x01
#define REG_DAT          0x02
#define REG_RESET        0x03
    #define RESET_OLED       0x06
#define REG_VERSION      0x1F
#define REG_SLEEP        0x04
    #define SLEEP_ON     0xA5
    #define SLEEP_OFF    0xA1
#define REG_VCOMH         0x05
#define REG_STATUS        0x06
    #define STATUS_RUN            0x00
    #define STATUS_RUN            0x00
    #define STATUS_SLEEP          0x01
    #define STATUS_SET_ADDRESS    0x02
    #define STATUS_TEST           0x04
    #define STATUS_BUSY           0x10

#define REG_ADDRESS      0x08
#define REG_BRIGHTNESS   0x0A
#define REG_8X16STR      0x52
#define REG_OLED_XY      0x60
#define REG_FILL_AREA    0x61
#define REG_SCROHOR      0x62
#define REG_SCROVER      0x63
#define REG_SCROVERHOR   0x64

#define PAGE0           0x00
#define PAGE1           0x01
#define PAGE2           0x02
#define PAGE3           0x03
#define PAGE4           0x04
#define PAGE5           0x05
#define PAGE6           0x06
#define PAGE7           0x07

#define SCROLL_UP       0x01
#define SCROLL_DOWN     0x00
#define SCROLL_RIGHT    0x26
#define SCROLL_LEFT     0x27
#define SCROLL_VR       0x29
#define SCROLL_VL       0x2A

#define FRAMS_2         0x07
#define FRAMS_3         0x04
#define FRAMS_4         0x05
#define FRAMS_5         0x00
#define FRAMS_25        0x06
#define FRAMS_64        0x01
#define FRAMS_128       0x02
#define FRAMS_256       0x03

class ZtLib
{
  private:

  public:
    void I2cInit(void);
// Module ZT.SEG8B4A036A FUNCTION
    int Seg8b4a036aSleep(uint8_t);
    int Seg8b4a036aUnSleep(uint8_t);
    int Seg8b4a036aReadState(uint8_t addr);
    int Seg8b4a036aReadVersion(uint8_t addr, uint8_t *buf);
    int Seg8b4a036aDisplayDec(uint8_t,unsigned short, uint8_t, uint8_t);
    int Seg8b4a036aDisplayHex(uint8_t,unsigned short, uint8_t, uint8_t);
    int Seg8b4a036aSetBrightness(uint8_t, uint8_t, uint8_t);
    int Seg8b4a036aSetAddress(uint8_t);
    int Seg8b4a036aDisplayBuff(uint8_t,uint8_t *);
// Module ZT.SC-I2CMx
    int ScI2cMxReadState(uint8_t);
    int ScI2cMxReadVersion(uint8_t, uint8_t *);
    int ScI2cMxSetAddress(uint8_t);
    int ScI2cMxSetBrightness(uint8_t, uint8_t);
    int ScI2cMxSetVcomH(uint8_t, uint8_t);
    int ScI2cMxDisplay8x16Str(uint8_t, uint8_t, uint8_t, const char *);
    int ScI2cMxFillArea(uint8_t, uint8_t, uint8_t,uint8_t, uint8_t,uint8_t);
    int ScI2cMxScrollingHorizontal(uint8_t, uint8_t, uint8_t, uint8_t,uint8_t);
    int ScI2cMxScrollingVertical(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
    int ScI2cMxScrollingVerticalHorizontal(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
    int ScI2cMxDeactivateScroll(uint8_t);
    int ScI2cMxReset(uint8_t);
    int ScI2cMxSetLocation(uint8_t, uint8_t, uint8_t);
    void ScI2cMxDisplayDot(uint8_t, const PROGMEM uint8_t* buffer, uint8_t len);
    void ScI2cMxDisplayDot16x16(uint8_t, uint8_t, uint8_t, unsigned char *);
    void ScI2cMxDisplayArea(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, const char *);
};


extern ZtLib ZT;

#endif

