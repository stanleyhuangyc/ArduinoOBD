/*
  ZtLib.cpp - ZT module Drive Library for Wiring & Arduino
  Copyright (c) 2012 Alvin Li(Kozig/www.kozig.com).  All right reserved.
  This library is free software;
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  Version:V1.1
*/

#include <Arduino.h>
extern "C" {
  #include "../Wire/utility/twi.h"
}

#include "ZtLib.h"

///ZT.SEG8B4A036A PART///-------------------------------------------------------------------s
unsigned char codetable[] =
{
   0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F, 0x77, 0x7C,0x39,0x5E,0x79,0x71,0x00
};

// Public Methods //////////////////////////////////////////////////////////////
/*
 * Function I2cInit
 * Desc     TWI/I2C init
 * Input    none
 * Output   none
 */
void ZtLib::I2cInit(void)
{
   twi_init();
}
/*
 * Function Seg8b4a036aSleep
 * Desc     Set ZT.SEG8B4A036A Go to Sleep
 * Input    addr:ZT.SEG8B4A036A Address
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::Seg8b4a036aSleep(uint8_t addr)
{
  uint8_t buff[5]={REG_SLEEP, SLEEP_ON, 0, 0, 0};

  return twi_writeTo(addr, buff, 5, 1, 1);
}
/*
 * Function Seg8b4a036aUnSleep
 * Desc     Set ZT.SEG8B4A036A Wait Up From Sleep
 * Input    addr:ZT.SEG8B4A036A Address
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::Seg8b4a036aUnSleep(uint8_t addr)
{
  uint8_t buff[5]={REG_SLEEP, SLEEP_OFF, 0, 0, 0};

  return twi_writeTo(addr, buff, 5, 1, 1);
}
/*
 * Function Seg8b4a036aReadState
 * Desc     Read ZT.SEG8B4A036A Status
 * Input    addr:ZT.SEG8B4A036A Address
 * Output   !=0xFF ZT.SC-I2CMx Status
 *          0xFF .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::Seg8b4a036aReadState(uint8_t addr)
{
   uint8_t state = 0xFF;
   uint8_t temp;
   uint8_t buff[1] = {REG_STATUS};
   temp = twi_writeTo(addr, buff, 1, 1, 0); // no stop
   if (temp ==0)
   {
      temp = twi_readFrom(addr, buff, 1, 1);
   }
   if (temp==1)
   {
      state = buff[0];
   }

   return state;
}
/*
 * Function Seg8b4a036aReadVersion
 * Desc     Read ZT.SEG8B4A036A Fireware Version
 * Input    addr:ZT.SEG8B4A036A Address
            *buf:Version Buffer
 * Output   .. number bytes of Version Read out
 */
int ZtLib::Seg8b4a036aReadVersion(uint8_t addr, uint8_t *buf)
{
   uint8_t state = 0xFF;
   uint8_t temp;
   uint8_t regv[1] = {REG_VERSION};
   temp = twi_writeTo(addr, regv, 1, 1, 0); // no stop
   if (temp ==0)
   {
      temp = twi_readFrom(addr, &(*buf), 19, 1);
   }
   return temp;
}
/*
 * Function Seg8b4a036aDisplayDec
 * Desc     ZT.SEG8B4A036A Display decimal numeral
 * Input    addr:ZT.SEG8B4A036A Address
            val: Display Val
            bitnum:Display Bit Number
            dotbit: Dot Display
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::Seg8b4a036aDisplayDec(uint8_t addr,unsigned short val, uint8_t bitnum, uint8_t dotbit)
{
  uint8_t i;
  uint8_t segnum[5];
  if (val>9999) return 0xFF;

  segnum[0] = REG_DAT;
  segnum[1] = val%10;
  segnum[2] = (val%100)/10;
  segnum[3] = (val/100)%10;
  segnum[4] = val/1000;
  for (i=1; i<5; i++)
  {
      segnum[i] = codetable[segnum[i]];
      if (dotbit&0x01)
      {
          segnum[i] |= 0x80;
      }
      dotbit >>= 1;
  }

  if (bitnum==DISP_0BIT)      {segnum[4] = 0;segnum[3] = 0;segnum[2] = 0;segnum[1] = 0;}
  else if (bitnum==DISP_1BIT) {segnum[4] = 0;segnum[3] = 0;segnum[2] = 0;}
  else if (bitnum==DISP_2BIT) {segnum[4] = 0;segnum[3] = 0;}
  else if (bitnum==DISP_3BIT) {segnum[4] = 0;}
  else if (bitnum==DISP_AUTO)
  {
     if (val<10)        {segnum[4] = 0;segnum[3] = 0;segnum[2] = 0;}
     else if (val<100)  {segnum[4] = 0;segnum[3] = 0;}
     else if (val<1000) {segnum[4] = 0;}
  }

  return twi_writeTo(addr, segnum, 5, 1, 1);
}
/*
 * Function Seg8b4a036aDisplayHex
 * Desc     Read ZT.SEG8B4A036A Display hexadecimal number
 * Input    addr:ZT.SEG8B4A036A Address
            val: Display Val
            bitnum:Display Bit Number
            dotbit: Dot Display
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::Seg8b4a036aDisplayHex(uint8_t addr,unsigned short val, uint8_t bitnum, uint8_t dotbit)
{
  uint8_t i;
  unsigned short temp;
  uint8_t segnum[5];
  segnum[0] = REG_DAT;
  temp = val;
  for (i=1; i<5; i++)
  {
      segnum[i] = temp&0x000F;
      temp >>= 4;
      segnum[i] = codetable[segnum[i]];
      if (dotbit&0x01)
      {
          segnum[i] |= 0x80;
      }
      dotbit >>= 1;
  }

  if (bitnum==DISP_0BIT)      {segnum[4] = 0;segnum[3] = 0;segnum[2] = 0;segnum[1] = 0;}
  else if (bitnum==DISP_1BIT) {segnum[4] = 0;segnum[3] = 0;segnum[2] = 0;}
  else if (bitnum==DISP_2BIT) {segnum[4] = 0;segnum[3] = 0;}
  else if (bitnum==DISP_3BIT) {segnum[4] = 0;}
  else if (bitnum==DISP_AUTO)
  {
     if (!(val&0xFFF0))      {segnum[4] = 0;segnum[3] = 0;segnum[2] = 0;}
     else if (!(val&0xFF00)) {segnum[4] = 0;segnum[3] = 0;}
     else if (!(val&0xF000)) {segnum[4] = 0;}
  }

  return twi_writeTo(addr, segnum, 5, 1, 1);
}
/*
 * Function Seg8b4a036aSetBrightness
 * Desc     Set ZT.SEG8B4A036A Brightness
 * Input    addr:ZT.SEG8B4A036A Address
            OnDelay:
            OffDelay:
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::Seg8b4a036aSetBrightness(uint8_t addr, uint8_t OnDelay, uint8_t OffDelay)
{
  uint8_t buff[5] = {REG_BRIGHTNESS, OnDelay, OffDelay, 0, 0};
  return twi_writeTo(addr, buff, 5, 1, 1);
}
/*
 * Function Seg8b4a036aSetAddress
 * Desc     Set ZT.SEG8B4A036A New Address
 * Input    val:ZT.SEG8B4A036A Address New Address
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::Seg8b4a036aSetAddress(uint8_t val)
{
   uint8_t buff[2] = {REG_ADDRESS, val};
   return twi_writeTo(ZTSEG8B4A036A_DADDR, buff, 2, 1, 1);
}
/*
 * Function Seg8b4a036aDisplayBuff
 * Desc     Set ZT.SEG8B4A036A Brightness
 * Input    addr:ZT.SEG8B4A036A Address
            *buf: Display buffer
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::Seg8b4a036aDisplayBuff(uint8_t addr,uint8_t *buf)
{
  uint8_t buff[5]={REG_DAT, buf[0], buf[1], buf[2], buf[3]};

  return twi_writeTo(addr, buff, 5, 1, 1);
}


///ZT.ScI2cMx PART///-------------------------------------------------------------------
/*
 * Function ScI2cMxReadState
 * Desc     Read ZT.SC-I2CMx Status
 * Input    addr:ZT.SC-I2CMx Address
 * Output   !=0xFF ZT.SC-I2CMx Status
 *          0xFF .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxReadState(uint8_t addr)
{
   uint8_t state = 0xFF;
   uint8_t temp;
   uint8_t buff[1] = {REG_STATUS};
   temp = twi_writeTo(addr, buff, 1, 1, 0); // no stop
   if (temp ==0)
   {
      temp = twi_readFrom(addr, buff, 1, 1);
   }
   if (temp==1)
   {
      state = buff[0];
   }

   return state;
}

/*
 * Function ScI2cMxReadVersion
 * Desc     Read ZT.SC-I2CMx Fireware Version
 * Input    addr:ZT.SC-I2CMx Address
            *buf:Version Buffer
 * Output   !=0xFF ZT.SC-I2CMx Status
 *          othe .. number bytes of Version Read out
 */
int ZtLib::ScI2cMxReadVersion(uint8_t addr, uint8_t *buf)
{
   uint8_t state = 0xFF;
   uint8_t temp;
   uint8_t regv[1] = {REG_VERSION};
   temp = twi_writeTo(addr, regv, 1, 1, 0); // no stop
   if (temp ==0)
   {
      temp = twi_readFrom(addr, &(*buf), 16, 1);
   }
   return temp;
}
/*
 * Function ScI2cMxSetAddress
 * Desc     Set ZT.SC-I2CMx New Address
 * Input    val:ZT.SC-I2CMx Address New Address
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxSetAddress(uint8_t newaddr)
{
   uint8_t buff[2] = {REG_ADDRESS, newaddr};
   return twi_writeTo(ZTSCI2CMX_DADDRESS, buff, 2, 1, 1);
}

/*
 * Function ScI2cMxSetBrightness
 * Desc     Set ZT.SC-I2CMx Brightness
 * Input    addr:ZT.SC-I2CMx Address
            val: Brightness 0~0xFF
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxSetBrightness(uint8_t addr, uint8_t val)
{
   uint8_t buff[2] = {REG_BRIGHTNESS, val};
   return twi_writeTo(addr, buff, 2, 1, 1);
}
/*
 * Function ScI2cMxSetVcomH
 * Desc     Set ZT.SC-I2CMx VcomH
 * Input    addr:ZT.SC-I2CMx Address
            val: Brightness 0~7
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxSetVcomH(uint8_t addr, uint8_t val)
{
   uint8_t buff[2] = {REG_VCOMH, val};
   return twi_writeTo(addr, buff, 2, 1, 1);
}

/*
 * Function ScI2cMxDisplay8x16Str
 * Desc     ZT.SC-I2CMx Display 8x16 English String
 * Input    addr:ZT.SC-I2CMx Address
            page: location page
            column: location column
            *str: 8X16 English String
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxDisplay8x16Str(uint8_t addr, uint8_t page, uint8_t column, const char *str)
{
    uint8_t i=0;
    uint8_t buff[19];
    buff[0] = REG_8X16STR;
    buff[1] = page;
    buff[2] = column;
    i=0;
    while ((*str != '\0') && (i<16))
    {
       buff[i+3] = (uint8_t)*str++;
       i++;
    }
    return twi_writeTo(addr, buff, i+3, 1, 1);
}
/*
 * Function ScI2cMxFillArea
 * Desc     ZT.SC-I2CMx Fill Area
 * Input    addr:ZT.SC-I2CMx Address
            spage: start page
            epage: end page
            scolumn: start column
            ecolumn: end column
            filldata: fill data
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxFillArea(uint8_t addr, uint8_t spage, uint8_t epage,uint8_t scolumn, uint8_t ecolumn,uint8_t filldata)
{
    uint8_t buff[6] = {REG_FILL_AREA, spage, epage, scolumn, ecolumn, filldata};
    return twi_writeTo(addr, buff, 6, 1, 1);
}
/*
 * Function ScI2cMxScrollingHorizontal
 * Desc     ZT.SC-I2CMx Scrolling Horizontal
 * Input    addr:ZT.SC-I2CMx Address
            lr: Scroll direction
            spage: start page
            epage: end page
            frames: Scroll fram
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxScrollingHorizontal(uint8_t addr, uint8_t lr, uint8_t spage, uint8_t epage,uint8_t frames)
{
    uint8_t buff[9] = {REG_CMD, 0x2E, 0x00, spage, frames, epage, 0x00, 0xFF, 0x2F};
    twi_writeTo(addr, buff, 2, 1, 1);
    buff[0] = REG_CMD;
    buff[1] = lr;
    for (int i=0; i<10; i++);
    return twi_writeTo(addr, buff, 9, 1, 1);
}
/*
 * Function ScI2cMxScrollingHorizontal
 * Desc     ZT.SC-I2CMx Scrolling Vertical
 * Input    addr:ZT.SC-I2CMx Address
            lr: Scroll direction
            rowsfixed: rows fixed
            rowsscroll: rows scroll
            scrollstep: scroll step
            stepdelay: step delay
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxScrollingVertical(uint8_t addr, uint8_t scrollupdown, uint8_t rowsfixed, uint8_t rowsscroll, uint8_t scrollstep, uint8_t stepdelay)
{
    uint8_t buff[6] = {REG_SCROVER, scrollupdown, rowsfixed, rowsscroll, scrollstep, stepdelay};
    return twi_writeTo(addr, buff, 6, 1, 1);
}
/*
  * function  ScI2cMxScrollingVerticalHorizontal
  * Desc  Continuous Vertical / Horizontal / Diagonal Scrolling (Partial or Full Screen)
  * input :
    Sdirection: Scrolling Direction
        "0x00" (Vertical & Rightward)
        "0x01" (Vertical & Leftward)
    spage: Define Start Page Address (Horizontal / Diagonal Scrolling)
    epage: Define End Page Address (Horizontal / Diagonal Scrolling)
    fixedarea: Set Top Fixed Area (Vertical Scrolling)
    scrollarea: Set Vertical Scroll Area (Vertical Scrolling)
    frames: Set Time Interval between Each Scroll Step in Terms of Frame Frequency
    offset: Set Numbers of Row Scroll per Step (Vertical / Diagonal Scrolling)
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxScrollingVerticalHorizontal(uint8_t addr, uint8_t Sdirection, uint8_t spage, uint8_t epage, uint8_t fixedarea, uint8_t scrollarea, uint8_t offset, uint8_t frames)
{
    uint8_t buff[8] = {REG_SCROVERHOR, Sdirection, spage, epage, fixedarea, scrollarea, offset, frames};
    return twi_writeTo(addr, buff, 8, 1, 1);
}
/*
 * Function ScI2cMxDeactivateScroll
 * Desc     ZT.SC-I2CMx Deactivate Scroll
 * Input    addr:ZT.SC-I2CMx Address
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxDeactivateScroll(uint8_t addr)
{
    uint8_t buff[2] = {REG_CMD, 0x2E};
    return twi_writeTo(addr, buff, 2, 1, 1);
}

/*
 * Function ScI2cMxReset
 * Desc     ZT.SC-I2CMx Reset OLED
 * Input    addr:ZT.SC-I2CMx Address
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxReset(uint8_t addr)
{
    uint8_t buff[2] = {REG_RESET,RESET_OLED};
    return twi_writeTo(addr, buff, 2, 1, 1);
}

/*
 * Function ScI2cMxSetLocation
 * Desc     Set ZT.SC-I2CMx SetLocation
 * Input    addr:ZT.SC-I2CMx Address
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
int ZtLib::ScI2cMxSetLocation(uint8_t addr, uint8_t page,uint8_t column)
{
    uint8_t buff[4] = {REG_CMD, (0xB0|page), (column%16), (column/16+0x10)};
    return twi_writeTo(addr, buff, 4, 1, 1);
}
/*
 * Function ScI2cMxDisplayDot16x16
 * Desc     Set ZT.SC-I2CMx Display 16*16 Dot
 * Input    addr:ZT.SC-I2CMx Address
            page:page
            column:column
            *str:16*16 Dot Data
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
void ZtLib::ScI2cMxDisplayDot16x16(uint8_t addr, uint8_t page, uint8_t column, unsigned char *str)
{
    uint8_t buff[17];
    buff[0] = REG_DAT;
    ScI2cMxSetLocation(addr, page, column);
    for (int i=0; i<16; i++)
    {
       buff[i+1] = str[i];
    }
    twi_writeTo(addr, buff, 17, 1, 1);
    ScI2cMxSetLocation(addr, page+1, column);
    for (int i=0; i<16; i++)
    {
       buff[i+1] = str[i+16];
    }
    twi_writeTo(addr, buff, 17, 1, 1);
}

void ZtLib::ScI2cMxDisplayDot(uint8_t addr, const PROGMEM uint8_t* buffer, uint8_t len)
{
    uint8_t buff[9] = {REG_DAT};
    memcpy_P(buff + 1, buffer, len);
    twi_writeTo(addr, buff, 9, 1, 1);
}

/*
 * Function ScI2cMxDisplayArea
 * Desc     Set ZT.SC-I2CMx Display Area
 * Input    addr:ZT.SC-I2CMx Address
            spage: start page
            epage: end page
            scolumn: start column
            ecolumn: end column
            *pt: Data
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
void ZtLib::ScI2cMxDisplayArea(uint8_t addr, uint8_t spage, uint8_t epage, uint8_t scolumn, uint8_t ecolumn, const char *pt)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t h = 0;
    uint8_t w = 0;
    uint16_t cnt = 0;
    uint8_t buff[32];
    buff[0] = REG_DAT;

    h = epage - spage;
    w = ecolumn - scolumn;

    while ( j<h )
    {
        ScI2cMxSetLocation(addr, spage + j, scolumn);
        uint8_t p=w;
        while(p)
        {
            if(p>=31)
            {
                for (int n=0; n<31; n++)
                {
                    buff[1+n] = pt[cnt++];
                }
                twi_writeTo(addr, buff, 32, 1, 1);
                p -= 31;
            }
            else
            {
                int n;
                for (n=0; n<p; n++)
                {
                    buff[1+n] = pt[cnt++];
                }
                twi_writeTo(addr, buff, n+1, 1, 1);
                p -= n;
            }
        }
        j++;
    }
}

// Preinstantiate Objects //////////////////////////////////////////////////////

