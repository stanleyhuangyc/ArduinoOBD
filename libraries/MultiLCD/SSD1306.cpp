#include <avr/pgmspace.h>
//#include <util/delay.h>
#include <stdlib.h>
#include <Wire.h>
#include "SSD1306.h"

SSD1306::SSD1306(int8_t SCLK, int8_t DC, int8_t RST, int8_t CS) {
  cs = CS;
  rst = RST;
  dc = DC;
  sclk = SCLK;
}

// initializer for I2C - we only indicate the reset pin!
  SSD1306::SSD1306(int8_t reset) {
  sclk = dc = cs = -1;
  rst = reset;
}


void SSD1306::begin(uint8_t vccstate, uint8_t i2caddr) {
  _i2caddr = i2caddr;


  // set pin directions
    // I2C Init
    Wire.begin(); // Is this the right place for this?

  // Setup reset pin direction (used by both SPI and I2C)
  pinMode(rst, OUTPUT);
  digitalWrite(rst, HIGH);
  // VDD (3.3V) goes high at start, lets just chill for a ms
  delay(1);
  // bring reset low
  digitalWrite(rst, LOW);
  // wait 10ms
  delay(10);
  // bring out of reset
  digitalWrite(rst, HIGH);
  // turn on VCC (9V?)
   #if defined SSD1306_128_32
    // Init sequence for 128x32 OLED module
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                                  // the suggested ratio 0x80
    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(0x1F);
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x0);                                   // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC)
      { ssd1306_command(0x10); }
    else
      { ssd1306_command(0x14); }
    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);                                  // 0x0 act like ks0108
	ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x02);
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    ssd1306_command(0x8F);
    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC)
      { ssd1306_command(0x22); }
    else
      { ssd1306_command(0xF1); }
    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6
  #endif

  #if defined SSD1306_128_64
    // Init sequence for 128x64 OLED module
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                                  // the suggested ratio 0x80
    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(0x3F);
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x0);                                   // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC)
      { ssd1306_command(0x10); }
    else
      { ssd1306_command(0x14); }
    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);                                  // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x12);
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC)
      { ssd1306_command(0x9F); }
    else
      { ssd1306_command(0xCF); }
    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC)
      { ssd1306_command(0x22); }
    else
      { ssd1306_command(0xF1); }
    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6
  #endif

  ssd1306_command(SSD1306_DISPLAYON);//--turn on oled panel

  // clear screen
    delay(5);

    ssd1306_command(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
    ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0); // line #0

    for (byte i = 0; i < SSD1306_LCDHEIGHT / 8; i++) {
      // send a bunch of data in one xmission
        ssd1306_command(0xB0 + i);//set page address
        ssd1306_command(0);//set lower column address
        ssd1306_command(0x10);//set higher column address

        for(byte j = 0; j < 8; j++){
            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte k = 0; k < SSD1306_LCDWIDTH / 8; k++) {
                Wire.write(0);
            }
            Wire.endTransmission();
        }
    }
}


void SSD1306::invertDisplay(uint8_t i) {
  if (i) {
    ssd1306_command(SSD1306_INVERTDISPLAY);
  } else {
    ssd1306_command(SSD1306_NORMALDISPLAY);
  }
}

void SSD1306::ssd1306_command(uint8_t c) {
    // I2C
    uint8_t control = 0x00;   // Co = 0, D/C = 0
    Wire.beginTransmission(_i2caddr);
    Wire.write(control);
    Wire.write(c);
    Wire.endTransmission();
}

// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrollright(uint8_t start, uint8_t stop){
	ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(0XFF);
	ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrollleft(uint8_t start, uint8_t stop){
	ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(0XFF);
	ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrolldiagright(uint8_t start, uint8_t stop){
	ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
	ssd1306_command(0X00);
	ssd1306_command(SSD1306_LCDHEIGHT);
	ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrolldiagleft(uint8_t start, uint8_t stop){
	ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
	ssd1306_command(0X00);
	ssd1306_command(SSD1306_LCDHEIGHT);
	ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

void SSD1306::stopscroll(void){
	ssd1306_command(SSD1306_DEACTIVATE_SCROLL);
}

void SSD1306::ssd1306_data(uint8_t c) {
    // I2C
    uint8_t control = 0x40;   // Co = 0, D/C = 1
    Wire.beginTransmission(_i2caddr);
    Wire.write(control);
    Wire.write(c);
    Wire.endTransmission();
}

void SSD1306::fill(unsigned char dat)
{
    unsigned char i,j;

    ssd1306_command(0x00);//set lower column address
    ssd1306_command(0x10);//set higher column address
    ssd1306_command(0xB0);//set page address

    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!
    for (byte i=0; i<(SSD1306_LCDHEIGHT/8); i++)
    {
        // send a bunch of data in one xmission
        ssd1306_command(0xB0 + i);//set page address
        ssd1306_command(0);//set lower column address
        ssd1306_command(0x10);//set higher column address

        for(byte j = 0; j < 8; j++){
            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte k = 0; k < 16; k++) {
                Wire.write(dat);
            }
            Wire.endTransmission();
        }
    }
    TWBR = twbrbackup;
}

void SSD1306::draw8x8(byte* buffer, uint8_t x, uint8_t y)
{
    // send a bunch of data in one xmission
    ssd1306_command(0xB0 + y);//set page address
    ssd1306_command(x & 0xf);//set lower column address
    ssd1306_command(0x10 | (x >> 4));//set higher column address

    Wire.beginTransmission(_i2caddr);
    Wire.write(0x40);
    Wire.write(buffer, 8);
    Wire.endTransmission();
}
