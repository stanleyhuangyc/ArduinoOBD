#include <Arduino.h>
#include <SPI.h>
#include "MultiLCD.h"

/* Pins
D4 : RESET
D5 : CS
D6 : D/C
D7 : LED
D11 : MOSI
D12 : MISO
D13 : SCK
*/

#define PIN_RESET 4
#define PIN_CS 5
#define PIN_DC 6
#define PIN_LED 7

//Basic Colors
#define RED		0xf800
#define GREEN	0x07e0
#define BLUE	0x001f
#define BLACK	0x0000
#define YELLOW	0xffe0
#define WHITE	0xffff

//Other Colors
#define CYAN		0x07ff
#define BRIGHT_RED	0xf810
#define GRAY1		0x8410
#define GRAY2		0x4208

//TFT resolution 240*320
#define MIN_X	0
#define MIN_Y	0
#define MAX_X	239
#define MAX_Y	319

#define TFT_CS_LOW  digitalWrite(PIN_CS, LOW)
#define TFT_CS_HIGH digitalWrite(PIN_CS, HIGH)
#define TFT_DC_LOW  digitalWrite(PIN_DC, LOW)
#define TFT_DC_HIGH digitalWrite(PIN_DC, HIGH)
#define TFT_RST_OFF digitalWrite(PIN_RESET, HIGH)
#define TFT_RST_ON  digitalWrite(PIN_RESET, LOW)


#define YP A2   // must be an analog pin, use "An" notation!
#define XM A1   // must be an analog pin, use "An" notation!
#define YM 14   // can be a digital pin, this is A0
#define XP 17   // can be a digital pin, this is A3

#define TS_MINX 116*2
#define TS_MAXX 890*2
#define TS_MINY 83*2
#define TS_MAXY 913*2

void LCD_ILI9341::sendCMD(uint8_t index)
{
    TFT_DC_LOW;
    TFT_CS_LOW;
    SPI.transfer(index);
    TFT_CS_HIGH;
}

void LCD_ILI9341::WRITE_DATA(uint8_t data)
{
    TFT_DC_HIGH;
    TFT_CS_LOW;
    SPI.transfer(data);
    TFT_CS_HIGH;
}

void LCD_ILI9341::sendData(uint16_t data)
{
    uint8_t data1 = data>>8;
    uint8_t data2 = data&0xff;
    TFT_DC_HIGH;
    TFT_CS_LOW;
    SPI.transfer(data1);
    SPI.transfer(data2);
    TFT_CS_HIGH;
}

void LCD_ILI9341::WRITE_Package(uint16_t *data, uint8_t howmany)
{
    uint16_t    data1 = 0;
    uint8_t   data2 = 0;

    TFT_DC_HIGH;
    TFT_CS_LOW;
    uint8_t count=0;
    for(count=0;count<howmany;count++)
    {
        data1 = data[count]>>8;
        data2 = data[count]&0xff;
        SPI.transfer(data1);
        SPI.transfer(data2);
    }
    TFT_CS_HIGH;
}

uint8_t LCD_ILI9341::Read_Register(uint8_t Addr, uint8_t xParameter)
{
    uint8_t data=0;
    sendCMD(0xd9);                                                      /* ext command                  */
    WRITE_DATA(0x10+xParameter);                                        /* 0x11 is the first Parameter  */
    TFT_DC_LOW;
    TFT_CS_LOW;
    SPI.transfer(Addr);
    TFT_DC_HIGH;
    data = SPI.transfer(0);
    TFT_CS_HIGH;
    return data;
}

void LCD_ILI9341::begin (void)
{
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_DC, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_RESET, OUTPUT);

    SPI.begin();
    TFT_CS_HIGH;
    TFT_DC_HIGH;
    uint8_t i=0, TFTDriver=0;

	TFT_RST_ON;
	delay(10);
	TFT_RST_OFF;

    for(i=0;i<3;i++)
    {
        TFTDriver = readID();
    }

	sendCMD(0xCB);
	WRITE_DATA(0x39);
	WRITE_DATA(0x2C);
	WRITE_DATA(0x00);
	WRITE_DATA(0x34);
	WRITE_DATA(0x02);

	sendCMD(0xCF);
	WRITE_DATA(0x00);
	WRITE_DATA(0XC1);
	WRITE_DATA(0X30);

	sendCMD(0xE8);
	WRITE_DATA(0x85);
	WRITE_DATA(0x00);
	WRITE_DATA(0x78);

	sendCMD(0xEA);
	WRITE_DATA(0x00);
	WRITE_DATA(0x00);

	sendCMD(0xED);
	WRITE_DATA(0x64);
	WRITE_DATA(0x03);
	WRITE_DATA(0X12);
	WRITE_DATA(0X81);

	sendCMD(0xF7);
	WRITE_DATA(0x20);

	sendCMD(0xC0);    	//Power control
	WRITE_DATA(0x23);   	//VRH[5:0]

	sendCMD(0xC1);    	//Power control
	WRITE_DATA(0x10);   	//SAP[2:0];BT[3:0]

	sendCMD(0xC5);    	//VCM control
	WRITE_DATA(0x3e);   	//Contrast
	WRITE_DATA(0x28);

	sendCMD(0xC7);    	//VCM control2
	WRITE_DATA(0x86);  	 //--

	sendCMD(0x36);    	// Memory Access Control
	WRITE_DATA(0x48);  	//C8

	sendCMD(0x3A);
	WRITE_DATA(0x55);

	sendCMD(0xB1);
	WRITE_DATA(0x00);
	WRITE_DATA(0x18);

	sendCMD(0xB6);    	// Display Function Control
	WRITE_DATA(0x08);
	WRITE_DATA(0x82);
	WRITE_DATA(0x27);

	sendCMD(0xF2);    	// 3Gamma Function Disable
	WRITE_DATA(0x00);

	sendCMD(0x26);    	//Gamma curve selected
	WRITE_DATA(0x01);

	sendCMD(0xE0);    	//Set Gamma
	WRITE_DATA(0x0F);
	WRITE_DATA(0x31);
	WRITE_DATA(0x2B);
	WRITE_DATA(0x0C);
	WRITE_DATA(0x0E);
	WRITE_DATA(0x08);
	WRITE_DATA(0x4E);
	WRITE_DATA(0xF1);
	WRITE_DATA(0x37);
	WRITE_DATA(0x07);
	WRITE_DATA(0x10);
	WRITE_DATA(0x03);
	WRITE_DATA(0x0E);
	WRITE_DATA(0x09);
	WRITE_DATA(0x00);

	sendCMD(0XE1);    	//Set Gamma
	WRITE_DATA(0x00);
	WRITE_DATA(0x0E);
	WRITE_DATA(0x14);
	WRITE_DATA(0x03);
	WRITE_DATA(0x11);
	WRITE_DATA(0x07);
	WRITE_DATA(0x31);
	WRITE_DATA(0xC1);
	WRITE_DATA(0x48);
	WRITE_DATA(0x08);
	WRITE_DATA(0x0F);
	WRITE_DATA(0x0C);
	WRITE_DATA(0x31);
	WRITE_DATA(0x36);
	WRITE_DATA(0x0F);

	sendCMD(0x11);    	//Exit Sleep
	delay(120);

	sendCMD(0x29);    //Display on
	sendCMD(0x2c);
	clear();

	backlight(true);
	setTextColor(0xffff);
	SetBGColor(0);
}

uint8_t LCD_ILI9341::readID(void)
{
    uint8_t i=0;
    uint8_t data[3] ;
    uint8_t ID[3] = {0x00, 0x93, 0x41};
    uint8_t ToF=1;
    for(i=0;i<3;i++)
    {
        data[i]=Read_Register(0xd3,i+1);
        if(data[i] != ID[i])
        {
            ToF=0;
        }
    }
    if(!ToF)                                                            /* data!=ID                     */
    {
#if 0
        Serial.print("Read TFT ID failed, ID should be 0x09341, but read ID = 0x");
        for(i=0;i<3;i++)
        {
            Serial.print(data[i],HEX);
        }
        Serial.println();
#endif
    }
    return ToF;
}

void LCD_ILI9341::setCol(uint16_t StartCol,uint16_t EndCol)
{
    sendCMD(0x2A);                                                      /* Column Command address       */
    sendData(StartCol);
    sendData(EndCol);
}

void LCD_ILI9341::setPage(uint16_t StartPage,uint16_t EndPage)
{
    sendCMD(0x2B);                                                      /* Column Command address       */
    sendData(StartPage);
    sendData(EndPage);
}

void LCD_ILI9341::fill(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1, uint16_t color)
{
    uint8_t Hcolor = color>>8;
    uint8_t Lcolor = color&0xff;

    setCol(239 - y1, 239 - y0);
    setPage(x0, x1);
    sendCMD(0x2c);                                                  /* start to write to display ra */
                                                                        /* m                            */
    TFT_DC_HIGH;
    TFT_CS_LOW;
    for(uint16_t n = (x1-x0+1) * (y1-y0+1); n > 0; n--)
    {
        SPI.transfer(Hcolor);
        SPI.transfer(Lcolor);
    }
    TFT_CS_HIGH;
}

void LCD_ILI9341::clear(void)
{
    setCol(0, 239);
    setPage(0, 319);
    sendCMD(0x2c);                                                  /* start to write to display ra */
                                                                        /* m                            */

    TFT_DC_HIGH;
    TFT_CS_LOW;
    for(uint16_t i=0; i<38400; i++)
    {
        SPI.transfer(0);
        SPI.transfer(0);
        SPI.transfer(0);
        SPI.transfer(0);
    }
    TFT_CS_HIGH;

    m_x = 0;
    m_y = 0;
}


void LCD_ILI9341::setXY(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1)
{
    setCol(239 - x1, 239 - x0);
    setPage(y0, y1);
    sendCMD(0x2c);
}

void LCD_ILI9341::setPixel(uint16_t poX, uint16_t poY,uint16_t color)
{
    setXY(poY, poY, poX, poX);
    sendData(color);
}

void LCD_ILI9341::backlight(bool on)
{
    digitalWrite(PIN_LED, on);
}

void LCD_ILI9341::clearPixels(uint16_t pixels)
{
    TFT_DC_HIGH;
    TFT_CS_LOW;
    for(uint16_t i = 0; i < pixels; i++)
    {
        SPI.transfer(0);
        SPI.transfer(0);
    }
    TFT_CS_HIGH;
}

void LCD_ILI9341::sendPixelData(byte d)
{
    for (byte j = 0; j < 8; j++, d <<= 1) {
        if (d & 0x80) {
            SPI.transfer(m_color[1][1]);
            SPI.transfer(m_color[1][0]);
        } else {
            SPI.transfer(m_color[0][1]);
            SPI.transfer(m_color[0][0]);
        }
    }
}

size_t LCD_ILI9341::write(uint8_t c)
{
    if (c == '\n') {
        m_y += (m_font + 1) << 3;
        return 0;
    } else if (c == '\r') {
        m_x = 0;
        return 0;
    }

#ifndef MEMORY_SAVING
    if (m_font == FONT_SIZE_SMALL) {
#endif
        setXY(m_y, m_y + 7, m_x, m_x + 4);
        m_x += 6;
        if (m_x >= 320) {
            m_y += 8;
            m_x = 0;
            if (m_y >= 240) m_y = 0;
        }
        if (c > 0x20 && c < 0x7f) {
            byte pgm_buffer[5];
            memcpy_P(pgm_buffer, &font5x8[c - 0x21], 5);
            TFT_DC_HIGH;
            TFT_CS_LOW;
            for (byte i = 0; i < 5; i++) {
                sendPixelData(pgm_buffer[i]);
            }
            TFT_CS_HIGH;
        } else {
            clearPixels(5 * 8);
        }
#ifndef MEMORY_SAVING
    } else {
        setXY(m_y, m_y + 15, m_x, m_x + 7);
        m_x += 9;
        if (m_x >= 320) {
            m_y += 16;
            m_x = 0;
            if (m_y >= 240) m_y = 0;
        }
        if (c > 0x20 && c < 0x7f) {
            byte pgm_buffer[16];
            memcpy_P(pgm_buffer, &font8x16_terminal[c - 0x21], 16);
            TFT_DC_HIGH;
            TFT_CS_LOW;
            for (byte i = 0; i < 16; i += 2) {
                sendPixelData(pgm_buffer[i + 1]);
                sendPixelData(pgm_buffer[i]);
            }
            TFT_CS_HIGH;
        } else {
            clearPixels(8 * 16);
        }
    }
#endif
}

void LCD_ILI9341::writeDigit(byte n)
{
    if (m_font == FONT_SIZE_LARGE) {
        setXY(m_y, m_y + 15, m_x, m_x + 15);
        m_x += 16;
        if (n <= 9) {
            byte pgm_buffer[32];
            memcpy_P(pgm_buffer, &digits16x16[n], sizeof(pgm_buffer));
            TFT_DC_HIGH;
            TFT_CS_LOW;
            for (byte i = 0; i < 16; i++) {
                sendPixelData(pgm_buffer[16 + i]);
                sendPixelData(pgm_buffer[i]);
            }
            TFT_CS_HIGH;
        } else {
            clearPixels(16 * 16);
        }
    } else if (m_font == FONT_SIZE_XLARGE) {
        setXY(m_y, m_y + 23, m_x, m_x + 15);
        m_x += 17;
        if (n <= 9) {
            byte pgm_buffer[48];
            memcpy_P(pgm_buffer, &digits16x24[n], sizeof(pgm_buffer));
            TFT_DC_HIGH;
            TFT_CS_LOW;
            for (int i = 0; i < 48; i += 3) {
                sendPixelData(pgm_buffer[i + 2]);
                sendPixelData(pgm_buffer[i + 1]);
                sendPixelData(pgm_buffer[i]);
            }
            TFT_CS_HIGH;
        } else {
            clearPixels(16 * 24);
        }
    } else {
        write(n <= 9 ? ('0' + n) : ' ');
    }
}

void LCD_ILI9341::draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height)
{
    byte rows = height >> 3;
    setXY(m_y, m_y + height - 1, m_x, m_x + width - 1);
    uint16_t i = width - 1;
    TFT_DC_HIGH;
    TFT_CS_LOW;
    for (uint16_t i = 0; i < width; i++) {
        for (int8_t h = rows - 1; h >= 0; h--) {
            byte d = pgm_read_byte(buffer + i + width * h);
            sendPixelData(d);
        }
    }
    TFT_CS_HIGH;
    m_x += width;
}

void LCD_ILI9341::draw2x(const PROGMEM byte* buffer, byte width, byte height)
{
    byte rows = height >> 3;
    setXY(m_y, m_y + height * 2 - 1, m_x, m_x + width * 2 - 1);
    uint16_t i = width - 1;
    uint16_t w = width << 1;
    TFT_DC_HIGH;
    TFT_CS_LOW;
    for (uint16_t i = 0; i < w; i++) {
        for (int8_t h = rows - 1; h >= 0; h--) {
            byte d = pgm_read_byte(buffer + (i >> 1) + width * h);
            for (byte j = 0; j < 8; j++, d <<= 1) {
                if (d & 0x80) {
                    SPI.transfer(m_color[1][1]);
                    SPI.transfer(m_color[1][0]);
                    SPI.transfer(m_color[1][1]);
                    SPI.transfer(m_color[1][0]);
                } else {
                    SPI.transfer(m_color[0][1]);
                    SPI.transfer(m_color[0][0]);
                    SPI.transfer(m_color[0][1]);
                    SPI.transfer(m_color[0][0]);
                }
            }
        }
    };
    TFT_CS_HIGH;
    m_x += (width << 1);
}
