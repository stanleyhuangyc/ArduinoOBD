/*************************************************************************
* Arduino Text & Bitmap Display Library for color LCDs
* Distributed under GPL v2.0
* Developed by Stanley Huang <stanleyhuangyc@gmail.com>
* For more information, please visit http://arduinodev.com
*************************************************************************/

#include <Arduino.h>
#include "MultiLCD.h"

/**********************************************
Define zone
**********************************************/

#define PIN_BACKLIGHT 8

#define PIN_CLK 6
#define PIN_CS 5
#define PIN_DIN 4
#define PIN_DOUT 3
#define PIN_IRQ 2

#define PREC_TOUCH_CONST 10

#define PixSizeX	136
#define PixOffsX	425

#define PixSizeY	106
#define PixOffsY	366


/**********************************************
Standard C functions zone
**********************************************/
void LCD_SSD1289::begin()
{
	_hw_special_init();

	pinMode(__p1,OUTPUT);
	pinMode(__p2,OUTPUT);
	pinMode(__p3,OUTPUT);
	if (__p4 != NOTINUSE)
		pinMode(__p4,OUTPUT);
	if ((display_transfer_mode==LATCHED_16) or ((display_transfer_mode==1) and (display_serial_mode==SERIAL_5PIN)))
		pinMode(__p5,OUTPUT);
	if (display_transfer_mode!=1)
		_set_direction_registers(display_transfer_mode);

	sbi(P_RST, B_RST);
	delay(5); 
	cbi(P_RST, B_RST);
	delay(15);
	sbi(P_RST, B_RST);
	delay(15);

	setColor(0xffff);
	setBackColor(0);
	_transparent = false;

	cbi(P_CS, B_CS);

	LCD_Write_COM_DATA(0x00,0x0001);
	LCD_Write_COM_DATA(0x03,0xA8A4);
	LCD_Write_COM_DATA(0x0C,0x0000);
	LCD_Write_COM_DATA(0x0D,0x080C);
	LCD_Write_COM_DATA(0x0E,0x2B00);
	LCD_Write_COM_DATA(0x1E,0x00B7);
	LCD_Write_COM_DATA(0x01,0x2B3F);
	LCD_Write_COM_DATA(0x02,0x0600);
	LCD_Write_COM_DATA(0x10,0x0000);
	LCD_Write_COM_DATA(0x11,0x6070);
	LCD_Write_COM_DATA(0x05,0x0000);
	LCD_Write_COM_DATA(0x06,0x0000);
	LCD_Write_COM_DATA(0x16,0xEF1C);
	LCD_Write_COM_DATA(0x17,0x0003);
	LCD_Write_COM_DATA(0x07,0x0233);
	LCD_Write_COM_DATA(0x0B,0x0000);
	LCD_Write_COM_DATA(0x0F,0x0000);
	LCD_Write_COM_DATA(0x41,0x0000);
	LCD_Write_COM_DATA(0x42,0x0000);
	LCD_Write_COM_DATA(0x48,0x0000);
	LCD_Write_COM_DATA(0x49,0x013F);
	LCD_Write_COM_DATA(0x4A,0x0000);
	LCD_Write_COM_DATA(0x4B,0x0000);
	LCD_Write_COM_DATA(0x44,0xEF00);
	LCD_Write_COM_DATA(0x45,0x0000);
	LCD_Write_COM_DATA(0x46,0x013F);
	LCD_Write_COM_DATA(0x30,0x0707);
	LCD_Write_COM_DATA(0x31,0x0204);
	LCD_Write_COM_DATA(0x32,0x0204);
	LCD_Write_COM_DATA(0x33,0x0502);
	LCD_Write_COM_DATA(0x34,0x0507);
	LCD_Write_COM_DATA(0x35,0x0204);
	LCD_Write_COM_DATA(0x36,0x0204);
	LCD_Write_COM_DATA(0x37,0x0502);
	LCD_Write_COM_DATA(0x3A,0x0302);
	LCD_Write_COM_DATA(0x3B,0x0302);
	LCD_Write_COM_DATA(0x23,0x0000);
	LCD_Write_COM_DATA(0x24,0x0000);
	LCD_Write_COM_DATA(0x25,0x8000);
	LCD_Write_COM_DATA(0x4f,0x0000);
	LCD_Write_COM_DATA(0x4e,0x0000);
	LCD_Write_COM(0x22);   

	sbi (P_CS, B_CS); 

	clear();

	pinMode(PIN_BACKLIGHT, OUTPUT);
	digitalWrite(PIN_BACKLIGHT, HIGH);

	// set up pins for touch controller
	pinMode(PIN_CLK,  OUTPUT);
	pinMode(PIN_CS,   OUTPUT);
	pinMode(PIN_DIN,  OUTPUT);
	pinMode(PIN_DOUT, INPUT);
	pinMode(PIN_IRQ,  INPUT);

	digitalWrite(PIN_CS,  HIGH);
	digitalWrite(PIN_CLK, HIGH);
	digitalWrite(PIN_DIN, HIGH);
	digitalWrite(PIN_CLK, HIGH);

}

void LCD_SSD1289::setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    swap(word, x1, y1);
    swap(word, x2, y2)
    y1=disp_y_size-y1;
    y2=disp_y_size-y2;
    swap(word, y1, y2)
    // begin hardware specific code
	LCD_Write_COM_DATA(0x44,(x2<<8)+x1);
	LCD_Write_COM_DATA(0x45,y1);
	LCD_Write_COM_DATA(0x46,y2);
	LCD_Write_COM_DATA(0x4e,x1);
	LCD_Write_COM_DATA(0x4f,y1);
	LCD_Write_COM(0x22); 
}

void LCD_SSD1289::setBackLight(byte brightness)
{
    analogWrite(PIN_BACKLIGHT, brightness);
}

void LCD_SSD1289::Enable()
{
    cbi(P_CS, B_CS);
}
void LCD_SSD1289::Disable()
{
    sbi(P_CS, B_CS);
}

void LCD_SSD1289::drawPixel(uint16_t poX, uint16_t poY,uint16_t color)
{
    Enable();
    setXY(poX, poY, poX, poY);
    setPixel(color);
    Disable();
}

void LCD_SSD1289::clearPixels(uint32_t pixels)
{
    Enable();
    do {
        setPixel(bch, bcl);
    } while(--pixels);
    Disable();
}

void LCD_SSD1289::clear(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    setColor(0);
    fillRect(x, y, x + width, y + height);
    setColor(0xffff);
    m_x = 0;
    m_y = 0;
}

size_t LCD_SSD1289::write(uint8_t c)
{
    if (c == '\n') {
        m_y += (m_font + 1) << 3;
        return 0;
    } else if (c == '\r') {
        m_x = 0;
        return 0;
    }
    if (m_x > disp_y_size) return 0;
    Enable();
    if (m_font == FONT_SIZE_SMALL) {
        setXY(m_x, m_y, m_x + 4, m_y + 7);
        if (c > 0x20 && c < 0x7f) {
            byte i = 4;
            do {
                unsigned char d = pgm_read_byte(&font5x8[c - 0x21][i]);
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
            } while(i--);
        } else {
            clearPixels(5 * 8);
        }
        m_x += 6;
    } else {
        setXY(m_x, m_y, m_x + 7, m_y + 15);
        if (c > 0x20 && c < 0x7f) {
            byte pgm_buffer[16];
            memcpy_P(pgm_buffer, &font8x16_terminal[c - 0x21], 16);
            for (byte i = 0; i < 16; i += 2) {
                unsigned char d = pgm_buffer[14 - i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
                d = pgm_buffer[15 - i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
            }
        } else {
            clearPixels(16 * 8);
        }
        m_x += 9;
    }
    Disable();
}

void LCD_SSD1289::writeDigit(byte n)
{
    Enable();
    if (m_font == FONT_SIZE_LARGE) {
        setXY(m_x, m_y, m_x + 15, m_y + 15);
        if (n <= 9) {
            byte pgm_buffer[32];
            memcpy_P(pgm_buffer, &digits16x16[n], sizeof(pgm_buffer));
            for (byte i = 0; i < 16; i++) {
                unsigned char d = pgm_buffer[15 - i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
                d = pgm_buffer[31 - i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
            }
        } else {
            clearPixels(16 * 16);
        }
        m_x += 16;
    } else if (m_font == FONT_SIZE_XLARGE) {
        setXY(m_x, m_y, m_x + 15, m_y + 23);
        if (n <= 9) {
            byte pgm_buffer[48];
            memcpy_P(pgm_buffer, &digits16x24[n], sizeof(pgm_buffer));
            for (int i = 0; i < 48; i += 3) {
                unsigned char d = pgm_buffer[45 - i];
                for (int j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
                d = pgm_buffer[46 - i];
                for (int j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
                d = pgm_buffer[47 - i];
                for (int j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
            }
        } else {
            clearPixels(16 * 24);
        }
        m_x += 18;
    } else {
        write(n <= 9 ? ('0' + n) : ' ');
    }
    Disable();
}

void LCD_SSD1289::draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height)
{
    byte rows = height >> 3;
    Enable();
    setXY(m_x, m_y, m_x + width - 1, m_y + height - 1);
    for (int16_t i = width - 1; i >= 0; i--) {
        for (uint8_t h = 0; h < rows; h++) {
#ifndef __arm__
           byte d = pgm_read_byte(buffer + i + width * h);
#else
           byte d = buffer[i + width * h];
#endif
            for (byte j = 0; j < 8; j++, d >>= 1) {
                if (d & 1) {
                    setPixel(fch, fcl);
                } else {
                    setPixel(bch, bcl);
                }
            }
        }
    }
    Disable();
    m_x += width;
}

void LCD_SSD1289::draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height, byte scaleX, byte scaleY)
{
    byte rows = height >> 3;
    if (scaleY == 0) scaleY = scaleX;
    Enable();
    setXY(m_x, m_y, m_x + width * scaleX - 1, m_y + height * scaleY - 1);
    for (int16_t i = width - 1; i >= 0; i--) {
        for (byte n = 0; n < scaleX; n++) {
            for (uint8_t h = 0; h < rows; h++) {
#ifndef __arm__
                byte d = pgm_read_byte(buffer + i + width * h);
#else
                byte d = buffer[i + width * h];
#endif
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1) {
                        for (byte m = 0; m < scaleY; m++) {
                            setPixel(fch, fcl);
                        }
                    } else {
                        for (byte m = 0; m < scaleY; m++) {
                            setPixel(bch, bcl);
                        }
                    }
                }
            }
        }
    }
    Disable();
    m_x += width * scaleX;
}

void LCD_SSD1289::draw4bpp(const PROGMEM byte* buffer, uint16_t width, uint16_t height)
{
    char buf[240];
    Enable();
    setXY(m_x, m_y, m_x + width * 2 - 1, m_y + height * 2 - 1);
    uint16_t i = width - 1;
    do {
        memcpy_P(buf, buffer + i * height * 2, height * 2);
        for (byte j = 0; j < height * 2; j += 2) {
            setPixel(buf[j + 1], buf[j]);
            setPixel(buf[j + 1], buf[j]);
        }
        for (byte j = 0; j < height * 2; j += 2) {
            setPixel(buf[j + 1], buf[j]);
            setPixel(buf[j + 1], buf[j]);
        }
    } while (i--);
    Disable();
    m_x += width * 2;
}

void LCD_SSD1289::shiftOutTouchData(unsigned char data)
{
	unsigned char nop;
	unsigned char count;

	digitalWrite(PIN_CLK,LOW);

	for(count=0; count<8; count++)
	{
		digitalWrite(PIN_DIN, data & 0x80 ? HIGH : LOW);
		digitalWrite(PIN_CLK, LOW);                
		nop++;
		digitalWrite(PIN_CLK, HIGH);
		data <<= 1; 
		//nop++;
	}
}

unsigned int LCD_SSD1289::shiftInTouchData()
{
	unsigned char nop;
	unsigned int data = 0;
	unsigned char count;
	for(count=0; count<12; count++)
	{
		data <<= 1;
		digitalWrite(PIN_CLK, HIGH);               
		nop++;
		digitalWrite(PIN_CLK, LOW);
		//nop++;
		data |= digitalRead(PIN_DOUT);
	}
	return data;
}

byte LCD_SSD1289::getTouchData(int& x, int& y)
{
	long tx = 0;
	long ty = 0;

	if (digitalRead(PIN_IRQ) == HIGH)
		return 0;
	
	digitalWrite(PIN_CS,LOW);                    
	for (int i=0; i < PREC_TOUCH_CONST; i++)
	{
		shiftOutTouchData(0x90);        
		digitalWrite(PIN_CLK,HIGH);
		digitalWrite(PIN_CLK,LOW); 
		int d = shiftInTouchData();
		if (d < PixOffsY) return 0;
		ty += d;

		shiftOutTouchData(0xD0);      
		digitalWrite(PIN_CLK,HIGH);
		digitalWrite(PIN_CLK,LOW);
		tx+=shiftInTouchData();
	}
	digitalWrite(PIN_CS,HIGH);
	y = (tx - PixOffsX * PREC_TOUCH_CONST) / PixSizeX;
	x = (ty - PixOffsY * PREC_TOUCH_CONST) / PixSizeY;
	return 1;
}

