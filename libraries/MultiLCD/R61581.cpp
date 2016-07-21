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

#define PIN_CLK 6
#define PIN_CS 5
#define PIN_DIN 4
#define PIN_DOUT 3
#define PIN_IRQ 2

#define PIN_BACKLIGHT 8

#define PixSizeX	1143
#define PixOffsX	1780

#define PixSizeY	793
#define PixOffsY	1380

/**********************************************
Standard C functions zone
**********************************************/
void LCD_R61581::begin()
{
	digitalWrite(PIN_BACKLIGHT, LOW);
	delay(50);
	pinMode(__p1,OUTPUT);
	pinMode(__p2,OUTPUT);
	pinMode(__p3,OUTPUT);
	if (__p4 != NOTINUSE)
		pinMode(__p4,OUTPUT);
	if ((display_transfer_mode==LATCHED_16) or ((display_transfer_mode==1) and (display_serial_mode==SERIAL_5PIN)))
		pinMode(__p5,OUTPUT);
	if (display_transfer_mode!=1)
		_set_direction_registers(display_transfer_mode);

	_hw_special_init();

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

	LCD_Write_COM(0xB0);		
	LCD_Write_DATA(0x1E);	    

	LCD_Write_COM(0xB0);
	LCD_Write_DATA(0x00);

	LCD_Write_COM(0xB3);
	LCD_Write_DATA(0x02);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x10);

	LCD_Write_COM(0xB4);
	LCD_Write_DATA(0x00);//0X10

// 		LCD_Write_COM(0xB9); //PWM Settings for Brightness Control
// 		LCD_Write_DATA(0x01);// Disabled by default. 
// 		LCD_Write_DATA(0xFF); //0xFF = Max brightness
// 		LCD_Write_DATA(0xFF);
// 		LCD_Write_DATA(0x18);

	LCD_Write_COM(0xC0);
	LCD_Write_DATA(0x03);
	LCD_Write_DATA(0x3B);//
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x01);
	LCD_Write_DATA(0x00);//NW
	LCD_Write_DATA(0x43);

	LCD_Write_COM(0xC1);
	LCD_Write_DATA(0x08);
	LCD_Write_DATA(0x15);//CLOCK
	LCD_Write_DATA(0x08);
	LCD_Write_DATA(0x08);

	LCD_Write_COM(0xC4);
	LCD_Write_DATA(0x15);
	LCD_Write_DATA(0x03);
	LCD_Write_DATA(0x03);
	LCD_Write_DATA(0x01);

	LCD_Write_COM(0xC6);
	LCD_Write_DATA(0x02);

	LCD_Write_COM(0xC8);
	LCD_Write_DATA(0x0c);
	LCD_Write_DATA(0x05);
	LCD_Write_DATA(0x0A);//0X12
	LCD_Write_DATA(0x6B);//0x7D
	LCD_Write_DATA(0x04);
	LCD_Write_DATA(0x06);//0x08
	LCD_Write_DATA(0x15);//0x0A
	LCD_Write_DATA(0x10);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x60);//0x23

	LCD_Write_COM(0x36);
	LCD_Write_DATA(0x0A);

	LCD_Write_COM(0x0C);
	LCD_Write_DATA(0x55);

	LCD_Write_COM(0x3A);
	LCD_Write_DATA(0x55);

	LCD_Write_COM(0x38);

	LCD_Write_COM(0xD0);
	LCD_Write_DATA(0x07);
	LCD_Write_DATA(0x07);//VCI1
	LCD_Write_DATA(0x14);//VRH 0x1D
	LCD_Write_DATA(0xA2);//BT 0x06

	LCD_Write_COM(0xD1);
	LCD_Write_DATA(0x03);
	LCD_Write_DATA(0x5A);//VCM  0x5A
	LCD_Write_DATA(0x10);//VDV

	LCD_Write_COM(0xD2);
	LCD_Write_DATA(0x03);
	LCD_Write_DATA(0x04);//0x24
	LCD_Write_DATA(0x04);

	LCD_Write_COM(0x11);
	delay(150);

	LCD_Write_COM(0x2A);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x01);
	LCD_Write_DATA(0xDF);//320

	LCD_Write_COM(0x2B);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x00);
	LCD_Write_DATA(0x01);
	LCD_Write_DATA(0x3F);//480


	delay(100);

	LCD_Write_COM(0x29);
	delay(30);

	LCD_Write_COM(0x2C);
	delay(30);

	sbi (P_CS, B_CS);

	clear();

	pinMode(PIN_BACKLIGHT, OUTPUT);
	digitalWrite(PIN_BACKLIGHT, HIGH);

	// init pins for touch controller
	pinMode(PIN_CLK,  OUTPUT);
	pinMode(PIN_CS,   OUTPUT);
	pinMode(PIN_DIN,  OUTPUT);
	pinMode(PIN_DOUT, INPUT);
	pinMode(PIN_IRQ,  INPUT_PULLUP);

	digitalWrite(PIN_CS,  HIGH);
	digitalWrite(PIN_CLK, HIGH);
	digitalWrite(PIN_DIN, HIGH);
	digitalWrite(PIN_CLK, HIGH);
}

void LCD_R61581::setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	swap(word, x1, y1);
	swap(word, x2, y2)
	//y1=disp_y_size-y1;
	//y2=disp_y_size-y2;
	//swap(word, y1, y2)
	// begin hardware specific code
	LCD_Write_COM(0x2a); 
	LCD_Write_DATA(x1>>8);
	LCD_Write_DATA(x1);
	LCD_Write_DATA(x2>>8);
	LCD_Write_DATA(x2);
	LCD_Write_COM(0x2b); 
	LCD_Write_DATA(y1>>8);
	LCD_Write_DATA(y1);
	LCD_Write_DATA(y2>>8);
	LCD_Write_DATA(y2);
	LCD_Write_COM(0x2c); 
}

void LCD_R61581::setBackLight(byte brightness)
{
	analogWrite(PIN_BACKLIGHT, brightness);
}

void LCD_R61581::Enable()
{
    cbi(P_CS, B_CS);
}
void LCD_R61581::Disable()
{
    sbi(P_CS, B_CS);
}

void LCD_R61581::drawPixel(uint16_t poX, uint16_t poY,uint16_t color)
{
    Enable();
    setXY(poX, poY, poX, poY);
    setPixel(color);
    Disable();
}

void LCD_R61581::clearPixels(uint32_t pixels)
{
    Enable();
    do {
        setPixel(bch, bcl);
    } while(--pixels);
    Disable();
}

void LCD_R61581::clear(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    setColor(0);
    fillRect(x, y, x + width, y + height);
    setColor(0xffff);
    m_x = 0;
    m_y = 0;
}

size_t LCD_R61581::write(uint8_t c)
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
            for (byte i = 0; i < 5; i++) {
                unsigned char d = pgm_read_byte(&font5x8[c - 0x21][i]);
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
            }
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
                unsigned char d = pgm_buffer[i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
                d = pgm_buffer[i + 1];
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
    return 1;
}

void LCD_R61581::writeDigit(byte n)
{
    Enable();
    if (m_font == FONT_SIZE_LARGE) {
        setXY(m_x, m_y, m_x + 15, m_y + 15);
        if (n <= 9) {
            byte pgm_buffer[32];
            memcpy_P(pgm_buffer, &digits16x16[n], sizeof(pgm_buffer));
            for (byte i = 0; i < 16; i++) {
                unsigned char d = pgm_buffer[i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
                d = pgm_buffer[i + 16];
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
                unsigned char d = pgm_buffer[i];
                for (int j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
                d = pgm_buffer[i + 1];
                for (int j = 0; j < 8; j++, d >>= 1) {
                    if (d & 1)
                        setPixel(fch, fcl);
                    else
                        setPixel(bch, bcl);
                }
                d = pgm_buffer[i + 2];
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

void LCD_R61581::draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height)
{
    byte rows = height >> 3;
    Enable();
    setXY(m_x, m_y, m_x + width - 1, m_y + height - 1);
    for (int16_t i = 0; i < width; i++) {
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

void LCD_R61581::draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height, byte scaleX, byte scaleY)
{
    byte rows = height >> 3;
    if (scaleY == 0) scaleY = scaleX;
    Enable();
    setXY(m_x, m_y, m_x + width * scaleX - 1, m_y + height * scaleY - 1);
    for (int16_t i = 0; i < width; i++) {
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

void LCD_R61581::draw2x(const PROGMEM byte* buffer, uint16_t width, uint16_t height)
{
    char buf[240];
    Enable();
    setXY(m_x, m_y, m_x + width * 2 - 1, m_y + height * 2 - 1);
    for (uint16_t i = 0; i < width; i++) {
        memcpy_P(buf, buffer + i * height * 2, height * 2);
        for (byte j = 0; j < height * 2; j += 2) {
            setPixel(buf[j + 1], buf[j]);
            setPixel(buf[j + 1], buf[j]);
        }
        for (byte j = 0; j < height * 2; j += 2) {
            setPixel(buf[j + 1], buf[j]);
            setPixel(buf[j + 1], buf[j]);
        }
    }
    Disable();
    m_x += width * 2;
}

void LCD_R61581::shiftOutTouchData(unsigned char data)
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

unsigned int LCD_R61581::shiftInTouchData()
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

byte LCD_R61581::getTouchData(int& x, int& y)
{
	unsigned long tx = 0;
	unsigned long ty = 0;

	if (digitalRead(PIN_IRQ) == HIGH)
		return 0;
	
	digitalWrite(PIN_CS,LOW);                    
	for (int i=0; i < 10; i++)
	{
		shiftOutTouchData(0x90);        
		digitalWrite(PIN_CLK,HIGH);
		digitalWrite(PIN_CLK,LOW); 
		int d = shiftInTouchData();
		if (d >= 4000) {
			digitalWrite(PIN_CS,HIGH);
                     return 0;
		}
		ty += d;

		shiftOutTouchData(0xD0);      
		digitalWrite(PIN_CLK,HIGH);
		digitalWrite(PIN_CLK,LOW);
		tx+=shiftInTouchData();
	}
	digitalWrite(PIN_CS,HIGH);
	x = 480 - (ty - PixOffsY) * 10 / PixSizeY;
	y = (tx - PixOffsX) * 10 / PixSizeX;
	return 1;
}

