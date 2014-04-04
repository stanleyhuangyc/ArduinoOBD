/*************************************************************************
* Arduino Text Display Library for Multiple LCDs
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@live.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include "MultiLCD.h"

/**********************************************
Define zone
**********************************************/

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

#define RS 59
#define WR 58
#define CS 57
#define RST 56

#define T_CLK 55
#define T_CS 60
#define T_DIN 54
#define T_DOUT 8
#define T_IRQ 9

#define X_CONST 240
#define Y_CONST 320

#define PREC_TOUCH_CONST 10

#define PixSizeX	13.78
#define PixOffsX	411

#define PixSizeY	11.01
#define PixOffsY	378

#define WINDOW_XADDR_START	0x0050 // Horizontal Start Address Set
#define WINDOW_XADDR_END	0x0051 // Horizontal End Address Set
#define WINDOW_YADDR_START	0x0052 // Vertical Start Address Set
#define WINDOW_YADDR_END	0x0053 // Vertical End Address Set
#define GRAM_XADDR		    0x0020 // GRAM Horizontal Address Set
#define GRAM_YADDR		    0x0021 // GRAM Vertical Address Set
#define GRAMWR 			    0x0022 // memory write

/* LCD color */
#define White          0xFFFF
#define Black          0x0000
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0

/**********************************************
Standard C functions zone
**********************************************/
void LCD_ILI9325D::Enable() { digitalWrite(CS,LOW); }
void LCD_ILI9325D::Disable() { digitalWrite(CS,HIGH); }

void LCD_ILI9325D::SetCommandMode()
{
    digitalWrite(CS,HIGH);
    digitalWrite(RS, LOW);
    digitalWrite(CS,LOW);
}

void LCD_ILI9325D::SetDataMode()
{
    digitalWrite(CS,HIGH);
    digitalWrite(RS, HIGH);
    digitalWrite(CS,LOW);
}

void LCD_ILI9325D::WriteData(byte l, byte h)
{
    if (h != lastData) {
        PORTE = (h & 0x3) | ((h & 0xC) << 2) | ((h & 0x20) >> 2);
        PORTG = (h & 0x10) << 1;
        PORTH = (h & 0xC0) >> 3;
        lastData = h;
    }

    digitalWrite(WR,LOW);//LCD_WR=0;
    digitalWrite(WR,HIGH);//LCD_WR=1;

    if (l != lastData) {
        PORTE = (l & 0x3) | ((l & 0xC) << 2) | ((l & 0x20) >> 2);
        PORTG = (l & 0x10) << 1;
        PORTH = (l & 0xC0) >> 3;
        lastData = l;
    }

    digitalWrite(WR,LOW);//LCD_WR=0;
    digitalWrite(WR,HIGH);//LCD_WR=1;
}

void LCD_ILI9325D::WriteData(uint16_t c)
{
    byte value = *((unsigned char*)&c + 1);
    if (value != lastData) {
        PORTE = (value & 0x3) | ((value & 0xC) << 2) | ((value & 0x20) >> 2);
        PORTG = (value & 0x10) << 1;
        PORTH = (value & 0xC0) >> 3;
        lastData = value;
    }

    digitalWrite(WR,LOW);//LCD_WR=0;
    digitalWrite(WR,HIGH);//LCD_WR=1;

    value = (unsigned char)c;
    if (value != lastData) {
        PORTE = (value & 0x3) | ((value & 0xC) << 2) | ((value & 0x20) >> 2);
        PORTG = (value & 0x10) << 1;
        PORTH = (value & 0xC0) >> 3;
        lastData = value;
    }

	digitalWrite(WR,LOW);//LCD_WR=0;
	digitalWrite(WR,HIGH);//LCD_WR=1;
}

void LCD_ILI9325D::WriteCommandData(uint16_t cmd,uint16_t dat)
{
    SetCommandMode();
	WriteData(cmd);
	SetDataMode();
	WriteData(dat);
}

void LCD_ILI9325D::begin()
{
    pinMode(RS,OUTPUT);
    pinMode(WR,OUTPUT);
    pinMode(CS,OUTPUT);
    pinMode(RST,OUTPUT);

    //DDRD = 0xFF;
    for(int a=0;a < 8;a++)
    {
        pinMode(a,OUTPUT);
        digitalWrite(a, LOW);
    }

    digitalWrite(RST,HIGH);
    delay(5);
    digitalWrite(RST,LOW);
    delay(5);

    digitalWrite(RST,HIGH);
    digitalWrite(CS,HIGH);
    digitalWrite(WR,HIGH);
    delay(50);

    PORTE = 0;
    PORTG = 0;
    PORTH = 0;
    lastData = 0;

	WriteCommandData(0x00E5, 0x78F0); // set SRAM internal timing
	WriteCommandData(0x0001, 0x0100); // set Driver Output Control
	WriteCommandData(0x0002, 0x0200); // set 1 line inversion
	WriteCommandData(0x0003, 0x1030); // set GRAM write direction and BGR=1.
	WriteCommandData(0x0004, 0x0000); // Resize register
	WriteCommandData(0x0008, 0x0207); // set the back porch and front porch
	WriteCommandData(0x0009, 0x0000); // set non-display area refresh cycle ISC[3:0]
	WriteCommandData(0x000A, 0x0000); // FMARK function
	WriteCommandData(0x000C, 0x0000); // RGB interface setting
	WriteCommandData(0x000D, 0x0000); // Frame marker Position
	WriteCommandData(0x000F, 0x0000); // RGB interface polarity
	//*************Power 00On sequence ****************//
	WriteCommandData(0x0010, 0x0000); // SAP, BT[3:0], AP, DSTB, SLP, STB
	WriteCommandData(0x0011, 0x0007); // DC1[2:0], DC0[2:0], VC[2:0]
	WriteCommandData(0x0012, 0x0000); // VREG1OUT voltage
	WriteCommandData(0x0013, 0x0000); // VDV[4:0] for VCOM amplitude
	WriteCommandData(0x0007, 0x0001);
	delay(200); // Dis-ch00arge capacitor power voltage
	WriteCommandData(0x0010, 0x1690); // SAP, BT[3:0], AP, DSTB, SLP, STB
	WriteCommandData(0x0011, 0x0227); // Set DC1[2:0], DC0[2:0], VC[2:0]
	delay(50); // Delay 50ms
	WriteCommandData(0x0012, 0x000D); // 0012
	delay(50); // Delay 50ms
	WriteCommandData(0x0013, 0x1200); // VDV[4:0] for VCOM amplitude
	WriteCommandData(0x0029, 0x000A); // 04  VCM[5:0] for VCOMH
	WriteCommandData(0x002B, 0x000D); // Set Frame Rate
	delay(50); // Delay 50ms
	WriteCommandData(0x0020, 0x0000); // GRAM horizontal Address
	WriteCommandData(0x0021, 0x0000); // GRAM Vertical Address
	// ----------- Adjust00 the Gamma Curve ----------//
	WriteCommandData(0x0030, 0x0000);
	WriteCommandData(0x0031, 0x0404);
	WriteCommandData(0x0032, 0x0003);
	WriteCommandData(0x0035, 0x0405);
	WriteCommandData(0x0036, 0x0808);
	WriteCommandData(0x0037, 0x0407);
	WriteCommandData(0x0038, 0x0303);
	WriteCommandData(0x0039, 0x0707);
	WriteCommandData(0x003C, 0x0504);
	WriteCommandData(0x003D, 0x0808);
	//------------------ 00Set GRAM area ---------------//
	WriteCommandData(0x0050, 0x0000); // Horizontal GRAM Start Address
	WriteCommandData(0x0051, 0x00EF); // Horizontal GRAM End Address
	WriteCommandData(0x0052, 0x0000); // Vertical GRAM Start Address
	WriteCommandData(0x0053, 0x013F); // Vertical GRAM Start Address
	WriteCommandData(0x0060, 0xA700); // Gate Scan Line
	WriteCommandData(0x0061, 0x0001); // NDL,VLE, REV
	WriteCommandData(0x006A, 0x0000); // set scrolling line
	//-------------- Part00ial Display Control ---------//
	WriteCommandData(0x0080, 0x0000);
	WriteCommandData(0x0081, 0x0000);
	WriteCommandData(0x0082, 0x0000);
	WriteCommandData(0x0083, 0x0000);
	WriteCommandData(0x0084, 0x0000);
	WriteCommandData(0x0085, 0x0000);
	//-------------- Pane00l Control -------------------//
	WriteCommandData(0x0090, 0x0010);
	WriteCommandData(0x0092, 0x0000);
	WriteCommandData(0x0007, 0x0133);

    Disable();

    m_color[0] = 0;
    m_color[1] = 0xffff;
    clear();
}

void LCD_ILI9325D::setXY(uint16_t x0,uint16_t x1,uint16_t y1,uint16_t y0)
{
    y1 = 319 - y1;
    y0 = 319 - y0;

    WriteCommandData(WINDOW_XADDR_START,x0);
    WriteCommandData(WINDOW_XADDR_END,x1);
    WriteCommandData(WINDOW_YADDR_START,y0);
    WriteCommandData(WINDOW_YADDR_END,y1);
    WriteCommandData(GRAM_XADDR,x0);
    WriteCommandData(GRAM_YADDR,y0);
    SetCommandMode();
    WriteData(0x0022);//LCD_WriteCMD(GRAMWR);
    SetDataMode();
}

void LCD_ILI9325D::clearPixels(uint16_t pixels)
{
    digitalWrite(RS,HIGH);//LCD_RS=0;
    digitalWrite(CS,LOW);//LCD_CS =0;
    PORTE = 0;
    PORTG = 0;
    PORTH = 0;
    lastData = 0;
    do {
        digitalWrite(WR,LOW);//LCD_WR=0;
        digitalWrite(WR,HIGH);//LCD_WR=1;
        digitalWrite(WR,LOW);//LCD_WR=0;
        digitalWrite(WR,HIGH);//LCD_WR=1;
    } while (--pixels);
    digitalWrite(CS,HIGH);//LCD_CS =0;
}

void LCD_ILI9325D::clear(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    unsigned long count = (unsigned long)width * height;
    setXY(y, y + height - 1, x, x + width - 1);

    digitalWrite(RS,HIGH);//LCD_RS=0;
    digitalWrite(CS,LOW);//LCD_CS =0;
    PORTE = 0;
    PORTG = 0;
    PORTH = 0;
    lastData = 0;
    do {
        digitalWrite(WR,LOW);//LCD_WR=0;
        digitalWrite(WR,HIGH);//LCD_WR=1;
        digitalWrite(WR,LOW);//LCD_WR=0;
        digitalWrite(WR,HIGH);//LCD_WR=1;
    } while (--count);
    digitalWrite(CS,HIGH);//LCD_CS =0;
    m_x = x;
    m_y = y;
}

size_t LCD_ILI9325D::write(uint8_t c)
{
    if (c == '\n') {
        m_x += (m_font + 1) << 3;
        return 0;
    } else if (c == '\r') {
        setXY(m_x, m_x + 7, m_y, 319);
        clearPixels((320 - m_y) * 8);
        m_y = 0;
        return 0;
    }

    if (m_font == FONT_SIZE_SMALL) {
        setXY(m_x, m_x + 7, m_y, m_y + 4);
        m_y += 6;
        if (m_y >= 320) {
            m_x += (m_font + 1) << 3;
            m_y = 0;
            if (m_x >= 240) {
                m_x = 0;
            }
        }
        if (c > 0x20 && c < 0x7f) {
            byte pgm_buffer[5];
            memcpy_P(pgm_buffer, &font5x8[c - 0x21], 5);
            byte i = 4;
            do {
                unsigned char d = pgm_buffer[i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    WriteData(m_color[d & 1]);
                }
            } while (i--);
        } else {
            clearPixels(5 * 8);
        }
    } else {
        setXY(m_x, m_x + 15, m_y, m_y + 7);
        m_y += 9;
        if (m_y >= 320) {
            m_x += (m_font + 1) << 3;
            m_y = 0;
            if (m_x >= 240) {
                m_x = 0;
            }
        }
        if (c > 0x20 && c < 0x7f) {
            byte pgm_buffer[16];
            memcpy_P(pgm_buffer, &font8x16_terminal[c - 0x21], 16);
            for (byte i = 0; i < 16; i += 2) {
                unsigned char d = pgm_buffer[14 - i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    WriteData(m_color[d & 1]);
                }
                d = pgm_buffer[15 - i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    WriteData(m_color[d & 1]);
                }
            }
        } else {
            clearPixels(8 * 16);
        }
    }
}

void LCD_ILI9325D::writeDigit(byte n)
{
    if (m_font == FONT_SIZE_SMALL) {
        setXY(m_x, m_x + 7, m_y, m_y + 7);
        m_y += 8;
        if (n <= 9) {
            byte pgm_buffer[8];
            memcpy_P(pgm_buffer, &digits8x8[n], 8);
            byte i = 7;
            do {
                unsigned char d = pgm_buffer[i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    WriteData(m_color[d & 1]);
                }
            } while (i--);

        } else {
            clearPixels(8 * 8);
        }
    } else if (m_font == FONT_SIZE_MEDIUM) {
        write(n <= 9 ? ('0' + n) : ' ');
    } else if (m_font == FONT_SIZE_LARGE) {
        setXY(m_x, m_x + 15, m_y, m_y + 15);
        m_y += 16;
        if (n <= 9) {
            byte pgm_buffer[32];
            memcpy_P(pgm_buffer, &digits16x16[n], sizeof(pgm_buffer));
            for (byte i = 0; i < 16; i++) {
                unsigned char d = pgm_buffer[15 - i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    WriteData(m_color[d & 1]);
                }
                d = pgm_buffer[31 - i];
                for (byte j = 0; j < 8; j++, d >>= 1) {
                    WriteData(m_color[d & 1]);
                }
            }
        } else {
            clearPixels(16 * 16);
        }
    } else if (m_font == FONT_SIZE_XLARGE) {
        setXY(m_x, m_x + 23, m_y, m_y + 15);
        m_y += 18;
        if (n <= 9) {
            byte pgm_buffer[48];
            memcpy_P(pgm_buffer, &digits16x24[n], sizeof(pgm_buffer));
            for (int i = 0; i < 48; i += 3) {
                unsigned char d = pgm_buffer[45 - i];
                for (int j = 0; j < 8; j++, d >>= 1) {
                    WriteData(m_color[d & 1]);
                }
                d = pgm_buffer[46 - i];
                for (int j = 0; j < 8; j++, d >>= 1) {
                    WriteData(m_color[d & 1]);
                }
                d = pgm_buffer[47 - i];
                for (int j = 0; j < 8; j++, d >>= 1) {
                    WriteData(m_color[d & 1]);
                }
            }
        } else {
            clearPixels(16 * 24);
        }
    }
}

void LCD_ILI9325D::draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height)
{
    byte rows = height >> 3;
    setXY(m_x, m_x + height - 1, m_y, m_y + width - 1);
    uint16_t i = width - 1;
    do {
        for (uint8_t h = 0; h < rows; h++) {
            byte d = pgm_read_byte_far(buffer + i + width * h);
            for (byte j = 0; j < 8; j++, d >>= 1) {
                WriteData(m_color[d & 1]);
            }
        }
    } while (i--);
    m_y += width;
}

void LCD_ILI9325D::draw2x(const PROGMEM byte* buffer, byte width, byte height)
{
    char buf[240];
    setXY(m_x, m_x + height * 2 - 1, m_y, m_y + width * 2- 1);
    uint16_t i = width - 1;
    do {
        memcpy_P(buf, buffer + (uint16_t)i * height * 2, height * 2);
        for (byte j = 0; j < height * 2; j += 2) {
            WriteData(buf[j], buf[j + 1]);
            WriteData(buf[j], buf[j + 1]);
        }
        for (byte j = 0; j < height * 2; j += 2) {
            WriteData(buf[j], buf[j + 1]);
            WriteData(buf[j], buf[j + 1]);
        }
    } while (i--);
    m_y += width * 2;
}

#endif
