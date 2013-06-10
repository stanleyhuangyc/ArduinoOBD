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
    }

	digitalWrite(RST,HIGH);
    delay(1);
	digitalWrite(RST,LOW);
	delay(1);

	digitalWrite(RST,HIGH);
	digitalWrite(CS,HIGH);
	digitalWrite(WR,HIGH);
	delay(50);

    PORTE = 0;
    PORTG = 0;
    PORTH = 0;
    lastData = 0;

	WriteCommandData(0x0001,0x0100);
	WriteCommandData(0x0002,0x0700);
	WriteCommandData(0x0003,0x1030);
	WriteCommandData(0x0004,0x0000);
	WriteCommandData(0x0008,0x0207);
	WriteCommandData(0x0009,0x0000);
	WriteCommandData(0x000A,0x0000);
	WriteCommandData(0x000C,0x0000);
	WriteCommandData(0x000D,0x0000);
	WriteCommandData(0x000F,0x0000);
	//power on sequence VGHVGL
	WriteCommandData(0x0010,0x0000);
	WriteCommandData(0x0011,0x0007);
	WriteCommandData(0x0012,0x0000);
	WriteCommandData(0x0013,0x0000);
	//vgh
	WriteCommandData(0x0010,0x1290);
	WriteCommandData(0x0011,0x0227);
	//delays(100);
	//vregiout
	WriteCommandData(0x0012,0x001d); //0x001b
	//delays(100);
	//vom amplitude
	WriteCommandData(0x0013,0x1500);
	//delays(100);
	//vom H
	WriteCommandData(0x0029,0x0018);
	WriteCommandData(0x002B,0x000D);

	//gamma
	WriteCommandData(0x0030,0x0004);
	WriteCommandData(0x0031,0x0307);
	WriteCommandData(0x0032,0x0002);// 0006
	WriteCommandData(0x0035,0x0206);
	WriteCommandData(0x0036,0x0408);
	WriteCommandData(0x0037,0x0507);
	WriteCommandData(0x0038,0x0204);//0200
	WriteCommandData(0x0039,0x0707);
	WriteCommandData(0x003C,0x0405);// 0504
	WriteCommandData(0x003D,0x0F02);
	//ram
	WriteCommandData(0x0050,0x0000);
	WriteCommandData(0x0051,0x00EF);
	WriteCommandData(0x0052,0x0000);
	WriteCommandData(0x0053,0x013F);
	WriteCommandData(0x0060,0xA700);
	WriteCommandData(0x0061,0x0001);
	WriteCommandData(0x006A,0x0000);
	//
	WriteCommandData(0x0080,0x0000);
	WriteCommandData(0x0081,0x0000);
	WriteCommandData(0x0082,0x0000);
	WriteCommandData(0x0083,0x0000);
	WriteCommandData(0x0084,0x0000);
	WriteCommandData(0x0085,0x0000);
	//
	WriteCommandData(0x0090,0x0010);
	WriteCommandData(0x0092,0x0600);
	WriteCommandData(0x0093,0x0003);
	WriteCommandData(0x0095,0x0110);
	WriteCommandData(0x0097,0x0000);
	WriteCommandData(0x0098,0x0000);
	WriteCommandData(0x0007,0x0133);
    Disable();

	m_color[0] = 0;
	m_color[1] = 0xffff;
    clear();
}

void LCD_ILI9325D::SetXY(uint16_t x0,uint16_t x1,uint16_t y0,uint16_t y1)
{
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

/*
void Pant(uint16_t color)
{
	int i,j;
	SetXY(0,239,0,319);

    for(i=0;i<320;i++)
	 {
	  for (j=0;j<240;j++)
	   	{
         WriteData(color);
	    }

	  }
}
*/

void LCD_ILI9325D::clear(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
	unsigned long count = (unsigned long)width * height;
	SetXY(y, y + height - 1, x, x + width - 1);

    digitalWrite(RS,HIGH);//LCD_RS=0;
    digitalWrite(CS,LOW);//LCD_CS =0;
    PORTE = 0;
    PORTG = 0;
    PORTH = 0;
    lastData = 0;
    while (count--) {
        digitalWrite(WR,LOW);//LCD_WR=0;
        digitalWrite(WR,HIGH);//LCD_WR=1;
        digitalWrite(WR,LOW);//LCD_WR=0;
        digitalWrite(WR,HIGH);//LCD_WR=1;
    }
    digitalWrite(CS,HIGH);//LCD_CS =0;
	m_x = x;
	m_y = y;
}

size_t LCD_ILI9325D::write(uint8_t c)
{
    if (c == '\n') {
        m_x += 8;
        return 0;
    } else if (c == '\r') {
        SetXY(m_x, m_x + 7, m_y, 319);
        uint16_t count = (320 - m_y) * 8;
        for (uint16_t i=0; i < count; i++) {
             WriteData(0, 0);
        }
        m_y = 0;
        return 0;
    } else if (c < ' ') {
        return 0;
    }

    if (m_font == FONT_SIZE_SMALL) {
        byte pgm_buffer[6] = {0};
        if (c > 0x20 && c < 0x7f) {
            memcpy_P(pgm_buffer, &font5x8[c - 0x21], 5);
        }

        SetXY(m_x, m_x + 7, m_y, m_y + 5);
        for (byte i = 0; i < 5; i++) {
            unsigned char d = pgm_buffer[i];
            for (byte j = 0; j < 8; j++) {
                WriteData(m_color[d & 1]);
                d >>= 1;
            }
        }

        m_y += 6;
        if (m_y >= 320) {
            m_x += 8;
            m_y = 0;
            if (m_x >= 240) {
                m_x = 0;
            }
        }
    } else {
        byte pgm_buffer[16];
        if (c > 0x20 && c < 0x7f) {
            memcpy_P(pgm_buffer, &font8x16_terminal[c - 0x21], 16);
        } else {
            memset(pgm_buffer, 0, sizeof(pgm_buffer));
        }

        SetXY(m_x, m_x + 15, m_y, m_y + 7);
        for (byte i = 0; i < 16; i++) {
            unsigned char d = pgm_buffer[i];
            for (byte j = 0; j < 8; j++) {
                WriteData(m_color[d & 1]);
                d >>= 1;
            }
        }

        m_y += 9;
        if (m_y >= 320) {
            m_x += 8;
            m_y = 0;
            if (m_x >= 240) {
                m_x = 0;
            }
        }
    }
}

void LCD_ILI9325D::writeDigit(byte n)
{
    if (m_font == FONT_SIZE_SMALL) {
        /*
        byte pgm_buffer[16] = {0};
        if (n >= 0 && n <= 9) {
            memcpy_P(pgm_buffer, &font5x8[n + '0' - 0x21], 5);
        }

        SetXY(m_x, m_x + 7, m_y, m_y + 5);
        for (byte i = 0; i < 5; i++) {
            unsigned char d = pgm_buffer[i];
            for (byte j = 0; j < 8; j++, d >>= 1) {
                WriteData(m_color[d & 1]);
            }
        }
        m_y += 6;
        */
        byte pgm_buffer[8];
        if (n >= 0 && n <= 9) {
            memcpy_P(pgm_buffer, &digits8x8[n], 8);
        } else {
            memset(pgm_buffer, 0, 8);
        }

        SetXY(m_x, m_x + 7, m_y, m_y + 7);
        for (byte i = 0; i < 8; i++) {
            unsigned char d = pgm_buffer[i];
            for (byte j = 0; j < 8; j++, d >>= 1) {
                WriteData(m_color[d & 1]);
            }
        }
        m_y += 8;
    } else if (m_font == FONT_SIZE_MEDIUM) {
        byte pgm_buffer[16];
        if (n >= 0 && n <= 9) {
            memcpy_P(pgm_buffer, &font8x16_terminal[n + '0' - 0x21], 16);
        } else {
            return;
        }
        SetXY(m_x, m_x + 15, m_y, m_y + 7);
        for (byte i = 0; i < 16; i++) {
            unsigned char d = pgm_buffer[i];
            for (byte j = 0; j < 8; j++, d >>= 1) {
                WriteData(m_color[d & 1]);
            }
        }
        m_y += 9;
    } else if (m_font == FONT_SIZE_LARGE) {
        byte pgm_buffer[32];
        if (n >= 0 && n <= 9) {
            memcpy_P(pgm_buffer, &digits16x16[n], sizeof(pgm_buffer));
        } else {
            memset(pgm_buffer, 0, sizeof(pgm_buffer));
        }

        SetXY(m_x, m_x + 15, m_y, m_y + 15);
        for (byte i = 0; i < 16; i++) {
            unsigned char d = pgm_buffer[i];
            for (byte j = 0; j < 8; j++, d >>= 1) {
                WriteData(m_color[d & 1]);
            }
            d = pgm_buffer[i + 16];
            for (byte j = 0; j < 8; j++, d >>= 1) {
                WriteData(m_color[d & 1]);
            }
        }
        m_y += 16;
    } else if (m_font == FONT_SIZE_XLARGE) {
        byte pgm_buffer[48];
        if (n >= 0 && n <= 9) {
            memcpy_P(pgm_buffer, &digits16x24[n], sizeof(pgm_buffer));
        } else {
            memset(pgm_buffer, 0, sizeof(pgm_buffer));
        }
        SetXY(m_x, m_x + 23, m_y, m_y + 15);
        for (int i = 0; i < 48; i++) {
            unsigned char d = pgm_buffer[i];
            for (int j = 0; j < 8; j++, d >>= 1) {
                WriteData(m_color[d & 1]);
            }
        }
        m_y += 18;
    }
}

void LCD_ILI9325D::draw(const PROGMEM byte* buffer, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    uint16_t pixels = (uint16_t)width * height;
    SetXY(y, y + height - 1, x, x + width - 1);
    do {
        WriteData(pgm_read_byte_near(buffer++), pgm_read_byte_near(buffer++));
    } while (--pixels);
}

void LCD_ILI9325D::draw2x(const PROGMEM byte* buffer, uint16_t x, uint16_t y, byte width, byte height)
{
    char buf[240];
    uint16_t pixels = (uint16_t)width * height;
    SetXY(y, y + height * 2 - 1, x, x + width * 2- 1);
    for (byte i = 0; i < width; i++) {
        memcpy_P(buf, buffer + (uint16_t)i * height * 2, height * 2);
        for (byte j = 0; j < height * 2; j += 2) {
            WriteData(buf[j], buf[j + 1]);
            WriteData(buf[j], buf[j + 1]);
        }
        for (byte j = 0; j < height * 2; j += 2) {
            WriteData(buf[j], buf[j + 1]);
            WriteData(buf[j], buf[j + 1]);
        }
    }
}

void LCD_ILI9325D::draw4bpp(const PROGMEM byte* buffer, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    uint16_t count = (uint16_t)width * height / 2;
    SetXY(y, y + height - 1, x, x + width - 1);
    do {
        byte d = pgm_read_byte_near(buffer++);
        byte dl = d & 0xf;
        byte rg = (dl << 3) | (dl > 1) | 0x8;
        byte gb = (dl << 7) | (dl << 1) | 0x61;
        WriteData(rg, gb);
        dl = d >> 4;
        rg = (dl << 3) | (dl > 1) | 0x8;
        gb = (dl << 7) | (dl << 1) | 0x61;
        WriteData(rg, gb);
    } while (--count);
}

/*
void Print16x24(int x, int y, uint16_t color, const char* data)
{
    SetXY(x, x + 23, y, y + 15);
    for (int i = 0; i < 48; i++) {
        unsigned char d = data[i];
        for (int j = 0; j < 8; j++) {
            WriteData((d & 1 ) ? color : 0);
            d >>= 1;
        }
    }
}

void Print8x16(int x, int y, uint16_t color, const char* data)
{
    SetXY(x, x + 7, y, y + 15);
    for (int i = 0; i < 16; i++) {
        unsigned char d = *data;
        for (int j = 0; j < 8; j++) {
            WriteData((d & 1 ) ? color : 0);
            d >>= 1;
        }
        data++;
    }

}
*/

#endif
