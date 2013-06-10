/*************************************************************************
* Arduino Text Display Library for Multiple LCDs
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@live.com>
* All rights reserved.
*************************************************************************/

typedef enum {
    FONT_SIZE_SMALL = 0,
    FONT_SIZE_MEDIUM,
    FONT_SIZE_LARGE,
    FONT_SIZE_XLARGE
} FONT_SIZE;

extern const PROGMEM unsigned char font5x8[][5];
extern const PROGMEM unsigned char digits8x8[][8] ;
extern const PROGMEM unsigned char digits16x16[][32];
extern const PROGMEM unsigned char digits16x24[][48];
extern const PROGMEM unsigned char font8x16_doslike[][16];
extern const PROGMEM unsigned char font8x16_terminal[][16];
#include "PCD8544.h"

class LCD_Common
{
public:
    LCD_Common():m_font(0) {}
    void setFont(FONT_SIZE size) { m_font = size; }
    virtual void backlight(bool on) {}
    virtual byte getLines() = 0;
    virtual byte getCols() = 0;
    virtual void changeLine() {}
    virtual void clearLine(byte line) {}
    void draw(const PROGMEM byte* buffer, byte x, byte y, byte width, byte height) {}
    void printInt(uint16_t value, char padding = -1);
    void printLong(unsigned long value, char padding = -1);
protected:
    virtual void writeDigit(byte n) {}
    byte m_font;
};

class LCD_PCD8544 : public LCD_Common, public PCD8544
{
public:
    byte getLines() { return 6; }
    byte getCols() { return 14; }
    void backlight(bool on)
    {
        pinMode(7, OUTPUT);
        digitalWrite(7, on ? HIGH : LOW);
    }
    void clearLine(byte line)
    {
        setCursor(0, line);
        for (byte i = 14; i > 0; i--) write(' ');
    }
    void changeLine()
    {
        column = 0;
        line ++;
    }
    void draw(const PROGMEM byte* buffer, byte x, byte y, byte width, byte height);
private:
    void writeDigit(byte n);
};

#include "ZtLib.h"

#define OLED_ADDRESS 0x27

class LCD_ZTOLED : public LCD_Common, public ZtLib, public Print
{
public:
    byte getLines() { return 4; }
    byte getCols() { return 16; }
    void setCursor(byte column, byte line);
    void changeLine()
    {
        m_column = 0;
        m_page += 2;
    }
    size_t write(uint8_t c);
    //void print(const char* s);
    void writeDigit(byte n);
    void clear();
    void begin();
    void backlight(bool on) {}
    void clearLine(byte line)
    {
        setCursor(0, line);
        for (byte i = 16; i > 0; i--) write(' ');
    }
private:
    unsigned char m_column;
    unsigned char m_page;
};

#include "LCD4Bit_mod.h"
class LCD_1602 : public LCD_Common, public LCD4Bit_mod
{
public:
    byte getLines() { return 2; }
    byte getCols() { return 16; }
    void writeDigit(byte n)
    {
        write(n >= 0 && n <= 9 ? '0' + n : ' ');
    }
    void clearLine(byte line)
    {
        setCursor(0, line);
        for (byte i = 16; i > 0; i--) write(' ');
    }
};

#include "SSD1306.h"

class LCD_SSD1306 : public LCD_Common, public SSD1306, public Print
{
public:
    void setCursor(byte column, byte line);
    void draw(const PROGMEM byte* buffer, byte x, byte y, byte width, byte height);
    size_t write(uint8_t c);
    void clear(byte x = 0, byte y = 0, byte width = 128, byte height = 64);
    void clearLine(byte line);
    byte getLines() { return 21; }
    byte getCols() { return 8; }
private:
    void writeDigit(byte n);
    byte m_col;
    byte m_row;
};

class LCD_ILI9325D : public LCD_Common, public Print
{
public:
    LCD_ILI9325D():m_lineHeight(10) {}
    void setCursor(uint16_t column, uint16_t line)
    {
        m_y = column;
        m_x = line * m_lineHeight;
    }
    void setColor(uint16_t textColor, uint16_t bgColor = 0)
    {
        m_color[0] = bgColor;
        m_color[1] = textColor;
    }
    void begin();
    void clear(uint16_t x = 0, uint16_t y = 0, uint16_t width = 320, uint16_t height = 240);
    void draw(const PROGMEM byte* buffer, uint16_t x, uint16_t y, uint16_t width, uint16_t height);
    void draw2x(const PROGMEM byte* buffer, uint16_t x, uint16_t y, byte width, byte height);
    void draw4bpp(const PROGMEM byte* buffer, uint16_t x, uint16_t y, uint16_t width, uint16_t height);
    size_t write(uint8_t);
    void clearLine(byte line)
    {
        clear(0, line * m_lineHeight, 320, 8);
    }
    void setLineHeight(byte lineHeight) { m_lineHeight = lineHeight; }
    byte getLines() { return 53; }
    byte getCols() { return 30; }
private:
    void writeDigit(byte n);
    void SetXY(uint16_t x0,uint16_t x1,uint16_t y0,uint16_t y1);
    void WriteData(uint16_t c);
    void WriteData(byte l, byte h);
    void WriteCommandData(uint16_t cmd,uint16_t dat);
    void Enable();
    void Disable();
    void SetCommandMode();
    void SetDataMode();
    int m_x;
    int m_y;
    uint16_t m_color[2];
    byte m_lineHeight;
    byte lastData;
};
