/*************************************************************************
* Arduino Text Display Library for Multiple LCDs
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@live.com>
* All rights reserved.
*************************************************************************/

#if !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega644P__) && !defined(__SAM3X8E__)
//#define MEMORY_SAVING
#endif

typedef enum {
    FONT_SIZE_SMALL = 0,
    FONT_SIZE_MEDIUM,
    FONT_SIZE_LARGE,
    FONT_SIZE_XLARGE
} FONT_SIZE;

#define FLAG_PAD_ZERO 1
#define FLAG_PIXEL_DOUBLE_H 2
#define FLAG_PIXEL_DOUBLE_V 4
#define FLAG_PIXEL_DOUBLE (FLAG_PIXEL_DOUBLE_H | FLAG_PIXEL_DOUBLE_V)

#define RGB16(r,g,b) (((uint16_t)(r >> 3) << 11) | ((uint16_t)(g >> 2) << 5) | (b >> 2))

#define RGB16_RED 0xF800
#define RGB16_GREEN 0x7E0
#define RGB16_BLUE 0x1F
#define RGB16_YELLOW 0xFFE0
#define RGB16_CYAN 0x7FF
#define RGB16_PINK 0xF81F
#define RGB16_WHITE 0xFFFF


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
    LCD_Common():m_font(FONT_SIZE_SMALL),m_flags(0) {}
    void setFont(FONT_SIZE size) { m_font = size; }
    void setFlags(byte flags) { m_flags = flags; }
    virtual void backlight(bool on) {}
    virtual void draw(const PROGMEM byte* buffer, byte width, byte height) {}
    void printInt(uint16_t value, int8_t padding = -1);
    void printLong(uint32_t value, int8_t padding = -1);
protected:
    virtual void writeDigit(byte n) {}
    byte m_font;
    byte m_flags;
};

class LCD_Null : public LCD_Common, public Print
{
public:
    byte getLines() { return 0; }
    byte getCols() { return 0; }
    void clearLine(byte line) {}
    void clear() {}
    void begin() {}
    void setCursor(byte column, byte line) {}
    size_t write(uint8_t c) { return 0; }
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
    void draw(const PROGMEM byte* buffer, byte width, byte height);
private:
    void writeDigit(byte n);
};

#include "SSD1306.h"

class LCD_SSD1306 : public LCD_Common, public SSD1306, public Print
{
public:
    void setCursor(byte column, byte line);
    void draw(const PROGMEM byte* buffer, byte width, byte height);
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

class LCD_SH1106 : public LCD_Common, public Print
{
public:
    void begin();
    void setCursor(byte column, byte line);
    void draw(const PROGMEM byte* buffer, byte width, byte height);
    size_t write(uint8_t c);
    void clear(byte x = 0, byte y = 0, byte width = 128, byte height = 64);
    void clearLine(byte line);
    byte getLines() { return 21; }
    byte getCols() { return 8; }
private:
    void WriteCommand(unsigned char ins);
    void WriteData(unsigned char dat);
    void writeDigit(byte n);
    byte m_col;
    byte m_row;
};

#define TFT_LINE_HEIGHT 8

class LCD_ILI9325D : public LCD_Common, public Print
{
public:
    LCD_ILI9325D() { m_font = FONT_SIZE_MEDIUM; }
    void setCursor(uint16_t column, uint8_t line)
    {
        m_y = column;
        m_x = (uint16_t)line * TFT_LINE_HEIGHT;
    }
    void setXY(uint16_t x, uint16_t y)
    {
        m_y = x;
        m_x = y;
    }
    void setTextColor(uint16_t color)
    {
        m_color[1] = color;
    }
    void setTextColor(uint8_t R, uint8_t G, uint8_t B)
    {
        m_color[1] = ((uint16_t)R << 11) | ((uint16_t)G << 5) | B;
    }
    void SetBGColor(uint16_t color)
    {
        m_color[0] = color;
    }
    void SetBGColor(uint8_t R, uint8_t G, uint8_t B)
    {
        m_color[0] = ((uint16_t)R << 11) | ((uint16_t)G << 5) | B;
    }
    void begin();
    void clear(uint16_t x = 0, uint16_t y = 0, uint16_t width = 320, uint16_t height = 240);
    void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height);
    void draw2x(const PROGMEM byte* buffer, byte width, byte height);
    void draw4bpp(const PROGMEM byte* buffer, uint16_t x, uint16_t y, uint16_t width, uint16_t height);
    size_t write(uint8_t);
    void clearLine(byte line)
    {
        clear(0, line * TFT_LINE_HEIGHT, 320, 8);
    }
    byte getLines() { return 53; }
    byte getCols() { return 30; }
private:
    void setXY(uint16_t x0,uint16_t x1,uint16_t y0,uint16_t y1);
    void writeDigit(byte n);
    void clearPixels(uint16_t pixels);
    void WriteData(uint16_t c);
    void WriteData(byte l, byte h);
    void WriteCommandData(uint16_t cmd,uint16_t dat);
    void Enable();
    void Disable();
    void SetCommandMode();
    void SetDataMode();
    uint16_t m_x;
    uint16_t m_y;
    uint16_t m_color[2];
    byte lastData;
};

class LCD_ILI9341 : public LCD_Common, public Print
{
public:
    LCD_ILI9341() { m_font = FONT_SIZE_MEDIUM; }
    void setCursor(uint16_t column, uint8_t line)
    {
        m_x = column;
        m_y = (uint16_t)line * TFT_LINE_HEIGHT;
    }
    void setXY(uint16_t x, uint16_t y)
    {
        m_x = x;
        m_y = y;
    }
    void setTextColor(uint16_t color)
    {
        m_color[1][0] = color & 0xff;
        m_color[1][1] = color >> 8;
    }
    void setTextColor(uint8_t R, uint8_t G, uint8_t B)
    {
        uint16_t color = ((uint16_t)R << 11) | ((uint16_t)G << 5) | B;
        m_color[1][0] = color & 0xff;
        m_color[1][1] = color >> 8;
    }
    void SetBGColor(uint16_t color)
    {
        m_color[0][0] = color & 0xff;
        m_color[0][1] = color >> 8;
    }
    void SetBGColor(uint8_t R, uint8_t G, uint8_t B)
    {
        uint16_t color = ((uint16_t)R << 11) | ((uint16_t)G << 5) | B;
        m_color[0][0] = color & 0xff;
        m_color[0][1] = color >> 8;
    }
    void clearLine(byte line)
    {
        fill(0, line * TFT_LINE_HEIGHT, 320, 8);
    }
    void begin (void);
    void setPixel(uint16_t poX, uint16_t poY,uint16_t color);
    void fill(uint16_t XL,uint16_t XR,uint16_t YU,uint16_t YD,uint16_t color = 0);
    void clear(void);
    size_t write(uint8_t);
    void backlight(bool on);
    void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height);
    void draw2x(const PROGMEM byte* buffer, byte width, byte height);
private:
    void setXY(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1);
    void sendPixelData(byte d);
    void writeDigit(byte n);
    void clearPixels(uint16_t pixels);
    void setCol(uint16_t StartCol,uint16_t EndCol);
    void setPage(uint16_t StartPage,uint16_t EndPage);
    void sendCMD(uint8_t index);
    void WRITE_Package(uint16_t *data,uint8_t howmany);
    void WRITE_DATA(uint8_t data);
    void sendData(uint16_t data);
    uint8_t Read_Register(uint8_t Addr,uint8_t xParameter);
    uint8_t readID(void);
    uint8_t m_color[2][2];
    uint16_t m_x;
    uint16_t m_y;
};
