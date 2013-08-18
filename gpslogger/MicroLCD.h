/*************************************************************************
* Arduino Text Display Library for Multiple LCDs
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@live.com>
* All rights reserved.
*************************************************************************/

#if !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega644P__)
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

extern const PROGMEM unsigned char font5x8[][5];
extern const PROGMEM unsigned char digits8x8[][8] ;
extern const PROGMEM unsigned char digits16x16[][32];
extern const PROGMEM unsigned char digits16x24[][48];
extern const PROGMEM unsigned char font8x16_doslike[][16];
extern const PROGMEM unsigned char font8x16_terminal[][16];

class LCD_Common
{
public:
    LCD_Common():m_font(FONT_SIZE_SMALL),m_flags(0) {}
    void setFont(FONT_SIZE size) { m_font = size; }
    void setFlags(byte flags) { m_flags = flags; }
    virtual void backlight(bool on) {}
    virtual void draw(const PROGMEM byte* buffer, byte x, byte y, byte width, byte height) {}
    void printInt(uint16_t value, char padding = -1);
    void printLong(unsigned long value, char padding = -1);
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
