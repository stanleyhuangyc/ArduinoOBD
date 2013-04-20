extern const PROGMEM unsigned char font16x32[][32];
extern const PROGMEM unsigned char font5x8[][5];

#include "PCD8544.h"

class LCD_Common
{
public:
    virtual void backlight(bool on) {}
    virtual byte getLines() = 0;
    virtual byte getCols() = 0;
};

class LCD_PCD8544 : public LCD_Common, public PCD8544
{
public:
    byte getLines() { return 6; }
    byte getCols() { return 14; }
    void printLarge(const char* s);
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
};

#include "ZtLib.h"

#define OLED_ADDRESS 0x27

class LCD_OLED : public LCD_Common, public ZtLib
{
public:
    byte getLines() { return 4; }
    byte getCols() { return 16; }
    void setCursor(byte column, byte line)
    {
        m_column = column << 3;
        m_line = line << 1;
    }
    void write(char c);
    void print(const char* s);
    void printLarge(const char* s);
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
    unsigned char m_line;
};

#include "LCD4Bit_mod.h"
class LCD_1602 : public LCD_Common, public LCD4Bit_mod
{
public:
    byte getLines() { return 2; }
    byte getCols() { return 16; }
    void printLarge(const char* s)
    {
        print(s);
    }
    void clearLine(byte line)
    {
        setCursor(0, line);
        for (byte i = 16; i > 0; i--) write(' ');
    }
};

