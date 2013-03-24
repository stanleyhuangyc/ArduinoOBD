extern const PROGMEM unsigned char font16x32[][32];
extern const PROGMEM unsigned char font5x8[][5];

#include "PCD8544.h"

class LCD_PCD8544 : public PCD8544 {
public:
    void printLarge(const char* s);
    void backlight(bool on)
    {
        pinMode(7, OUTPUT);
        digitalWrite(7, on ? HIGH : LOW);
    }
};

#include "ZtLib.h"

#define OLED_ADDRESS 0x27

class LCD_OLED : public ZtLib {
public:
    void setCursor(unsigned char column, unsigned char line)
    {
        m_column = column << 3;
        m_line = line << 1;
    }
    void print(const char* s);
    void printLarge(const char* s);
    void clear();
    void begin();
    void backlight(bool on) {}
private:
    unsigned char m_column;
    unsigned char m_line;
};

#include "LCD4Bit_mod.h"
class LCD_1602 : public LCD4Bit_mod {
public:
    void printLarge(const char* s) { print(s); }
    void backlight(bool on) {}
};

