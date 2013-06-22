/*************************************************************************
* Arduino Text Display Library for Multiple LCDs
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@live.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "MultiLCD.h"

void LCD_Common::printInt(unsigned int value, char padding)
{
    unsigned int den = 10000;
    for (byte i = 5; i > 0; i--) {
        byte v = (byte)(value / den);
        value -= v * den;
        den /= 10;
        if (v == 0 && padding && den) {
            if (padding >= i) {
                writeDigit(-1);
            }
            continue;
        }
        padding = 0;
        writeDigit(v);
    }
}

void LCD_Common::printLong(unsigned long value, char padding)
{
    unsigned long den = 1000000000;
    for (byte i = 10; i > 0; i--) {
        byte v = (byte)(value / den);
        value -= v * den;
        den /= 10;
        if (v == 0 && padding && den) {
            if (padding >= i) {
                writeDigit(-1);
            }
            continue;
        }
        padding = 0;
        writeDigit(v);
    }
}

void LCD_ZTOLED::setCursor(byte column, byte line)
{
    m_column = column;
    m_page = line;
    ScI2cMxSetLocation(OLED_ADDRESS, m_page, m_column);
    delay(1);
}

size_t LCD_ZTOLED::write(uint8_t c)
{
    if (m_font == FONT_SIZE_SMALL) {
        if (c <= 0x20 || c >= 0x7f) {
            ScI2cMxFillArea(OLED_ADDRESS, m_column, m_column + 5, m_page, m_page, 0);
        } else {
            ScI2cMxDisplayDot(OLED_ADDRESS, font5x8[c - 0x21], 5);
        }
        m_column += 6;
        ScI2cMxSetLocation(OLED_ADDRESS, m_page, m_column);
    } else {
        char s[2] = {c};
        ScI2cMxDisplay8x16Str(OLED_ADDRESS, m_page, m_column, s);
        m_column += 8;
        ScI2cMxSetLocation(OLED_ADDRESS, m_page, m_column);
    }
    return 1;
}

void LCD_ZTOLED::writeDigit(byte n)
{
    if (m_font == FONT_SIZE_SMALL) {
        if (n <= 9)
            ScI2cMxDisplayDot(OLED_ADDRESS, font5x8[n + ('0' - 0x21)], 5);
        else
            ScI2cMxFillArea(OLED_ADDRESS, m_column, m_column + 5, m_page, m_page, 0);
        m_column += 6;
        ScI2cMxSetLocation(OLED_ADDRESS, m_page, m_column);
    } else if (m_font == FONT_SIZE_MEDIUM) {
        if (n <= 9) {
            ScI2cMxDisplayDot(OLED_ADDRESS, digits8x8[n], 8);
        } else {
            ScI2cMxFillArea(OLED_ADDRESS, m_column, m_column + 7, m_page, m_page, 0);

        }
        m_column += 8;
        ScI2cMxSetLocation(OLED_ADDRESS, m_page, m_column);
    } else if (m_font == FONT_SIZE_LARGE) {
        write('0' + n);
    } else {
        unsigned char data[32];
        if (n <= 9) {
            memcpy_P(data, digits16x16[n], 32);
        } else {
            memset(data, 0, sizeof(data));
        }
        ScI2cMxDisplayDot16x16(OLED_ADDRESS, m_page, m_column, data);
        m_column += 16;
        ScI2cMxSetLocation(OLED_ADDRESS, m_page, m_column);
    }
}

void LCD_ZTOLED::clear()
{
    ScI2cMxFillArea(OLED_ADDRESS, 0, 7, 0, 127, 0);
    delay(10);
    setCursor(0, 0);
}

void LCD_ZTOLED::begin()
{
    I2cInit();
    ScI2cMxReset(OLED_ADDRESS);
    clear();
}

void LCD_PCD8544::writeDigit(byte n)
{
    if (m_font == FONT_SIZE_SMALL) {
        write(n >= 0 && n <= 9 ? '0' + n : ' ');
    } else if (m_font == FONT_SIZE_MEDIUM) {
        unsigned char data[8];
        if (n >= 0 && n <= 9) {
            memcpy_P(data, digits8x8[n], 8);
        } else {
            memset(data, 0, sizeof(data));
        }
        draw8x8(data);
    } else {
        unsigned char data[32];
        if (n >= 0 && n <= 9) {
            memcpy_P(data, digits16x16[n], 32);
        } else {
            memset(data, 0, sizeof(data));
        }
        draw16x16(data);
        //column += 16;
    }
}


void LCD_PCD8544::draw(const unsigned char *data, unsigned char x, unsigned char y, unsigned char width, unsigned char height)
{
    height >>= 3;
    for (unsigned char y = 0; y < height; y++) {
        setCursor(x, y);
        for (unsigned char x = 0; x < width; x++) {
            send(PCD8544_DATA, data[y * width + x]);
        }
    }
}

void LCD_SSD1306::setCursor(byte column, byte line)
{
    m_col = column;
    m_row = line;
    ssd1306_command(0xB0 + m_row);//set page address
    ssd1306_command(m_col & 0xf);//set lower column address
    ssd1306_command(0x10 | (m_col >> 4));//set higher column address
}

size_t LCD_SSD1306::write(uint8_t c)
{
    if (m_font == FONT_SIZE_SMALL) {
        Wire.beginTransmission(_i2caddr);
        Wire.write(0x40);
        if (c > 0x20 && c < 0x7f) {
            c -= 0x21;
            for (byte i = 0; i < 5; i++) {
                Wire.write(pgm_read_byte_near(&font5x8[c][i]));
            }
            Wire.write(0);
        } else {
            for (byte i = 0; i < 6; i++) {
                Wire.write(0);
            }
        }
        Wire.endTransmission();
        m_col += 8;
    } else {
        if (c > 0x20 && c < 0x7f) {
            c -= 0x21;

            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 0; i <= 14; i += 2) {
                Wire.write(pgm_read_byte_near(&font8x16_terminal[c][i]));
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 1; i <= 15; i += 2) {
                Wire.write(pgm_read_byte_near(&font8x16_terminal[c][i]));
            }
            Wire.endTransmission();
        } else {
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 0; i < 8; i ++) {
                Wire.write(0);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 0; i < 8; i ++) {
                Wire.write(0);
            }
            Wire.endTransmission();
        }
        m_col += 9;
    }
    return 1;
}

void LCD_SSD1306::writeDigit(byte n)
{
    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!
    if (m_font == FONT_SIZE_SMALL) {
        n += '0' - 0x21;
        Wire.beginTransmission(_i2caddr);
        Wire.write(0x40);
        if (n <= 9) {
            for (byte i = 0; i < 5; i++) {
                Wire.write(pgm_read_byte_near(&font5x8[n][i]));
            }
            Wire.write(0);
        } else {
            for (byte i = 0; i < 6; i++) {
                Wire.write(0);
            }
        }
        Wire.endTransmission();
        m_col += 6;
    } else if (m_font == FONT_SIZE_MEDIUM) {
        Wire.beginTransmission(_i2caddr);
        Wire.write(0x40);
        if (n <= 9) {
            for (byte i = 0; i < 8; i++) {
                Wire.write(pgm_read_byte_near(&digits8x8[n][i]));
            }
        } else {
            for (byte i = 0; i < 8; i++) {
                Wire.write(0);
            }
        }
        Wire.endTransmission();
        m_col += 8;
    } else if (m_font == FONT_SIZE_LARGE) {
        if (n <= 9) {
            n += '0' - 0x21;
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 0; i <= 14; i += 2) {
                Wire.write(pgm_read_byte_near(&font8x16_terminal[n][i]));
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 1; i <= 15; i += 2) {
                Wire.write(pgm_read_byte_near(&font8x16_terminal[n][i]));
            }
            Wire.endTransmission();
        } else {
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 0; i < 8; i++) {
                Wire.write(0);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 0; i < 8; i++) {
                Wire.write(0);
            }
            Wire.endTransmission();
        }
        m_col += 9;
    } else if (m_font == FONT_SIZE_XLARGE) {
        if (n <= 9) {
            byte i;
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (i = 0; i < 16; i ++) {
                Wire.write(pgm_read_byte_near(&digits16x16[n][i]));
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (; i < 32; i ++) {
                Wire.write(pgm_read_byte_near(&digits16x16[n][i]));
            }
            Wire.endTransmission();
        } else {
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 0; i < 16; i++) {
                Wire.write(0);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 0; i < 16; i++) {
                Wire.write(0);
            }
            Wire.endTransmission();
        }
        m_col += 16;
    }
    TWBR = twbrbackup;
}

void LCD_SSD1306::draw(const PROGMEM byte* buffer, byte x, byte y, byte width, byte height)
{
    ssd1306_command(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
    ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0); // line #0

    // save I2C bitrate
    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!

    const PROGMEM byte *p = buffer;
    height >>= 3;
    width >>= 3;
    y >>= 3;
    for (byte i = 0; i < height; i++) {
      // send a bunch of data in one xmission
        ssd1306_command(0xB0 + i + y);//set page address
        ssd1306_command(x & 0xf);//set lower column address
        ssd1306_command(0x10 | (x >> 4));//set higher column address

        for(byte j = 0; j < 8; j++){
            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte k = 0; k < width; k++, p++) {
                Wire.write(pgm_read_byte_near(p));
            }
            Wire.endTransmission();
        }
    }
    TWBR = twbrbackup;
}

void LCD_SSD1306::clearLine(byte line)
{
    ssd1306_command(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
    ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0); // line #0

    // save I2C bitrate
    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!

    // send a bunch of data in one xmission
    ssd1306_command(0xB0 + line);//set page address
    ssd1306_command(0);//set lower column address
    ssd1306_command(0x10);//set higher column address

    for(byte j = 0; j < 8; j++){
        Wire.beginTransmission(_i2caddr);
        Wire.write(0x40);
        for (byte k = 0; k < 16; k++) {
            Wire.write(0);
        }
        Wire.endTransmission();
    }

    TWBR = twbrbackup;
}

void LCD_SSD1306::clear(byte x, byte y, byte width, byte height)
{
    ssd1306_command(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
    ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0); // line #0

    // save I2C bitrate
    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!

    height >>= 3;
    width >>= 3;
    y >>= 3;
    for (byte i = 0; i < height; i++) {
      // send a bunch of data in one xmission
        ssd1306_command(0xB0 + i + y);//set page address
        ssd1306_command(x & 0xf);//set lower column address
        ssd1306_command(0x10 | (x >> 4));//set higher column address

        for(byte j = 0; j < 8; j++){
            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte k = 0; k < width; k++) {
                Wire.write(0);
            }
            Wire.endTransmission();
        }
    }
    TWBR = twbrbackup;
}
