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
                writeDigit((m_flags & FLAG_PAD_ZERO) ? 0 : -1);
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
                writeDigit((m_flags & FLAG_PAD_ZERO) ? 0 : -1);
            }
            continue;
        }
        padding = 0;
        writeDigit(v);
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
    if (c == '\n') {
        setCursor(0, m_row + ((m_font == FONT_SIZE_SMALL) ? 1 : 2));
        return 1;
    } else if (c == '\r') {
        m_col = 0;
        return 1;
    }
#ifndef MEMORY_SAVING
    if (m_font == FONT_SIZE_SMALL) {
#endif
        Wire.beginTransmission(_i2caddr);
        Wire.write(0x40);
        if (c > 0x20 && c < 0x7f) {
            c -= 0x21;
            for (byte i = 0; i < 5; i++) {
                byte d = pgm_read_byte_near(&font5x8[c][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.write(0);
        } else {
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 11 : 6; i > 0; i--) {
                Wire.write(0);
            }
        }
        Wire.endTransmission();
        m_col += (m_flags & FLAG_PIXEL_DOUBLE_H) ? 11 : 6;
        if (m_col >= 128) {
            m_col = 0;
            m_row ++;
        }
#ifndef MEMORY_SAVING
    } else {
        if (c > 0x20 && c < 0x7f) {
            c -= 0x21;

            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 0; i <= 14; i += 2) {
                byte d = pgm_read_byte_near(&font8x16_terminal[c][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 1; i <= 15; i += 2) {
                byte d = pgm_read_byte_near(&font8x16_terminal[c][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();
        } else {
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 16 : 8; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 16 : 8; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();
        }
        m_col += (m_flags & FLAG_PIXEL_DOUBLE_H) ? 17 : 9;
        if (m_col >= 128) {
            m_col = 0;
            m_row += 2;
        }
    }
#endif
    return 1;
}

void LCD_SSD1306::writeDigit(byte n)
{
    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!
    if (m_font == FONT_SIZE_SMALL) {
        Wire.beginTransmission(_i2caddr);
        Wire.write(0x40);
        if (n <= 9) {
            n += '0' - 0x21;
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
#ifndef MEMORY_SAVING
        if (n <= 9) {
            n += '0' - 0x21;
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 0; i <= 14; i += 2) {
                byte d = pgm_read_byte_near(&font8x16_terminal[n][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = 1; i <= 15; i += 2) {
                byte d = pgm_read_byte_near(&font8x16_terminal[n][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();
        } else {
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 16 : 8; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 16 : 8; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();
        }
        m_col += (m_flags & FLAG_PIXEL_DOUBLE_H) ? 17 : 9;
#else
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
#endif
    } else if (m_font == FONT_SIZE_LARGE) {
        if (n <= 9) {
            byte i;
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (i = 0; i < 16; i ++) {
                byte d = pgm_read_byte_near(&digits16x16[n][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (; i < 32; i ++) {
                byte d = pgm_read_byte_near(&digits16x16[n][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();
        } else {
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 32 : 16; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 32 : 16; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();
        }
        m_col += (m_flags & FLAG_PIXEL_DOUBLE_H) ? 30 : 16;
    } else if (m_font == FONT_SIZE_XLARGE) {
        if (n <= 9) {
            byte i;
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (i = 0; i < 16; i ++) {
                byte d = pgm_read_byte_near(&digits16x24[n][i * 3]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (i = 0; i < 16; i ++) {
                byte d = pgm_read_byte_near(&digits16x24[n][i * 3 + 1]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 2);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (i = 0; i < 16; i ++) {
                byte d = pgm_read_byte_near(&digits16x24[n][i * 3 + 2]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();
        } else {
            ssd1306_command(0xB0 + m_row);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 32 : 16; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 1);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 32 : 16; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();

            ssd1306_command(0xB0 + m_row + 2);//set page address
            ssd1306_command(m_col & 0xf);//set lower column address
            ssd1306_command(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(_i2caddr);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 32 : 16; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();
        }
        m_col += (m_flags & FLAG_PIXEL_DOUBLE_H) ? 30 : 16;
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

    setCursor(0, 0);
    TWBR = twbrbackup;
}
