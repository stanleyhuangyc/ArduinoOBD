#include <Arduino.h>
#include <Wire.h>
#include "MultiLCD.h"

#define I2C_ADDR 0x78 >> 1

void LCD_SH1106::WriteCommand(unsigned char ins)
{
  Wire.beginTransmission(I2C_ADDR);//0x78 >> 1
  Wire.write(0x00);//0x00
  Wire.write(ins);
  Wire.endTransmission();
}

void LCD_SH1106::WriteData(unsigned char dat)
{
  Wire.beginTransmission(I2C_ADDR);//0x78 >> 1
  Wire.write(0x40);//0x40
  Wire.write(dat);
  Wire.endTransmission();
}

void LCD_SH1106::setCursor(unsigned char x, unsigned char y)
{
    m_col = x + 2;
    m_row = y;
    WriteCommand(0xb0 + m_row);
    WriteCommand(m_col & 0xf);//set lower column address
    WriteCommand(0x10 | (m_col >> 4));//set higher column address
}

void LCD_SH1106::clear(byte x, byte y, byte width, byte height)
{
    WriteCommand(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
    WriteCommand(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
    WriteCommand(SSD1306_SETSTARTLINE | 0x0); // line #0

    // save I2C bitrate
    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!

    height >>= 3;
    width >>= 3;
    y >>= 3;
    for (byte i = 0; i < height; i++) {
      // send a bunch of data in one xmission
        WriteCommand(0xB0 + i + y);//set page address
        WriteCommand((x + 2) & 0xf);//set lower column address
        WriteCommand(0x10 | (x >> 4));//set higher column address

        for(byte j = 0; j < 8; j++){
            Wire.beginTransmission(I2C_ADDR);
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

size_t LCD_SH1106::write(uint8_t c)
{
    if (c == '\n') {
        setCursor(0, m_row + ((m_font == FONT_SIZE_SMALL) ? 1 : 2));
        return 1;
    } else if (c == '\r') {
        m_col = 0;
        return 1;
    }

    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!
#ifndef MEMORY_SAVING
    if (m_font == FONT_SIZE_SMALL) {
#endif
        Wire.beginTransmission(I2C_ADDR);
        Wire.write(0x40);
        if (c > 0x20 && c < 0x7f) {
            c -= 0x21;
            for (byte i = 0; i < 5; i++) {
                byte d = pgm_read_byte(&font5x8[c][i]);
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

            WriteCommand(0xB0 + m_row);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (byte i = 0; i <= 14; i += 2) {
                byte d = pgm_read_byte(&font8x16_terminal[c][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();

            WriteCommand(0xB0 + m_row + 1);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (byte i = 1; i <= 15; i += 2) {
                byte d = pgm_read_byte(&font8x16_terminal[c][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();
        } else {
            WriteCommand(0xB0 + m_row);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 16 : 8; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();

            WriteCommand(0xB0 + m_row + 1);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
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
    TWBR = twbrbackup;
    return 1;
}

void LCD_SH1106::writeDigit(byte n)
{
    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!

    if (m_font == FONT_SIZE_SMALL) {
        Wire.beginTransmission(I2C_ADDR);
        Wire.write(0x40);
        if (n <= 9) {
            n += '0' - 0x21;
            for (byte i = 0; i < 5; i++) {
                Wire.write(pgm_read_byte(&font5x8[n][i]));
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
        write(n <= 9 ? ('0' + n) : ' ');
#ifndef MEMORY_SAVING
    } else if (m_font == FONT_SIZE_LARGE) {
        if (n <= 9) {
            byte i;
            WriteCommand(0xB0 + m_row);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (i = 0; i < 16; i ++) {
                byte d = pgm_read_byte(&digits16x16[n][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();

            WriteCommand(0xB0 + m_row + 1);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (; i < 32; i ++) {
                byte d = pgm_read_byte(&digits16x16[n][i]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();
        } else {
            WriteCommand(0xB0 + m_row);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 32 : 16; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();

            WriteCommand(0xB0 + m_row + 1);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 32 : 16; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();
        }
        m_col += (m_flags & FLAG_PIXEL_DOUBLE_H) ? 30 : 16;
#endif
    } else {
        if (n <= 9) {
            byte i;
            WriteCommand(0xB0 + m_row);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (i = 0; i < 16; i ++) {
                byte d = pgm_read_byte(&digits16x24[n][i * 3]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();

            WriteCommand(0xB0 + m_row + 1);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (i = 0; i < 16; i ++) {
                byte d = pgm_read_byte(&digits16x24[n][i * 3 + 1]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();

            WriteCommand(0xB0 + m_row + 2);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (i = 0; i < 16; i ++) {
                byte d = pgm_read_byte(&digits16x24[n][i * 3 + 2]);
                Wire.write(d);
                if (m_flags & FLAG_PIXEL_DOUBLE_H) Wire.write(d);
            }
            Wire.endTransmission();
        } else {
            WriteCommand(0xB0 + m_row);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 32 : 16; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();

            WriteCommand(0xB0 + m_row + 1);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (byte i = (m_flags & FLAG_PIXEL_DOUBLE_H) ? 32 : 16; i > 0; i--) {
                Wire.write(0);
            }
            Wire.endTransmission();

            WriteCommand(0xB0 + m_row + 2);//set page address
            WriteCommand(m_col & 0xf);//set lower column address
            WriteCommand(0x10 | (m_col >> 4));//set higher column address

            Wire.beginTransmission(I2C_ADDR);
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

void LCD_SH1106::draw(const PROGMEM byte* buffer, byte width, byte height)
{
    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!

    WriteCommand(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
    WriteCommand(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
    WriteCommand(SSD1306_SETSTARTLINE | 0x0); // line #0

    const PROGMEM byte *p = buffer;
    height >>= 3;
    width >>= 3;
    for (byte i = 0; i < height; i++) {
      // send a bunch of data in one xmission
        WriteCommand(0xB0 + i + m_row);//set page address
        WriteCommand(m_col & 0xf);//set lower column address
        WriteCommand(0x10 | (m_col >> 4));//set higher column address

        for(byte j = 0; j < 8; j++){
            Wire.beginTransmission(I2C_ADDR);
            Wire.write(0x40);
            for (byte k = 0; k < width; k++, p++) {
                Wire.write(pgm_read_byte(p));
            }
            Wire.endTransmission();
        }
    }
    TWBR = twbrbackup;
    m_col += width;
}

void LCD_SH1106::begin()
{
  Wire.begin();

  WriteCommand(0xAE);    /*display off*/

  WriteCommand(0x02);    /*set lower column address*/
  WriteCommand(0x10);    /*set higher column address*/

  WriteCommand(0x40);    /*set display start line*/

  WriteCommand(0xB0);    /*set page address*/

  WriteCommand(0x81);    /*contract control*/
  WriteCommand(0x80);    /*128*/

  WriteCommand(0xA1);    /*set segment remap*/

  WriteCommand(0xA6);    /*normal / reverse*/

  WriteCommand(0xA8);    /*multiplex ratio*/
  WriteCommand(0x3F);    /*duty = 1/32*/

  WriteCommand(0xad);    /*set charge pump enable*/
  WriteCommand(0x8b);     /*external VCC   */

  WriteCommand(0x30);    /*0X30---0X33  set VPP   9V liangdu!!!!*/

  WriteCommand(0xC8);    /*Com scan direction*/

  WriteCommand(0xD3);    /*set display offset*/
  WriteCommand(0x00);   /*   0x20  */

  WriteCommand(0xD5);    /*set osc division*/
  WriteCommand(0x80);

  WriteCommand(0xD9);    /*set pre-charge period*/
  WriteCommand(0x1f);    /*0x22*/

  WriteCommand(0xDA);    /*set COM pins*/
  WriteCommand(0x12);

  WriteCommand(0xdb);    /*set vcomh*/
  WriteCommand(0x40);

  WriteCommand(0xAF);    /*display ON*/

  clear();
}
