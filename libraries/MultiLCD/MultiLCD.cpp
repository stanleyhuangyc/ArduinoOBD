/*************************************************************************
* Arduino Text Display Library for Multiple LCDs
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@live.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <UTFT.h>
#include "MultiLCD.h"
#include "fonts.h"

void LCD_Common::printInt(uint16_t value, int8_t padding)
{
    uint16_t den = 10000;
    for (int8_t i = 5; i > 0; i--) {
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

void LCD_Common::printLong(uint32_t value, int8_t padding)
{
    uint32_t den = 1000000000;
    for (int8_t i = 10; i > 0; i--) {
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
