#include <Arduino.h>
#include <Wire.h>
#include <MultiLCD.h>

//LCD_OLED lcd; /* for I2C OLED module */
//LCD_PCD8544 lcd; /* for LCD4884 shield or Nokia 5100 screen module */
LCD_1602 lcd; /* for LCD1602 shield */

void setup()
{
	lcd.begin();
	lcd.setCursor(0, 0);
	lcd.print("Hello, World");
	lcd.setCursor(0, 1);
	lcd.printLarge("12345");
}

void loop()
{
}
