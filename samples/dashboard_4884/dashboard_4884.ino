/*************************************************************************
* Sample sketch based on OBD-II library for Arduino
* Using a LCD4884 shield to display realtime vehicle data
* Distributed under GPL v2.0
* Copyright (c) 2012 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include "OBD.h"
#include "LCD4884.h"

// the following line toggles between hardware serial and software serial
// #define USE_SOFTSERIAL

#ifdef USE_SOFTSERIAL
#include <SoftwareSerial.h>
SoftwareSerial mySerial(11, 12); // RX, TX
#endif

//keypad debounce parameter
#define DEBOUNCE_MAX 15
#define DEBOUNCE_ON  10
#define DEBOUNCE_OFF 3 

#define NUM_KEYS 5
#define NUM_MODES 3

// joystick number
#define LEFT_KEY 0
#define CENTER_KEY 1
#define DOWN_KEY 2
#define RIGHT_KEY 3
#define UP_KEY 4

int  adc_key_val[5] ={
  50, 200, 400, 600, 800 };

// debounce counters
byte button_count[NUM_KEYS];
// button status - pressed/released
byte button_status[NUM_KEYS];
// button on flags for user program 
byte button_flag[NUM_KEYS];
// initial gauge mode
char mode = 0;

// The followinging are interrupt-driven keypad reading functions
// which includes DEBOUNCE ON/OFF mechanism, and continuous pressing detection

// Convert ADC value to key number
char get_key(unsigned int input)
{
  char k;

  for (k = 0; k < NUM_KEYS; k++)
  {
    if (input < adc_key_val[k])
    {

      return k;
    }
  }

  if (k >= NUM_KEYS)
    k = -1;     // No valid key pressed

  return k;
}

void update_adc_key(){
  int adc_key_in;
  char key_in;
  byte i;

  adc_key_in = analogRead(0);
  key_in = get_key(adc_key_in);
  for(i=0; i<NUM_KEYS; i++)
  {
    if(key_in==i)  //one key is pressed 
    { 
      if(button_count[i]<DEBOUNCE_MAX)
      {
        button_count[i]++;
        if(button_count[i]>DEBOUNCE_ON)
        {
          if(button_status[i] == 0)
          {
            button_flag[i] = 1;
            button_status[i] = 1; //button debounced to 'pressed' status
          }

        }
      }

    }
    else // no button pressed
    {
      if (button_count[i] >0)
      {  
        button_flag[i] = 0;	
        button_count[i]--;
        if(button_count[i]<DEBOUNCE_OFF){
          button_status[i]=0;   //button debounced to 'released' status
        }
      }
    }

  }
}

void ShowProgressBarV(byte x, byte y, byte val /* 0~10 */)
{
	byte j = 10 - val;
	for (char y1 = j >> 1; y1 >= 0; y1--) {
		lcd.LCD_set_XY(x, y1 + y);
		for (byte x = 0; x < 14; x++) {
			lcd.LCD_write_byte(0, 1);
		}
	}
	if (j & 1 == 1) {
		j >>= 1;
		lcd.LCD_set_XY(x, y + j);
		for (byte x = 0; x < 14; x++) {
			lcd.LCD_write_byte(0xE0, 1);
		}
		j++;
	} else {
		j >>= 1;
	}
	for (byte y1 = j; y1 <= 5; y1++) {
		lcd.LCD_set_XY(x, y1 + y);
		for (byte x = 0; x < 14; x++) {
			lcd.LCD_write_byte(0xEE, 1);
		}
	}
}

byte CheckPressedKey()
{
    for(int i=0; i<NUM_KEYS; i++){
      if(button_flag[i] !=0){
        button_flag[i]=0;  // reset button flag
        return i;
      }
    }
    return -1;
}

// waiting for center key press
void waitfor_OKkey(){
  byte i;
  byte key = 0xFF;
  update_adc_key();
  while (key!= CENTER_KEY){
    for(i=0; i<NUM_KEYS; i++){
      if(button_flag[i] !=0){
        button_flag[i]=0;  // reset button flag
        if(i== CENTER_KEY) key=CENTER_KEY;
      }
    }
  }

}

class COBDDash : public COBD
{
public:
        void Connect()
        {
                lcd.LCD_clear();
                lcd.LCD_write_string(0, 0, "Connecting..", MENU_NORMAL);
                for (int n = 0; !Init(); n++) {
                  lcd.LCD_putchar('.');
                  if (n == 3) lcd.backlight(OFF);
                }

                lcd.backlight(ON); //Turn on the backlight
                lcd.LCD_clear();
                lcd.LCD_write_string(0, 0, "Connected!", MENU_NORMAL);

                int value;
                lcd.LCD_write_string(0, 1, "Wait ECU start", MENU_NORMAL);
                do {
                  delay(1000);
                } while (!ReadSensor(PID_RPM, value));
                lcd.LCD_write_string(0, 2, "ECU started   ", MENU_NORMAL);
                lcd.LCD_write_string(0, 3, "Wait ignition ", MENU_NORMAL);
                do {
                  delay(100);
                } while (!ReadSensor(PID_RPM, value) || value == 0);                         
                lcd.LCD_write_string(0, 4, "Engine started", MENU_NORMAL);
                delay(1000);                
        }
	void Loop()
	{
                unsigned long lastTime = millis();
                
		Connect();
  
		byte count = 0;
		byte key;
		DisplayBG(mode);
		dataMode = 1;
                lcd.backlight(ON);
		for (;;) {
                        update_adc_key();
			key = CheckPressedKey();
			if (key != -1) {
				switch (key) {
                                case CENTER_KEY:
                                    mode = (mode + 1) % NUM_MODES;
                                    DisplayBG(mode);
				    count = 0;
                                    break;
				case LEFT_KEY:
					if (mode > 0) {
						mode--;
						DisplayBG(mode);
						count = 0;
					}
					break;
				case RIGHT_KEY:
					if (mode < NUM_MODES - 1) {
						mode++;
						DisplayBG(mode);
						count = 0;
					}
					break;
				case UP_KEY:
					lcd.backlight(ON);
					break;
				case DOWN_KEY:
					lcd.backlight(OFF);
					break;
				}
			}
                        if (millis() - lastTime < 250) {
                          continue;
                        }
                        lastTime = millis();

			switch (mode) {
			case 0:
				DisplayData1();
				break;
			case 1:
				DisplayData2();
				switch (count) {
				case 0:
					DisplayData21();
					break;
				case 5:
					DisplayData22();
					break;
				case 10:
					DisplayData23();
					break;
				}
				break;
			case 2:
				DisplayData3();
				break;
			}
                        if (errors > 5) {
                            lcd.backlight(OFF);
                            return;
                        }
                        count++;
		}
	}
private:
	void DisplayData1()
	{
            if (ReadSensor(PID_RPM, value)) {
		ShowRPM(value);
            }
            if (ReadSensor(PID_SPEED, value)) {
		ShowSpeed(value);
            }
            if (ReadSensor(PID_ENGINE_LOAD, value)) {
		ShowEngineLoad(value);
            }
	}
	void DisplayData2()
	{
            if (ReadSensor(PID_RPM, value)) {
		ShowRPM(value);
            }
            if (ReadSensor(PID_SPEED, value)) {
		ShowSpeed2(value);
            }
	}
	void DisplayData21()
	{
            if (ReadSensor(PID_COOLANT_TEMP, value)) {
		ShowTemperature(value, 42, 3);
            }
	}
	void DisplayData22()
	{
            if (ReadSensor(PID_INTAKE_TEMP, value)) {
		ShowTemperature(value, 42, 4);
            }
	}
	void DisplayData23()
	{
            if (ReadSensor(PID_AMBIENT_TEMP, value)) {
		ShowTemperature(value, 42, 5);
            }
	}
	void DisplayData3()
	{
            if (ReadSensor(PID_SPEED, value)) {
		ShowSpeed2(value);
            }
            if (ReadSensor(PID_INTAKE_PRESSURE, value)) {
              char buf[8];
              sprintf(buf, "%3u", value);
  	      lcd.LCD_write_string(24, 4, buf, MENU_NORMAL);
              int boost = (value - 101);
              if (boost < 0) boost = 0;
              sprintf(buf, "%d.%02d", boost / 100, boost % 100);
	      lcd.LCD_write_string_big(0, 0, buf, MENU_NORMAL);
            }
            if (ReadSensor(PID_FUEL_PRESSURE, value)) {
              char buf[8];
              sprintf(buf, "%3u", value);
              lcd.LCD_write_string(24, 5, buf, MENU_NORMAL);
            }
	}
	void ShowEngineLoad(uint8_t value)
	{
		ShowProgressBarV(70, 1, value / 10);
		lcd.LCD_write_string(78, 1, "%", MENU_NORMAL);
	}
	void ShowRPM(int value)
	{
		char buf[15];
		if (value <= 9999) {
			sprintf(buf, "%4u", value);
			lcd.LCD_write_string_big(0, 0, buf, MENU_NORMAL);
			lcd.LCD_write_string(48, 2, "R", MENU_NORMAL);
		}
	}
	void ShowSpeed(uint8_t value)
	{
		char buf[8];
		sprintf(buf, "%3u", value);
		lcd.LCD_write_string_big(6, 3, buf, MENU_NORMAL);
		lcd.LCD_write_string(42, 5, "k", MENU_NORMAL);
	}
	void ShowSpeed2(uint8_t value)
	{
		char buf[8];
		ShowProgressBarV(70, 1, value / 25);
		sprintf(buf, "%3u", value);
		lcd.LCD_write_string(66, 0, buf, MENU_NORMAL);
		lcd.LCD_write_string(66, 1, "kph", MENU_NORMAL);
	}
	void ShowTemperature(uint8_t value, byte x, byte y)
	{
		char buf[8];
		sprintf(buf, "%3d", value);
		lcd.LCD_write_string(x, y, buf, MENU_NORMAL);
	}
	void DisplayBG(char mode)
	{
		lcd.LCD_clear();
		switch (mode) {
		case 0:
			lcd.LCD_write_string(48, 2, "RPM", MENU_NORMAL);
			lcd.LCD_write_string(42, 5, "kph", MENU_NORMAL);
			lcd.LCD_write_string(66, 0, "ENG", MENU_NORMAL);
			break;
		case 1:
			lcd.LCD_write_string(48, 2, "RPM", MENU_NORMAL);
			lcd.LCD_write_string(0, 3, "COOLANT   C", MENU_NORMAL);
			lcd.LCD_write_string(0, 4, "INTAKE    C", MENU_NORMAL);
			lcd.LCD_write_string(0, 5, "AMBIENT   C", MENU_NORMAL);
			break;
		case 2:
			lcd.LCD_write_string(48, 2, "bar", MENU_NORMAL);
			lcd.LCD_write_string(0, 3, "PRESSURES", MENU_NORMAL);
			lcd.LCD_write_string(0, 4, "AIR     kpa", MENU_NORMAL);
			lcd.LCD_write_string(0, 5, "FUEL    kpa", MENU_NORMAL);
			break;
		}
	}
#ifdef USE_SOFTSERIAL
        // override data communication functions
        bool DataAvailable() { return mySerial.available(); }
        char ReadData()
        {
          char c = mySerial.read();
          Serial.write(c);
          return c;
        }
        void WriteData(const char* s) { mySerial.write(s); }
        void WriteData(const char c) { mySerial.write(c); }
#endif
	char displayMode;
        int value;
};

COBDDash obd;

void loop()
{
  obd.Loop();
}

void setup()
{

  // setup interrupt-driven keypad arrays  
  // reset button arrays
  for(byte i=0; i<NUM_KEYS; i++){
    button_count[i]=0;
    button_status[i]=0;
    button_flag[i]=0;
  }

#ifdef __AVR_ATmega32U4__
  // Setup timer2 -- Prescaler/256
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
  TCCR2B &= ~(1<<WGM22);
  TCCR2B = (1<<CS22)|(1<<CS21);      

  ASSR |=(0<<AS2);

  // Use normal mode  
  TCCR2A =0;    
  //Timer2 Overflow Interrupt Enable  
  TIMSK2 |= (0<<OCIE2A);
  TCNT2=0x6;  // counting starts from 6;  
  TIMSK2 = (1<<TOIE2);    

  SREG|=1<<SREG_I;
#endif

  lcd.LCD_init();
  lcd.LCD_clear();

  lcd.backlight(ON); // Turn on the backlight  
  
  pinMode(13, OUTPUT);

#ifndef USE_SOFTSERIAL
  OBDUART.begin(OBD_SERIAL_BAUDRATE);
#else
  Serial.begin(9600);
  mySerial.begin(OBD_SERIAL_BAUDRATE);
#endif
}

// Timer2 interrupt routine -
// 1/(160000000/256/(256-6)) = 4ms interval

#ifdef __AVR_ATmega32U4__

ISR(TIMER2_OVF_vect) {  
  TCNT2  = 6;
  update_adc_key();
}

#endif

