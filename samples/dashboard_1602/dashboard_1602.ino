/*************************************************************************
* Sample sketch based on OBD-II library for Arduino
* Using a LCD1602 shield to display realtime vehicle data
* Distributed under GPL v2.0
* Copyright (c) 2012 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <arduino.h>
#include "OBD.h"
#include "LCD4Bit_mod.h"

//create object to control an LCD.
LCD4Bit_mod lcd = LCD4Bit_mod(2);

COBD obd;

//Key message
unsigned int  adc_key_val[5] ={30, 150, 360, 535, 760 };
int NUM_KEYS = 5;
int adc_key_in;
char key=-1;
char oldkey=-1;
unsigned long lastTick = millis();
uint8_t modes[2] = {0, 2};

const char modePids[] = {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD,
	PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_MAF_FLOW,
	PID_ABS_ENGINE_LOAD, PID_FUEL_PRESSURE, PID_INTAKE_MAP, PID_BAROMETRIC,
	PID_TIMING_ADVANCE, PID_FUEL_LEVEL, PID_RUNTIME, PID_DISTANCE};

const char* modeLabels[] = {
	"Engine       rpm", "Speed       km/h", "Throttle       %", "Engine Load    %",
	"Coolant        C", "Intake Air     C", "Env. Temp      C", "MAF Flow     kpa",
	"Abs. Load      %", "Fuel         kpa", "Intake       kpa", "Barometer    kpa",
	"Timing Adv.     ", "Fuel Level     %", "Run Time", "Distance      km"};

const char modePos[] = {8, 8, 11, 12,
	11, 11, 11, 9,
	11, 9, 9, 10,
	12, 11, 8, 10};

const char* modeFmts[] = {"%4u", "%3u", "%3u", "%u",
	"%3d", "%3d", "%3d", "%3u",
	"%3u", "%3u", "%3u", "%u",
	"%3d", "%3u", "%4u:%02u", "%04u"};

#define TOTAL_PIDS (sizeof(modePids) / sizeof(modePids[0]))

// Convert ADC value to key number
char get_key(unsigned int input)
{
  char k;
  for (k = 0; k < NUM_KEYS; k++) {
    if (input < adc_key_val[k])
      return k;
  }
  return -1;
}

void updateMode()
{
	lcd.cursorTo(1, 0);
	lcd.printIn((char*)modeLabels[modes[0]]);
	lcd.cursorTo(2, 0);
	lcd.printIn((char*)modeLabels[modes[1]]);
}

bool showData(int index)
{
	char buf[16];
	int value;
	uint8_t mode = modes[index];
	uint8_t pid = modePids[mode];
	digitalWrite(13, HIGH);   // set the LED on
	if (!obd.read(pid, value)) {
                // display received data on error
		lcd.cursorTo(index + 1, 0);
		lcd.printIn("Error");
		delay(2000);
		updateMode();
		return false;
	}
	digitalWrite(13, LOW);   // set the LED off

	if (pid == PID_RUNTIME) {
		sprintf(buf, modeFmts[mode], (unsigned int)value / 60, (unsigned int)value % 60);
	} else {
		sprintf(buf, modeFmts[mode], value);
	}
	lcd.cursorTo(index + 1, modePos[mode]);
	lcd.printIn(buf);
	return true;
}

bool setupConnection()
{
  uint8_t errors = 0;
  char buf[16];
  lcd.clear();
  lcd.printIn("Connecting...");
  while (!obd.init()) {
	  lcd.cursorTo(2, 0);
	  sprintf(buf, "Attempts #%d", ++errors);
	  lcd.printIn(buf);
  }
  lcd.clear();
  lcd.printIn("Connected!");
  delay(1000);
  updateMode();
  return true;
}

void setup()
{
  pinMode(13, OUTPUT);  //we'll use the debug LED to output a heartbeat
  lcd.init();
  obd.begin();
  setupConnection();
}

void loop()
{
	adc_key_in = analogRead(0);    // read the value from the sensor
	key = get_key(adc_key_in);		        // convert into key press

	if (key != oldkey) {
		delay(50);		// wait for debounce time
		adc_key_in = analogRead(0);    // read the value from the sensor
		key = get_key(adc_key_in);		        // convert into key press
		if (key != oldkey)
		{
			oldkey = key;
			if (key >=0){
				switch (key) {
				case 3: // left key
					do {
						modes[0] = modes[0] > 0 ? modes[0] - 1 : TOTAL_PIDS - 1;
					} while (modes[0] == modes[1]);
					break;
				case 0: // right key
					do {
						modes[0] = modes[0] < TOTAL_PIDS - 1 ? modes[0] + 1 : 0;
					} while (modes[0] == modes[1]);
					break;
				case 1: // up key
					do {
						modes[1] = modes[1] > 0 ? modes[1] - 1 : TOTAL_PIDS - 1;
					} while (modes[0] == modes[1]);
					break;
				case 2: // down key
					do {
						modes[1] = modes[1] < TOTAL_PIDS - 1 ? modes[1] + 1 : 0;
					} while (modes[0] == modes[1]);
					break;
				}
				updateMode();
			}
		}
	}

	unsigned long curTick = millis();
	if (curTick - lastTick > 500) {
		showData(0);
		showData(1);
		if (obd.errors > 10) {
			setupConnection();
		}
		lastTick = curTick;
	}
}

