#include <Arduino.h>
#include <Wire.h>
#include "MultiLCD.h"
#include "OBD.h"
#include "MPU6050.h"

#define INIT_CMD_COUNT 8
#define MAX_CMD_LEN 6

const char initcmd[INIT_CMD_COUNT][MAX_CMD_LEN] = {"ATZ\r","ATE0\r","ATL1\r","0100\r","0120\r","0140\r","0145\r"};

//SoftwareSerial softSerial(2, 3); // RX, TX

unsigned int  adc_key_val[5] ={30, 150, 360, 535, 760 };
int NUM_KEYS = 5;
int adc_key_in;
char key=-1;
char oldkey=-1;

byte index = 0;
uint16_t pid = 0x0145;
int stateMPU6050;

//create object to control an LCD.
LCD_OLED lcd;

class COBDTester : public COBD
{
public:
    bool Init(bool passive = false)
    {
        unsigned long currentMillis;
        unsigned char n;
        char prompted;
        char buffer[OBD_RECV_BUF_SIZE];

        for (unsigned char i = 0; i < INIT_CMD_COUNT; i++) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(initcmd[i]);
            lcd.setCursor(0, 1);
            WriteData(initcmd[i]);
            n = 0;
            prompted = 0;
            currentMillis = millis();
            for (;;) {
                if (DataAvailable()) {
                    char c = ReadData();
                    if (c == '>') {
                        buffer[n] = 0;
                        prompted++;
                    } else if (n < OBD_RECV_BUF_SIZE - 1) {
                        buffer[n++] = c;

                        if (c == '\r' || c == '\n')
                            lcd.setCursor(0, 1);
                        else
                            lcd.write(c);
                    }
                } else if (prompted) {
                    break;
                } else {
                    unsigned long elapsed = millis() - currentMillis;
                    if (elapsed > OBD_TIMEOUT_INIT) {
                        // init timeout
                        //WriteData("\r");
                        return false;
                    }
                }
            }
            delay(1000);
        }
        errors = 0;
        return true;
    }
};

COBDTester obd;

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

void readMPU6050()
{
  int error;
  float dT;
  accel_t_gyro_union accel_t_gyro;

  // Read the raw values.
  // Read 14 bytes at once,
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.

  lcd.setCursor(0, 1);
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  if (error != 0) {
    lcd.print("ACC Error!");
    return;
  }


  // Swap all high and low bytes.
  // After this, the registers values are swapped,
  // so the structure name like x_accel_l does no
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);


  // Print the raw acceleration values

  char buf[20];
  dT = ( (float) accel_t_gyro.value.temperature + 12412.0) / 340.0;
  sprintf(buf, "%d %d %d %d    ",
    (int)dT,
    accel_t_gyro.value.x_accel / 128,
    accel_t_gyro.value.y_accel / 128,
    accel_t_gyro.value.z_accel / 128);
  //Serial.println(buf);
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

void query()
{
    char buf[17];

    switch (index) {
    case 0:
        sprintf(buf, "[%04X]", pid);
        break;
    case 1:
        sprintf(buf, "%03X[%01X]", pid >> 4, pid & 0xf);
        break;
    case 2:
        sprintf(buf, "%02X[%01X]%01X", pid >> 8, (pid >> 4) & 0xf, pid & 0xf);
        break;
    case 3:
        sprintf(buf, "%01X[%01X]%02X", pid >> 12, (pid >> 8) & 0xf, pid & 0xff);
        break;
    case 4:
        sprintf(buf, "[%01X]%03X", pid >> 12, pid & 0xfff);
        break;
    }

    lcd.setCursor(0, 0);
    lcd.print(buf);
    lcd.setCursor(6, 0);

    obd.dataMode = (byte)(pid >> 8);
    obd.Query((byte)pid);

    if (stateMPU6050 == 0) {
      readMPU6050();
    }
}

void setup()
{
    Wire.begin();
    lcd.begin();
    OBDUART.begin(38400);


    lcd.clear();
    lcd.print("Init MPU6050...");
    lcd.setCursor(0, 1);

    stateMPU6050 = MPU6050_init();

    char buf[16];
    if (stateMPU6050 != 0) {
        sprintf(buf, "Error: %d", stateMPU6050);
        lcd.print(buf);
    } else {
       unsigned long t = millis();
      do {
       readMPU6050();
       delay(100);
      } while (millis() - t <= 10000);
    }
    delay(1000);

    do {
        lcd.clear();
        lcd.print("Init OBD...");

      if (stateMPU6050 == 0) {
         readMPU6050();
      }
      delay(500);
    } while(!obd.Init());

    lcd.setCursor(0, 1);
    lcd.print("CONNECTED!   ");
    delay(1000);
    lcd.clear();
    query();
}

void loop()
{
    if (Serial.available()) {
        char c = Serial.read();
        if (c == '\r' || c == '\n') {
            lcd.setCursor(6, 0);
        } else if (c == '>') {
            lcd.setCursor(15, 0);
            lcd.write(c);
            lcd.setCursor(6, 0);
            delay(100);
            query();
        } else {
            lcd.write(c);
            delay(10);
        }
    }

#if 0
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
				case 2: // down key
					switch (index) {
					case 0:
					    pid--;
					    break;
					case 1:
					    pid = (pid & 0xfff0) | (((pid & 0xf) - 1) & 0xf);
					    break;
					case 2:
					    pid = (pid & 0xff0f) | (((pid & 0xf0) - 0x10) & 0xf0);
					    break;
					case 3:
					    pid = (pid & 0xf0ff) | (((pid & 0xf00) - 0x100) & 0xf00);
					    break;
					case 4:
					    pid = (pid & 0x0fff) | (((pid & 0xf000) - 0x1000) & 0xf000);
					    break;
					}
					break;
				case 1: // up key
					switch (index) {
					case 0:
					    pid++;
					    break;
					case 1:
					    pid = (pid & 0xfff0) | (((pid & 0xf) + 1) & 0xf);
					    break;
					case 2:
					    pid = (pid & 0xff0f) | (((pid & 0xf0) + 0x10) & 0xf0);
					    break;
					case 3:
					    pid = (pid & 0xf0ff) | (((pid & 0xf00) + 0x100) & 0xf00);
					    break;
					case 4:
					    pid = (pid & 0x0fff) | (((pid & 0xf000) + 0x1000) & 0xf000);
					}
					break;
				case 0: // right key
					if (index > 0) index--;
					break;
				case 3: // left key
					if (index < 4) index++;
					break;
				}
				lcd.clear();
				query();
			}
		}
	}
#endif
}
