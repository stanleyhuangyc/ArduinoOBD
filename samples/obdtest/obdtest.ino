#include <Arduino.h>
#include <Wire.h>
#include <MultiLCD.h>
#include <OBD.h>
#include <MPU6050.h>

#define INIT_CMD_COUNT 8
#define MAX_CMD_LEN 6

const char initcmd[INIT_CMD_COUNT][MAX_CMD_LEN] = {"ATZ\r","ATE0\r","ATL1\r","ATI\r","0100\r","0120\r","0140\r","0145\r"};

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
LCD_1602 lcd;

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
            delay(200);
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

    obd.dataMode = (byte)(pid >> 8);
    obd.Query((byte)pid);

    if (stateMPU6050 == 0) {
        accel_t_gyro_union data;
        char buf[20];
        int ret = MPU6050_readout(&data);
        if (ret == 0) {
            sprintf(buf, "%d/%d/%d", data.value.x_accel, data.value.y_accel, data.value.z_accel);
        } else {
            sprintf(buf, "6050 error: %d", ret);
        }
    }
}

void setup()
{
    lcd.begin();
    pinMode(13, OUTPUT);  //we'll use the debug LED to output a heartbeat
    digitalWrite(13, LOW);
    OBDUART.begin(38400);
    digitalWrite(13, HIGH);


    lcd.clear();
    lcd.print("Init MPU6050...");
    lcd.setCursor(0, 1);
    stateMPU6050 = MPU6050_init();
    if (stateMPU6050 == 0) {
        lcd.print("Success!");
    } else {
        char buf[16];
        sprintf(buf, "Error: %d", stateMPU6050);
        lcd.print(buf);
    }
    delay(1000);

    do {
        lcd.clear();
        lcd.print("Init OBD...");
        delay(500);
    } while(!obd.Init());

    char buf[16];
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
            lcd.setCursor(1, 6);
        } else if (c == '>') {
            lcd.setCursor(15, 0);
            lcd.write(c);
            lcd.setCursor(1, 6);
            query();
        } else {
            lcd.write(c);
        }
    }

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
}
