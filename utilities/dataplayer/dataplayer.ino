#include <Arduino.h>
#include <SPI.h>
#include <MultiLCD.h>
#include <Wire.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <OBD.h>
#include "config.h"
#include "datalogger.h"

CDataLogger sender;

#define LOG_DATA_LEN sizeof(LOG_DATA)
#define REPLAY_SPEED 1

///////////////////////////////////////////////////////////////////////////
#define PID_GPS_LATITUDE 0xF00A
#define PID_GPS_LONGITUDE 0xF00B
#define PID_GPS_ALTITUDE 0xF00C
#define PID_GPS_SPEED 0xF00D

void sendLogFile(File logfile, byte noWait);
//
//unsigned int hex2uint16(const char *p)
//{
//	char c = *p;
//	unsigned int i = 0;
//	for (char n = 0; c && n < 4; c = *(++p)) {
//		if (c >= 'A' && c <= 'F') {
//			c -= 7;
//		} else if (c>='a' && c<='f') {
//			c -= 39;
//        } else if (c == ' ') {
//            continue;
//        } else if (c < '0' || c > '9') {
//			break;
//        }
//		i = (i << 4) | (c & 0xF);
//		n++;
//	}
//	return i;
//}

void initScreen()
{
    lcd.clear();
    lcd.backlight(true);
    lcd.setFont(FONT_SIZE_SMALL);
    lcd.setTextColor(RGB16_CYAN);
    lcd.setCursor(4, 0);
    lcd.print("ENGINE RPM");
    lcd.setCursor(104, 0);
    lcd.print("SPEED");
    lcd.setCursor(164, 0);
    lcd.print("ENGINE LOAD");
    lcd.setCursor(248, 0);
    lcd.print("INTAKE TEMP");

    lcd.setCursor(4, 7);
    lcd.print("COOLANT TEMP");
    lcd.setCursor(104, 7);
    lcd.print("DISTANCE");
    lcd.setCursor(164, 7);
    lcd.print("INTAKE MAP");

    lcd.setCursor(260, 7);
    lcd.print("ELAPSED");
    lcd.setCursor(260, 10);
    lcd.print("LOG SIZE");

    lcd.setTextColor(RGB16_YELLOW);
    lcd.setCursor(24, 5);
    lcd.print("rpm");
    lcd.setCursor(110, 5);
    lcd.print("km/h");
    lcd.setCursor(216, 4);
    lcd.print("%");
    lcd.setCursor(304, 4);
    lcd.print("C");
    lcd.setCursor(64, 11);
    lcd.print("C");
    lcd.setCursor(110, 12);
    lcd.print("km");
    lcd.setCursor(200, 12);
    lcd.print("kpa");
    lcd.setCursor(296, 12);
    lcd.print("KB");

    lcd.setTextColor(RGB16_WHITE);


    //lcd.setCursor(0, 5);
    //lcd.print("THR:   %");
    //lcd.setCursor(80, 5);
    //lcd.print("AIR:   C");
}

void showChart(int value)
{
    static uint16_t pos = 0;
    if (value < 500) return;
    byte n = (value - 600) / 30;
    lcd.fill(pos, pos, 239 - n, 239, RGB16_CYAN);
    pos = (pos + 1) % 320;
    lcd.fill(pos, pos, 120, 239);
}

void showData(byte pid, int value)
{
    switch (pid) {
    case PID_RPM:
        lcd.setCursor(0, 2);
        lcd.setFont(FONT_SIZE_XLARGE);
        lcd.printInt((unsigned int)value % 10000, 4);
        showChart(value);
        break;
    case PID_SPEED:
        lcd.setCursor(90, 2);
        lcd.setFont(FONT_SIZE_XLARGE);
        lcd.printInt((unsigned int)value % 1000, 3);
        break;
    case PID_ENGINE_LOAD:
        lcd.setCursor(164, 2);
        lcd.setFont(FONT_SIZE_XLARGE);
        lcd.printInt(value % 100, 3);
        break;
    case PID_INTAKE_TEMP:
        if ((uint16_t)value < 1000) {
            lcd.setCursor(248, 2);
            lcd.setFont(FONT_SIZE_XLARGE);
            lcd.printInt(value, 3);
        }
        break;
    case PID_INTAKE_MAP:
        lcd.setCursor(164, 9);
        lcd.setFont(FONT_SIZE_XLARGE);
        lcd.printInt((uint16_t)value % 1000, 3);
        break;
    case PID_COOLANT_TEMP:
        lcd.setCursor(8, 9);
        lcd.setFont(FONT_SIZE_XLARGE);
        lcd.printInt((uint16_t)value % 1000, 3);
        break;
    case PID_DISTANCE:
        lcd.setFont(FONT_SIZE_XLARGE);
        lcd.setCursor(90, 9);
        lcd.printInt((uint16_t)value % 1000, 3);
        break;
    }
}

///////////////////////////////////////////////////////////////////////////

void setup()
{
    // Open serial communications and wait for port to open:
    Serial.begin(115200);

    lcd.begin();
    lcd.clear();
    sender.initSender();

    // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
    // Note that even if it's not used as the CS pin, the hardware SS pin
    // (10 on Arduino Uno boards, 53 on the Mega) must be left as an output
    // or the SD library functions will not work.
    pinMode(SD_CS_PIN, OUTPUT);

    Sd2Card card;

    if (card.init(SPI_FULL_SPEED, SD_CS_PIN)) {
    SdVolume volume;
#if 0
        char buf[20];
        if (!volume.init(card)) {
            Serial.println("No FAT!");
        } else {
            uint32_t volumesize = volume.blocksPerCluster();
            volumesize >>= 1; // 512 bytes per block
            volumesize *= volume.clusterCount();
            volumesize >>= 10;

            sprintf(buf, "%dGB", (int)((volumesize + 511) / 1000));
            Serial.println(buf);
            lcd.print("SD ");
            lcd.println(buf);
        }
#endif
        if (!SD.begin(SD_CS_PIN)) {
            Serial.println("SD error");
        }
    } else {
        lcd.print("No SD");
        Serial.println("No SD");
    }
}

///////////////////////////////////////////////////////////////////////////
#if 0
void ShowSensorData(uint16_t pid, float data)
{
    uint16_t value;
    Serial.print('[');
    Serial.print(pid, HEX);
    Serial.print("]=");
    if (pid < 0xf000) {
      value = (uint16_t)data;
      Serial.print(value);
    }

    char buf[16];
    switch (pid) {
    case PID_RPM:
        lcd.setCursor(64, 0);
        lcd.setFont(FONT_SIZE_XLARGE);
        lcd.printInt(value, 4);
        break;
    case PID_SPEED:
        lcd.setCursor(0, 0);
        lcd.setFont(FONT_SIZE_XLARGE);
        lcd.printInt(value, 3);
        break;
    case PID_THROTTLE:
        lcd.setCursor(24, 5);
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.printInt(value, 3);
        break;
    case PID_INTAKE_TEMP:
        lcd.setCursor(104, 5);
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.printInt(value, 3);
        break;
    case PID_DISTANCE:
        sprintf(buf, "%5ukm", value);
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(86, 6);
        lcd.print(buf);
        break;
    case PID_GPS_LATITUDE: {
        int32_t lat = (int32_t)(data * 100000);
        sprintf(buf, "LAT:%d.%05ld  ", (int)(lat / 100000), lat % 100000);
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(0, 6);
        lcd.print(buf);
        //Serial.print(buf);
        break;
        }
    case PID_GPS_LONGITUDE: {
        int32_t lon = (int32_t)(data * 100000);
        sprintf(buf, "LON:%d.%05ld  ", (int)(lon / 100000), lon % 100000);
        lcd.setFont(FONT_SIZE_SMALL);
        lcd.setCursor(0, 7);
        lcd.print(buf);
        //Serial.print(buf);
        break;
        }
    }
}
#endif

///////////////////////////////////////////////////////////////////////////
uint8_t getDataInt8(LOG_DATA_COMMAND& msg, int offset = 0)
{
  return msg.data[offset];
}

uint16_t getDataInt16(LOG_DATA_COMMAND& msg, int offset = 0)
{
  return *(uint16_t *)(&msg.data[offset]);
}

uint32_t getDataInt32(LOG_DATA_COMMAND& msg, int offset = 0)
{
  return *(uint32_t *)(&msg.data[offset]);
}

void sendLogFile(File logfile, byte noWait = 0)
{
    uint32_t timestamp = 0;
    uint32_t dataSize = 0;
    int value;
    int pid;
    char buf[64];
    byte n = 0;

    Serial.println("Start");
    while (logfile.available()) {
        byte c = logfile.read();
        dataSize++;
        if (c <= 13) {
            // process one line
            if (n > 0) do {
                buf[n] = 0;
                n = 0;
                // TODO: parse line
                int t = atoi(buf);
                char *p = strchr(buf, ',');
                if (!p++) break;
                Serial.print(p);
                Serial.write('\n');
                int pid = hex2uint16(p);
                p = strchr(p, ',');
                if (!p++) break;
                int value = atoi(p);
                timestamp += t;
                showData((byte)pid, value);
                if (Serial.available()) {
                    lcd.setCursor(0, 20);
                    do {
                        lcd.write(Serial.read());
                    } while (Serial.available());
                }
                if (!noWait && t > 0) delay(t / REPLAY_SPEED);
            } while(0);

            // show time elapsed
            uint16_t elapsed = timestamp / 1000;
            lcd.setFont(FONT_SIZE_MEDIUM);
            lcd.setCursor(260, 8);
            lcd.printInt(elapsed / 60, 2);
            lcd.write(':');
            lcd.setFlags(FLAG_PAD_ZERO);
            lcd.printInt(elapsed % 60, 2);
            lcd.setFlags(0);

            lcd.setCursor(260, 11);
            lcd.printInt(dataSize >> 10, 4);

            continue;
        }
        if (n >= sizeof(buf) - 1) continue;
        buf[n++] = c;

        //Show on LCD:

        //Serial.print(" T:");
        //Serial.print(ld.time);
        //Serial.print(" F:");
        //Serial.print(ld.flags);

        // display KB counter
#if 0
        lcd.setCursor(96, 7);
        lcd.setFont(FONT_SIZE_SMALL);
        byte percent = (byte)(dataSendLen * 100 / logsize);
        lcd.printInt(percent, 3);

        Serial.print(' ');
        Serial.print(percent);
        Serial.println('%');
#endif
        //checkReceiveCommand();
    } while (logfile.available());
}

void enumDirectoryAndSend(File dir)
{
    // Begin at the start of the directory
    dir.rewindDirectory();

    while(true) {
        File entry =  dir.openNextFile();
        if (! entry) {
            // no more files
            //Serial.println("**nomorefiles**");
            break;
        }

        if (entry.isDirectory()) {
            enumDirectoryAndSend(entry);
        } else {
            lcd.clear();
            lcd.setFont(FONT_SIZE_MEDIUM);
            lcd.print(entry.name());
            delay(1000);
            initScreen();
            sendLogFile(entry);
        }
        entry.close();
    }
}

void loop()
{
#if 0
   if (!checkReceiveCommand()) {
        delay(100);
   }
#else
    File root = SD.open("/FRMATICS");
    if (!root || !root.isDirectory()) {
        lcd.println("NO LOG");
        Serial.println("NO LOG");
    } else {
        enumDirectoryAndSend(root);
        root.close();
    }
    delay(5000);
#endif

}
