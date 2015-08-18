/*************************************************************************
* Reference code for generic GPS data logger
* Works with Freematics GPS Data Logger Kit
* Visit http://freematics.com for more information
* Distributed under GPL v2.0
* Written by Stanley Huang
*************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include "config.h"
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "datalogger.h"

//bool acc = false;

#if USE_MPU6050
#include <Wire.h>
#include <MPU6050.h>
#endif

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define GPSUART Serial2
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega32U4__)
#define GPSUART Serial1
#else
#define GPSUART Serial
#endif

TinyGPS gps;

CDataLogger logger;

#if USE_MPU6050
bool initACC()
{
  if (MPU6050_init() != 0)
    return false;
  return true;

}
void processACC()
{
  accel_t_gyro_union data;
  MPU6050_readout(&data);
  logger.dataTime = millis();
  // log x/y/z of accelerometer
  logger.logData(PID_ACC, data.value.x_accel, data.value.y_accel, data.value.z_accel);
  delay(10);
  // log x/y/z of gyro meter
  logger.logData(PID_GYRO, data.value.x_gyro, data.value.y_gyro, data.value.z_gyro);
}
#endif

void processGPS()
{
  long lat, lon;
  int speed = 0;
  int16_t alt = 0;
  uint32_t date, time;

  logger.dataTime = millis();

  gps.get_datetime(&date, &time, 0);
  logger.logData(PID_GPS_TIME, (int32_t)time);
  delay(10);

  speed = (int)(gps.speed() * 1852 / 100000);
  logger.logData(PID_GPS_SPEED, speed);
  delay(10);

  //logger.logData(PID_GPS_SAT_COUNT, (int)gps.satellites());

  alt = gps.altitude() / 100;
  if (alt > -10000 && alt < 10000) {
    logger.logData(PID_GPS_ALTITUDE, alt);
    delay(10);
  }

  gps.get_position(&lat, &lon, 0);
  logger.logData(PID_GPS_LATITUDE, lat);
  delay(10);
  logger.logData(PID_GPS_LONGITUDE, lon);
}

bool CheckSD()
{
  Sd2Card card;
  //SdVolume volume;

  pinMode(SS, OUTPUT);
  if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
    const char* type;
    char buf[20];

    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        type = "SD1";
        break;
      case SD_CARD_TYPE_SD2:
        type = "SD2";
        break;
      case SD_CARD_TYPE_SDHC:
        type = "SDHC";
        break;
      default:
        type = "SDx";
    }
    SerialRF.print(type);
    SerialRF.print(' ');
  } else {
    SerialRF.println("No SD");
    return false;
  }

  if (!SD.begin(SD_CS_PIN)) {
    SerialRF.println("Bad SD");
    return false;
  }
  return true;
}

void setup()
{
  logger.initSender();
  SerialRF.println("Init...");

#if ENABLE_DATA_LOG
  if (CheckSD()) {
    int index = logger.openFile();
    SerialRF.print("File: ");
    SerialRF.println(index);
  }
#endif // ENABLE_DATA_LOG

#if USE_MPU6050
  Wire.begin();
  acc = initACC();

  SerialRF.print("ACC:");
  SerialRF.println(acc ? "YES" : "NO");
#endif

  GPSUART.begin(GPS_BAUDRATE);
  delay(500);

  if (GPSUART.available()) {
    // init GPS
    SerialRF.println("Searching...");
    for (;;) {
      if (!GPSUART.available()) continue;
      char c = GPSUART.read();
      if (!gps.encode(c)) break;
    }
  } else {
    SerialRF.println("No GPS");
  }
}

void loop()
{
  static uint32_t lastTime = 0;

  if (GPSUART.available()) {
    char c = GPSUART.read();
    if (gps.encode(c)) {
      processGPS();
    }
  }

#if ENABLE_DATA_LOG
  // flush file every 3 seconds
  if (logger.dataTime - lastTime >= 3000) {
    lastTime = logger.dataTime;
    logger.flushFile();
  }
#endif

#if USE_MPU6050
  processACC();
#endif
}
