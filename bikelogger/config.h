#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#define ENABLE_DATA_OUT 1
#define ENABLE_DATA_LOG 1
#define USE_SOFTSERIAL 0
//this defines the format of log file
#define LOG_FORMAT FORMAT_CSV
#define STREAM_FORMAT FORMAT_CSV
#define STREAM_BAUDRATE 115200

/**************************************
* Choose SD pin here
**************************************/
#define SD_CS_PIN SS // generic
//#define SD_CS_PIN 4 // ethernet shield
//#define SD_CS_PIN 7 // microduino
//#define SD_CS_PIN 10 // SD breakout

/**************************************
* Choose LCD model here
**************************************/
LCD_SSD1289 lcd;
//LCD_ILI9341 lcd;
//LCD_Null lcd;

/**************************************
* Other options
**************************************/
#define USE_MPU6050 0
#define GPS_BAUDRATE 38400
#define GPS_DATA_TIMEOUT 2000 /* ms */
//#define DEBUG Serial
#define DEBUG_BAUDRATE 9600

#endif // CONFIG_H_INCLUDED
