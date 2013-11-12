#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#define ENABLE_DATA_OUT 0
#define ENABLE_DATA_LOG 1
//this defines the format of log file
#define LOG_FORMAT FORMAT_CSV

/**************************************
* Default working mode
**************************************/
#define MODE_DEFAULT MODE_TIMER /* MODE_LOGGER */
//#define MODE_SWITCH_PIN 8

/**************************************
* Choose SD pin here
**************************************/
//#define SD_CS_PIN SS // generic
//#define SD_CS_PIN 4 // ethernet shield
//#define SD_CS_PIN 7 // microduino
#define SD_CS_PIN 10 // SD breakout

/**************************************
* Config GPS here
**************************************/
#define USE_GPS 0
#define GPS_BAUDRATE 38400 /* bps */
//#define GPS_OPEN_BAUDRATE 4800 /* bps */

/**************************************
* Choose LCD model here
**************************************/
LCD_SSD1306 lcd;
//LCD_ZTOLED lcd;

/**************************************
* Other options
**************************************/
#define USE_MPU6050 0
//#define OBD_MIN_INTERVAL 50 /* ms */
#define GPS_DATA_TIMEOUT 2000 /* ms */
//#define DEBUG Serial

#endif // CONFIG_H_INCLUDED
