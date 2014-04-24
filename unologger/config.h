#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#define OBD_MODEL_UART 0
#define OBD_MODEL_I2C 1

/**************************************
* OBD-II options
**************************************/
#define OBD_MODEL OBD_MODEL_I2C
#define OBD_PROTOCOL 0 /* 0 for auto */

/**************************************
* Data logging/streaming out
**************************************/
#define ENABLE_DATA_OUT 0
#define ENABLE_DATA_LOG 1
#define USE_SOFTSERIAL 0
//this defines the format of log file
#define LOG_FORMAT FORMAT_CSV
#define STREAM_FORMAT FORMAT_CSV
#define STREAM_BAUDRATE 115200

/**************************************
* Default working mode
**************************************/
#define MODE_DEFAULT MODE_LOGGER /* MODE_LOGGER/MODE_TIMER */
//#define MODE_SWITCH_PIN 8

/**************************************
* Choose SD pin here
**************************************/
//#define SD_CS_PIN SS // generic
//#define SD_CS_PIN 4 // ethernet shield
//#define SD_CS_PIN 7 // microduino
#define SD_CS_PIN 10 // SD breakout

/**************************************
* Choose LCD model here
**************************************/
LCD_ILI9341 lcd;
//LCD_SSD1306 lcd;
//LCD_Null lcd;

/**************************************
* Other options
**************************************/
#define USE_MPU6050 0
#define GPS_DATA_TIMEOUT 2000 /* ms */
//#define DEBUG Serial
#define DEBUG_BAUDRATE 9600

#endif // CONFIG_H_INCLUDED
