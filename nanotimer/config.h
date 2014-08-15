#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#define ENABLE_DATA_OUT 0
#define ENABLE_DATA_LOG 0
#define USE_SOFTSERIAL 0
//this defines the format of log file
#define LOG_FORMAT FORMAT_CSV

/**************************************
* Choose SD pin here
**************************************/
#define SD_CS_PIN 10

/**************************************
* Choose LCD model here
**************************************/
//LCD_SSD1306 lcd;
LCD_SH1106 lcd;
//LCD_Null lcd;

/**************************************
* Other options
**************************************/
//#define OBD_MIN_INTERVAL 50 /* ms */
//#define DEBUG Serial
#define DEBUG_BAUDRATE 9600

#endif // CONFIG_H_INCLUDED
