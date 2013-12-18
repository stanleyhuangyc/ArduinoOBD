#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

// configurations
/**************************************
* Choose SD pin here
**************************************/
//#define SD_CS_PIN 4 // ethernet shield
//#define SD_CS_PIN 7 // microduino
//#define SD_CS_PIN 10 // SD breakout
#define SD_CS_PIN SS

/**************************************
* Config GPS here
**************************************/
#define USE_GPS
#define MAX_GPS_PROCESS_TIME 50 /* ms */
#define GPS_BAUDRATE 38400 /* bps */
//#define GPS_OPEN_BAUDRATE 4800 /* bps */

/**************************************
* Timeout/interval options
**************************************/
//#define OBD_MIN_INTERVAL 200 /* ms */
#define ACC_DATA_INTERVAL 200 /* ms */
#define GPS_DATA_TIMEOUT 2000 /* ms */

/**************************************
* Data logging/streaming options
**************************************/
#define ENABLE_DATA_OUT 0
#define ENABLE_DATA_LOG 1
#define USE_OBD_BT 0
#define LOG_FORMAT FORMAT_CSV /* options: FORMAT_CSV, FORMAT_BIN */

/**************************************
* LCD module
**************************************/
LCD_ILI9325D lcd; /* for ILI9325 based TFT shield */
//LCD_ILI9341 lcd; /* for ILI9341 based SPI TFT */

#endif
