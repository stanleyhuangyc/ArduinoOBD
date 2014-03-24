#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

// definitions
#define OBD_MODEL_UART 0
#define OBD_MODEL_I2C 1

// configurations
/**************************************
* Choose model of OBD-II Adapter
**************************************/
#define OBD_MODEL OBD_MODEL_UART
#define OBD_PROTOCOL 0 /* 0 for auto */

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
* Accelerometer & Gyro
**************************************/
#define USE_MPU6050 1

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
