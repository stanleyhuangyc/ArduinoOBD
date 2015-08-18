#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#define ENABLE_DATA_OUT 1
#define ENABLE_DATA_LOG 1
#define USE_SOFTSERIAL 1
//this defines the format of log file
#define STREAM_FORMAT FORMAT_TEXT
#define STREAM_BAUDRATE 9600

/**************************************
* Choose SD pin here
**************************************/
#define SD_CS_PIN SS

/**************************************
* Other options
**************************************/
#define USE_MPU6050 0
#define GPS_BAUDRATE 115200
#define GPS_DATA_TIMEOUT 2000 /* ms */
//#define DEBUG Serial
#define DEBUG_BAUDRATE 9600

#endif // CONFIG_H_INCLUDED
