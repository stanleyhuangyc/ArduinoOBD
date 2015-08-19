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
#define GPS_BAUDRATE 115200
#define USE_MPU6050 1

#endif
