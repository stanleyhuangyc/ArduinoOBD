#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/

// enable(1)/disable(0) data logging (if SD card is present)
#define ENABLE_DATA_LOG 1

// enable(1)/disable(0) data streaming
#define ENABLE_DATA_OUT 1

#define ENABLE_DATA_CACHE 0
#define MAX_CACHE_SIZE 256  /* bytes */

#define STREAM_FORMAT FORMAT_TEXT
//#define STREAM_FORMAT FORMAT_BIN

// serial baudrate for data out streaming
#define STREAM_BAUDRATE 115200

// maximum size per file, a new file will be created on reaching this size
#define MAX_LOG_FILE_SIZE 1024 /* KB */

/**************************************
* Hardware setup
**************************************/

// max allowed time for connecting OBD-II (0 for forever)
#define OBD_ATTEMPT_TIME 0 /* seconds */

// SD pin
#define SD_CS_PIN 10

// enable(1)/disable(0) MEMS sensor
#define USE_MPU6050 1
#define ACC_DATA_RATIO 172
#define GYRO_DATA_RATIO 256
#define COMPASS_DATA_RATIO 8

// enable(1)/disable(0) GPS module
#define USE_GPS 0

// GPS parameters
#define GPS_SERIAL_BAUDRATE 115200L

// motion detection
#define START_MOTION_THRESHOLD 10000 /* for wakeup on movement */

#endif // CONFIG_H_INCLUDED
