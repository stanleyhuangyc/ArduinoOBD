#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Choose model of OBD-II Adapter
**************************************/
// OBD_MODEL_I2C for I2C version
// OBD_MODEL_UART for UART version
#define OBD_MODEL OBD_MODEL_UART
#define OBD_PROTOCOL 0 /* 0 for auto */

/**************************************
* Data logging options
**************************************/
// enable(1)/disable(0) data logging (if SD card is present)
#define ENABLE_DATA_LOG 1
#define SD_CS_PIN 10

/**************************************
* Data streaming options
**************************************/
// enable(1)/disable(0) data streaming
#define ENABLE_DATA_OUT 0

// uses software(1)/hardware(0) serial for data streaming
#define USE_SOFTSERIAL 0

// this defines the format of data streaming
// FORMAT_BIN is required by Freematics OBD iOS App
#define STREAM_FORMAT FORMAT_CSV

/* Default streaming baudrates:
   9600bps for BLE
   38400bps for BT 2.1
*/
#define STREAM_BAUDRATE 9600

// outputs debug information
#define VERBOSE 0

/**************************************
* Accelerometer & Gyro
**************************************/
#define USE_MPU6050 1
#define ACC_DATA_RATIO 160
#define GYRO_DATA_RATIO 256

/**************************************
* Timeout/interval options
**************************************/
#define OBD_MIN_INTERVAL 20 /* ms */
#define ACC_DATA_INTERVAL 200 /* ms */
#define GPS_DATA_TIMEOUT 2000 /* ms */

/**************************************
* LCD module (uncomment only one)
**************************************/
LCD_ILI9341 lcd; /* 2.4" ILI9341 based SPI TFT LCD */
//LCD_Null lcd;

#endif
