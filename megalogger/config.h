#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* OBD-II Adapter options
**************************************/
#define OBD_ADAPTER_MODEL OBD_MODEL_I2C
//#define OBD_ADAPTER_MODEL OBD_MODEL_UART
#define OBD_PROTOCOL PROTO_AUTO

/**************************************
* Data logging options
**************************************/
// enable(1)/disable(0) data logging (if SD card is present)
#define ENABLE_DATA_LOG 1
#define SD_CS_PIN SS

/**************************************
* Data streaming options
**************************************/
// enable(1)/disable(0) data streaming
#define ENABLE_DATA_OUT 1

// uses software(1)/hardware(0) serial for data streaming
#define USE_SOFTSERIAL 0
#define RF_SERIAL Serial3

// this defines the format of data streaming
// FORMAT_BIN is required by Freematics OBD iOS App
// FORMAT_TEXT for text-based, text names for PID
#define STREAM_FORMAT FORMAT_TEXT

/* Default streaming baudrates:
   9600bps for BLE
   38400bps for BT 2.1
*/
#define STREAM_BAUDRATE 9600

/**************************************
* GPS configuration
**************************************/
#define USE_GPS 1
#define GPSUART Serial2
#define MAX_GPS_PROCESS_TIME 50 /* ms */
#define GPS_DATA_TIMEOUT 2000 /* ms */

// 38400bps for G6010 5Hz GPS receiver
// 115200bps for G7020 10Hz GPS receiver
#define GPS_BAUDRATE 115200 /* bps */

/**************************************
* Accelerometer & Gyro
**************************************/
//#define USE_MPU6050 1
#define USE_MPU9150 1
#define ACC_DATA_RATIO 160
#define GYRO_DATA_RATIO 256
#define COMPASS_DATA_RATIO 8

/**************************************
* Timeout/interval options
**************************************/
#define OBD_MIN_INTERVAL 20 /* ms */
#define ACC_DATA_INTERVAL 200 /* ms */

/**************************************
* LCD module (uncomment only one)
**************************************/
LCD_SSD1289 lcd; /* 3.2" SSD12389 based TFT LCD */
//LCD_ILI9325D lcd; /* 2.8" ILI9325 based TFT LCD */
//LCD_ILI9341 lcd; /* 2.4" ILI9341 based SPI TFT LCD */
//LCD_Null lcd;

#endif
