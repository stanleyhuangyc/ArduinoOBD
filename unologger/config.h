#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Choose model of OBD-II Adapter
**************************************/
// OBD_MODEL_I2C for I2C version
// OBD_MODEL_UART for UART version
#define OBD_MODEL OBD_MODEL_UART
#define OBD_PROTOCOL PROTO_AUTO

/**************************************
* Data logging options
**************************************/
// enable(1)/disable(0) data logging (if SD card is present)
#define ENABLE_DATA_LOG 1
#define SD_CS_PIN 10

/**************************************
* Data streaming options
**************************************/
// data streaming is not supported on Arduino UNO
#define ENABLE_DATA_OUT 0

/**************************************
* Accelerometer & Gyro
**************************************/
//#define USE_MPU6050 1
//#define USE_MPU9150 1

/**************************************
* Timeout/interval options
**************************************/
#define OBD_MIN_INTERVAL 20 /* ms */

/**************************************
* LCD module (uncomment only one)
**************************************/
LCD_ILI9341 lcd; /* 2.4" ILI9341 based SPI TFT LCD */
//LCD_Null lcd;

#endif
