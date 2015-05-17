#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Choose model of OBD-II Adapter
**************************************/
// OBD_MODEL_I2C for I2C version
// OBD_MODEL_UART for UART version
#define OBD_MODEL OBD_MODEL_I2C
#define OBD_PROTOCOL PROTO_AUTO

/**************************************
* Accelerometer & Gyro
**************************************/
//#define USE_MPU6050 1
#define USE_MPU9150 1

/**************************************
* LCD module (uncomment only one)
**************************************/
LCD_ILI9341 lcd; /* 2.4" ILI9341 based SPI TFT LCD */
//LCD_Null lcd;

/**************************************
* Data streaming options
**************************************/
// enable(1)/disable(0) data streaming
#define ENABLE_DATA_OUT 1

// uses software(1)/hardware(0) serial for data streaming
#define USE_SOFTSERIAL 0

// followings define the format of data streaming, enable one of them only
// FORMAT_BIN is required by Freematics OBD iOS App
//#define STREAM_FORMAT FORMAT_BIN
// FORMAT_CSV is for CSV based text output
//#define STREAM_FORMAT FORMAT_CSV
// FORMAT_LINE is for readable text output
#define STREAM_FORMAT FORMAT_TEXT

#define STREAM_BAUDRATE 115200

#endif
