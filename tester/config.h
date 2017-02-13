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
* LCD module (uncomment only one)
**************************************/
LCD_ILI9341 lcd; /* 2.2" ILI9341 based SPI TFT LCD */
//LCD_SSD1289 lcd; /* 3.2" SSD12389 based TFT LCD */
//LCD_Null lcd;

/**************************************
* Benchmark option
**************************************/
#define OBD_BENCHMARK_TIME 5 /* seconds */
#define MEMS_BENCHMARK_TIME 3 /* seconds */

/**************************************
* Accelerometer & Gyro
**************************************/
#define ACC_DATA_RATIO 172
#define GYRO_DATA_RATIO 256

/**************************************
* Data streaming options
**************************************/
// enable(1)/disable(0) data streaming
#define ENABLE_DATA_OUT 0
#define ENABLE_DATA_LOG 0
#define SD_CS_PIN 10

#define DELAY_AFTER_SENDING 10

// uses software(1)/hardware(0) serial for data streaming
#define USE_SOFTSERIAL 0

// followings define the format of data streaming, enable one of them only
// FORMAT_CSV is for CSV based text output
//#define STREAM_FORMAT FORMAT_CSV
// FORMAT_LINE is for readable text output
#define STREAM_FORMAT FORMAT_TEXT

#define STREAM_BAUDRATE 115200

#endif
