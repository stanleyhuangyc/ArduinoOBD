MegaLogger
==========

MegaLogger is a sketch developed for Arduino Telematics Kits based on Arduino MEGA or MEGA ADK. The sketch features OBD-II, GPS and motion sensor data access, data logging, data streaming and data display.

The sketch utilize following custom Arduino libraries:

 OBD-II Library
 TinyGPS Library
 MPU6050 Library
 MultiLCD Library

Configuration
-------------

MegaLogger has several configurable options all stay in the config.h file. Major options include:

 OBD_MODEL - defines the model of OBD-II Adapter (UART or I2C)
 USE_GPS - defines whether GPS is used
 USE_MPU6050 - defines whether
 MPU6050 (accelerometer & gyro) is used
 ENABLE_DATA_LOG - toggles data logging on microSD card
 ENABLE_DATA_OUT - toggles data streaming (to mobile devices via Bluetooth)
 STREAM_FORMAT - sets streaming data format (binary or text)

OBD-II
------

The kit connects to the OBD-II port of a motor vehicle via OBD-II Adapter for Arduino. There are two versions of OBD-II Adapter available, UART and I2C. The UART version provides ELM327 command-set via serial UART at baudrate of 38400bps. The I2C version works as an I2C device which is accessed by I2C address for OBD-II data access. It also integrates I2C MEMS sensor which provides accelerometer and gyro. OBD_MODEL define in config.h should be corresponding to the model of the adapter. The OBD-II Adapter should be connected at Serial1 or I2C interface and is access from Arduino by OBD-II library. 

GPS
---
The kit comes with optional GPS receiver which should be connected at Serial2. The incoming NMEA data stream is parsed by TinyGPS library.

Motion Sensor
-------------

Motion sensor (accelerometer & gyro) are embedded in I2C version of OBD-II Adapter. They are accessed via I2C by MPU6050 library.

Data Logging
------------

The kit provides complete data logging for OBD-II, GPS and motion sensor data. The data is stored on microSD card if present. Standard Arduino SD library used for accessing microSD card.

The data is logged in a simple CSV text format.Each line represents a record with time, data type and data value like this:

    [Time Elapsed],[Data Type],[Data Value]

Time Elapsed is the time elapsed in milliseconds since previous data record. Data Type is the OBD-II PID which is defined in the OBD-II library (e.g. 0x10C is engine RPM). Here is an example data clip:

 169,10D,0
 171,111,12
 170,143,21
 165,10B,30
 175,10C,705
 169,10D,0
 170,111,12
 ...

GPS and motion sensor data are defined as special PID which OBD-II standard does not use and are defined in datalogger.h file.

 #define PID_GPS_LATITUDE 0xA
 #define PID_GPS_LONGITUDE 0xB
 #define PID_GPS_ALTITUDE 0xC
 #define PID_GPS_SPEED 0xD
 #define PID_GPS_HEADING 0xE
 #define PID_GPS_SAT_COUNT 0xF
 #define PID_GPS_TIME 0x10
 #define PID_GPS_DATE 0x11
 #define PID_ACC 0x20
 #define PID_GYRO 0x21

Data Streaming
--------------

Live data can be streamed to a mobile device via Bluetooth (BLE or 2.1) module which is optionally connected at Serial3. Two stream data formats are available, text or binary based. Text based format is same as data logging format in form of CSV. The binary format is based on a framed data format and is used by streaming data to iOS device with Freematics OBD App installed.

Data Display
------------

The kit provides a 3.2" color TFT LCD display. The live data will be displayed on the screen. The LCD display is driven by MultiLCD library.