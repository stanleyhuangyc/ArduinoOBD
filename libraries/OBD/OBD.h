/*************************************************************************
* Arduino Library for OBD-II UART/I2C Adapter
* Distributed under BSD License
* Visit http://freematics.com for more information
* (C)2012-2016 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>

#define OBD_MODEL_UART 0
#define OBD_MODEL_I2C 1

#define OBD_TIMEOUT_SHORT 1000 /* ms */
#define OBD_TIMEOUT_LONG 5000 /* ms */
#define OBD_TIMEOUT_GPS 200 /* ms */

#ifndef OBDUART
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168P__)
#define OBDUART Serial
#else
#define OBDUART Serial1
#endif
#endif

// Mode 1 PIDs
#define PID_ENGINE_LOAD 0x04
#define PID_COOLANT_TEMP 0x05
#define PID_SHORT_TERM_FUEL_TRIM_1 0x06
#define PID_LONG_TERM_FUEL_TRIM_1 0x07
#define PID_SHORT_TERM_FUEL_TRIM_2 0x08
#define PID_LONG_TERM_FUEL_TRIM_2 0x09
#define PID_FUEL_PRESSURE 0x0A
#define PID_INTAKE_MAP 0x0B
#define PID_RPM 0x0C
#define PID_SPEED 0x0D
#define PID_TIMING_ADVANCE 0x0E
#define PID_INTAKE_TEMP 0x0F
#define PID_MAF_FLOW 0x10
#define PID_THROTTLE 0x11
#define PID_AUX_INPUT 0x1E
#define PID_RUNTIME 0x1F
#define PID_DISTANCE_WITH_MIL 0x21
#define PID_COMMANDED_EGR 0x2C
#define PID_EGR_ERROR 0x2D
#define PID_COMMANDED_EVAPORATIVE_PURGE 0x2E
#define PID_FUEL_LEVEL 0x2F
#define PID_WARMS_UPS 0x30
#define PID_DISTANCE 0x31
#define PID_EVAP_SYS_VAPOR_PRESSURE 0x32
#define PID_BAROMETRIC 0x33
#define PID_CATALYST_TEMP_B1S1 0x3C
#define PID_CATALYST_TEMP_B2S1 0x3D
#define PID_CATALYST_TEMP_B1S2 0x3E
#define PID_CATALYST_TEMP_B2S2 0x3F
#define PID_CONTROL_MODULE_VOLTAGE 0x42
#define PID_ABSOLUTE_ENGINE_LOAD 0x43
#define PID_AIR_FUEL_EQUIV_RATIO 0x44
#define PID_RELATIVE_THROTTLE_POS 0x45
#define PID_AMBIENT_TEMP 0x46
#define PID_ABSOLUTE_THROTTLE_POS_B 0x47
#define PID_ABSOLUTE_THROTTLE_POS_C 0x48
#define PID_ACC_PEDAL_POS_D 0x49
#define PID_ACC_PEDAL_POS_E 0x4A
#define PID_ACC_PEDAL_POS_F 0x4B
#define PID_COMMANDED_THROTTLE_ACTUATOR 0x4C
#define PID_TIME_WITH_MIL 0x4D
#define PID_TIME_SINCE_CODES_CLEARED 0x4E
#define PID_ETHANOL_FUEL 0x52
#define PID_FUEL_RAIL_PRESSURE 0x59
#define PID_HYBRID_BATTERY_PERCENTAGE 0x5B
#define PID_ENGINE_OIL_TEMP 0x5C
#define PID_FUEL_INJECTION_TIMING 0x5D
#define PID_ENGINE_FUEL_RATE 0x5E
#define PID_ENGINE_TORQUE_DEMANDED 0x61
#define PID_ENGINE_TORQUE_PERCENTAGE 0x62
#define PID_ENGINE_REF_TORQUE 0x63

// non-OBD/custom PIDs (no mode number)
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
#define PID_COMPASS 0x22
#define PID_MEMS_TEMP 0x23
#define PID_BATTERY_VOLTAGE 0x24

// custom PIDs for calculated data
#define PID_TRIP_DISTANCE 0x30

typedef enum {
    PROTO_AUTO = 0,
    PROTO_ISO_9141_2 = 3,
    PROTO_KWP2000_5KBPS = 4,
    PROTO_KWP2000_FAST = 5,
    PROTO_CAN_11B_500K = 6,
    PROTO_CAN_29B_500K = 7,
    PROTO_CAN_29B_250K = 8,
    PROTO_CAN_11B_250K = 9,
} OBD_PROTOCOLS;

// states
typedef enum {
    OBD_DISCONNECTED = 0,
    OBD_CONNECTING = 1,
    OBD_CONNECTED = 2,
	OBD_FAILED = 3
} OBD_STATES;

uint16_t hex2uint16(const char *p);
uint8_t hex2uint8(const char *p);

class COBD
{
public:
	COBD():dataMode(1),errors(0),m_state(OBD_DISCONNECTED) {}
	// begin serial UART, return the version number on success and 0 on failure
	virtual byte begin();
	// initialize OBD-II connection
	virtual bool init(OBD_PROTOCOLS protocol = PROTO_AUTO);
	// un-initialize OBD-II connection
	virtual void end();
	// set serial baud rate
	virtual bool setBaudRate(unsigned long baudrate);
	// get connection state
	virtual OBD_STATES getState() { return m_state; }
	// read specified OBD-II PID value
	virtual bool readPID(byte pid, int& result);
	// read multiple (up to 8) OBD-II PID values, return number of values obtained
	virtual byte readPID(const byte pid[], byte count, int result[]);
	// set device into low power mode
	virtual void enterLowPowerMode();
	// wake up device from low power mode
	virtual void leaveLowPowerMode();
	// send AT command and receive response
	virtual byte sendCommand(const char* cmd, char* buf, byte bufsize, int timeout = OBD_TIMEOUT_LONG);
	// read diagnostic trouble codes (return number of DTCs read)
	virtual byte readDTC(uint16_t codes[], byte maxCodes = 1);
	// clear diagnostic trouble code
	virtual void clearDTC();
	// get battery voltage (works without ECU)
	virtual float getVoltage();
	// get VIN as a string, buffer length should be >= OBD_RECV_BUF_SIZE
	virtual bool getVIN(char* buffer, byte bufsize);
	// initialize MEMS sensor
	virtual bool memsInit();
	// read out MEMS data (acc for accelerometer, gyr for gyroscope, temp in 0.1 celcius degree)
	virtual bool memsRead(int16_t* acc, int16_t* gyr = 0, int16_t* mag = 0, int16_t* temp = 0);
	// send query for specified PID
	virtual void sendQuery(byte pid);
	// retrive and parse the response of specifie PID
	virtual bool getResult(byte& pid, int& result);
	// determine if the PID is supported
	virtual bool isValidPID(byte pid);
	// get adapter firmware version
	virtual byte getVersion();
	// set current PID mode
	byte dataMode;
	// number of subsequent errors
	byte errors;
	// bit map of supported PIDs
	byte pidmap[4 * 4];
protected:
	virtual char* getResponse(byte& pid, char* buffer, byte bufsize);
	virtual byte receive(char* buffer, byte bufsize, int timeout = OBD_TIMEOUT_SHORT);
	virtual void write(const char* s);
	virtual void dataIdleLoop() {}
	void recover();
	void debugOutput(const char* s);
	int normalizeData(byte pid, char* data);
	OBD_STATES m_state;
private:
	virtual uint8_t getPercentageValue(char* data)
	{
		return (uint16_t)hex2uint8(data) * 100 / 255;
	}
	virtual uint16_t getLargeValue(char* data)
	{
		return hex2uint16(data);
	}
	virtual uint8_t getSmallValue(char* data)
	{
		return hex2uint8(data);
	}
	virtual int16_t getTemperatureValue(char* data)
	{
		return (int)hex2uint8(data) - 40;
	}
	char* getResultValue(char* buf);
};

#define I2C_ADDR 0x62

#define MAX_PAYLOAD_SIZE 32
#define MAX_PIDS 8

#define CMD_QUERY_STATUS 0x10
#define CMD_SEND_AT_COMMAND 0x11
#define CMD_APPLY_OBD_PIDS 0x12
#define CMD_LOAD_OBD_DATA 0x13
#define CMD_GPS_SETUP 0x14
#define CMD_GPS_QUERY 0x15

typedef struct {
    uint16_t age;
    uint16_t value;
} PID_INFO;

typedef struct {
    uint16_t time;
    uint8_t message;
    uint8_t data;
} COMMAND_BLOCK;

class COBDI2C : public COBD {
public:
	byte begin();
	void end();
	void write(const char* s);
	// API not applicable
	bool setBaudRate(unsigned long baudrate) { return false; }
	// Asynchronized access API
	void setQueryPID(byte pid, byte obdPid[]);
	void applyQueryPIDs(byte obdPid[]);
	void loadQueryData(PID_INFO obdInfo[]);
	// initialize MEMS sensor
	bool memsInit();
	// read out MEMS sensor data (acc for accelerometer, gyr for gyroscope, temp in 0.1 celcius degree)
	bool memsRead(int* acc, int* gyr = 0, int* mag = 0, int* temp = 0);
protected:
	byte receive(char* buffer, byte bufsize, int timeout = OBD_TIMEOUT_SHORT);
	bool sendCommandBlock(byte cmd, uint8_t data = 0, byte* payload = 0, byte payloadBytes = 0);
private:
	bool MPU6050_read(int start, uint8_t* buffer, int size);
	bool MPU6050_write(int start, const uint8_t* pData, int size);
	bool MPU6050_write_reg(int reg, uint8_t data);
	void MPU6050_store(int* pData, uint8_t data_l, uint8_t data_h);
};
