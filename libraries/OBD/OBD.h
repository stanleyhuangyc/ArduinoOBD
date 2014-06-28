/*************************************************************************
* Arduino Library for OBD-II UART/I2C Adapter
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2012-2014 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>

#define OBD_MODEL_UART 0
#define OBD_MODEL_I2C 1

#define OBD_TIMEOUT_SHORT 2000 /* ms */
#define OBD_TIMEOUT_LONG 7000 /* ms */
#define OBD_SERIAL_BAUDRATE 38400
#define OBD_RECV_BUF_SIZE 128

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
#define PID_FUEL_LEVEL_INPUT 0x2F
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

// states
#define OBD_DISCONNECTED 0
#define OBD_CONNECTING 1
#define OBD_CONNECTED 2

uint16_t hex2uint16(const char *p);
uint8_t hex2uint8(const char *p);

class COBD
{
public:
	COBD():dataMode(1),errors(0),m_state(OBD_DISCONNECTED) {}
	/*
       Serial baudrate is only adjustable for Arduino OBD-II Adapters V2
       Check out http://arduinodev.com/hardware/obd-kit/
	*/
	virtual void begin(unsigned long baudrate = 0);
	virtual bool init(byte protocol = 0);
	virtual void uninit();
	virtual bool read(byte pid, int& result);
	virtual void sleep();
	virtual void wakeup();
	virtual void setProtocol(byte h = -1);
	// Query and GetResponse for advanced usage only
	virtual void sendQuery(byte pid);
	virtual bool getResult(byte& pid, int& result);
	bool isValidPID(byte pid);
	byte getState() { return m_state; }
	byte dataMode;
	byte errors;
	byte pidmap[4 * 4];
	byte vin[17];
protected:
	virtual char* getResponse(byte& pid, char* buffer);
	virtual byte receive(char* buffer = 0, int timeout = OBD_TIMEOUT_SHORT);
	virtual bool available();
	virtual char read();
	virtual void write(const char* s);
	virtual void write(char c);
	virtual void dataIdleLoop() {}
	void recover();
	void debugOutput(const char* s);
	int normalizeData(byte pid, char* data);
	byte m_state;
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
};

#define I2C_ADDR 0x62

#define MAX_PAYLOAD_SIZE 32
#define MAX_PIDS 8

#define CMD_QUERY_STATUS 0x10
#define CMD_SEND_AT_COMMAND 0x11
#define CMD_APPLY_OBD_PIDS 0x12
#define CMD_LOAD_OBD_DATA 0x13
#define CMD_GPS_SETUP 0x20
#define CMD_GPS_QUERY 0x22

typedef struct {
    uint16_t age;
    uint16_t value;
} PID_INFO;

typedef struct {
    uint16_t time;
    uint8_t message;
    uint8_t data;
} COMMAND_BLOCK;

typedef struct {
    uint32_t time;
    uint32_t date;
    float lat;
    float lon;
    float speed;
    float alt;
    uint8_t sat;
    uint8_t state;
    uint16_t age;
    uint8_t reserved[4];
} GPS_DATA;

class COBDI2C : public COBD {
public:
    void begin(byte addr = I2C_ADDR);
    bool init(byte protocol = 0);
    bool read(byte pid, int& result);
    void write(const char* s);
    void setProtocol(bool auto, byte h);
    // Asynchronized access API
    void setPID(byte pid);
    void applyPIDs();
    void loadData();
    uint16_t getData(byte pid, int& result);
    // GPS API
    bool gpsQuery(GPS_DATA* gpsdata);
    void gpsSetup(uint32_t baudrate, const char* cmds = 0);
protected:
    bool sendCommand(byte cmd, uint8_t data = 0, byte* payload = 0, byte payloadBytes = 0);
    byte receive(char* buffer, int timeout = OBD_TIMEOUT_SHORT);
    byte m_addr;
    PID_INFO obdInfo[MAX_PIDS];
    byte obdPid[MAX_PIDS];
};
