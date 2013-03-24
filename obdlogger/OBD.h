/*************************************************************************
* OBD-II (ELM327) data accessing library for Arduino
* Distributed under GPL v2.0
* Copyright (c) 2012 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#define OBD_TIMEOUT_SHORT 2000 /* ms */
#define OBD_TIMEOUT_LONG 7000 /* ms */
#define OBD_TIMEOUT_INIT 3000 /* ms */
#define OBD_SERIAL_BAUDRATE 38400
#define OBD_RECV_BUF_SIZE 48

#ifndef OBDUART
#ifdef __AVR_ATmega32U4__ /* for Leonardo */
#define OBDUART Serial1
#else
#define OBDUART Serial
#endif
#endif

// mode 0 pids
#define PID_RPM 0x0C
#define PID_SPEED 0x0D
#define PID_THROTTLE 0x11
#define PID_ENGINE_LOAD 0x04
#define PID_COOLANT_TEMP 0x05
#define PID_INTAKE_TEMP 0x0F
#define PID_MAF_FLOW 0x10
#define PID_ABS_ENGINE_LOAD 0x43
#define PID_AMBIENT_TEMP 0x46
#define PID_FUEL_PRESSURE 0x0A
#define PID_INTAKE_PRESSURE 0x0B
#define PID_BAROMETRIC 0x33
#define PID_TIMING_ADVANCE 0x0E
#define PID_FUEL_LEVEL 0x2F
#define PID_RUNTIME 0x1F
#define PID_DISTANCE 0x31

unsigned int hex2uint16(const char *p);
unsigned char hex2uint8(const char *p);

class COBD
{
public:
    COBD()
    {
        dataMode = 1;
        errors = 0;
    }
	bool Init(bool passive = false);
	bool ReadSensor(byte pid, int& result, bool passive = false);
	void Sleep(int seconds);
	// Query and GetResponse for advanced usage only
	void Query(byte pid);
	virtual char* GetResponse(byte pid, char* buffer);
	virtual bool GetResponse(byte pid, int& result);
	virtual bool GetResponsePassive(byte& pid, int& result);
	virtual bool DataAvailable();
	byte dataMode;
	byte errors;
	//char recvBuf[OBD_RECV_BUF_SIZE];
protected:
    static bool GetParsedData(byte pid, char* data, int& result);
	static int GetPercentageValue(char* data)
	{
		return (int)hex2uint8(data) * 100 / 255;
	}
	static int GetLargeValue(char* data)
	{
		return hex2uint16(data);
	}
	static int GetSmallValue(char* data)
	{
		return hex2uint8(data);
	}
	static int GetTemperatureValue(char* data)
	{
		return (int)hex2uint8(data) - 40;
	}
	virtual char ReadData();
	virtual void WriteData(const char* s);
	virtual void WriteData(const char c);
};
