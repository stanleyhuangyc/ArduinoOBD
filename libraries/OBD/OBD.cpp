/*************************************************************************
* Arduino Library for OBD-II UART/I2C Adapter
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2012-2014 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>
#include "OBD.h"

//#define DEBUG Serial
//#define REDIRECT Serial

uint16_t hex2uint16(const char *p)
{
	char c = *p;
	uint16_t i = 0;
	for (char n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		} else if (c>='a' && c<='f') {
			c -= 39;
        } else if (c == ' ') {
            continue;
        } else if (c < '0' || c > '9') {
			break;
        }
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

byte hex2uint8(const char *p)
{
	byte c1 = *p;
	byte c2 = *(p + 1);
	if (c1 >= 'A' && c1 <= 'F')
		c1 -= 7;
	else if (c1 >='a' && c1 <= 'f')
	    c1 -= 39;
	else if (c1 < '0' || c1 > '9')
		return 0;

	if (c2 >= 'A' && c2 <= 'F')
		c2 -= 7;
	else if (c2 >= 'a' && c2 <= 'f')
	    c2 -= 39;
	else if (c2 < '0' || c2 > '9')
		return 0;

	return c1 << 4 | (c2 & 0xf);
}

/*************************************************************************
* OBD-II UART Adapter
*************************************************************************/
#include <Wire.h>

void COBD::sendQuery(byte pid)
{
	char cmd[8];
	sprintf(cmd, "%02X%02X\r", dataMode, pid);
#ifdef DEBUG
	debugOutput(cmd);
#endif
	write(cmd);
}

bool COBD::read(byte pid, int& result)
{
	// send a query command
	sendQuery(pid);
	// receive and parse the response
	return getResult(pid, result);
}

bool COBD::available()
{
	return OBDUART.available();
}

char COBD::read()
{
	char c = OBDUART.read();
#ifdef REDIRECT
    REDIRECT.write(c);
#endif
	return c;
}

void COBD::write(const char* s)
{
	OBDUART.write(s);
}

void COBD::write(char c)
{
	OBDUART.write(c);
}

int COBD::normalizeData(byte pid, char* data)
{
	int result;
	switch (pid) {
	case PID_RPM:
		result = getLargeValue(data) >> 2;
		break;
	case PID_FUEL_PRESSURE:
		result = getSmallValue(data) * 3;
		break;
	case PID_COOLANT_TEMP:
	case PID_INTAKE_TEMP:
	case PID_AMBIENT_TEMP:
	case PID_ENGINE_OIL_TEMP:
		result = getTemperatureValue(data);
		break;
	case PID_THROTTLE:
	case PID_ENGINE_LOAD:
	case PID_FUEL_LEVEL:
	case PID_ABSOLUTE_ENGINE_LOAD:
	case PID_ETHANOL_PERCENTAGE:
	case PID_HYBRID_BATTERY_PERCENTAGE:
		result = getPercentageValue(data);
		break;
	case PID_MAF_FLOW:
		result = getLargeValue(data) / 100;
		break;
	case PID_TIMING_ADVANCE:
		result = (int)(getSmallValue(data) / 2) - 64;
		break;
	case PID_DISTANCE: // km
	case PID_RUNTIME: // second
	case PID_FUEL_RAIL_PRESSURE: // kPa
	case PID_ENGINE_REF_TORQUE: // Nm
		result = getLargeValue(data);
		break;
	case PID_CONTROL_MODULE_VOLTAGE: // V
		result = getLargeValue(data) / 1000;
		break;
	case PID_ENGINE_FUEL_RATE: // L/h
		result = getLargeValue(data) / 20;
		break;
	case PID_ENGINE_TORQUE_PERCENTAGE: // %
		result = (int)getSmallValue(data) - 125;
		break;
	default:
		result = getSmallValue(data);
	}
	return result;
}

char* COBD::getResponse(byte& pid, char* buffer)
{
	while (receive(buffer, OBD_TIMEOUT_SHORT) > 0) {
		char *p = buffer;
		while ((p = strstr(p, "41 "))) {
		    p += 3;
		    byte curpid = hex2uint8(p);
		    if (pid == 0) pid = curpid;
		    if (curpid == pid) {
		        errors = 0;
		        p += 2;
		        if (*p == ' ')
		            return p + 1;
		    }
		}
	}
	return 0;
}

bool COBD::getResult(byte& pid, int& result)
{
	char buffer[OBD_RECV_BUF_SIZE];
	char* data = getResponse(pid, buffer);
	if (!data) {
		recover();
		errors++;
		return false;
	}
	result = normalizeData(pid, data);
	return true;
}

void COBD::setProtocol(byte h)
{
	if (h == -1) {
		write("ATSP00\r");
	} else {
		char cmd[8];
		sprintf(cmd, "ATSP%d\r", h);
		write(cmd);
	}
}

void COBD::sleep()
{
	write("ATLP\r");
}

void COBD::wakeup()
{
	write('\r');
}

bool COBD::isValidPID(byte pid)
{
	if (pid >= 0x7f)
		return true;
	pid--;
	byte i = pid >> 3;
	byte b = 0x80 >> (pid & 0x7);
	return pidmap[i] & b;
}

void COBD::begin(unsigned long baudrate)
{
	OBDUART.begin(OBD_SERIAL_BAUDRATE);
	if (baudrate) {
        OBDUART.print("ATBR1 ");
        OBDUART.print(baudrate);
        OBDUART.print('\r');
        OBDUART.end();
        delay(100);
        OBDUART.begin(baudrate);
	}
}

byte COBD::receive(char* buffer, int timeout)
{
	unsigned char n = 0;
	for (unsigned long startTime = millis();;) {
	    if (available()) {
	        char c = read();
	        if (n > 2 && c == '>') {
	            // prompt char received
	            break;
	        } else if (!buffer) {
                n++;
	        } else if (n < OBD_RECV_BUF_SIZE - 1) {
                if (c == '.' && n > 2 && buffer[n - 1] == '.' && buffer[n - 2] == '.') {
                    n = 0;
                    timeout = OBD_TIMEOUT_LONG;
                } else {
                    buffer[n++] = c;
                }
	        }
	    } else {
	        if (millis() - startTime > timeout) {
	            // timeout
	            return 0;
	        }
	        dataIdleLoop();
	    }
	}
    if (buffer) buffer[n] = 0;
	return n;
}

void COBD::recover()
{
	write('\r');
	delay(100);
	while (available()) read();
}

bool COBD::init(byte protocol)
{
	const char *initcmd[] = {"ATZ\r","ATE0\r","ATL1\r"};
	char buffer[OBD_RECV_BUF_SIZE];

	m_state = OBD_CONNECTING;
	recover();

	for (unsigned char i = 0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
#ifdef DEBUG
	    debugOutput(initcmd[i]);
#endif
	    write(initcmd[i]);
		if (receive(buffer) == 0) {
	        return false;
		}
		delay(50);
	}
	if (protocol) {
	        write("ATSP");
	        write('0' + protocol);
	        write('\r');
	        receive(buffer);
	        delay(50);
	}
	while (available()) read();

	// load pid map
	memset(pidmap, 0, sizeof(pidmap));
	for (byte i = 0; i < 4; i++) {
		byte pid = i * 0x20;
		sendQuery(pid);
		char* data = getResponse(pid, buffer);
		if (!data) break;
		data--;
		for (byte n = 0; n < 4; n++) {
			if (data[n * 3] != ' ')
				break;
			pidmap[i * 4 + n] = hex2uint8(data + n * 3 + 1);
		}
		delay(100);
	}
	while (available()) read();

	m_state = OBD_CONNECTED;
	errors = 0;
	return true;
}

#ifdef DEBUG
void COBD::debugOutput(const char *s)
{
	DEBUG.print('[');
	DEBUG.print(millis());
	DEBUG.print(']');
	DEBUG.print(s);
}
#endif

/*************************************************************************
* OBD-II I2C Adapter
*************************************************************************/

void COBDI2C::begin(byte addr)
{
	m_addr = addr;
	Wire.begin();
	memset(obdPid, 0, sizeof(obdPid));
	memset(obdInfo, 0, sizeof(obdInfo));
}

bool COBDI2C::init(byte protocol)
{
	m_state = OBD_CONNECTING;
	sendCommand(CMD_QUERY_STATUS);

	char recvbuf[MAX_PAYLOAD_SIZE];
	for (byte n = 0; n < 3; n++) {
		memset(recvbuf, 0, sizeof(recvbuf));
		receive(recvbuf);
		if (!memcmp(recvbuf, "OBD ", 4))
			break;
	}
	if (recvbuf[4] == 'Y') {
		memcpy(pidmap, recvbuf + 16, sizeof(pidmap));
		int value;
		if (read(PID_RPM, value)) {
			m_state = OBD_CONNECTED;
			return true;
		}
	}
	m_state = OBD_DISCONNECTED;
	return false;
}

bool COBDI2C::read(byte pid, int& result)
{
	sendQuery(pid);
	dataIdleLoop();
	return getResult(pid, result);
}

void COBDI2C::write(const char* s)
{
	COMMAND_BLOCK cmdblock = {millis(), CMD_SEND_AT_COMMAND};
	Wire.beginTransmission(m_addr);
	Wire.write((byte*)&cmdblock, sizeof(cmdblock));
	Wire.write(s);
	Wire.endTransmission();
}

bool COBDI2C::sendCommand(byte cmd, uint8_t data, byte* payload, byte payloadBytes)
{
	COMMAND_BLOCK cmdblock = {millis(), cmd, data};
	Wire.beginTransmission(m_addr);
	bool success = Wire.write((byte*)&cmdblock, sizeof(COMMAND_BLOCK)) == sizeof(COMMAND_BLOCK);
	if (payload) Wire.write(payload, payloadBytes);
	Wire.endTransmission();
	return success;
}

byte COBDI2C::receive(char* buffer, int timeout)
{
	uint32_t start = millis();
	byte offset = 0;
	do {
		Wire.requestFrom((byte)m_addr, (byte)MAX_PAYLOAD_SIZE, (byte)1);

		bool hasEnd = false;
		for (byte i = 0; i < MAX_PAYLOAD_SIZE; i++) {
			if ((buffer[offset + i] = Wire.read()) == 0)
				hasEnd = true;
		}

		if (buffer[0] == 0) {
			// data not ready
			dataIdleLoop();
			continue;
		}

		offset += MAX_PAYLOAD_SIZE;
		if (!hasEnd) {
			continue;
		}

		return offset;
	} while(millis() - start < OBD_TIMEOUT_LONG);
	return 0;
}

bool COBDI2C::gpsQuery(GPS_DATA* gpsdata)
{
	if (!sendCommand(CMD_GPS_QUERY, 0)) return false;
	Wire.requestFrom((byte)m_addr, (byte)MAX_PAYLOAD_SIZE, (byte)1);
	Wire.readBytes((char*)gpsdata, MAX_PAYLOAD_SIZE);
	return true;
}

void COBDI2C::gpsSetup(uint32_t baudrate, const char* cmds)
{
	sendCommand(CMD_GPS_SETUP, baudrate / 1200, (byte*)cmds, cmds ? strlen(cmds) : 0);
}

void COBDI2C::setPID(byte pid)
{
	byte n = 0;
	for (; n < MAX_PIDS && obdPid[n]; n++) {
		if (obdPid[n] == pid)
			return;
	}
	if (n == MAX_PIDS) {
		memmove(obdPid, obdPid + 1, sizeof(obdPid[0]) * (MAX_PIDS - 1));
		n = MAX_PIDS - 1;
	}
	obdPid[n] = pid;
}

void COBDI2C::applyPIDs()
{
	sendCommand(CMD_APPLY_OBD_PIDS, 0, (byte*)obdPid, sizeof(obdPid));
	delay(200);
}

void COBDI2C::loadData()
{
	sendCommand(CMD_LOAD_OBD_DATA);
	dataIdleLoop();
	Wire.requestFrom((byte)m_addr, (byte)MAX_PAYLOAD_SIZE, (byte)0);
	Wire.readBytes((char*)obdInfo, MAX_PAYLOAD_SIZE);
}

uint16_t COBDI2C::getData(byte pid, int& result)
{
	byte n;
	for (n = 0; n < MAX_PIDS && obdPid[n] != pid; n++);
	if (n == MAX_PIDS)
		return -1;

	PID_INFO* pi = obdInfo + n;
	switch (pid) {
	case PID_RPM:
		result = pi->value >> 2;
		break;
	case PID_FUEL_PRESSURE:
		result = (int)pi->value * 3;
		break;
	case PID_COOLANT_TEMP:
	case PID_INTAKE_TEMP:
	case PID_AMBIENT_TEMP:
	case PID_ENGINE_OIL_TEMP:
		result = (int)pi->value - 40;
		break;
	case PID_THROTTLE:
	case PID_ENGINE_LOAD:
	case PID_FUEL_LEVEL:
	case PID_ABSOLUTE_ENGINE_LOAD:
	case PID_ETHANOL_PERCENTAGE:
	case PID_HYBRID_BATTERY_PERCENTAGE:
		result = pi->value * 100 / 255; // %
		break;
	case PID_MAF_FLOW:
		result = pi->value / 100;
		break;
	case PID_TIMING_ADVANCE:
		result = (int)pi->value / 2 - 64;
		break;
	case PID_CONTROL_MODULE_VOLTAGE: // V
		result = pi->value / 1000;
		break;
	case PID_ENGINE_FUEL_RATE: // L/h
		result = pi->value / 20;
		break;
	case PID_ENGINE_TORQUE_PERCENTAGE: // %
		result = (int)pi->value - 125;
		break;
	default:
		result = pi->value;
	}
	return result;
}
