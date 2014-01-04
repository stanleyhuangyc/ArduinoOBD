/*************************************************************************
* Arduino Library for OBD-II UART Adapter
* Distributed under GPL v2.0
* Copyright (c) 2012~2013 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "OBD.h"

//#define DEBUG Serial
//#define REDIRECT Serial

#define MAX_CMD_LEN 6

const char PROGMEM s_initcmd[][MAX_CMD_LEN] = {"ATZ\r","ATE0\r","ATL1\r","0902\r"};
const char PROGMEM s_cmd_fmt[] = "%02X%02X 1\r";
const char PROGMEM s_cmd_sleep[] = "atlp\r";
#define STR_SEARCHING "SEARCHING..."

unsigned int hex2uint16(const char *p)
{
	char c = *p;
	unsigned int i = 0;
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

unsigned char hex2uint8(const char *p)
{
	unsigned char c1 = *p;
	unsigned char c2 = *(p + 1);
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

void COBD::sendQuery(unsigned char pid)
{
	char cmd[8];
	sprintf_P(cmd, s_cmd_fmt, dataMode, pid);
#ifdef DEBUG
	debugOutput(cmd);
#endif
	write(cmd);
}

bool COBD::read(byte pid, int& result, bool passive)
{
	// send a query command
	sendQuery(pid);
	// wait for reponse
	bool hasData;
	unsigned long tick = millis();
	do {
		dataIdleLoop();
	} while (!(hasData = available()) && millis() - tick < OBD_TIMEOUT_SHORT);
	if (!hasData) {
		errors++;
		return false;
	}
	// receive and parse the response
	return getResponseParsed(pid, result);
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

void COBD::write(char* s)
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
	case PID_ABSOLUTE_ENGINE_LOAD:
	case PID_ETHANOL_PERCENTAGE:
	case PID_HYBRID_BATTERY_PERCENTAGE:
		result = getLargeValue(data) * 100 / 255; // %
		break;
	case PID_MAF_FLOW:
		result = getLargeValue(data) / 100;
		break;
	case PID_THROTTLE:
	case PID_ENGINE_LOAD:
	case PID_FUEL_LEVEL:
		result = getPercentageValue(data);
		break;
	case PID_TIMING_ADVANCE:
		result = (getSmallValue(data) - 128) >> 1;
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
	case PID_ENGINE_FUEL_RATE: // L/min
		result = getLargeValue(data) * 3;
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
    byte n = receive(buffer);
	if (n > 6) {
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
		};
	}
	return 0;
}

bool COBD::getResponseParsed(byte& pid, int& result)
{
	char buffer[OBD_RECV_BUF_SIZE];
	char* data = getResponse(pid, buffer);
	if (!data) {
		// try recover next time
		//write('\r');
		return false;
	}
	result = normalizeData(pid, data);
	return true;
}

void COBD::sleep(int seconds)
{
	char cmd[MAX_CMD_LEN];
	strcpy_P(cmd, s_cmd_sleep);
	write(cmd);
	if (seconds) {
		delay((unsigned long)seconds << 10);
		write('\r');
	}
}

bool COBD::isValidPID(byte pid)
{
	if (pid >= 0x7f)
		return false;
	pid--;
	byte i = pid >> 3;
	byte b = 0x80 >> (pid & 0x7);
	return pidmap[i] & b;
}

void COBD::begin()
{
	OBDUART.begin(OBD_SERIAL_BAUDRATE);
}

byte COBD::receive(char* buffer)
{
    unsigned long startTime = millis();
	unsigned char n = 0;
    int timeout = OBD_TIMEOUT_SHORT;
    bool prompted = false;

    buffer[0] = 0;
    for (;;) {
        if (available()) {
            char c = read();
            if (n > 2 && c == '>') {
                // prompt char received
                prompted = true;
            } else if (n < OBD_RECV_BUF_SIZE - 1) {
                buffer[n++] = c;
                buffer[n] = 0;
                if (strstr(buffer, STR_SEARCHING)) {
                    strcpy(buffer, buffer + sizeof(STR_SEARCHING));
                    n -= sizeof(STR_SEARCHING);
                    timeout = OBD_TIMEOUT_LONG;
                }
            }
        } else if (prompted) {
            break;
        } else {
            if (millis() - startTime > timeout) {
                // timeout
                return 0;
            }
            dataIdleLoop();
        }
    }

    return n;
}

bool COBD::init(bool passive)
{
	unsigned long currentMillis;
	char buffer[OBD_RECV_BUF_SIZE];

    m_state = OBD_CONNECTING;

    write('\r');
    delay(100);
    while (available()) read();

	for (unsigned char i = 0; i < sizeof(s_initcmd) / sizeof(s_initcmd[0]); i++) {
		if (!passive) {
			char cmd[MAX_CMD_LEN];
			strcpy_P(cmd, s_initcmd[i]);
#ifdef DEBUG
			debugOutput(cmd);
#endif
			write(cmd);
		}
		if (receive(buffer) == 0) {
            return false;
		}
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
}

bool COBDI2C::init()
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
        m_state = OBD_CONNECTED;
        return true;
    } else {
        m_state = OBD_DISCONNECTED;
        return false;
    }
}

bool COBDI2C::read(byte pid, int& result, bool passive)
{
    uint32_t t = millis();
    sendQuery(pid);
    dataIdleLoop();
    return getResponseParsed(pid, result);
}

void COBDI2C::write(char* s)
{
    COMMAND_BLOCK cmdblock = {millis(), CMD_SEND_COMMAND};
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

byte COBDI2C::receive(char* buffer)
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

bool COBDI2C::btInit(uint16_t baudrate)
{
    return sendCommand(CMD_UART_BEGIN, baudrate / 1200);
}

bool COBDI2C::btSend(byte* data, byte length)
{
    return sendCommand(CMD_UART_SEND, 0, data, length);
}

bool COBDI2C::btReceive(byte* buffer, byte bufsize)
{
    if (!sendCommand(CMD_UART_RECV, bufsize)) return false;
    memset(buffer, 0, MAX_PAYLOAD_SIZE);
    Wire.requestFrom((byte)m_addr, (byte)MAX_PAYLOAD_SIZE, (byte)1);
    Wire.readBytes((char*)buffer, MAX_PAYLOAD_SIZE);
    return true;
}
