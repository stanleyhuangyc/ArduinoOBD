/*************************************************************************
* OBD-II (ELM327) data accessing library for Arduino
* Distributed under GPL v2.0
* Copyright (c) 2012 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "OBD.h"

#define INIT_CMD_COUNT 4
#define MAX_CMD_LEN 6

const char PROGMEM s_initcmd[INIT_CMD_COUNT][MAX_CMD_LEN] = {"ATZ\r","ATE0\r","ATL1\r","ATI\r"};
const char PROGMEM s_searching[] = "SEARCHING";
const char PROGMEM s_cmd_fmt[] = "%02X%02X 1\r";
const char PROGMEM s_cmd_sleep[] = "atlp\r";
const char PROGMEM s_cmd_vin[] = "0902\r";
const char PROGMEM s_response_begin[] = "41 ";

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

void COBD::Query(unsigned char pid)
{
	char cmd[8];
	sprintf_P(cmd, s_cmd_fmt, dataMode, pid);
	write(cmd);
}

bool COBD::ReadSensor(byte pid, int& result, bool passive)
{
    // send a query command
    Query(pid);
    // wait for reponse
    bool hasData;
    unsigned long tick = millis();
    do {
        DataIdleLoop();
    } while (!(hasData = available()) && millis() - tick < OBD_TIMEOUT_SHORT);
    if (!hasData) {
        errors++;
        return false;
    }
    // receive and parse the response
    return GetResponseParsed(pid, result);
}

bool COBD::available()
{
	return OBDUART.available();
}

char COBD::read()
{
	return OBDUART.read();
}

void COBD::write(const char* s)
{
	OBDUART.write(s);
}

void COBD::write(const char c)
{
	OBDUART.write(c);
}

int COBD::GetConvertedValue(byte pid, char* data)
{
    int result;
	switch (pid) {
	case PID_RPM:
		result = GetLargeValue(data) >> 2;
		break;
	case PID_FUEL_PRESSURE:
		result = GetSmallValue(data) * 3;
		break;
	case PID_COOLANT_TEMP:
	case PID_INTAKE_TEMP:
	case PID_AMBIENT_TEMP:
		result = GetTemperatureValue(data);
		break;
	case PID_ABS_ENGINE_LOAD:
		result = GetLargeValue(data) * 100 / 255;
		break;
	case PID_MAF_FLOW:
		result = GetLargeValue(data) / 100;
		break;
	case PID_THROTTLE:
	case PID_ENGINE_LOAD:
	case PID_FUEL_LEVEL:
		result = GetPercentageValue(data);
		break;
	case PID_TIMING_ADVANCE:
		result = (GetSmallValue(data) - 128) >> 1;
		break;
	case PID_DISTANCE:
	case PID_RUNTIME:
		result = GetLargeValue(data);
		break;
	default:
		result = GetSmallValue(data);
	}
	return result;
}

char* COBD::GetResponse(byte& pid, char* buffer)
{
	unsigned long startTime = millis();
	byte i = 0;

	for (;;) {
		if (available()) {
			char c = read();
			buffer[i] = c;
			if (++i == OBD_RECV_BUF_SIZE - 1) {
                // buffer overflow
                break;
			}
			if (c == '>' && i > 6) {
				// prompt char reached
				break;
			}
		} else {
		    buffer[i] = 0;
		    unsigned int timeout;
		    if (dataMode != 1 || strstr_P(buffer, s_searching)) {
                timeout = OBD_TIMEOUT_LONG;
		    } else {
		        timeout = OBD_TIMEOUT_SHORT;
		    }
		    if (millis() - startTime > timeout) {
                // timeout
                errors++;
                break;
		    }
		}
	}
	buffer[i] = 0;

	char *p = buffer;
	while ((p = strstr_P(p, s_response_begin))) {
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
	return 0;
}

bool COBD::GetResponseParsed(byte& pid, int& result)
{
    char buffer[OBD_RECV_BUF_SIZE];
    char* data = GetResponse(pid, buffer);
    if (!data) {
        // try recover next time
        write('\r');
		return false;
    }
    result = GetConvertedValue(pid, data);
    return true;
}

void COBD::Sleep(int seconds)
{
    char cmd[MAX_CMD_LEN];
    strcpy_P(cmd, s_cmd_sleep);
    write(cmd);
    if (seconds) {
        delay((unsigned long)seconds << 10);
        write('\r');
    }
}

bool COBD::IsValidPID(byte pid)
{
    if (pid >= 0x7f)
        return false;
    pid--;
    byte i = pid >> 3;
    byte b = 0x80 >> (pid & 0x7);
    return pidmap[i] & b;
}

bool COBD::Init(bool passive)
{
	unsigned long currentMillis;
	unsigned char n;
	char prompted;
	char buffer[OBD_RECV_BUF_SIZE];

	for (unsigned char i = 0; i < INIT_CMD_COUNT; i++) {
        if (!passive) {
            char cmd[MAX_CMD_LEN];
            strcpy_P(cmd, s_initcmd[i]);
            write(cmd);
        }
        n = 0;
        prompted = 0;
        currentMillis = millis();
        for (;;) {
            if (available()) {
                char c = read();
                if (c == '>') {
                    buffer[n] = 0;
                    prompted++;
                } else if (n < OBD_RECV_BUF_SIZE - 1) {
                    buffer[n++] = c;
                }
            } else if (prompted) {
                break;
            } else {
                unsigned long elapsed = millis() - currentMillis;
                if (elapsed > OBD_TIMEOUT_INIT) {
                    // init timeout
                    //WriteData("\r");
                    return false;
                }
                InitIdleLoop();
            }
        }
    }

    // load pid map
    memset(pidmap, 0, sizeof(pidmap));
    for (byte i = 0; i < 4; i++) {
        byte pid = i * 0x20;
        Query(pid);
        char* data = GetResponse(pid, buffer);
        if (!data) break;
        data--;
        for (byte n = 0; n < 4; n++) {
            if (data[n * 3] != ' ')
                break;
            pidmap[i * 4 + n] = hex2uint8(data + n * 3 + 1);
        }
    }
    errors = 0;
	return true;
}
