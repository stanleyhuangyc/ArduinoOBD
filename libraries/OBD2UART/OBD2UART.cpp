/*************************************************************************
* Arduino Library for Freematics OBD-II UART Adapter
* Distributed under BSD License
* Visit https://freematics.com for more information
* (C)2012-2018 Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include "OBD2UART.h"

//#define DEBUG Serial

uint16_t hex2uint16(const char *p)
{
	char c = *p;
	uint16_t i = 0;
	for (uint8_t n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		} else if (c>='a' && c<='f') {
			c -= 39;
        } else if (c == ' ' && n == 2) {
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

	if (c2 == 0)
		return (c1 & 0xf);
	else if (c2 >= 'A' && c2 <= 'F')
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

byte COBD::sendCommand(const char* cmd, char* buf, byte bufsize, int timeout)
{
	write(cmd);
	idleTasks();
	return receive(buf, bufsize, timeout);
}

bool COBD::readPID(byte pid, int& result)
{
	char cmd[8];
	sprintf(cmd, "%02X%02X\r", dataMode, pid);
	write(cmd);
	// receive and parse the response
	return getResult(pid, result);
}

byte COBD::readPID(const byte pid[], byte count, int result[])
{
	byte results = 0; 
	for (byte n = 0; n < count; n++) {
		if (readPID(pid[n], result[n])) {
			results++;
		}
	}
	return results;
}

byte COBD::readDTC(uint16_t codes[], byte maxCodes)
{
	/*
	Response example:
	0: 43 04 01 08 01 09 
	1: 01 11 01 15 00 00 00
	*/ 
	byte codesRead = 0;
 	for (byte n = 0; n < 6; n++) {
		char buffer[128];
		sprintf_P(buffer, n == 0 ? PSTR("03\r") : PSTR("03%02X\r"), n);
		write(buffer);
		if (receive(buffer, sizeof(buffer)) > 0) {
			if (!strstr_P(buffer, PSTR("NO DATA"))) {
				char *p = strstr(buffer, "43");
				if (p) {
					while (codesRead < maxCodes && *p) {
						p += 6;
						if (*p == '\r') {
							p = strchr(p, ':');
							if (!p) break;
							p += 2; 
						}
						uint16_t code = hex2uint16(p);
						if (code == 0) break;
						codes[codesRead++] = code;
					}
				}
				break;
			}
		}
	}
	return codesRead;
}

void COBD::clearDTC()
{
	char buffer[32];
	write("04\r");
	receive(buffer, sizeof(buffer));
}

void COBD::write(const char* s)
{
#ifdef DEBUG
	DEBUG.print("<<<");
	DEBUG.println(s);
#endif
	OBDUART.write(s);
}

int COBD::normalizeData(byte pid, char* data)
{
	int result;
	switch (pid) {
	case PID_RPM:
	case PID_EVAP_SYS_VAPOR_PRESSURE: // kPa
		result = getLargeValue(data) >> 2;
		break;
	case PID_FUEL_PRESSURE: // kPa
		result = getSmallValue(data) * 3;
		break;
	case PID_COOLANT_TEMP:
	case PID_INTAKE_TEMP:
	case PID_AMBIENT_TEMP:
	case PID_ENGINE_OIL_TEMP:
		result = getTemperatureValue(data);
		break;
	case PID_THROTTLE:
	case PID_COMMANDED_EGR:
	case PID_COMMANDED_EVAPORATIVE_PURGE:
	case PID_FUEL_LEVEL:
	case PID_RELATIVE_THROTTLE_POS:
	case PID_ABSOLUTE_THROTTLE_POS_B:
	case PID_ABSOLUTE_THROTTLE_POS_C:
	case PID_ACC_PEDAL_POS_D:
	case PID_ACC_PEDAL_POS_E:
	case PID_ACC_PEDAL_POS_F:
	case PID_COMMANDED_THROTTLE_ACTUATOR:
	case PID_ENGINE_LOAD:
	case PID_ABSOLUTE_ENGINE_LOAD:
	case PID_ETHANOL_FUEL:
	case PID_HYBRID_BATTERY_PERCENTAGE:
		result = getPercentageValue(data);
		break;
	case PID_MAF_FLOW: // grams/sec
		result = getLargeValue(data) / 100;
		break;
	case PID_TIMING_ADVANCE:
		result = (int)(getSmallValue(data) / 2) - 64;
		break;
	case PID_DISTANCE: // km
	case PID_DISTANCE_WITH_MIL: // km
	case PID_TIME_WITH_MIL: // minute
	case PID_TIME_SINCE_CODES_CLEARED: // minute
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
	case PID_ENGINE_TORQUE_DEMANDED: // %
	case PID_ENGINE_TORQUE_PERCENTAGE: // %
		result = (int)getSmallValue(data) - 125;
		break;
	case PID_SHORT_TERM_FUEL_TRIM_1:
	case PID_LONG_TERM_FUEL_TRIM_1:
	case PID_SHORT_TERM_FUEL_TRIM_2:
	case PID_LONG_TERM_FUEL_TRIM_2:
	case PID_EGR_ERROR:
		result = ((int)getSmallValue(data) - 128) * 100 / 128;
		break;
	case PID_FUEL_INJECTION_TIMING:
		result = ((int32_t)getLargeValue(data) - 26880) / 128;
		break;
	case PID_CATALYST_TEMP_B1S1:
	case PID_CATALYST_TEMP_B2S1:
	case PID_CATALYST_TEMP_B1S2:
	case PID_CATALYST_TEMP_B2S2:
		result = getLargeValue(data) / 10 - 40;
		break;
	case PID_AIR_FUEL_EQUIV_RATIO: // 0~200
		result = (long)getLargeValue(data) * 200 / 65536;
		break;
	default:
		result = getSmallValue(data);
	}
	return result;
}

char* COBD::getResponse(byte& pid, char* buffer, byte bufsize)
{
	while (receive(buffer, bufsize) > 0) {
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
	char buffer[64];
	char* data = getResponse(pid, buffer, sizeof(buffer));
	if (!data) {
		recover();
		errors++;
		return false;
	}
	result = normalizeData(pid, data);
	return true;
}

void COBD::enterLowPowerMode()
{
  	char buf[32];
	sendCommand("ATLP\r", buf, sizeof(buf));
}

void COBD::leaveLowPowerMode()
{
	// simply send any command to wake the device up
	char buf[32];
	sendCommand("ATI\r", buf, sizeof(buf), 1000);
}

char* COBD::getResultValue(char* buf)
{
	char* p = buf;
	for (;;) {
		if (isdigit(*p) || *p == '-') {
			return p;
		}
		p = strchr(p, '\r');
		if (!p) break;
		if (*(++p) == '\n') p++;
	}
	return 0;
}

float COBD::getVoltage()
{
    char buf[32];
	if (sendCommand("ATRV\r", buf, sizeof(buf)) > 0) {
		char* p = getResultValue(buf);
		if (p) return (float)atof(p);
    }
    return 0;
}

bool COBD::getVIN(char* buffer, byte bufsize)
{
	for (byte n = 0; n < 5; n++) {
		if (sendCommand("0902\r", buffer, bufsize)) {
			int len = hex2uint16(buffer);
			char *p = strstr_P(buffer + 4, PSTR("0: 49 02 01"));
			if (p) {
				char *q = buffer;
				p += 11; // skip the header
				do {
					while (*(++p) == ' ');
					for (;;) {
						*(q++) = hex2uint8(p);
						while (*p && *p != ' ') p++;
						while (*p == ' ') p++;
						if (!*p || *p == '\r') break;
					}
					p = strchr(p, ':');
				} while(p);
				*q = 0;
				if (q - buffer == len - 3) {
					return true;
				}
			}
		}
		delay(100);
    }
    return false;
}

bool COBD::isValidPID(byte pid)
{
	if (pid >= 0x7f)
		return true;
	pid--;
	byte i = pid >> 3;
	byte b = 0x80 >> (pid & 0x7);
	return (pidmap[i] & b) != 0;
}

byte COBD::begin()
{
	long baudrates[] = {115200, 38400};
	byte version = 0;
	for (byte n = 0; n < sizeof(baudrates) / sizeof(baudrates[0]); n++) {
#ifndef ESP32
		OBDUART.begin(baudrates[n]);
#else
		OBDUART.begin(baudrates[n], SERIAL_8N1, 16, 17);
#endif
		version = getVersion();
		if (version != 0) break;
		OBDUART.end();		 
	}
	return version;	
}

byte COBD::getVersion()
{
	byte version = 0;
	for (byte n = 0; n < 3; n++) {
		char buffer[32];
		if (sendCommand("ATI\r", buffer, sizeof(buffer), 200)) {
			char *p = strchr(buffer, ' ');
			if (p) {
				p += 2;
				version = (*p - '0') * 10 + (*(p + 2) - '0');
				break;
			}
		}
	}
	return version;
}

int COBD::receive(char* buffer, int bufsize, unsigned int timeout)
{
	unsigned char n = 0;
	unsigned long startTime = millis();
	char c = 0;
	for (;;) {
		if (OBDUART.available()) {
			c = OBDUART.read();
			if (!buffer) {
			       n++;
			} else if (n < bufsize - 1) {
				if (c == '.' && n > 2 && buffer[n - 1] == '.' && buffer[n - 2] == '.') {
					// waiting siginal
					n = 0;
					timeout = OBD_TIMEOUT_LONG;
				} else {
					if (c == '\r' || c == '\n' || c == ' ') {
						if (n == 0 || buffer[n - 1] == '\r' || buffer[n - 1] == '\n') continue;
					}
					buffer[n++] = c;
				}
			}
		} else {
			if (c == '>') {
				// prompt char received
				break;
			}
			if ((int)(millis() - startTime) > timeout) {
			    // timeout
			    break;
			}
			idleTasks();
		}
	}
	if (buffer) {
		buffer[n] = 0;
	}
#ifdef DEBUG
	DEBUG.print(">>>");
	DEBUG.println(buffer);
#endif
	return n;
}

void COBD::recover()
{
	sendCommand("\r", 0, 0);
}

bool COBD::init(OBD_PROTOCOLS protocol)
{
	const char *initcmd[] = {"ATZ\r", "ATE0\r", "ATH0\r"};
	char buffer[64];
	byte stage;

	m_state = OBD_DISCONNECTED;
	for (byte n = 0; n < 2; n++) {
		stage = 0;
		if (n != 0) reset();
		for (byte i = 0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
			delay(10);
			if (!sendCommand(initcmd[i], buffer, sizeof(buffer), OBD_TIMEOUT_SHORT)) {
				continue;
			}
		}
		stage = 1;
		if (protocol != PROTO_AUTO) {
			sprintf(buffer, "ATSP%u\r", protocol);
			delay(10);
			if (!sendCommand(buffer, buffer, sizeof(buffer), OBD_TIMEOUT_SHORT) || !strstr(buffer, "OK")) {
				continue;
			}
		}
		stage = 2;
		delay(10);
		if (!sendCommand("010D\r", buffer, sizeof(buffer), OBD_TIMEOUT_LONG) || checkErrorMessage(buffer)) {
			continue;
		}
		stage = 3;
		// load pid map
		memset(pidmap, 0xff, sizeof(pidmap));
		for (byte i = 0; i < 4; i++) {
			byte pid = i * 0x20;
			sprintf(buffer, "%02X%02X\r", dataMode, pid);
			delay(10);
			write(buffer);
			delay(10);
			if (!receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) || checkErrorMessage(buffer)) break;
			for (char *p = buffer; (p = strstr(p, "41 ")); ) {
				p += 3;
				if (hex2uint8(p) == pid) {
					p += 2;
					for (byte n = 0; n < 4 && *(p + n * 3) == ' '; n++) {
						pidmap[i * 4 + n] = hex2uint8(p + n * 3 + 1);
					}
				}
			}
		}
		break;
	}
	if (stage == 3) {
		m_state = OBD_CONNECTED;
		errors = 0;
		return true;
	} else {
#ifdef DEBUG
		Serial.print("Stage:");
		Serial.println(stage);
#endif
		reset();
		return false;
	}
}

void COBD::end()
{
	m_state = OBD_DISCONNECTED;
	OBDUART.end();
}

bool COBD::setBaudRate(unsigned long baudrate)
{
    OBDUART.print("ATBR1 ");
    OBDUART.print(baudrate);
    OBDUART.print('\r');
    delay(50);
    OBDUART.end();
    OBDUART.begin(baudrate);
    recover();
    return true;
}

void COBD::reset()
{
	char buf[32];
	sendCommand("ATR\r", buf, sizeof(buf));
	delay(3000);
	sendCommand("ATZ\r", buf, sizeof(buf));
}

void COBD::uninit()
{
	char buf[32];
	sendCommand("ATPC\r", buf, sizeof(buf));
}

byte COBD::checkErrorMessage(const char* buffer)
{
	const char *errmsg[] = {"UNABLE", "ERROR", "TIMEOUT", "NO DATA"};
	for (byte i = 0; i < sizeof(errmsg) / sizeof(errmsg[0]); i++) {
		if (strstr(buffer, errmsg[i])) return i + 1;
	}
	return 0;
}

uint8_t COBD::getPercentageValue(char* data)
{
  return (uint16_t)hex2uint8(data) * 100 / 255;
}

uint16_t COBD::getLargeValue(char* data)
{
  return hex2uint16(data);
}

uint8_t COBD::getSmallValue(char* data)
{
  return hex2uint8(data);
}

int16_t COBD::getTemperatureValue(char* data)
{
  return (int)hex2uint8(data) - 40;
}

bool COBD::memsInit(bool fusion)
{
	char buf[16];
	if (sendCommand("ATTEMP\r", buf, sizeof(buf)) <= 0 || strchr(buf, '?'))
		return false;
	if (fusion) {
		m_fusion = true;
		return sendCommand("ATQU1\r", buf, sizeof(buf));
	} else {
		if (m_fusion) {
			m_fusion = false;
			sendCommand("ATQU0\r", buf, sizeof(buf));
		}
		return true;
	}
}

bool COBD::memsRead(int16_t* acc, int16_t* gyr, int16_t* mag, int16_t* temp)
{
	char buf[64];
	bool success;
	if (acc) {
		success = false;
		if (sendCommand("ATACL\r", buf, sizeof(buf)) > 0) do {
			char* p = getResultValue(buf);
			if (!p) break;
			acc[0] = atoi(p++);
			if (!(p = strchr(p, ','))) break;
			acc[1] = atoi(++p);
			if (!(p = strchr(p, ','))) break;
			acc[2] = atoi(++p);
			success = true;
		} while (0);
		if (!success) return false;
	}
	if (gyr) {
		success = false;
		if (sendCommand("ATGYRO\r", buf, sizeof(buf)) > 0) do {
			char* p = getResultValue(buf);
			if (!p) break;
			gyr[0] = atoi(p++);
			if (!(p = strchr(p, ','))) break;
			gyr[1] = atoi(++p);
			if (!(p = strchr(p, ','))) break;
			gyr[2] = atoi(++p);
			success = true;
		} while (0);
		if (!success) return false;
	}
	if (mag) {
		success = false;
		if (sendCommand("ATMAG\r", buf, sizeof(buf)) > 0) do {
			char* p = getResultValue(buf);
			if (!p) break;
			mag[0] = atoi(p++);
			if (!(p = strchr(p, ','))) break;
			mag[1] = atoi(++p);
			if (!(p = strchr(p, ','))) break;
			mag[2] = atoi(++p);
			success = true;
		} while (0);
		if (!success) return false;
	}
	if (temp) {
		success = false;
		if (sendCommand("ATTEMP\r", buf, sizeof(buf)) > 0) {
			char* p = getResultValue(buf);
			if (p) {
				*temp = (atoi(p) + 12412) / 34;
				success = true;
			}
		}
		if (!success) return false;
	}
	return true;	
}

bool COBD::memsOrientation(float& yaw, float& pitch, float& roll)
{
	char buf[64];
	bool success = false;
	if (sendCommand("ATORI\r", buf, sizeof(buf)) > 0) do {
		char* p = getResultValue(buf);
		if (!p) break;
		yaw = (float)atof(p++);
		if (!(p = strchr(p, ','))) break;
		pitch = (float)atoi(++p);
		if (!(p = strchr(p, ','))) break;
		roll = (float) atof(++p);
		success = true;
	} while (0);
	return success;
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

