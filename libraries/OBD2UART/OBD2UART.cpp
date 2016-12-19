/*************************************************************************
* Arduino Library for Freematics OBD-II UART Adapter
* Distributed under BSD License
* Visit http://freematics.com for more information
* (C)2012-2016 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include "OBD2UART.h"

//#define DEBUG Serial

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

byte COBD::sendCommand(const char* cmd, char* buf, byte bufsize, int timeout)
{
	write(cmd);
	dataIdleLoop();
	return receive(buf, bufsize, timeout);
}

void COBD::sendQuery(byte pid)
{
	char cmd[8];
	sprintf(cmd, "%02X%02X\r", dataMode, pid);
#ifdef DEBUG
	debugOutput(cmd);
#endif
	write(cmd);
}

bool COBD::readPID(byte pid, int& result)
{
	// send a query command
	sendQuery(pid);
	// receive and parse the response
	return getResult(pid, result);
}

byte COBD::readPID(const byte pid[], byte count, int result[])
{
	// send a multiple query command
	char buffer[128];
	char *p = buffer;
	byte results = 0;
	for (byte n = 0; n < count; n++) {
		p += sprintf(p, "%02X%02X\r", dataMode, pid[n]);		
	}
	write(buffer);
	// receive and parse the response
	for (byte n = 0; n < count; n++) {
		byte curpid = pid[n];
		if (getResult(curpid, result[n]))
			results++;
	}
	return results;
}

byte COBD::readDTC(uint16_t codes[], byte count)
{
	/*
	Response example:
	0: 43 04 01 08 01 09 
	1: 01 11 01 15 00 00 00
	*/ 
	byte codesRead = 0;
 	for (byte n = 0; n < 6; n++) {
		char buffer[128];
		sprintf(buffer, "03%02X\r", n);
		write(buffer);
		Serial.println(buffer);
		if (receive(buffer, sizeof(buffer)) > 0) {
			Serial.println(buffer);
			if (!strstr(buffer, "NO DATA")) {
				char *p = strstr(buffer, "43");
				if (p) {
					while (codesRead < count && *p) {
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

bool COBD::setProtocol(OBD_PROTOCOLS h)
{
    char buf[32];
	if (h == PROTO_AUTO) {
		write("ATSP00\r");
	} else {
		sprintf(buf, "ATSP%d\r", h);
		write(buf);
	}
	if (receive(buf, sizeof(buf), OBD_TIMEOUT_LONG) > 0 && strstr(buf, "OK"))
        return true;
    else
        return false;
}

void COBD::sleep()
{
  	char buf[32];
	sendCommand("ATLP\r", buf, sizeof(buf));
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
		if (p) return atof(p);
    }
    return 0;
}

bool COBD::getVIN(char* buffer, byte bufsize)
{
	if (sendCommand("0902\r", buffer, bufsize)) {
        char *p = strstr(buffer, "0: 49 02");
        if (p) {
            char *q = buffer;
            p += 10;
            do {
                for (++p; *p == ' '; p += 3) {
                    if (*q = hex2uint8(p + 1)) q++;
                }
                p = strchr(p, ':');
            } while(p);
            *q = 0;
            return true;
        }
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
	return pidmap[i] & b;
}

void COBD::begin()
{
	OBDUART.begin(OBD_SERIAL_BAUDRATE);
#ifdef DEBUG
	DEBUG.begin(115200);
#endif
	recover();

	char buffer[32];
	version = 0;
	if (sendCommand("ATI\r", buffer, sizeof(buffer), 200)) {
		char *p = strstr(buffer, "OBDUART");
		if (p) {
			p += 9;
			version = (*p - '0') * 10 + (*(p + 2) - '0');
		}
	}
}

byte COBD::receive(char* buffer, byte bufsize, int timeout)
{
	unsigned char n = 0;
	unsigned long startTime = millis();
	for (;;) {
		if (OBDUART.available()) {
			char c = OBDUART.read();
			if (n > 2 && c == '>') {
				// prompt char received
				break;
			} else if (!buffer) {
			       n++;
			} else if (n < bufsize - 1) {
				if (c == '.' && n > 2 && buffer[n - 1] == '.' && buffer[n - 2] == '.') {
					// waiting siginal
					n = 0;
					timeout = OBD_TIMEOUT_LONG;
				} else {
					buffer[n++] = c;
				}
			}
		} else {
			if (millis() - startTime > timeout) {
			    // timeout
			    break;
			}
			dataIdleLoop();
		}
	}
	if (buffer) buffer[n] = 0;
	return n;
}

void COBD::recover()
{
	char buf[16];
	sendCommand("AT\r", buf, sizeof(buf));
}

bool COBD::init(OBD_PROTOCOLS protocol)
{
	const char *initcmd[] = {"ATZ\r","ATE0\r","ATL1\r","0100\r"};
	char buffer[64];

	m_state = OBD_CONNECTING;

	for (unsigned char i = 0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
#ifdef DEBUG
		debugOutput(initcmd[i]);
#endif
		write(initcmd[i]);
		if (receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) == 0) {
			m_state = OBD_DISCONNECTED;
			return false;
		}
		delay(50);
	}

	if (protocol != PROTO_AUTO) {
		setProtocol(protocol);
	}

	// load pid map
	memset(pidmap, 0, sizeof(pidmap));
	for (byte i = 0; i < 4; i++) {
		byte pid = i * 0x20;
		sendQuery(pid);
		char* data = getResponse(pid, buffer, sizeof(buffer));
		if (!data) break;
		data--;
		for (byte n = 0; n < 4; n++) {
			if (data[n * 3] != ' ')
				break;
			pidmap[i * 4 + n] = hex2uint8(data + n * 3 + 1);
		}
		delay(100);
	}

	m_state = OBD_CONNECTED;
	errors = 0;
	return true;
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

float COBD::getTemperature()
{
	char buf[32];
	if (sendCommand("ATTEMP\r", buf, sizeof(buf)) > 0) {
		char* p = getResultValue(buf);
		if (p) return (float)(atoi(p) + 12412) / 340;
	}
	else {
		return -1000;
	}
}

bool COBD::readAccel(int& x, int& y, int& z)
{
	char buf[32];
	if (sendCommand("ATACL\r", buf, sizeof(buf)) > 0) do {
		char* p = getResultValue(buf);
		if (!p) break;
		x = atoi(p++);
		if (!(p = strchr(p, ','))) break;
		y = atoi(++p);
		if (!(p = strchr(p, ','))) break;
		z = atoi(++p);
		return true;
	} while (0);
	return false;
}

bool COBD::readGyro(int& x, int& y, int& z)
{
	char buf[32];
	if (sendCommand("ATGYRO\r", buf, sizeof(buf)) > 0) do {
		char* p = getResultValue(buf);
		if (!p) break;
		x = atoi(p++);
		if (!(p = strchr(p, ','))) break;
		y = atoi(++p);
		if (!(p = strchr(p, ','))) break;
		z = atoi(++p);
		return true;
	} while (0);
	return false;
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

