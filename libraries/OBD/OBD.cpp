/*************************************************************************
* Arduino Library for OBD-II UART/I2C Adapter
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2012-2015 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "OBD.h"

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

byte COBD::sendCommand(const char* cmd, char* buf, byte bufsize)
{
	write(cmd);
	dataIdleLoop();
	return receive(buf, bufsize, OBD_TIMEOUT_LONG);
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

bool COBD::read(byte pid, int& result)
{
	// send a query command
	sendQuery(pid);
	// receive and parse the response
	return getResult(pid, result);
}

/* 
 * getDTCStatus
 *  send request for number of DTC codes available. Will also return value if MIL ("Check Engine") light is on.
 *  if number of codes is > 0, then request the actual codes as well.
 */    
bool COBD::getDTCStatus(int* numCodes)
{
	char buffer[MAX_PAYLOAD_SIZE + 1];
	char val3str[3];
	int val3;
	bool milOn = false;

	write("01 01\r");  // send OBD request for DTCs
	int bytesReceived = receive(buffer, MAX_PAYLOAD_SIZE, 1000); // record the number of bytes of data received, timeout after 1000 milliseconds
	buffer[bytesReceived + 1] = '\0'; // set the entry following the last code to "0"

	Serial.println("Engine response to 01 01...");
	Serial.println(buffer);

	if (bytesReceived > 0) {

		// Response should be "41 01 xx yy yy yy yy"
		// looking for the value in xx

		int response = strncmp(buffer, "41", 2);
		if (response == 0) {

			// Extract the number of codes from the string
			memcpy(val3str, &buffer[6], 2);

			val3 = strtol(val3str, NULL, 16);  // Convert hex string to decimal

			if ((val3 & 0x80) > 0) { // MIL is on
				milOn = true;
			}

			*numCodes = val3 & 0x7F;

		} else {
			Serial.println("Error receiving DTC information");
		}
	}

	return milOn;
}

/* getDTCs
 *  Send request for trouble codes.  Will receive multiple codes in one message.
 */
int COBD::getDTCs(int numCodes, char *retval, int bufsize) 
{
	write("03\r");  // send OBD request for DTCs
	int bytesReceived = 0;
	int len;
	char buffer[MAX_PAYLOAD_SIZE];
	while ((len = receive(buffer, bufsize, 1000)) != 0) {
		// loop until no data is received
		buffer[len] = '\0';
		strcat(retval, buffer);
		bytesReceived += len;
	}

	int numBytes = bytesReceived;
	retval[bytesReceived] = '\0';

	Serial.println("Raw DTC Data from 03...");
	Serial.println(retval);

	if (numCodes > 0 && bytesReceived > 0 && strncmp(retval, "43", 2) == 0) {
		int i;
		int j;
		char response[MAX_PAYLOAD_SIZE + 1];

		// Remove spaces from the string
		for (i = 0, j = 0; retval[i] != '\0'; i++) {
			if (retval[i] != ' ') {
				response[j++] = retval[i];
			} else {
				numBytes--;
			}
		}

		response[j] = '\0';

		char oneCode[15];
		char category[2];
		char codeType[3];
		char codeTypes[5] = "PCBU";
		char code[4];  // Actual trouble code
		Serial.println("-------------------");

		// Loop through each code
		for (i = 0; i < numCodes; i++) {
			if ((15 * i) > numBytes) {
				Serial.println("No other errors were available!");
				break;
			}

			// Grab one chunk of codes at a time
			memcpy(oneCode, &response[15 * i], 14);
			oneCode[14] = '\0'; 

			Serial.printf("Error code %d=%s\n", i + 1, oneCode);

			memcpy(category, &oneCode[2], 1);
			category[2] = '\0';
			unsigned int numCategory = strtol(category, NULL, 16);

			// numCategory represents 0x0X, the letter is the bits 3 and 4, the value is bits 1 and 2
			sprintf(codeType, "%c%d", codeTypes[numCategory >> 2], (numCategory & 0x03));

			memcpy(code, &oneCode[3], 3);
			code[4] = '\0';

			Serial.printf("Formatted error=%d/%d: %s%s\n", i + 1, numCodes, codeType, code);
			Serial.println("-------------------");
		}

	}

	return bytesReceived;
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
	case PID_EVAP_SYS_VAPOR_PRESSURE:
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
	case PID_MAF_FLOW:
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
	write("ATLP\r");
	receive(buf, sizeof(buf));
}

float COBD::getVoltage()
{
    char buf[32];
    write("ATRV\r");
    byte n = receive(buf, sizeof(buf));
    if (n > 0) {
        return atof(buf);
    }
    return 0;
}

bool COBD::getVIN(char* buffer, byte bufsize)
{
    write("0902\r");
    if (receive(buffer, bufsize, OBD_TIMEOUT_LONG)) {
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
	write("AT\r");
	receive(buf, sizeof(buf));
}

bool COBD::init(OBD_PROTOCOLS protocol)
{
	const char *initcmd[] = {"ATZ\r","ATE0\r","ATL1\r"};
	char buffer[64];

	m_state = OBD_CONNECTING;
	//recover();

	for (unsigned char i = 0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
#ifdef DEBUG
		debugOutput(initcmd[i]);
#endif
		write(initcmd[i]);
		if (receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) == 0) {
			if (i == 0) {
				// workaround for longer initialization time
				delay(2000);
			} else {
				m_state = OBD_DISCONNECTED;
				return false;
			}
		}
		delay(50);
	}
	//while (available()) read();

	if (protocol != PROTO_AUTO) {
		setProtocol(protocol);
	}
        int value;
	if (!read(PID_RPM, value)) {
		m_state = OBD_DISCONNECTED;
		return false;
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
	//while (available()) read();

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

bool COBD::initGPS(unsigned long baudrate)
{
    char buf[32];
    sprintf(buf, "ATBR2 %lu\r", baudrate);
    write(buf);
    return (receive(buf, sizeof(buf)) && strstr(buf, "OK"));
}

bool COBD::getGPSData(GPS_DATA* gdata)
{
    char buf[128];
    char *p;
    write("ATGPS\r");
    if (receive(buf, sizeof(buf)) == 0 || !(p = strstr(buf, "$GPS")))
        return false;

    byte index = 0;
    char *s = buf;
    s = p + 5;
    for (p = s; *p; p++) {
        char c = *p;
        if (c == ',' || c == '>' || c <= 0x0d) {
            long value = atol(s);
            switch (index) {
            case 0:
                gdata->date = (uint32_t)value;
                break;
            case 1:
                gdata->time = (uint32_t)value;
                break;
            case 2:
                gdata->lat = value;
                break;
            case 3:
                gdata->lon = value;
                break;
            case 4:
                gdata->alt = value;
                break;
            case 5:
                gdata->speed = value;
                break;
            case 6:
                gdata->heading = value;
                break;
            case 7:
                gdata->sat = value;
                break;
            }
            index++;
            s = p + 1;
        }
    }
    return index >= 4;
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

void COBDI2C::begin()
{
	Wire.begin();
#ifdef DEBUG
	DEBUG.begin(115200);
#endif
	recover();
}

void COBDI2C::end()
{
	m_state = OBD_DISCONNECTED;
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
	Wire.beginTransmission(I2C_ADDR);
	Wire.write((byte*)&cmdblock, sizeof(cmdblock));
	Wire.write(s);
	Wire.endTransmission();
}

bool COBDI2C::sendCommandBlock(byte cmd, uint8_t data, byte* payload, byte payloadBytes)
{
	COMMAND_BLOCK cmdblock = {millis(), cmd, data};
	Wire.beginTransmission(I2C_ADDR);
	bool success = Wire.write((byte*)&cmdblock, sizeof(COMMAND_BLOCK)) == sizeof(COMMAND_BLOCK);
	if (payload) Wire.write(payload, payloadBytes);
	Wire.endTransmission();
	return success;
}

byte COBDI2C::receive(char* buffer, byte bufsize, int timeout)
{
	uint32_t start = millis();
	byte offset = 0;
	do {
		Wire.requestFrom((byte)I2C_ADDR, (byte)MAX_PAYLOAD_SIZE, (byte)1);
		int c = Wire.read();
		if (offset == 0 && (c == 0 || c == -1)) {
			 // data not ready
			dataIdleLoop();
			continue; 
		}
		if (buffer) buffer[offset++] = c;
		for (byte i = 1; i < MAX_PAYLOAD_SIZE && Wire.available(); i++) {
			char c = Wire.read();
			if (c == '.' && offset > 2 && buffer[offset - 1] == '.' && buffer[offset - 2] == '.') {
				// waiting signal
				offset = 0;
				timeout = OBD_TIMEOUT_LONG;
			} else if (c == 0 || offset == bufsize - 1) {
				// string terminator encountered or buffer full
				if (buffer) buffer[offset] = 0;
				// discard the remaining data
				while (Wire.available()) Wire.read();
				return offset;
			} else {
				if (buffer) buffer[offset++] = c;
			}
		}
	} while(millis() - start < timeout);
	return 0;
}

void COBDI2C::setPID(byte pid, byte obdPid[])
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

void COBDI2C::applyPIDs(byte obdPid[])
{
	sendCommandBlock(CMD_APPLY_OBD_PIDS, 0, (byte*)obdPid, sizeof(obdPid[0])* MAX_PIDS);
	delay(200);
}

void COBDI2C::loadData(PID_INFO obdInfo[])
{
	sendCommandBlock(CMD_LOAD_OBD_DATA);
	dataIdleLoop();
	Wire.requestFrom((byte)I2C_ADDR, (byte)MAX_PAYLOAD_SIZE, (byte)0);
	Wire.readBytes((char*)obdInfo, sizeof(obdInfo[0]) * MAX_PIDS);
}
