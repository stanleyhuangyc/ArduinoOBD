/*************************************************************************
* SIM800 GPRS/HTTP Library
* Distributed under GPL v2.0
* Written by Stanley Huang <stanleyhuangyc@gmail.com>
* For more information, please visit http://arduinodev.com
*************************************************************************/

#include "SIM800.h"

bool CGPRS_SIM800::init()
{
    simser.begin(115200);
    pinMode(SIM800_RESET_PIN, OUTPUT);
    digitalWrite(SIM800_RESET_PIN, HIGH);
    delay(10);
    digitalWrite(SIM800_RESET_PIN, LOW);
    delay(100);
    // generate turn on pulse
    digitalWrite(SIM800_RESET_PIN, HIGH);
    delay(3000);
    if (sendCommand("AT")) {
        sendCommand("AT+IPR=115200");
        sendCommand("ATE0");
        return true;
    }
    return false;
}
byte CGPRS_SIM800::setup(const char* apn)
{
  bool success = false;
  for (byte n = 0; n < 30; n++) {
    if (sendCommand("AT+CREG?", 2000)) {
        char *p = strstr(buffer, "0,");
        if (p) {
          char mode = *(p + 2);
#if DEBUG
          con.print("Mode:");
          con.println(mode);
#endif
          if (mode == '1' || mode == '5') {
            success = true;
            break;
          }
        }
    }
    delay(1000);
  }
  		
  if (!success)
    return 1;
  
  if (!sendCommand("AT+CGATT?"))
    return 2;
    
  if (!sendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\""))
    return 3;
  
  simser.print("AT+SAPBR=3,1,\"APN\",\"");
  simser.print(apn);
  simser.println('\"');
  if (!sendCommand(0))
    return 4;
  
  for (byte n = 0; n < 10 && !(success = (sendCommand("AT+SAPBR=1,1") != 0)); n++)
    delay(500);
  
  if (!success)
    return 5;

  return 0;
}
bool CGPRS_SIM800::getOperatorName()
{
  // display operator name
  if (sendCommand("AT+COPS?", "OK\r", "ERROR\r") == 1) {
      char *p = strstr(buffer, ",\"");
      if (p) {
          p += 2;
          char *s = strchr(p, '\"');
          if (s) *s = 0;
          strcpy(buffer, p);
          return true;
      }
  }
  return false;
}
bool CGPRS_SIM800::checkSMS()
{
  if (sendCommand("AT+CMGR=1", "+CMGR:", "ERROR") == 1) {
    // reads the data of the SMS
    sendCommand(0, 100, "\r\n");
    if (sendCommand(0)) {
      // remove the SMS
      sendCommand("AT+CMGD=1");
      return true;
    }
  }
  return false; 
}
int CGPRS_SIM800::getSignalQuality()
{
  sendCommand("AT+CSQ");
  char *p = strstr(buffer, "CSQ: ");
  if (p) {
    int n = atoi(p + 2);
    if (n == 99 || n == -1) return 0;
    return n * 2 - 114;
  } else {
   return 0; 
  }
}
void CGPRS_SIM800::httpUninit()
{
  sendCommand("AT+HTTPTERM");
}

bool CGPRS_SIM800::httpInit()
{
  if  (!sendCommand("AT+HTTPINIT", 10000) || !sendCommand("AT+HTTPPARA=\"CID\",1", 5000)) {
    httpState = HTTP_DISABLED;
    return false;
  }
  httpState = HTTP_READY;
  return true;
}
bool CGPRS_SIM800::httpConnect(const char* url, const char* args)
{
    // Sets url
    simser.print("AT+HTTPPARA=\"URL\",\"");
    simser.print(url);
    if (args) {
        simser.print('?');
        simser.print(args);
    }

    simser.println('\"');
    if (sendCommand(0))
    {
        // Starts GET action
        simser.println("AT+HTTPACTION=0");
        httpState = HTTP_CONNECTING;
        bytesRecv = 0;
        checkTimer = millis();
    } else {
        httpState = HTTP_ERROR;
    }
    return false;
}
// check if HTTP connection is established
// return 0 for in progress, 1 for success, 2 for error
byte CGPRS_SIM800::httpIsConnected()
{
    byte ret = checkbuffer("0,200", "0,60", 10000);
    if (ret >= 2) {
        httpState = HTTP_ERROR;
        return -1;
    }
    return ret;
}
void CGPRS_SIM800::httpRead()
{
    simser.println("AT+HTTPREAD");
    httpState = HTTP_READING;
    bytesRecv = 0;
    checkTimer = millis();
}
// check if HTTP connection is established
// return 0 for in progress, -1 for error, number of http payload bytes on success
int CGPRS_SIM800::httpIsRead()
{
    byte ret = checkbuffer("+HTTPREAD: ", "Error", 10000) == 1;
    if (ret == 1) {
        bytesRecv = 0;
        // read the rest data
        sendCommand(0, 100, "\r\n");
        int bytes = atoi(buffer);
        sendCommand(0);
        bytes = min(bytes, sizeof(buffer) - 1);
        buffer[bytes] = 0;
        return bytes;
    } else if (ret >= 2) {
        httpState = HTTP_ERROR;
        return -1;
    }
    return 0;
}
byte CGPRS_SIM800::sendCommand(const char* cmd, unsigned int timeout, const char* expected)
{
  if (cmd) {
    purgeSerial();
#if DEBUG
    con.print('>');
    con.println(cmd);
#endif
    simser.println(cmd);
  }
  uint32_t t = millis();
  byte n = 0;
  do {
    if (simser.available()) {
      char c = simser.read();
      if (n >= sizeof(buffer) - 1) {
        // buffer full, discard first half
        n = sizeof(buffer) / 2 - 1;
        memcpy(buffer, buffer + sizeof(buffer) / 2, n);
      }
      buffer[n++] = c;
      buffer[n] = 0;
      if (strstr(buffer, expected ? expected : "OK\r")) {
#if DEBUG
       con.print("[1]");
       con.println(buffer);
#endif
       return n;
      }
    }
  } while (millis() - t < timeout);
#if DEBUG
   con.print("[0]");
   con.println(buffer);
#endif
  return 0;
}
byte CGPRS_SIM800::sendCommand(const char* cmd, const char* expected1, const char* expected2, unsigned int timeout)
{
  if (cmd) {
    purgeSerial();
#if DEBUG
    con.print('>');
    con.println(cmd);
#endif
    simser.println(cmd);
  }
  uint32_t t = millis();
  byte n = 0;
  do {
    if (simser.available()) {
      char c = simser.read();
      if (n >= sizeof(buffer) - 1) {
        // buffer full, discard first half
        n = sizeof(buffer) / 2 - 1;
        memcpy(buffer, buffer + sizeof(buffer) / 2, n);
      }
      buffer[n++] = c;
      buffer[n] = 0;
      if (strstr(buffer, expected1)) {
#if DEBUG
       con.print("[1]");
       con.println(buffer);
#endif
       return 1;
      }
      if (strstr(buffer, expected2)) {
#if DEBUG
       con.print("[2]");
       con.println(buffer);
#endif
       return 2;
      }
    }
  } while (millis() - t < timeout);
#if DEBUG
   con.print("[0]");
   con.println(buffer);
#endif
  return 0;
}

byte CGPRS_SIM800::checkbuffer(const char* expected1, const char* expected2, unsigned int timeout)
{
    while (simser.available()) {
        char c = simser.read();
        if (bytesRecv >= sizeof(buffer) - 1) {
            // buffer full, discard first half
            bytesRecv = sizeof(buffer) / 2 - 1;
            memcpy(buffer, buffer + sizeof(buffer) / 2, bytesRecv);
        }
        buffer[bytesRecv++] = c;
        buffer[bytesRecv] = 0;
        if (strstr(buffer, expected1)) {
            return 1;
        }
        if (expected2 && strstr(buffer, expected2)) {
            return 2;
        }
    }
    return (millis() - checkTimer < timeout) ? 0 : 3;
}

void CGPRS_SIM800::purgeSerial()
{
  while (simser.available()) simser.read();
}

