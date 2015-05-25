/*************************************************************************
* SIM800 GPRS/HTTP Library
* Distributed under GPL v2.0
* Written by Stanley Huang <stanleyhuangyc@gmail.com>
* For more information, please visit http://arduinodev.com
*************************************************************************/

#include <Arduino.h>

// change this to the pin connect with SIM800 reset pin
#define SIM800_RESET_PIN 7

// change this to the serial UART which SIM800 is attached to
#define simser Serial1

// change this to the serial UART used for console
#define con Serial

// change this to 1 to enable debug information output
#define DEBUG 0

typedef enum {
    HTTP_DISABLED = 0,
    HTTP_READY,
    HTTP_CONNECTING,
    HTTP_READING,
    HTTP_ERROR,
} HTTP_STATES;

typedef struct {
  float lat;
  float lon;
  uint8_t year; /* year past 2000, e.g. 15 for 2015 */
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} GSM_LOCATION;

class CGPRS_SIM800 {
public:
    CGPRS_SIM800():httpState(HTTP_DISABLED) {}
    bool init();
    byte setup(const char* apn);
    bool getOperatorName();
    bool checkSMS();
    int getSignalQuality();
    bool getLocation(GSM_LOCATION* loc);
    // initialize HTTP connection
    bool httpInit();
    // terminate  HTTP connection
    void httpUninit();
    // connect to HTTP server
    bool httpConnect(const char* url, const char* args = 0);
    // check if HTTP connection is established
    // return 0 for in progress, 1 for success, 2 for error
    byte httpIsConnected();
    // read data from HTTP connection
    void httpRead();
    // check if HTTP connection is established
    // return 0 for in progress, -1 for error, number of http payload bytes on success
    int httpIsRead();
    // send AT command and check for expected response
    byte sendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = 0);
    // send AT command and check for two possible responses
    byte sendCommand(const char* cmd, const char* expected1, const char* expected2, unsigned int timeout = 2000);
    // toggle low-power mode
    bool sleep(bool enabled)
    {
      return sendCommand(enabled ? "AT+CFUN=0" : "AT+CFUN=1");
    }
    // check if there is available serial data
    bool available()
    {
      return simser.available(); 
    }
    char buffer[256];
    byte httpState;
private:
    byte checkbuffer(const char* expected1, const char* expected2 = 0, unsigned int timeout = 2000);
    void purgeSerial();
    byte m_bytesRecv;
    uint32_t m_checkTimer;
};

