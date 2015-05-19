/******************************************************************************
* Vehicle Telematics Remote Data Logger Sketch (SIM900/GPRS)
* Developed by Stanley Huang <stanleyhuangyc@gmail.com>
* Distributed under GPL v2.0
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <OBD.h>
#if ENABLE_DATA_LOG
#include <SD.h>
#endif
#include <I2Cdev.h>
#include <MPU9150.h>
#include "config.h"
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_SLEEPING 0x20

static uint32_t lastFileSize = 0;
static uint16_t fileIndex = 0;
static uint32_t startTime = 0;
static uint32_t lastGPSAccess = 0;
static uint32_t lastGPSTime = 0;
static uint32_t lastGPSTimeSent = 0;
static uint32_t dataCount = 0;

static byte pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static byte pidTier2[] = {PID_INTAKE_MAP, PID_MAF_FLOW, PID_TIMING_ADVANCE};
static byte pidTier3[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_FUEL_LEVEL};

static int pidValueTier1[sizeof(pidTier1)];
static int pidValueTier2[sizeof(pidTier2)];
static int pidValueTier3[sizeof(pidTier3)];

#if USE_MPU6050 || USE_MPU9150
static int16_t ax = 0, ay = 0, az = 0;
static int16_t gx = 0, gy = 0, gz = 0;
#if USE_MPU9150
static int16_t mx = 0, my = 0, mz = 0;
#endif
static int temp = 0;
#endif

static GPS_DATA gd = {0};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)
#define TIER_NUM3 sizeof(pidTier3)

byte pidValue[TIER_NUM1];

#if USE_MPU6050 || USE_MPU9150
MPU6050 accelgyro;
static uint32_t lastMemsDataTime = 0;
#endif

#define sim Serial
#define GSM_ON              6
#define GSM_RESET           7

char gpsline[OBD_RECV_BUF_SIZE] = {0};

typedef enum {
    HTTP_DISABLED = 0,
    HTTP_READY,
    HTTP_CONNECTING,
    HTTP_READING,
    HTTP_ERROR,
} HTTP_STATES;

class CGPRS {
public:
    CGPRS():httpState(HTTP_DISABLED) {}
    bool init()
    {
        sim.begin(9600);
        pinMode(GSM_ON, OUTPUT);
        pinMode(GSM_RESET, OUTPUT);

        if (sendCommand("ATE0")) {
            digitalWrite(GSM_RESET, HIGH);
            delay(1000);
            // generate turn on pulse
            digitalWrite(GSM_RESET, LOW);
            delay(100);
            sendCommand("ATE0");
            return true;
        } else {
            for (byte n = 0; n < 5; n++) {
              digitalWrite(GSM_ON, HIGH);
              delay(1000);
              // generate turn on pulse
              digitalWrite(GSM_ON, LOW);
              delay(3000);
              if (sendCommand("ATE0"))
                return true;
            }
            return false;
        }
    }
    bool setup(const char* apn)
    {
        while (sendCommand("AT+CREG?", "+CREG: 0,1", "+CREG: 0,5", 2000) == 0);
        sendCommand("AT+CSQ");
        sendCommand("AT+CGATT?");

        if (!sendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\""))
            return false;

        sim.print("AT+SAPBR=3,1,\"APN\",\"");
        sim.print(apn);
        sim.println('\"');
        if (!sendCommand(0))
            return false;

        while (!sendCommand("AT+SAPBR=1,1"));
        return true;
    }
    bool getOperatorName()
    {
        // display operator name
        if (sendCommand("AT+COPS?", "OK\r", "ERROR\r") == 1) {
            char *p = strstr(response, ",\"");
            if (p) {
                p += 2;
                char *s = strchr(p, '\"');
                if (s) *s = 0;
                strcpy(response, p);
                return true;
            }
        }
        return false;
    }
    void httpUninit()
    {
      sendCommand("AT+HTTPTERM");
    }

    bool httpInit()
    {
      if  (!sendCommand("AT+HTTPINIT", 10000) || !sendCommand("AT+HTTPPARA=\"CID\",1", 5000)) {
        httpState = HTTP_DISABLED;
        return false;
      }
      httpState = HTTP_READY;
      return true;
    }
    bool httpConnect(const char* url, const char* args = 0)
    {
        // Sets url
        sim.print("AT+HTTPPARA=\"URL\",\"");
        sim.print(url);
        if (args) {
            sim.print('?');
            sim.print(args);
        }

        sim.println('\"');
        if (sendCommand(0))
        {
            // Starts GET action
            sim.println("AT+HTTPACTION=0");
            httpState = HTTP_CONNECTING;
            bytesRecv = 0;
            checkTimer = millis();
        } else {
            httpState = HTTP_ERROR;
        }
        return false;
    }
    byte httpIsConnected()
    {
        byte ret = checkResponse("+HTTPACTION:0,200", "+HTTPACTION:0,6", 10000);
        if (ret == 1) {
            return 1;
        } else if (ret >= 2) {
            httpState = HTTP_ERROR;
        }
        return false;
    }
    void httpRead()
    {
        sim.println("AT+HTTPREAD");
        httpState = HTTP_READING;
        bytesRecv = 0;
        checkTimer = millis();
    }
    bool httpIsRead()
    {
        byte ret = checkResponse("+HTTPREAD:", "Error", 10000) == 1;
        if (ret == 1) {
            bytesRecv = 0;
            sendCommand(0);
            byte n = atoi(response);
            char *p = strchr(response, '\n');
            if (p) memmove(response, p + 1, n);
            response[n] = 0;
            return 1;
        } else if (ret >= 2) {
            httpState = HTTP_ERROR;
        }
        return false;
    }
    char response[256];
    byte bytesRecv;
    byte httpState;
    uint32_t checkTimer;
private:
    byte checkResponse(const char* expected1, const char* expected2 = 0, unsigned int timeout = 2000)
    {
        while (sim.available()) {
            char c = sim.read();
            if (bytesRecv >= sizeof(response) - 1) {
                // buffer full, discard first half
                bytesRecv = sizeof(response) / 2 - 1;
                memcpy(response, response + sizeof(response) / 2, bytesRecv);
            }
            response[bytesRecv++] = c;
            response[bytesRecv] = 0;
            if (strstr(response, expected1)) {
                return 1;
            }
            if (expected2 && strstr(response, expected2)) {
                return 2;
            }
        }
        return (millis() - checkTimer < timeout) ? 0 : 3;
    }
    byte sendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = 0)
    {
      if (cmd) {
        while (sim.available()) sim.read();
        sim.println(cmd);
      }
      uint32_t t = millis();
      byte n = 0;
      do {
        if (sim.available()) {
          char c = sim.read();
          if (n >= sizeof(response) - 1) {
            // buffer full, discard first half
            n = sizeof(response) / 2 - 1;
            memcpy(response, response + sizeof(response) / 2, n);
          }
          response[n++] = c;
          response[n] = 0;
          if (strstr(response, expected ? expected : "OK\r")) {
           return n;
          }
        }
      } while (millis() - t < timeout);
      return 0;
    }
    byte sendCommand(const char* cmd, const char* expected1, const char* expected2, unsigned int timeout = 2000)
    {
      if (cmd) {
        while (sim.available()) sim.read();
        sim.println(cmd);
      }
      uint32_t t = millis();
      byte n = 0;
      do {
        if (sim.available()) {
          char c = sim.read();
          if (n >= sizeof(response) - 1) {
            // buffer full, discard first half
            n = sizeof(response) / 2 - 1;
            memcpy(response, response + sizeof(response) / 2, n);
          }
          response[n++] = c;
          response[n] = 0;
          if (strstr(response, expected1)) {
           return 1;
          }
          if (strstr(response, expected2)) {
           return 2;
          }
        }
      } while (millis() - t < timeout);
      return 0;
    }
};

class COBDLogger : public COBDI2C, public CDataLogger
{
public:
    COBDLogger():state(0) {}
    void setup()
    {
        SerialRF.print("#GPRS...");
        if (gprs.init()) {
            SerialRF.println("OK");
        } else {
            SerialRF.println(gprs.response);
        }

        SerialRF.print("#OBD..");
        do {
            SerialRF.print('.');
        } while (!init());
        SerialRF.println("OK");

        state |= STATE_OBD_READY;

#if USE_MPU6050 || USE_MPU9150
        Wire.begin();
        accelgyro.initialize();
        if (accelgyro.testConnection()) state |= STATE_MEMS_READY;
#endif

        SerialRF.print("#GPS...");
        if (initGPS()) {
            SerialRF.println("OK");
            state |= STATE_GPS_READY;
        } else {
            SerialRF.println("N/A");
        }
        delay(3000);

#if ENABLE_DATA_LOG
        uint16_t index = openFile();
        if (index) {
            SerialRF.print("#SD File:");
            SerialRF.println(index);
            state |= STATE_SD_READY;
        } else {
            SerialRF.print("#No SD");
            state &= ~STATE_SD_READY;
        }
#endif

        SerialRF.print("#Network...");
        if (gprs.setup("connect")) {
            SerialRF.println("OK");
        } else {
            SerialRF.print(gprs.response);
        }
        if (gprs.getOperatorName()) {
            SerialRF.print('#');
            SerialRF.println(gprs.response);
        }
        // init HTTP
        SerialRF.print("#HTTP...");
        while (!gprs.httpInit()) {
          SerialRF.print('.');
          gprs.httpUninit();
          delay(1000);
        }
        delay(3000);
    }
    void loop()
    {
        static byte index = 0;
        static byte index2 = 0;
        static byte index3 = 0;
        uint32_t start = millis();

        // poll OBD-II PIDs
        pidValueTier1[index] = logOBDData(pidTier1[index]);
        if (++index == TIER_NUM1) {
            index = 0;
            if (index2 == TIER_NUM2) {
                index2 = 0;
                pidValueTier3[index] = logOBDData(pidTier3[index3]);
                index3 = (index3 + 1) % TIER_NUM3;
            } else {
                pidValueTier2[index2] = logOBDData(pidTier2[index2]);
                index2++;
            }
        }

        if (state & STATE_GPS_READY) {
            if (millis() - lastGPSAccess > GPS_DATA_INTERVAL) {
                logGPSData();
                lastGPSAccess = millis();
            }
        }

#if ENABLE_DATA_LOG
        // flush SD data every 1KB
        if (dataSize - lastFileSize >= 1024) {
            flushFile();
            lastFileSize = dataSize;
            // display logged data size
            SerialRF.print("#Log Size:");
            SerialRF.println(dataSize);
        }
#endif

        if (errors >= 2) {
            reconnect();
        }

#ifdef OBD_MIN_INTERVAL
        while (millis() - start < OBD_MIN_INTERVAL) {
            dataIdleLoop();
        }
#endif
    }
private:
    void dataIdleLoop()
    {
        switch (gprs.httpState) {
        case HTTP_READY:
            {
                // generate URL
                char *p = gprs.response;
                p += sprintf(p, "C=%lu&", ++dataCount);
                for (byte n = 0; n < sizeof(pidTier1); n++) {
                    p += sprintf(p, "%x=%d&", pidTier1[n], pidValueTier1[n]);
                }
                p += sprintf(p, "A=%d,%d,%d&G=%d,%d,%d&M=%d,%d,%d", ax, ay, az, gx, gy, gz, mx, my, mz);
                //p += sprintf(p, "&T=%d", temp);
                if (gd.time && gd.time != lastGPSTimeSent) {
                    p += sprintf(p, "&GPS=%lu,%ld,%ld,%d,%d,%d", gd.time, gd.lat, gd.lon, gd.alt / 100, (int)gd.speed, gd.sat);
                    lastGPSTimeSent = gd.time;
                }

                #if 0
                char *q = strchr(gpsline, ',');
                if (q) {
                    char *s = strchr(q, '\r');
                    if (s) *s = 0;
                    strcat(p, q + 1);
                }
                #endif
                gprs.httpConnect("http://home.mediacoder.net.au:8000/tick", gprs.response);
            }
            break;
        case HTTP_CONNECTING:
            if (gprs.httpIsConnected()) {
                gprs.httpRead();
            }
            break;
        case HTTP_READING:
            if (gprs.httpIsRead()) {
                SerialRF.print("#HTTP:");
                SerialRF.println(gprs.response);
                // ready for next connection
                gprs.httpState = HTTP_READY;
            }
            break;
        case HTTP_ERROR:
            SerialRF.println("#HTTP ERROR");
            // re-initialize HTTP
            /*
            gprs.httpUninit();
            if (gprs.httpInit())
                gprs.httpState = HTTP_READY;
            */
            gprs.httpState = HTTP_READY;
            break;
        }
#if USE_MPU6050 || USE_MPU9150
        if (state & STATE_MEMS_READY) {
            processMEMS();
        }
#endif
    }
#if USE_MPU6050 || USE_MPU9150
    void processMEMS()
    {
        if (dataTime - lastMemsDataTime < ACC_DATA_INTERVAL) {
            return;
        }

    #if USE_MPU9150
        accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    #else
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    #endif

        dataTime = millis();

        temp = accelgyro.getTemperature();

        ax /= ACC_DATA_RATIO;
        ay /= ACC_DATA_RATIO;
        az /= ACC_DATA_RATIO;
        gx /= GYRO_DATA_RATIO;
        gy /= GYRO_DATA_RATIO;
        gz /= GYRO_DATA_RATIO;
    #if USE_MPU9150
        mx /= COMPASS_DATA_RATIO;
        my /= COMPASS_DATA_RATIO;
        mz /= COMPASS_DATA_RATIO;
    #endif

        // log x/y/z of accelerometer
        logData(PID_ACC, ax, ay, az);
        // log x/y/z of gyro meter
        logData(PID_GYRO, gx, gy, gz);
    #if USE_MPU9150
        // log x/y/z of compass
        logData(PID_COMPASS, mx, my, mz);
    #endif
        logData(PID_MEMS_TEMP, temp);

        lastMemsDataTime = dataTime;
    }
#endif
    int logOBDData(byte pid)
    {
        int value = 0;
        // send a query to OBD adapter for specified OBD-II pid
        if (read(pid, value)) {
            dataTime = millis();
            // log data to SD card
            logData(0x100 | pid, value);
        }
        return value;
    }
    bool logGPSData()
    {
        if (getGPSData(&gd) && gd.lat && gd.lon && gd.time != lastGPSTime) {
            logData(PID_GPS_TIME, gd.time);
            logData(PID_GPS_ALTITUDE, gd.lat);
            logData(PID_GPS_LONGITUDE, gd.lon);
            logData(PID_GPS_ALTITUDE, gd.alt);
            lastGPSTime = gd.time;
            return true;
        }
        return false;
    }
    void reconnect()
    {
#if ENABLE_DATA_LOG
        closeFile();
#endif
        SerialRF.println("#Sleeping");
        startTime = millis();
        state &= ~STATE_OBD_READY;
        state |= STATE_SLEEPING;
        //digitalWrite(SD_CS_PIN, LOW);
        for (uint16_t i = 0; ; i++) {
            if (init()) {
                int value;
                if (read(PID_RPM, value) && value > 0)
                    break;
            }
        }
        SerialRF.println("#Resuming");
        state &= ~STATE_SLEEPING;
        fileIndex++;
        recover();
        setup();
    }
    byte state;
    CGPRS gprs;
};

static COBDLogger logger;

void setup()
{
    logger.begin();
    logger.initSender();
    logger.setup();
}

void loop()
{
    logger.loop();
}
