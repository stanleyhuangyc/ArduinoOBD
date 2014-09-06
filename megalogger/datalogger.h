/*************************************************************************
* Arduino Data Logger Class
* Distributed under GPL v2.0
* Copyright (c) 2013-2014 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
* Visit http://freematics.com for more information
*************************************************************************/

#define FORMAT_BIN 0
#define FORMAT_CSV 1

typedef struct {
    uint32_t time;
    uint16_t pid;
    uint8_t flags;
    uint8_t checksum;
    float value[3];
} LOG_DATA_COMM;

#define HEADER_LEN 128 /* bytes */

#define PID_GPS_LATITUDE 0xA
#define PID_GPS_LONGITUDE 0xB
#define PID_GPS_ALTITUDE 0xC
#define PID_GPS_SPEED 0xD
#define PID_GPS_HEADING 0xE
#define PID_GPS_SAT_COUNT 0xF
#define PID_GPS_TIME 0x10
#define PID_GPS_DATE 0x11

#define PID_ACC 0x20
#define PID_GYRO 0x21
#define PID_COMPASS 0x22
#define PID_TEMP 0x23
#define PID_VOLTAGE 0x24

#define FILE_NAME_FORMAT "/DAT%05d.CSV"

#if ENABLE_DATA_OUT

#if USE_SOFTSERIAL

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    SoftwareSerial SerialBLE(A8, A9); /* for BLE Shield on MEGA*/
#elif defined(__AVR_ATmega644P__)
    SoftwareSerial SerialBLE(9, 10); /* for Microduino */
#else
    SoftwareSerial SerialBLE(A2, A3); /* for BLE Shield on UNO/leonardo*/
#endif

#else

#define SerialBLE Serial3

#endif

#endif

#if ENABLE_DATA_LOG
static File sdfile;
#endif

static const char* idstr = "FREEMATICS\r";

class CDataLogger {
public:
    void initSender()
    {
#if ENABLE_DATA_OUT
        SerialBLE.begin(STREAM_BAUDRATE);
        SerialBLE.print(idstr);
#endif
#if ENABLE_DATA_LOG
        m_lastDataTime = 0;
#endif
    }
    void logTimeElapsed()
    {
#if ENABLE_DATA_LOG
        dataSize += sdfile.print(dataTime - m_lastDataTime);
        sdfile.write(',');
        dataSize++;
        m_lastDataTime = dataTime;
#endif
    }
    void logData(char c)
    {
#if ENABLE_DATA_OUT && STREAM_FORMAT == FORMAT_CSV
        SerialBLE.write(c);
#endif
#if ENABLE_DATA_LOG
        if (c >= ' ') {
            sdfile.write(c);
            dataSize++;
        }
#endif
    }
    void logData(uint16_t pid, int value)
    {
        char buf[16];
        byte n = sprintf(buf, "%X,%d\r", pid, value);
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
        SerialBLE.write((uint8_t*)&ld, 12);
#else
        SerialBLE.write((uint8_t*)buf, n);
#endif
#endif
#if ENABLE_DATA_LOG
        logTimeElapsed();
        dataSize += sdfile.write((uint8_t*)buf, n);
#endif
    }
    void logData(uint16_t pid, int32_t value)
    {
        char buf[20];
        byte n = sprintf(buf, "%X,%ld\r", pid, value);
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
        SerialBLE.write((uint8_t*)&ld, 12);
#else
        SerialBLE.write((uint8_t*)buf, n);
#endif
#endif
#if ENABLE_DATA_LOG
        logTimeElapsed();
        dataSize += sdfile.write((uint8_t*)buf, n);
#endif
    }
    void logData(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte n = sprintf(buf, "%X,%d,%d,%d\r", pid, value1, value2, value3);
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 3, 0, {value1, value2, value3}};
        ld.checksum = getChecksum((char*)&ld, 20);
        SerialBLE.write((uint8_t*)&ld, 20);
#else
        SerialBLE.write((uint8_t*)buf, n);
#endif
#endif
#if ENABLE_DATA_LOG
        logTimeElapsed();
        dataSize += sdfile.write((uint8_t*)buf, n);
#endif
    }
#if ENABLE_DATA_LOG
    uint16_t openFile(uint16_t logFlags = 0, uint32_t dateTime = 0)
    {
        uint16_t fileIndex;
        char filename[24] = "/FRMATICS";

        dataSize = 0;
        if (SD.exists(filename)) {
            for (fileIndex = 1; fileIndex; fileIndex++) {
                sprintf(filename + 9, FILE_NAME_FORMAT, fileIndex);
                if (!SD.exists(filename)) {
                    break;
                }
            }
            if (fileIndex == 0)
                return 0;
        } else {
            SD.mkdir(filename);
            fileIndex = 1;
            sprintf(filename + 9, FILE_NAME_FORMAT, 1);
        }

        sdfile = SD.open(filename, FILE_WRITE);
        if (!sdfile) {
            return 0;
        }
        return fileIndex;
    }
    void closeFile()
    {
        sdfile.close();
    }
    void flushFile()
    {
        sdfile.flush();
    }
#endif
    uint32_t dataTime;
    uint32_t dataSize;
private:
    byte getChecksum(char* buffer, byte len)
    {
        uint8_t checksum = 0;
        for (byte i = 0; i < len; i++) {
          checksum ^= buffer[i];
        }
        return checksum;
    }
#if ENABLE_DATA_LOG
    uint32_t m_lastDataTime;
#endif
};
