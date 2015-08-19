/*************************************************************************
* Arduino Data Logger Class
* Distributed under GPL v2.0
* Written by Stanley Huang <stanleyhuangyc@gmail.com>
* Visit http://freematics.com for more information
*************************************************************************/

#define FORMAT_BIN 0
#define FORMAT_CSV 1
#define FORMAT_TEXT 2

typedef struct {
    uint32_t time;
    uint16_t pid;
    uint8_t flags;
    uint8_t checksum;
    float value[3];
} LOG_DATA_COMM;

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
#define PID_MEMS_TEMP 0x23
#define PID_BATTERY_VOLTAGE 0x24

#define PID_DATA_SIZE 0x80

#define FILE_NAME_FORMAT "/DAT%05d.CSV"

#if ENABLE_DATA_OUT

#if USE_SOFTSERIAL
SoftwareSerial SerialRF(A2, A3);
#else
#define SerialRF Serial
#endif

#endif

#if ENABLE_DATA_LOG
static File sdfile;
#endif

typedef struct {
    uint16_t pid;
    char name[3];
} PID_NAME;

const PID_NAME pidNames[] PROGMEM = {
{PID_ACC, {'A','C','C'}},
{PID_GYRO, {'G','Y','R'}},
{PID_COMPASS, {'M','A','G'}},
{PID_GPS_LATITUDE, {'L','A','T'}},
{PID_GPS_LONGITUDE, {'L','O','N'}},
{PID_GPS_ALTITUDE, {'A','L','T'}},
{PID_GPS_SPEED, {'S','P','D'}},
{PID_GPS_HEADING, {'C','R','S'}},
{PID_GPS_SAT_COUNT, {'S','A','T'}},
{PID_GPS_TIME, {'T','I','M'}},
{PID_GPS_DATE, {'D','T','E'}},
{PID_BATTERY_VOLTAGE, {'B','A','T'}},
{PID_DATA_SIZE, {'D','A','T'}},
};

class CDataLogger {
public:
    void initSender()
    {
#if ENABLE_DATA_OUT
        SerialRF.begin(STREAM_BAUDRATE);
        /*
        SerialRF.print("AT+NAMEFreematics");
        delay(10);
        while (SerialRF.available()) SerialRF.read();
        SerialRF.println();
        */
        m_lastSendTime = 0;
#endif
#if ENABLE_DATA_LOG
        m_lastDataTime = 0;
#endif
    }
    void recordData(const char* buf, byte len)
    {
#if ENABLE_DATA_LOG
        dataSize += sdfile.print(dataTime - m_lastDataTime);
        dataSize += sdfile.write(',');
        dataSize += sdfile.write(buf, len);
        m_lastDataTime = dataTime;
#endif
    }
    void sendData(const char* buf, byte len)
    {
        SerialRF.write(buf, len);
#if MIN_DATA_INTERVAL
        uint32_t t = millis();
        uint32_t elapsed = t - m_lastSendTime;
        if (elapsed < MIN_DATA_INTERVAL) delay(MIN_DATA_INTERVAL - elapsed);
        m_lastSendTime = t;
#else
        delay(10);
#endif
    }
    void logData(char c)
    {
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
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%d\r", value) + n;
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
        sendData((char*)&ld, 12);
#else
        sendData(buf, len);
#endif
#endif
        recordData(buf, len);
    }
    void logData(uint16_t pid, int32_t value)
    {
        char buf[20];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%ld\r", value) + n;
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
        sendData((uint8_t*)&ld, 12);
#else
        sendData(buf, len);
#endif
#endif
        recordData(buf, len);
    }
    void logData(uint16_t pid, uint32_t value)
    {
        char buf[20];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%lu\r", value) + n;
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
        sendData((uint8_t*)&ld, 12);
#else
        sendData(buf, len);
#endif
#endif
        recordData(buf, len);
    }
    void logData(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%d,%d,%d\r", value1, value2, value3) + n;
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 3, 0, {value1, value2, value3}};
        ld.checksum = getChecksum((char*)&ld, 20);
        SerialRF.write((uint8_t*)&ld, 20);
#else
        sendData(buf, len);
#endif
#endif
        recordData(buf, len);
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
        m_lastDataTime = dateTime;
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
#if STREAM_FORMAT == FORMAT_TEXT
    byte translatePIDName(uint16_t pid, char* text)
    {
#if STREAM_FORMAT == FORMAT_TEXT
        for (uint16_t n = 0; n < sizeof(pidNames) / sizeof(pidNames[0]); n++) {
            uint16_t id = pgm_read_word(&pidNames[n].pid);
            if (pid == id) {
                memcpy_P(text, pidNames[n].name, 3);
                text[3] = ',';
                return 4;
            }
        }
#endif
        return sprintf(text, "%X,", pid);
    }
#endif
#if ENABLE_DATA_LOG
    uint32_t m_lastDataTime;
#endif
#if ENABLE_DATA_OUT
    uint32_t m_lastSendTime;
#endif
};
