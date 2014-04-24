#define FORMAT_BIN 0
#define FORMAT_CSV 1

typedef struct {
    uint32_t time;
    uint16_t pid;
    uint8_t flags;
    uint8_t checksum;
    float value;
} LOG_DATA;

typedef struct {
    uint32_t time;
    uint16_t pid;
    uint8_t flags;
    uint8_t checksum;
    float value[3];
} LOG_DATA_COMM;

typedef struct {
    uint32_t time; /* e.g. 1307281259 */
    uint16_t pid;
    uint8_t message;
    uint8_t checksum;
    uint16_t fileIndex;
    uint16_t fileSize; /* KB */
    uint16_t logFlags;
    uint8_t logType;
    uint8_t data[5];
} LOG_DATA_FILE_INFO;

typedef struct {
    uint32_t time;
    uint16_t pid;
    uint8_t message;
    uint8_t checksum;
    uint8_t data[12];
} LOG_DATA_COMMAND;

typedef struct {
    uint32_t id;
    uint32_t dataOffset;
    uint8_t ver;
    uint8_t logType;
    uint16_t flags;
    uint32_t dateTime; //4, YYMMDDHHMM, e.g. 1305291359
    /*
    uint8_t devid[8];
    uint8_t vin[24];
    uint8_t unused[84];
    */
} HEADER;

#define HEADER_LEN 128 /* bytes */

#define PID_GPS_COORDINATES 0xF00A
#define PID_GPS_ALTITUDE 0xF00C
#define PID_GPS_SPEED 0xF00D
#define PID_GPS_HEADING 0xF00E
#define PID_GPS_SAT_COUNT 0xF00F
#define PID_GPS_TIME 0xF010

#define PID_ACC 0xF020
#define PID_GYRO 0xF021

#if LOG_FORMAT == FORMAT_BIN
#define FILE_NAME_FORMAT "/DAT%05d.LOG"
#else
#define FILE_NAME_FORMAT "/DAT%05d.CSV"
#endif

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

#define SerialBLE Serial

#endif

#endif

class CDataLogger {
public:
    void initSender()
    {
#if ENABLE_DATA_OUT
        SerialBLE.begin(STREAM_BAUDRATE);
#endif
#if ENABLE_DATA_LOG && LOG_FORMAT == FORMAT_CSV
        m_lastDataTime = 0;
#endif
    }
    void logData(char c)
    {
#if ENABLE_DATA_OUT && STREAM_FORMAT == FORMAT_CSV
        SerialBLE.write(c);
#endif
#if ENABLE_DATA_LOG
        if (sdfile) dataSize += sdfile.write(c);
#endif
    }
    void logData(uint16_t pid, int value)
    {
#if LOG_FORMAT == FORMAT_BIN || STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
#endif
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        SerialBLE.write((uint8_t*)&ld, 12);
#else
        SerialBLE.print(pid, HEX);
        SerialBLE.write(',');
        SerialBLE.print(value);
        SerialBLE.write('\n');
#endif
#endif
#if ENABLE_DATA_LOG
        if (!sdfile) return;
#if LOG_FORMAT == FORMAT_BIN
        sdfile.write((uint8_t*)&ld, 12);
        dataSize += 12;
#else
        dataSize += sdfile.print(dataTime - m_lastDataTime);
        sdfile.write(',');
        dataSize += sdfile.print(pid, HEX);
        sdfile.write(',');
        dataSize += sdfile.print(value);
        sdfile.write('\n');
        dataSize += 3;
        m_lastDataTime = dataTime;
#endif
#endif
    }
    void logData(uint16_t pid, float value)
    {
#if LOG_FORMAT == FORMAT_BIN || STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
#endif
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        SerialBLE.write((uint8_t*)&ld, 12);
#else
        SerialBLE.print(pid, HEX);
        SerialBLE.write(',');
        SerialBLE.print(value);
        SerialBLE.write('\n');
#endif
#endif
#if ENABLE_DATA_LOG
        if (!sdfile) return;
#if LOG_FORMAT == FORMAT_BIN
        sdfile.write((uint8_t*)&ld, 12);
        dataSize += 12;
#else
        dataSize += sdfile.print(dataTime - m_lastDataTime);
        sdfile.write(',');
        dataSize += sdfile.print(pid, HEX);
        sdfile.write(',');
        dataSize += sdfile.print(value);
        sdfile.write('\n');
        dataSize += 3;
        m_lastDataTime = dataTime;
#endif
#endif
    }
    void logData(uint16_t pid, float value1, float value2)
    {
#if LOG_FORMAT == FORMAT_BIN || STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 2, 0, {value1, value2}};
        ld.checksum = getChecksum((char*)&ld, 16);
#endif
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        SerialBLE.write((uint8_t*)&ld, 16);
#else
        SerialBLE.print(pid, HEX);
        SerialBLE.write(',');
        SerialBLE.print(value1, 6);
        SerialBLE.write(',');
        SerialBLE.print(value2, 6);
        SerialBLE.write('\n');
#endif
#endif
#if ENABLE_DATA_LOG
        if (!sdfile) return;
#if LOG_FORMAT == FORMAT_BIN
        sdfile.write((uint8_t*)&ld, 16);
        dataSize += 16;
#else
        dataSize += sdfile.print(dataTime - m_lastDataTime);
        sdfile.write(',');
        dataSize += sdfile.print(pid, HEX);
        sdfile.write(',');
        dataSize += sdfile.print(value1, 6);
        sdfile.write(',');
        dataSize += sdfile.print(value2, 6);
        sdfile.write('\n');
        dataSize += 4;
        m_lastDataTime = dataTime;
#endif
#endif
    }
    void logData(uint16_t pid, uint32_t value1, uint32_t value2)
    {
#if LOG_FORMAT == FORMAT_BIN || STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 2, 0, {value1, value2}};
        ld.checksum = getChecksum((char*)&ld, 16);
#endif
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        SerialBLE.write((uint8_t*)&ld, 16);
#else
        SerialBLE.print(pid, HEX);
        SerialBLE.write(',');
        SerialBLE.print(value1);
        SerialBLE.write(',');
        SerialBLE.print(value2);
        SerialBLE.write('\n');
#endif
#endif
#if ENABLE_DATA_LOG
        if (!sdfile) return;
#if LOG_FORMAT == FORMAT_BIN
        sdfile.write((uint8_t*)&ld, 16);
        dataSize += 16;
#else
        dataSize += sdfile.print(dataTime - m_lastDataTime);
        sdfile.write(',');
        dataSize += sdfile.print(pid, HEX);
        sdfile.write(',');
        dataSize += sdfile.print(value1);
        sdfile.write(',');
        dataSize += sdfile.print(value2);
        sdfile.write('\n');
        dataSize += 4;
        m_lastDataTime = dataTime;
#endif
#endif
    }
    void logData(uint16_t pid, int value1, int value2, int value3)
    {
#if LOG_FORMAT == FORMAT_BIN || STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 3, 0, {value1, value2, value3}};
        ld.checksum = getChecksum((char*)&ld, 20);
#endif
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        SerialBLE.write((uint8_t*)&ld, 20);
#else
        SerialBLE.print(pid, HEX);
        SerialBLE.write(',');
        SerialBLE.print(value1);
        SerialBLE.write(',');
        SerialBLE.print(value2);
        SerialBLE.write(',');
        SerialBLE.print(value3);
        SerialBLE.write('\n');
#endif
#endif
#if ENABLE_DATA_LOG
        if (!sdfile) return;
#if LOG_FORMAT == FORMAT_BIN
        sdfile.write((uint8_t*)&ld, 20);
        dataSize += 20;
#else
        dataSize += sdfile.print(dataTime - m_lastDataTime);
        sdfile.write(',');
        dataSize += sdfile.print(pid, HEX);
        sdfile.write(',');
        dataSize += sdfile.print(value1);
        sdfile.write(',');
        dataSize += sdfile.print(value2);
        sdfile.write(',');
        dataSize += sdfile.print(value3);
        sdfile.write('\n');
        dataSize += 5;
        m_lastDataTime = dataTime;
#endif
#endif
    }
#if ENABLE_DATA_LOG
    uint16_t openFile(uint16_t logFlags = 0, uint32_t dateTime = 0)
    {
        uint16_t fileIndex;
        char filename[24] = "/FRMATICS";

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

#if LOG_FORMAT == FORMAT_BIN
        HEADER hdr = {'UDUS', HEADER_LEN, 1, 0, logFlags, dateTime};
        sdfile.write((uint8_t*)&hdr, sizeof(hdr));
        for (byte i = 0; i < HEADER_LEN - sizeof(hdr); i++)
            sdfile.write((uint8_t)0);
        dataSize = HEADER_LEN;
#endif
        return fileIndex;
    }
    void closeFile()
    {
        sdfile.close();
    }
    void flushFile()
    {
        if (sdfile) sdfile.flush();
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
    File sdfile;
#if LOG_FORMAT == FORMAT_CSV
    uint32_t m_lastDataTime;
#endif
#endif
};
