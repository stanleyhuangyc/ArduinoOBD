// configurations
#define ENABLE_DATA_OUT 0
#define ENABLE_DATA_LOG 1

typedef enum {
    LOG_TYPE_DEFAULT = 0,
    LOG_TYPE_0_60,
    LOG_TYPE_0_100,
    LOG_TYPE_100_200,
    LOG_TYPE_400M,
    LOG_TYPE_LAPS,
    LOG_TYPE_ROUTE,
} LOG_TYPES;

#define FLAG_CAR 0x1
#define FLAG_CYCLING 0x2
#define FLAG_OBD 0x10
#define FLAG_GPS 0x20
#define FLAG_ACC 0x40

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

#define PID_MESSAGE 0xFE00

#define MSG_FILE_LIST_BEGIN 0x1
#define MSG_FILE_LIST_END 0x2
#define MSG_FILE_INFO 0x3
#define MSG_FILE_REQUEST 0x4

#define FILE_NAME_FORMAT "/DAT%05d.LOG"

#if ENABLE_DATA_OUT
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    SoftwareSerial mySerial(A8, A9); /* for BLE Shield on MEGA*/
#elif defined(__AVR_ATmega644P__)
    SoftwareSerial mySerial(9, 10); /* for Microduino */
#else
    SoftwareSerial mySerial(A2, A3); /* for BLE Shield on UNO*/
#endif
#endif

class CDataLogger {
public:
    void initSender()
    {
#if ENABLE_DATA_OUT
        mySerial.begin(9600);
#endif
    }
#if ENABLE_DATA_OUT
    void sendFileInfo(File& file)
    {
        if (file.size() < HEADER_LEN) return;

        LOG_DATA_FILE_INFO info = {0};
        info.fileIndex = atol(file.name() + 3);
        if (info.fileIndex == 0) return;

        HEADER hdr;
        if (file.readBytes((char*)&hdr, sizeof(hdr)) != sizeof(hdr)) return;

        info.pid = PID_MESSAGE;
        info.message = MSG_FILE_INFO;
        info.fileSize = file.size();
        info.time = hdr.dateTime;
        info.logType = hdr.logType;
        info.logFlags = hdr.flags;
        info.checksum = getChecksum((char*)&info, sizeof(info));
        mySerial.write((uint8_t*)&info, sizeof(info));
    }
    void sendCommand(byte message, void* data = 0, byte bytes = 0)
    {
        LOG_DATA_COMMAND msg = {0, PID_MESSAGE, message};
        if (data) memcpy(msg.data, data, bytes);
        msg.checksum = getChecksum((char*)&msg, sizeof(msg));
        mySerial.write((uint8_t*)&msg, sizeof(msg));
    }
    bool receiveCommand(LOG_DATA_COMMAND& msg)
    {
        if (!mySerial.available())
            return false;

        if (mySerial.readBytes((char*)&msg, sizeof(msg)) != sizeof(msg))
            return false;

        uint8_t checksum = msg.checksum;
        msg.checksum = 0;
        if (getChecksum((char*)&msg, sizeof(msg)) != msg.checksum) {
            return false;
        }
        return true;
    }
#endif
    void logData(uint16_t pid, float value)
    {
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
#if ENABLE_DATA_OUT
        mySerial.write((uint8_t*)&ld, 12);
#endif
#if ENABLE_DATA_LOG
        sdfile.write((uint8_t*)&ld, 12);
#endif
        dataSize += 12;
    }
    void logData(uint16_t pid, float value1, float value2)
    {
        LOG_DATA_COMM ld = {dataTime, pid, 2, 0, {value1, value2}};
        ld.checksum = getChecksum((char*)&ld, 16);
#if ENABLE_DATA_OUT
        mySerial.write((uint8_t*)&ld, 16);
#endif
#if ENABLE_DATA_LOG
        sdfile.write((uint8_t*)&ld, 16);
#endif
        dataSize += 16;
    }
    void logData(uint16_t pid, float value1, float value2, float value3)
    {
        LOG_DATA_COMM ld = {dataTime, pid, 3, 0, {value1, value2, value3}};
        ld.checksum = getChecksum((char*)&ld, 20);
#if ENABLE_DATA_OUT
        mySerial.write((uint8_t*)&ld, 20);
#endif
#if ENABLE_DATA_LOG
        sdfile.write((uint8_t*)&ld, 20);
#endif
        dataSize += 20;
    }
#if ENABLE_DATA_LOG
    uint16_t openFile(LOG_TYPES logType, uint16_t logFlags = 0, uint32_t dateTime = 0)
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

        HEADER hdr = {'UDUS', HEADER_LEN, 1, logType, logFlags, dateTime};
        sdfile.write((uint8_t*)&hdr, sizeof(hdr));
        for (byte i = 0; i < HEADER_LEN - sizeof(hdr); i++)
            sdfile.write((uint8_t)0);
        dataSize = HEADER_LEN;
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
    static byte getChecksum(char* buffer, byte len)
    {
        uint8_t checksum = 0;
        for (byte i = 0; i < len; i++) {
          checksum ^= buffer[i];
        }
        return checksum;
    }
#if ENABLE_DATA_LOG
    File sdfile;
#endif
};
