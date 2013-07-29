// configurations
#define ENABLE_DATA_OUT 1
#define ENABLE_DATA_LOG 1

typedef struct {
    uint32_t /*DWORD*/ time;
    uint16_t /*WORD*/ pid;
    uint8_t /*BYTE*/ flags;
    uint8_t /*BYTE*/ checksum;
    float value;
} LOG_DATA;

typedef struct {
    uint32_t /*DWORD*/ time;
    uint16_t /*WORD*/ pid;
    uint8_t /*BYTE*/ flags;
    uint8_t /*BYTE*/ checksum;
    float value[3];
} LOG_DATA_COMM;

#define PID_GPS_COORDINATES 0xF00A
#define PID_GPS_ALTITUDE 0xF00C
#define PID_GPS_SPEED 0xF00D
#define PID_GPS_HEADING 0xF00E
#define PID_GPS_SAT_COUNT 0xF00F
#define PID_GPS_TIME 0xF010

#define PID_ACC 0xF020
#define PID_GYRO 0xF021

#define FILE_NAME_FORMAT "DAT%05d.LOG"

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
    bool openFile(uint16_t index)
    {
        char filename[13];
        sprintf(filename, FILE_NAME_FORMAT, index);
        sdfile = SD.open(filename, FILE_WRITE);
        if (!sdfile) {
            return false;
        }

        uint32_t d;
        d = 'UDUS';
        sdfile.write((uint8_t*)&d, 4); // id
        d = 256;
        sdfile.write((uint8_t*)&d, 4); // offset
        sdfile.write((uint8_t)0x10); // VER
        sdfile.write((uint8_t)0);
        sdfile.write((uint8_t)0);
        sdfile.write((uint8_t)0);
        d = 0;
        sdfile.write((uint8_t*)&d, 4); // date
        sdfile.write((uint8_t*)&d, 4); // time
        for (byte i = 0; i < 256 - 20; i++)
            sdfile.write((uint8_t)0);

        dataSize = 256;
        return true;
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
