#define PID_RPM 0x10C
#define PID_SPEED 0x10D
#define PID_THROTTLE 0x111
#define PID_ENGINE_LOAD 0x104
#define PID_COOLANT_TEMP 0x105
#define PID_INTAKE_TEMP 0x10F
#define PID_MAF_FLOW 0x110
#define PID_ABS_ENGINE_LOAD 0x143
#define PID_AMBIENT_TEMP 0x146
#define PID_FUEL_PRESSURE 0x10A
#define PID_INTAKE_PRESSURE 0x10B
#define PID_BAROMETRIC 0x133
#define PID_TIMING_ADVANCE 0x10E
#define PID_FUEL_LEVEL 0x12F
#define PID_RUNTIME 0x11F
#define PID_DISTANCE 0x131

#define PID_TIME_DATE 0xF001
#define PID_TIME_TIME 0xF002
#define PID_TIME_PLAY_SPEED 0xF003

#define PID_STAT_0_60 0xF100
#define PID_STAT_0_100 0xF101
#define PID_STAT_0_160 0xF102
#define PID_STAT_0_400 0xF103
#define PID_STAT_CUR_LAP 0xF110
#define PID_STAT_LAST_LAP 0xF111
#define PID_STAT_BEST_LAP 0xF112
#define PID_STAT_LAP_PROGRESS 0xF113

#define PID_COMMAND 0xFFFE
#define PID_SYNC 0xFFFF

enum {
	PID_STAT_DISTANCE = 0xF200,
	PID_STAT_TRIP_TIME,
	PID_STAT_WAIT_TIME,
	PID_STAT_SPEED_MAX,
	PID_STAT_SPEED_AVG,
	PID_STAT_RPM_MAX,
	PID_STAT_RPM_MIN,
	PID_STAT_RPM_AVG,
	PID_STAT_INTAKE_MAX,
	PID_STAT_INTAKE_MIN,
	PID_STAT_INTAKE_SUM,
	PID_STAT_ACC_FORWARD,
	PID_STAT_ACC_BACKWARD,
};

enum {
	PID_LOCAL_DATA = 0xF300,
	PID_LOCAL_RATE,
	PID_REMOTE_DATA,
	PID_REMOTE_RATE,
	PID_TIME_REMAIN,
	PID_BATTERY,
	PID_DEVICE_TEMP,
};

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

#define PID_VIDEO_FRAME 0xFF00

typedef struct {
	uint32_t time;
	uint16_t pid;
	uint16_t flags;
	uint32_t value;
} LOG_DATA;

typedef struct {
	uint32_t id; //4
	uint32_t dataOffset; //4
	uint8_t ver; //1
	uint8_t ununsed; //1
	uint16_t flags; //2
	uint32_t date;
	uint32_t time;
	uint32_t startTick;
	uint32_t unused[2];
	uint64_t devid; //8
	uint8_t ununsed2[20];
	uint8_t vin[32]; //32
	int32_t videoOffset; //4
	uint8_t stats[84]; //84
	uint8_t reserved[72]; //72
	uint32_t tail; //4
} HEADER; //256
