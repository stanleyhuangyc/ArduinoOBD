#include <Arduino.h>
#include <Wire.h>
#include <OBD.h>
#include <MPU6050.h>
#include <SPI.h>
//#include <MultiLCD.h>

//LCD_ILI9341 lcd;

#define CON Serial

bool hasAAC = false;

COBDI2C obd;

bool initACC()
{
    if (MPU6050_init() != 0)
        return false;
    return true;

}
void processACC()
{
    accel_t_gyro_union data;
    MPU6050_readout(&data);
    CON.print('[');
    CON.print(millis());
    CON.print(']');
    // log x/y/z of accelerometer
    CON.print(" AX=");
    CON.print(data.value.x_accel);
    CON.print(" AY=");
    CON.print(data.value.y_accel);
    CON.print(" AZ=");
    CON.println(data.value.z_accel);
    // log x/y/z of gyro meter
    CON.print('[');
    CON.print(millis());
    CON.print(']');
    CON.print(" GX=");
    CON.print(data.value.x_gyro);
    CON.print(" GY=");
    CON.print(data.value.y_gyro);
    CON.print(" GZ=");
    CON.println(data.value.z_gyro);
}

void setup()
{
    CON.begin(115200);
    //lcd.begin();

    CON.println("OBD TESTER");

    obd.begin();

    hasAAC = initACC();
    if (hasAAC)
        CON.println("MPU6050 detected");

    for (;;) {
        CON.println("Connecting...");
        if (obd.init()) break;
        delay(1000);
    }
    CON.println("Connected");

    obd.gpsStart(38400);

    //obd.btInit(9600);
}

byte getChecksum(char* buffer, byte len)
{
    uint8_t checksum = 0;
    for (byte i = 0; i < len; i++) {
      checksum ^= buffer[i];
    }
    return checksum;
}

typedef struct {
    uint32_t time;
    uint16_t pid;
    uint8_t flags;
    uint8_t checksum;
    float value;
} LOG_DATA;

void sendData(uint16_t pid, int value)
{
    LOG_DATA ld = {millis(), pid, 1, 0, value};
    ld.checksum = getChecksum((char*)&ld, sizeof(LOG_DATA));
    obd.btSend((byte*)&ld, sizeof(LOG_DATA));
}

void loop()
{
    int value;

    //lcd.setCursor(0, 8);

    CON.print('[');
    CON.print(millis());
    CON.print(']');
    if (obd.read(PID_RPM, value, false)) {
        //sendData(0x100 | PID_RPM, value);
        CON.print(" RPM:");
        CON.print(value);
    }
    if (obd.read(PID_SPEED, value, false)) {
        //sendData(0x100 | PID_SPEED, value);
        CON.print(" SPD:");
        CON.print(value);
    }
    if (obd.read(PID_THROTTLE, value, false)) {
        //sendData(0x100 | PID_THROTTLE, value);
        CON.print(" THR:");
        CON.print(value);
    }
    CON.println("");
    if (hasAAC) {
        processACC();
    }
    //delay(500);

    GPS_DATA gpsdata;
    if (obd.gpsQuery(&gpsdata)) {
        CON.print('[');
        CON.print(millis());
        CON.print(']');
        CON.print(" LAT:");
        CON.print(gpsdata.lat, 6);
        CON.print(" LON:");
        CON.print(gpsdata.lon, 6);
        CON.print(" ALT:");
        CON.print(gpsdata.alt, 1);
        CON.print(" SAT:");
        CON.print(gpsdata.sat);
        CON.print(" AGE:");
        CON.print(gpsdata.age);
        CON.print(" TIME:");
        CON.println(gpsdata.time);
    }
    //delay(500);
}
