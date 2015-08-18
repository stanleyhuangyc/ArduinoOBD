Arduino OBD-II Adapter Library & Sketches
=========================================

(C)2012-2015 Freematics.com

OBD-II Adapter for Arduino is a product that works as a vehicle OBD-II data bridge for Arduino (literally all embedded platforms) with open-source Arduino library provided. Besides providing easy-to-use OBD-II data access, it also integrates 6-axis or 9-axis MEMS sensor module and a voltmeter for measuring vehicle battery power. The adapter draws power from OBD-II port and convert it to 5V for powering attached device.

OBD-II Adapter: http://freematics.com/pages/products/arduino-obd-adapter/

OBD-II Telematics DIY Kit: http://freematics.com/pages/products/arduino-telematics-kit-3/

![Image](http://www.arduinodev.com/wp-content/uploads/2012/03/obdkit1-150x150.jpg)

About the library
-----------------
Most commonly use PIDs are defined in OBD library as followings.

Engine

    PID_RPM – Engine RPM (rpm)
    PID_ENGINE_LOAD – Calculated engine load (%)
    PID_COOLANT_TEMP – Engine coolant temperature (°C)
    PID_ENGINE_LOAD – Calculated Engine load (%)
    PID_ABSOLUTE_ENGINE_LOAD – Absolute Engine load (%)
    PID_TIMING_ADVANCE – Ignition timing advance (°)
    PID_ENGINE_OIL_TEMP – Engine oil temperature (°C)
    PID_ENGINE_TORQUE_PERCENTAGE – Engine torque percentage (%)
    PID_ENGINE_REF_TORQUE – Engine reference torque (Nm)

Intake/Exhaust

    PID_INTAKE_TEMP – Intake temperature (°C)
    PID_INTAKE_PRESSURE – Intake manifold absolute pressure (kPa)
    PID_MAF_FLOW – MAF flow pressure (grams/s)
    PID_BAROMETRIC – Barometric pressure (kPa)

Speed/Time

    PID_SPEED – Vehicle speed (km/h)
    PID_RUNTIME – Engine running time (second)
    PID_DISTANCE – Vehicle running distance (km)

Driver

    PID_THROTTLE – Throttle position (%)
    PID_AMBIENT_TEMP – Ambient temperature (°C)

Electric Systems

    PID_CONTROL_MODULE_VOLTAGE – vehicle control module voltage (V)
    PID_HYBRID_BATTERY_PERCENTAGE – Hybrid battery pack remaining life (%)

Additional defines can be added to access other OBD-II PIDs.

Directory Descriptions
----------------------

libraries - all Arduino libraries needed for OBD-II adapter and kits

samples - several simple sketches for testing purpose

nanologger - OBD-II data logger working with 128x64 monochrome OLED display (for Arduino Nano)

megalogger - OBD-II and GPS data logger based on 320x240 TFT LCD display (for Arduino MEGA)

tester - a testing sketch for OBD-II communication and capability

utilites - useful utility source code for development

