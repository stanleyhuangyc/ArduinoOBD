Arduino OBD-II Adapter Library & Sketches
=========================================

(C)2012-2014 Freematics.com

The Arduino OBD-II Adapter is a product that works as a vehicle OBD-II data bridge for Arduino with open-source Arduino library provided. Besides providing OBD-II data access, it also provides power supply (converted and regulated from OBD-II port) for Arduino and its attached devices.

OBD-II Adapter: http://arduinodev.com/hardware/obd-kit/
OBD-II Data Logger Kits: http://arduinodev.com/hardware/#logger_kit

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

unologger - OBD-II data logger working with 320x240 TFT LCD display (for Arduino UNO & Bluno)

megalogger - OBD-II and GPS data logger based on 320x240 TFT LCD display (for Arduino MEGA, ADK and DUE)

utilites - useful utility source code for development

How to view logged data
-----------------------
Data2KML (http://arduinodev.com/data2kml-utility/) is an open-source command line utility which converts data logged by obdlogger or megalogger to KML file loading in Google Earth.

A web service (http://freematics.com/chart/) is provided to view data logged by obdlogger or megalogger.
