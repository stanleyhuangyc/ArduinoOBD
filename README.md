Arduino OBD-II Adapter Library & Examples
=========================================

(C)2012-2013 Stanley Huang

The Arduino OBD-II Adapter is a product that works as a vehicle OBD-II data bridge for Arduino with open-source Arduino library provided. Besides providing OBD-II data access, it also provides power supply (converted and regulated from OBD-II port) for Arduino and its attached devices.

Product page: http://arduinodev.com/hardware/obd-kit/

![Image](http://www.arduinodev.com/wp-content/uploads/2012/03/obdkit1-150x150.jpg)

About the library
-----------------
In current version of the library, following OBD-II PIDs are defined:

    Vehicle speed (PID_SPEED)
    Engine RPM (PID_RPM)
    Throttle position (PID_THROTTLE)
    Calculated Engine load (PID_ENGINE_LOAD)
    Absolute Engine load (PID_ABS_ENGINE_LOAD)
    Engine coolant temperature (PID_COOLANT_TEMP)
    Intake temperature (PID_INTAKE_TEMP)
    Intake MAP (PID_INTAKE_PRESSURE)
    MAF flow pressure (PID_MAF_FLOW)
    Fuel pressure (PID_FUEL_PRESSURE)
    Fuel level (PID_FUEL_LEVEL)
    Barometric pressure (PID_BAROMETRIC)
    Ignition timing advance (PID_TIMING_ADVANCE)
    Engine running time (PID_RUNTIME)
    Vehicle running distance (PID_DISTANCE)

Additional defines can be added to access all OBD-II PIDs which the car's ECU provides. 

Directory Descriptions
----------------------

libraries/OBD - Arduino library for OBD-II adapter

samples/rpm_led - a simplest example sketch implementing a RPM indicator with the pin 13 LED on Arduino board

samples/dashboard_1602 - an example sketch showing a set of vehicle data with a LCD1602 shield

samples/dashboard_4884 - a sketch providing extensive display of vehicle data with a LCD4884 shield

samples/dashboard_oled - a sketch providing extensive display of vehicle data with a OLED module

samples/obdtest - a testing sketch for OBD-II adapter

obdlogger - a complete OBD-II and GPS data logger and timer based on 128x64 OLED display

megalogger - a complete OBD-II and GPS data logger based on Arduino MEGA and TFT LCD shield

How to view logged data
-----------------------
Data2KML (http://arduinodev.com/data2kml-utility/) is an open-source command line utility which converts data logged by obdlogger or megalogger to KML file loading in Google Earth.

A web service (http://freematics.com/chart/) is provided to view data logged by obdlogger or megalogger.
