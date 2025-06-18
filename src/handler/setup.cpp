#include "setup.h"
#include "../../config.h"
#include "../../pin_config.h"
#include <Wire.h>

// Include GPS
#ifdef ENABLE_GPS
#include "../../src/sensors/GPS.h"
#endif

// Include IMU
#ifdef ENABLE_IMU
#include "../../src/sensors/IMU.h"
#endif

// Include Battery Monitor
#ifdef ENABLE_BATTERY_MONITOR
#include "../../src/sensors/BatteryMonitor.h"
#endif

unsigned long lastSensorUpdate = 0;
unsigned long lastNavigationUpdate = 0;
unsigned long lastSafetyCheck = 0;

// Include Motor Controller
#ifdef ENABLE_DRIVE_MOTORS
#include "../../src/motors/MotorController.h"
#include "../../src/functions/Maneuver.h"
#endif

#ifdef ENABLE_BATTERY_MONITOR
#include "../../src/sensors/BatteryMonitor.h"
#endif

#ifdef ENABLE_DISPLAY
#include "../../src/LCD/LCDManager.h"
#endif

#ifdef ENABLE_BUZZER
#include "../../src/actuators/Buzzer.h"
#endif

#ifdef ENABLE_RELAY
#include "../../src/actuators/Relay.h"
#endif

#ifdef ENABLE_BLADE_MOTORS
#include "../../src/motors/BladeController.h"
#endif

#ifdef ENABLE_WIFI
#include "../../src/communications/WiFiController.h"
#include "../../src/communications/CommandHandler.h"
#endif

#ifdef ENABLE_NAVIGATION
#include "../../src/functions/Navigation.h"
#endif

#ifdef ENABLE_SAFETY
#include "../../src/safety/SafetyManager.h"
#endif

#ifdef ENABLE_CHARGING
#include "../../src/functions/ChargingSystem.h"
#endif

#ifdef ENABLE_SCHEDULE
#include "../../src/functions/Scheduler.h"
#endif

#ifdef ENABLE_ULTRASONIC
#include "../../src/sensors/UltrasonicSensors.h"
#endif

#ifdef ENABLE_BUMP_SENSORS
#include "../../src/sensors/BumpSensors.h"
#endif

#ifdef ENABLE_PERIMETER
#include "../../src/sensors/PerimeterSensors.h"
#endif

#ifdef ENABLE_RAIN_SENSOR
#include "../../src/sensors/RainSensor.h"
#endif

#ifdef ENABLE_RELAY
#include "../../src/actuators/Relay.h"
#endif

MowerState currentState = IDLE;

void my_setup() {
    // Serial initialization
    #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD);
        while (!SERIAL_DEBUG) {
            ; // Wait for serial port to connect
        }
    #endif

    Wire.begin();

    // Initialize components
    #ifdef ENABLE_BATTERY_MONITOR
        batteryMonitor = INA226_WE(BATTERY_MONITOR_ADDRESS);
        
        if (!batteryMonitor.init()) {
            SERIAL_DEBUG.println("ERROR: Failed to initialize INA226!");
            while(1); // Block if initialization fails
        }   
        batteryMonitor.setAverage(AVERAGE_16);
        batteryMonitor.setConversionTime(CONV_TIME_1100);
        batteryMonitor.setMeasureMode(CONTINUOUS);
        batteryMonitor.setCorrectionFactor(0.95);
    #endif

    // Initialize motors
    #ifdef ENABLE_DRIVE_MOTORS
        motorController.begin();
        mowerManeuver.begin();
    #endif

    // Initialize sensors
    #ifdef ENABLE_ULTRASONIC
        ultrasonicSensors.begin();
    #endif

    #ifdef ENABLE_BUMP_SENSORS
        bumpSensors.begin();
    #endif

    #ifdef ENABLE_IMU
        imu.begin();
    #endif

    // Initialize GPS
    #ifdef ENABLE_GPS
        SERIAL_GPS.begin(SERIAL_GPS_BAUD);
        gps.begin();
    #endif

    // Initialize perimeter
    #ifdef ENABLE_PERIMETER
        perimeterSensors.begin();
    #endif

    // Initialize rain sensor
    #ifdef ENABLE_RAIN_SENSOR
        rainSensor.begin();
    #endif

    // Initialize actuators
    #ifdef ENABLE_BUZZER
        buzzer.begin();
    #endif

    #ifdef ENABLE_DISPLAY
        lcdManager.begin();
    #endif

    #ifdef ENABLE_RELAY
        relay.begin();
    #endif

    // Initialize blade motors
    #ifdef ENABLE_BLADE_MOTORS
        bladeController.begin();
    #endif

    // Initialize communication
    #ifdef ENABLE_WIFI
        SERIAL_WIFI.begin(SERIAL_WIFI_BAUD);
    #endif

    // Initialize navigation
    #ifdef ENABLE_NAVIGATION
        navigation.begin();
    #endif

    // Set initial state
    currentState = IDLE;
}
