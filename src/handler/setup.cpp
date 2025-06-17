#include "setup.h"
#include "../../config.h"
#include "../../pin_config.h"
#include "../../src/motors/MotorController.h"
#include <Wire.h>

unsigned long lastSensorUpdate = 0;
unsigned long lastNavigationUpdate = 0;
unsigned long lastSafetyCheck = 0;

extern MotorController motorController;  // Dichiarazione della variabile esterna

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
        batteryMonitor.begin();
    #endif

    #ifdef ENABLE_PERIMETER
//        perimeterManager.begin();
    #endif

    #ifdef ENABLE_IMU
//        imu.begin();
    #endif

    #ifdef ENABLE_SCHEDULE
//        scheduler.begin();
    #endif

    #ifdef ENABLE_DISPLAY
//        lcdManager.begin();
    #endif

    #ifdef ENABLE_WIFI
//        wifi.begin();
    #endif

    #ifdef ENABLE_DRIVE_MOTORS
        if (!motorController.begin()) {
    #ifdef SERIAL_DEBUG 
            Serial.println("Errore nell'inizializzazione del MotorController");
    #endif
        }   
    #endif

    #ifdef ENABLE_BLADE_MOTOR
//        bladeController.begin();
    #endif

    #ifdef ENABLE_BUMP_SENSORS
//        bumpSensors.begin();
    #endif

    #ifdef ENABLE_ULTRASONIC
//        ultrasonicSensors.begin();
    #endif

    #ifdef ENABLE_RAIN_SENSOR
//        rainSensor.begin();
    #endif

    #ifdef ENABLE_GPS
//        gpsSensor.begin();
    #endif

    #ifdef ENABLE_BUZZER
//        buzzer.begin();
    #endif

    #ifdef ENABLE_RELAY
//        relayController.begin();
    #endif

    // Initialize navigation system
    #ifdef ENABLE_NAVIGATION
//        navigationSystem.begin();
    #endif

    // Initialize state machine
    currentState = IDLE;
}
