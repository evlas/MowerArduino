#include "setup.h"
#include "../../config.h"
#include "../../pin_config.h"
#include <Wire.h>

// Include GPS
#ifdef ENABLE_GPS
#include "../../src/sensors/GPS.h"
extern GPSModule gps;
#endif

// Include Battery Monitor
#ifdef ENABLE_BATTERY_MONITOR
#include "../../src/sensors/BatteryMonitor.h"
#endif

unsigned long lastSensorUpdate = 0;
unsigned long lastNavigationUpdate = 0;
unsigned long lastSafetyCheck = 0;

// Include IMU
#ifdef ENABLE_IMU
#include <MPU6050.h>
extern MPU6050 imu;
#endif

// Include Motor Controller
#ifdef ENABLE_DRIVE_MOTORS
#include "../../src/motors/MotorController.h"
extern MotorController motorController;
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
    // Inizializza e configura l'INA226
//    if (!batteryMonitor.begin()) {
//        SERIAL_DEBUG.println("ERROR: Failed to initialize INA226!");
//        while(1); // Blocca se non riesce a inizializzare
//    }
    
    // Configura il sensore
//    batteryMonitor.setBusVoltageRange(INA226_WE::BusVoltageRange::BVOLTAGERANGE_32V);
//    batteryMonitor.setPGA(INA226_WE::PGA::PGA_8_320MV);
//    batteryMonitor.setBusADCResolution(INA226_WE::BusADCResolution::ADCRESOLUTION_12BIT);
//    batteryMonitor.setShuntADCResolution(INA226_WE::ShuntADCResolution::ADCRESOLUTION_12BIT);
//    batteryMonitor.setMode(INA226_WE::Mode::MODE_SANDBVOLT_CONTINUOUS);
    #endif

    #ifdef ENABLE_PERIMETER
//        perimeterManager.begin();
    #endif

    #ifdef ENABLE_IMU
        imu.initialize();
    #endif
    
    #ifdef ENABLE_GPS
        gps.begin();
    #endif

    #ifdef ENABLE_ODOMETRY
    if (!sensorFusion.initOdometry(WHEEL_DIAMETER, TICKS_PER_REVOLUTION)) {
        // Gestione errore
    }
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
