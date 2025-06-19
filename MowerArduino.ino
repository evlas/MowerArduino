/**
 * @file MowerArduino.ino
 * @brief Main file for the Arduino Mower project
 * @version 1.0
 * @date 2025
 * 
 * @copyright Copyright (c) 2025
 * 
 * This is the main entry point for the Arduino-based robotic lawn mower system.
 * The project is organized in a modular structure with the following components:
 * - config.h: Global configuration settings
 * - pin_config.h: Pin configuration for all hardware components
 * - setup.ino: System initialization code
 * - loop.ino: Main program loop
 * - src/functions: Core functionality modules
 * - src/sensors: Sensor interfaces and drivers
 * - src/motors: Motor control and drivers
 * - src/actuators: Actuator control (buzzers, relays, etc.)
 * - src/communications: Communication protocols and interfaces
 * - src/LCD: Display and user interface
 */

#include <Arduino.h>
#include <Wire.h>

// Include configuration
#include "config.h"
#include "pin_config.h"

// Include delle classi di sistema
#include "src/functions/StateMachine.h"

#include "src/handler/setup.h"
#include "src/handler/loop.h"

// SENSORS
#ifdef ENABLE_ULTRASONIC
#include "src/sensors/UltrasonicSensors.h"
UltrasonicSensors ultrasonicSensors;
#endif

#ifdef ENABLE_BUMP_SENSORS
#include "src/sensors/BumpSensors.h"
BumpSensors bumpSensors;
#endif 

#ifdef ENABLE_PERIMETER
#include "src/sensors/PerimeterSensors.h"
PerimeterSensors perimeterSensors;
#endif  

#ifdef ENABLE_IMU
#include "src/sensors/IMU.h"
IMUModule imu;
#endif

#ifdef ENABLE_GPS
#include "src/sensors/GPS.h"
GPSModule gps;
#endif

// RTC
#ifdef ENABLE_RTC
#include <DS1302.h>
DS1302 rtc(RTC_IO_PIN, RTC_SCLK_PIN, RTC_RST_PIN);
#endif

#ifdef ENABLE_RAIN_SENSOR
#include "src/sensors/RainSensor.h"
RainSensor rainSensor;
#endif

#ifdef ENABLE_BATTERY_MONITOR
#include <INA226_WE.h>
#include "src/sensors/BatteryMonitor.h"
INA226_WE batteryMonitor = INA226_WE(BATTERY_MONITOR_ADDRESS);
#endif

// MOTORS
#ifdef ENABLE_DRIVE_MOTORS
#include "src/motors/Motor.h"
#include "src/motors/MotorController.h"
#include "src/functions/Maneuver.h"
MotorController motorController;
Maneuver mowerManeuver(&motorController);
#endif

#ifdef ENABLE_BLADE_MOTOR
#include "src/motors/BladeController.h"
BladeController bladeController;
#endif

// ACTUATORS
#ifdef ENABLE_BUZZER
#include "src/actuators/Buzzer.h"
Buzzer buzzer;
#endif

#ifdef ENABLE_DISPLAY
#include <LiquidCrystal_I2C.h>
#include "src/LCD/LCDManager.h"
LCDManager lcdManager;
#endif

#ifdef ENABLE_RELAY
#include "src/actuators/Relay.h"
Relay relay;
#endif

// COMMUNICATIONS
#ifdef ENABLE_WIFI
#include <ArduinoJson.h>
#include "src/communications/WiFiSerialBridge.h"
#include "src/communications/CommandHandler.h"

// La variabile wifiBridge è definita in WiFiSerialBridge.cpp
extern WiFiSerialBridge wifiBridge;

// Dichiarazione anticipata
extern StateMachine mowerStateMachine;
#ifdef ENABLE_NAVIGATION
extern Navigation navigation;
#endif

// Definizione di commandHandler
CommandHandler commandHandler(&mowerStateMachine, 
#ifdef ENABLE_NAVIGATION
    &navigation
#else
    nullptr
#endif
);
#endif

// FUNCTIONS
// Include moduli funzionalità (solo se abilitati)
#ifdef ENABLE_NAVIGATION
#include "src/position/PositionManager.h"
#include "src/functions/Navigation.h"
PositionManager positionManager;
NavigationMode navigationMode;

// Initialize Navigation with proper perimeter handling
#ifdef ENABLE_PERIMETER
Navigation navigation(&mowerManeuver, 
                    &ultrasonicSensors, 
                    &bumpSensors, 
                    &perimeterSensors,
                    &positionManager);
#else
Navigation navigation(&mowerManeuver, 
                    &ultrasonicSensors, 
                    &bumpSensors, 
                    nullptr,
                    &positionManager);
#endif
#endif

#ifdef ENABLE_SAFETY
#include "src/safety/SafetyManager.h"
SafetyManager safety;
#endif

#ifdef ENABLE_CHARGING
#include "src/functions/ChargingSystem.h"
ChargingSystem chargingSystem;
#endif

#ifdef ENABLE_SCHEDULE
#include "src/functions/Scheduler.h"
Scheduler scheduler;
#endif

/**
 * @brief Arduino setup function - called once at startup
 * 
 * Initializes all system components and sets up the initial state.
 * This function is called once when the Arduino starts up.
 */
void setup()
{
    setupMower();
}

/**
 * @brief Arduino main loop - called repeatedly
 * 
 * This is the main program loop that runs continuously after setup().
 * It handles all the main functionality of the robotic mower.
 */
void loop()
{
    loopMower();
}
