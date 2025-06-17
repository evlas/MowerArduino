/*
 * Robot Tagliaerba Arduino Mega - File Principale
 * Versione Modulare Riorganizzata
 *
 * Struttura file:
 * - config.h (configurazione generale)
 * - pin_config.h (configurazione pin)
 * - setup.ino (setup)
 * - loop.ino (loop)
 * - MowerArduino.ino (file principale)
 * - src/functions (cartella funzioni)
 * - src/sensors (cartella sensori)
 * - src/motors (cartella motori)
 * - src/actuators (cartella attuatori)
 * - src/communications (cartella comunicazioni)
 * - src/LCD (cartella LCD)
 */

#include <Arduino.h>
#include <Wire.h>

// Include configuration
#include "config.h"
#include "pin_config.h"

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

#ifdef ENABLE_IMU
#include <MPU6050.h>
#include "src/sensors/IMU.h"
IMU imu;
#endif

#ifdef ENABLE_PERIMETER
#include "src/sensors/PerimeterSensors.h"
#include "src/functions/PerimeterManager.h"
PerimeterSensors perimeterSensors;
PerimeterManager perimeterManager(&motorController, &buzzer, &lcdManager);
#endif

#ifdef ENABLE_RAIN_SENSOR
#include "src/sensors/RainSensor.h"
RainSensor rainSensor;
#endif

#ifdef ENABLE_BATTERY_MONITOR
#include <INA226_WE.h>
#include "src/sensors/BatteryMonitor.h"
BatteryMonitor batteryMonitor;
#endif

#ifdef ENABLE_GPS
#include <TinyGPS++.h>
#include "src/sensors/GpsSensor.h"
GpsSensor gpsSensor;
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
#include "src/actuators/RelayController.h"
RelayController relayController;
#endif

// COMMUNICATIONS
#ifdef ENABLE_WIFI
#include <ArduinoJson.h>
#include "src/communications/WiFiController.h"
#include "src/communications/CommandHandler.h"

// Initialize WiFi serial
WiFiController wifi(&SERIAL_WIFI, &commandHandler);
#endif

// FUNCTIONS
// Include moduli funzionalità (solo se abilitati)
#ifdef ENABLE_NAVIGATION
#include "src/functions/Navigation.h"
Navigation navigationSystem(&motorController, &bladeController, &buzzer, &lcdManager);
#endif

#ifdef ENABLE_SAFETY
#include "src/functions/SafetySystem.h"
SafetySystem safetySystem;
#endif

#ifdef ENABLE_CHARGING
#include "src/functions/ChargingSystem.h"
ChargingSystem chargingSystem;
#endif

#ifdef ENABLE_SCHEDULE
#include "src/functions/Scheduler.h"
Scheduler scheduler;
#endif

// RTC
#ifdef ENABLE_RTC
#include <DS1302.h>
#endif

void setup() {

  my_setup();
}

void loop() {
  my_loop();
}
