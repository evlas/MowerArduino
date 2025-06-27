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
#include "src/config.h"
#include "src/pin_config.h"

// Include delle classi di sistema
#include "src/functions/Mower.h"

#include <LiquidCrystal_I2C.h>
#include <DS1302.h>

// Inizializza il RTC
DS1302 rtc(RTC_IO_PIN, RTC_SCLK_PIN, RTC_RST_PIN);

// Inizializza il display
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Initialize Mower with motor pins
Mower mower;

/**
 * @brief Arduino setup function - called once at startup
 * 
 * Initializes all system components and sets up the initial state.
 * This function is called once when the Arduino starts up.
 */
void setup() {
  mower.begin();
}

/**
 * @brief Arduino main loop - called repeatedly
 * 
 * This is the main program loop that runs continuously after setup().
 * It handles all the main functionality of the robotic mower.
 */
void loop() {
  mower.update();
}
