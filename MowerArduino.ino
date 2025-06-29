/**
 * @file MowerArduino.ino
 * @brief Main file for the Arduino Mower project
 * @version 2.0
 * @date 2025
 * 
 * @copyright Copyright (c) 2025
 * 
 * This is the main entry point for the Arduino-based robotic lawn mower system.
 * The project uses a state pattern for managing different operating modes.
 * 
 * Project Structure:
 * - config.h: Global configuration settings
 * - pin_config.h: Pin configuration for all hardware components
 * - src/functions: Core functionality modules
 * - src/states: State implementations for the state machine
 * - src/sensors: Sensor interfaces and drivers
 * - src/actuators: Motor and actuator control
 * - src/communications: Communication protocols and interfaces
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Include configuration
#include "src/config.h"
#include "src/pin_config.h"

// Include system classes
#include "src/functions/Mower.h"

// Create global Mower instance
Mower mower;

/**
 * @brief Arduino setup function - called once at startup
 * 
 * Initializes all system components and sets up the initial state.
 * This function is called once when the Arduino starts up.
 */
void setup() {
    // Initialize serial communication for debugging
    #ifdef ENABLE_DEBUG
    SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD);
    while (!SERIAL_DEBUG) {
        ; // Wait for serial port to connect (for native USB)
    }
    SERIAL_DEBUG.println(F("\n=== Mower System Starting ==="));
    #endif
    
    // Initialize the mower system
    mower.init();
    
    #ifdef ENABLE_DEBUG
    SERIAL_DEBUG.println(F("System initialized"));
    SERIAL_DEBUG.print(F("Initial state: "));
    SERIAL_DEBUG.println(mower.stateToString(mower.getState()));
    #endif
}

/**
 * @brief Arduino main loop - called repeatedly
 * 
 * This is the main program loop that runs continuously after setup().
 * It updates the mower's state and handles all operations.
 */
void loop() {
    // Update the mower (this will handle state updates, sensors, etc.)
    mower.update();
    
    // Add a small delay to prevent watchdog resets
    delay(10);
}
