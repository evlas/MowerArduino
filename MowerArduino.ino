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
#include "src/LCD/LCDMenu.h"
#include "src/functions/Mower.h"
#include "src/communications/RemoteCommand.h"
#include "src/communications/WiFiRemote.h"

// Create global LCDMenu instance
LCDMenu lcdMenu;

// Create global Mower instance and link it with LCDMenu
Mower mower(&lcdMenu);  // Passa il puntatore all'istanza LCDMenu

// Create remote command instance
RemoteCommand remoteCmd(mower);

// Create WiFi remote instance
WiFiRemote wifiRemote(remoteCmd, mower);

void setup() {
    // Inizializzazione seriale di debug
    #ifdef DEBUG_MODE
    // Inizializza la seriale di debug
    SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD);
    
    // Attendi che la porta sia pronta (solo su alcune schede)
    #ifndef ARDUINO_ARCH_AVR
    unsigned long startTime = millis();
    while (!SERIAL_DEBUG && (millis() - startTime < 3000)) {
        ; // Attendi la connessione della porta seriale (max 3 secondi)
    }
    #else
    // Su AVR, aspetta un po' per la stabilizzazione
    delay(2000);
    #endif
    
    // Invia un messaggio di benvenuto
    SERIAL_DEBUG.println("\n\n=== Mower System Starting ===");
    SERIAL_DEBUG.print("Firmware: ");
    SERIAL_DEBUG.println(FIRMWARE_VERSION);
    SERIAL_DEBUG.print("Build: ");
    SERIAL_DEBUG.println(__DATE__ " " __TIME__);
    SERIAL_DEBUG.print("Debug mode: ENABLED\n");
    SERIAL_DEBUG.println("======================\n");
    #endif

    DEBUG_PRINTLN("Inizializzazione sistema...");
    
    // Inizializza I2C
    Wire.begin();
    
    // Inizializza prima il menu LCD
    lcdMenu.setMower(&mower);
    lcdMenu.begin();
    
    // Poi inizializza il sistema mower
    mower.begin();

    // Inizializza la comunicazione WiFi
    wifiRemote.begin(SERIAL_WIFI_BAUD);
    
    DEBUG_PRINTLN("Sistema inizializzato");
    DEBUG_PRINT("Stato iniziale: ");
    DEBUG_PRINTLN(mower.stateToString(mower.getState()));
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
    
    // Aggiorna il menu LCD (stato e interazione pulsanti)
    lcdMenu.update();

    // Aggiorna il controllo remoto WiFi
    wifiRemote.update();
        
    // Add a small delay to prevent watchdog resets
    delay(10);
}
