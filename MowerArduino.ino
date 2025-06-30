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
//#include "src/communications/RemoteCommand.h"
//#include "src/communications/WiFiRemote.h"

// Create global LCDMenu instance
LCDMenu lcdMenu;

// Create global Mower instance with LCDMenu reference
Mower mower(lcdMenu);

// Create remote command instance
//RemoteCommand remoteCmd(mower);

// Create WiFi remote instance
//WiFiRemote wifiRemote(remoteCmd);

/**
 * @brief Arduino setup function - called once at startup
 * 
 * Initializes all system components and sets up the initial state.
 * This function is called once when the Arduino starts up.
 */
void setup() {
//    Wire.setClock(100000);
//    Wire.setWireTimeout(300 /* us */, true /* reset_on_timeout */);
    Wire.begin();

    #ifdef DEBUG_MODE
    // Inizializza la seriale debug solo se il debug è abilitato
    SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD);
    // Attendi che la seriale sia pronta (solo per alcune schede)
    // Piccola pausa per stabilizzare la connessione
    delay(100);
    // Invia caratteri di test per sincronizzazione
    SERIAL_DEBUG.println();
    #endif

    DEBUG_PRINTLN(F("=== Mower System Starting ==="));

    // Initialize the mower system
    mower.begin();
/*    
    // Inizializza il controllo remoto
    remoteCmd.begin();
    
    // Imposta i gestori degli eventi
    remoteCmd.setStatusUpdateHandler([](const struct RemoteStatus& status) {
        // Qui puoi gestire gli aggiornamenti di stato
        // Ad esempio, inviare notifiche o aggiornare display
        #ifdef ENABLE_DEBUG
        SERIAL_DEBUG.print("Status update - Battery: ");
        SERIAL_DEBUG.print(status.batteryLevel);
        SERIAL_DEBUG.print("% ");
        SERIAL_DEBUG.print("Moving: ");
        SERIAL_DEBUG.println(status.isMoving ? "Yes" : "No");
        #endif
    });
    
    // Inizializza la comunicazione WiFi
    wifiRemote.begin(SERIAL_WIFI_BAUD);
*/
    
    DEBUG_PRINTLN(F("System initialized"));
    DEBUG_PRINT(F("Initial state: "));
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
    
    /*    
    // Aggiorna il controllo remoto
    remoteCmd.update();
    // Gestisci la comunicazione WiFi
    wifiRemote.update();
    */    
    // Add a small delay to prevent watchdog resets
    delay(10);
}
