#include <Arduino.h>
#include "setup.h"
#include "../../config.h"
#include "../../pin_config.h"
#include <Wire.h>

// Dichiarazione esterna di commandHandler
#ifdef ENABLE_WIFI
#include "../../src/communications/CommandHandler.h"
extern CommandHandler commandHandler;
#endif

// Include EEPROM
#ifdef ENABLE_EEPROM
#include "../../src/eeprom/EEPROMConfig.h"
#include "../../src/eeprom/EEPROMManager.h"
#endif

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

// Le variabili globali sono definite in loop.cpp
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
#include "../../src/communications/WiFiSerialBridge.h"
#include "../../src/communications/CommandHandler.h"
#include "../../src/communications/WiFiCommands.h"

// La variabile wifiBridge è definita in WiFiSerialBridge.cpp
extern WiFiSerialBridge wifiBridge;
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

// La macchina a stati è ora gestita dalla classe StateMachine
// mowerStateMachine è definita in StateMachine.cpp

void setupMower() {
    // Inizializza la comunicazione seriale
    #ifdef SERIAL_DEBUG
    delay(2000);
        SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD);
        SERIAL_DEBUG.println(F("Inizializzazione Robot Tagliaerba..."));
    #endif

    // Inizializza I2C
    Wire.begin();
    #ifdef SERIAL_DEBUG
    SERIAL_DEBUG.println(F("Wire OK"));   // subito dopo Wire.begin()
    #endif
    #ifdef ENABLE_DISPLAY
    uint8_t bootStep = 0;
    const uint8_t BOOT_TOTAL_STEPS = 10; // aggiorna qui se aggiungi altri passi
    lcdManager.updateBootProgress(++bootStep / (float)BOOT_TOTAL_STEPS);
    #endif

    // Inizializza il display
    #ifdef ENABLE_DISPLAY
        lcdManager.begin();
        lcdManager.showBootScreen();
        // dopo aver mostrato lo schermo di boot consideriamo già completato il primo step
        bootStep = 1; // progress 1/10
        lcdManager.updateBootProgress(bootStep / (float)BOOT_TOTAL_STEPS);
    #endif

    // Carica le impostazioni dalla EEPROM
    #ifdef ENABLE_EEPROM
        #ifdef SERIAL_DEBUG
            SERIAL_DEBUG.println(F("Caricamento impostazioni EEPROM..."));
        #endif
        
        EEPROMSettings settings;
        if (EEPROMManager::loadSettings(settings)) {
            #ifdef SERIAL_DEBUG
                SERIAL_DEBUG.println(F("Impostazioni caricate dalla EEPROM"));
            #endif
        } else {
            // Carica impostazioni di default se la EEPROM è vuota o corrotta
            EEPROMManager::loadDefaultSettings(settings);
            EEPROMManager::saveSettings(settings);
            #ifdef SERIAL_DEBUG
                SERIAL_DEBUG.println(F("Impostazioni di default caricate e salvate"));
            #endif
        }
        
        #ifdef ENABLE_DISPLAY
            lcdManager.updateBootProgress(++bootStep / (float)BOOT_TOTAL_STEPS);
        #endif
    #endif
    
    // Inizializza i pin
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Accendi il LED durante l'inizializzazione
    #ifdef ENABLE_DISPLAY
    lcdManager.updateBootProgress(++bootStep / (float)BOOT_TOTAL_STEPS);
    #endif

    // Inizializza il GPS
    #ifdef ENABLE_GPS
        gps.begin();
        #ifdef ENABLE_DISPLAY
        lcdManager.updateBootProgress(++bootStep / (float)BOOT_TOTAL_STEPS);
        #endif
    #endif

    // Inizializza l'IMU
    #ifdef ENABLE_IMU
        imu.begin();
        #ifdef ENABLE_DISPLAY
        lcdManager.updateBootProgress(++bootStep / (float)BOOT_TOTAL_STEPS);
        #endif
    #endif

    // Inizializza il monitor della batteria
    #ifdef ENABLE_BATTERY_MONITOR
        batteryMonitor = INA226_WE(BATTERY_MONITOR_ADDRESS);
        if (!batteryMonitor.init()) {
            SERIAL_DEBUG.println("ERRORE: Impossibile inizializzare INA226!");
        }
        batteryMonitor.setAverage(AVERAGE_16);
        batteryMonitor.setConversionTime(CONV_TIME_1100);
        batteryMonitor.setMeasureMode(CONTINUOUS);
        batteryMonitor.setCorrectionFactor(0.95);
        #ifdef ENABLE_DISPLAY
        lcdManager.updateBootProgress(++bootStep / (float)BOOT_TOTAL_STEPS);
        #endif
    #endif

    // Inizializza il controller dei motori
    #ifdef ENABLE_DRIVE_MOTORS
        motorController.begin();
        #ifdef ENABLE_DISPLAY
        lcdManager.updateBootProgress(++bootStep / (float)BOOT_TOTAL_STEPS);
        #endif
    #endif

    // Inizializza il controller delle lame
    #ifdef ENABLE_BLADE_MOTORS
        bladeController.begin();
        #ifdef ENABLE_DISPLAY
        lcdManager.updateBootProgress(++bootStep / (float)BOOT_TOTAL_STEPS);
        #endif
    #endif

    // Inizializza il modulo WiFi
    #ifdef ENABLE_WIFI
        wifiBridge.begin();
        
        // Inizializza il gestore dei comandi WiFi
        initWiFiCommands(wifiBridge, commandHandler);
        
        #ifdef SERIAL_DEBUG
            SERIAL_DEBUG.println("WiFi Serial Bridge inizializzato");
        #endif
        #ifdef ENABLE_DISPLAY
        lcdManager.updateBootProgress(++bootStep / (float)BOOT_TOTAL_STEPS);
        #endif
    #endif

    // Inizializza il buzzer
    #ifdef ENABLE_BUZZER
        buzzer.begin();
        // Suono di conferma all'avvio
        buzzer.buttonPressSound();
        #ifdef ENABLE_DISPLAY
        lcdManager.updateBootProgress(++bootStep / (float)BOOT_TOTAL_STEPS);
        #endif
    #endif

    // Inizializza i relay
    #ifdef ENABLE_RELAY
        relay.begin();
    #endif

    // Inizializza la macchina a stati
    mowerStateMachine.begin();
    
    // Imposta lo stato iniziale a IDLE
    mowerStateMachine.sendEvent(MowerEvent::STOP_MOWING);

    #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.println("Inizializzazione completata");
    #endif

    #ifdef ENABLE_DISPLAY
    lcdManager.updateBootProgress(1.0f); // Completa la barra di avanzamento
    lcdManager.showSetupComplete();
    #endif

    // Spegni il LED di inizializzazione
    digitalWrite(LED_BUILTIN, LOW);
}
