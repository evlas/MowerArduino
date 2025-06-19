#include "setup.h"
#include "../../config.h"
#include "../../pin_config.h"
#include <Wire.h>

// Dichiarazione esterna di commandHandler
#ifdef ENABLE_WIFI
#include "../../src/communications/CommandHandler.h"
extern CommandHandler commandHandler;
#endif

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
        SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD);
        while (!SERIAL_DEBUG) {
            ; // Attendi che la porta seriale sia connessa
        }
        SERIAL_DEBUG.println("Inizializzazione Robot Tagliaerba...");
    #endif

    // Inizializza I2C
    Wire.begin();
    
    // Inizializza i pin
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Accendi il LED durante l'inizializzazione

    // Inizializza il GPS
    #ifdef ENABLE_GPS
        gps.begin();
    #endif

    // Inizializza l'IMU
    #ifdef ENABLE_IMU
        imu.begin();
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
    #endif

    // Inizializza il controller dei motori
    #ifdef ENABLE_DRIVE_MOTORS
        motorController.begin();
    #endif

    // Inizializza il controller delle lame
    #ifdef ENABLE_BLADE_MOTORS
        bladeController.begin();
    #endif

    // Inizializza il display
    #ifdef ENABLE_DISPLAY
        lcdManager.begin();
        // TODO: Implementare visualizzazione messaggio su LCD
        // lcdManager.display("Robot Avviato");
    #endif

    // Inizializza il modulo WiFi
    #ifdef ENABLE_WIFI
        wifiBridge.begin();
        
        // Inizializza il gestore dei comandi WiFi
        initWiFiCommands(wifiBridge, commandHandler);
        
        #ifdef SERIAL_DEBUG
            SERIAL_DEBUG.println("WiFi Serial Bridge inizializzato");
        #endif
    #endif

    // Inizializza il buzzer
    #ifdef ENABLE_BUZZER
        buzzer.begin();
        buzzer.beep(1000, 100); // Breve beep di conferma (frequenza 1000Hz, durata 100ms)
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

    // Spegni il LED di inizializzazione
    digitalWrite(LED_BUILTIN, LOW);
}
