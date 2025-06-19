#include "setup.h"
#include "loop.h"
#include "../../config.h"
#include "../../src/error/ErrorManager.h"

// Dichiarazione esterna di commandHandler
#ifdef ENABLE_WIFI
#include "../../src/communications/CommandHandler.h"
extern CommandHandler commandHandler;
#endif

// Includi i moduli necessari
#ifdef ENABLE_GPS
#include "../../src/sensors/GPS.h"
#endif

#ifdef ENABLE_BATTERY_MONITOR
#include <INA226_WE.h>
#endif

#ifdef ENABLE_ULTRASONIC
#include "../../src/sensors/UltrasonicSensors.h"
extern UltrasonicSensors ultrasonicSensors;
#endif

#ifdef ENABLE_PERIMETER
#include "../../src/sensors/PerimeterSensors.h"
extern PerimeterSensors perimeterSensors;
#endif

// Dichiarazione dell'istanza globale della macchina a stati
extern StateMachine mowerStateMachine;

// Definizione delle variabili di tempo
unsigned long lastBatteryUpdate = 0;
unsigned long lastSensorUpdate = 0;
unsigned long lastTelemetryUpdate = 0;
unsigned long lastNavigationUpdate = 0;

// Variabili per la gestione degli errori
static bool criticalError = false;
static unsigned long lastErrorTime = 0;

void loopMower() {
    unsigned long currentTime = millis();

    // 1. Gestione degli errori critici
    if (ErrorManager::hasCriticalError()) {
        if (!criticalError) {
            // Primo rilevamento di un errore critico
            criticalError = true;
            lastErrorTime = currentTime;
            mowerStateMachine.sendEvent(MowerEvent::EMERGENCY_STOP);
            
            #ifdef DEBUG_MODE
                SERIAL_DEBUG.println(F("Errore critico rilevato! Attesa risoluzione..."));
            #endif
        }
        
        // Se è passato abbastanza tempo, prova a ripristinare
        // TODO: Definire il timeout di recupero errori in config.h
        const unsigned long ERROR_RECOVERY_TIMEOUT = 60000; // 60 secondi
        if (currentTime - lastErrorTime > ERROR_RECOVERY_TIMEOUT) {
            if (ErrorManager::clearErrors()) {
                criticalError = false;
                mowerStateMachine.sendEvent(MowerEvent::ERROR_RESOLVED);
                
                #ifdef DEBUG_MODE
                    SERIAL_DEBUG.println(F("Errore risolto, ripristino operazioni normali"));
                #endif
            } else {
                lastErrorTime = currentTime; // Resetta il timer
            }
        }
        
        return; // Non eseguire altre operazioni in caso di errore critico
    }

    // 2. Aggiornamento dei sensori (ogni SENSOR_UPDATE_INTERVAL ms)
    if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
        lastSensorUpdate = currentTime;
        
        // Aggiornamento GPS se abilitato
        #ifdef ENABLE_GPS
            gps.update();
            // TODO: Implementare controllo validità GPS
            // if (!gps.isValid()) {
            //     ErrorManager::addError(ErrorCode::GPS_ERROR, "Errore aggiornamento GPS");
            // } else {
            //     #ifdef SERIAL_DEBUG
            //         gps.printDebug();
            //     #endif
            //     
            //     // Se il GPS è attivo e abbiamo una posizione valida, aggiorna la posizione
            //     // if (gps.isValid()) {
            //     //     Aggiorna la posizione nel sistema di navigazione
            //     //     navigation.updatePosition(gps.getLatitude(), gps.getLongitude());
            //     // }
            // }
        #endif
        
        // Aggiornamento sensori di ostacoli
        #ifdef ENABLE_ULTRASONIC
            // TODO: Implementare logica di rilevamento ostacoli
            // if (ultrasonicSensors.isObstacleDetected()) {
            //     mowerStateMachine.sendEvent(MowerEvent::OBSTACLE_DETECTED);
            //     
            //     #ifdef DEBUG_MODE
            //         SERIAL_DEBUG.println(F("Ostacolo rilevato!"));
            //     #endif
            // }
        #endif
        
        // Aggiornamento sensori perimetrali
        #ifdef ENABLE_PERIMETER
            // TODO: Implementare logica di rilevamento bordo
            // if (perimeterSensors.borderDetected()) {
            //     mowerStateMachine.sendEvent(MowerEvent::BORDER_DETECTED);
            //     
            //     #ifdef DEBUG_MODE
            //         SERIAL_DEBUG.println(F("Bordo rilevato!"));
            //     #endif
            // }
        #endif
    } // Fine del blocco di aggiornamento sensori
    
    // 3. Aggiornamento della batteria (meno frequente)
    if ((currentTime - lastBatteryUpdate) >= BATTERY_CHECK_INTERVAL) {
        lastBatteryUpdate = currentTime;
        
        #ifdef ENABLE_BATTERY_MONITOR
            // Leggi lo stato della batteria con gestione degli errori
            float voltage = 0.0f;
            float current = 0.0f;
            
            // TODO: Implementare lettura batteria
            // TODO: Implementare lettura batteria
            // if (batteryMonitor.readAndClearFlags() & INA226_MEASUREMENT_READY) {
            //     voltage = batteryMonitor.getBusVoltage();
            //     current = batteryMonitor.getCurrent_mA();
                
                // Se la tensione è sotto la soglia critica, attiva l'emergenza
                // TODO: Definire le soglie di tensione in config.h
                const float BATTERY_CRITICAL_VOLTAGE = 10.5f;
                const float BATTERY_LOW_VOLTAGE = 11.5f;
                
                if (voltage < BATTERY_CRITICAL_VOLTAGE) {
                    mowerStateMachine.sendEvent(MowerEvent::EMERGENCY_STOP);
                    ErrorManager::addError(ErrorCode::BATTERY_CRITICAL, 
                                         String("Tensione critica: ") + String(voltage, 2) + "V");
                } 
                // Se la tensione è bassa ma non critica, torna alla base
                else if (voltage < BATTERY_LOW_VOLTAGE) {
                    if (mowerStateMachine.getCurrentState() != MowerState::RETURN_TO_BASE) {
                        mowerStateMachine.sendEvent(MowerEvent::LOW_BATTERY);
                        ErrorManager::addError(ErrorCode::BATTERY_LOW, 
                                             String("Batteria scarica: ") + String(voltage, 2) + "V");
                    }
                }
                // Se la batteria è sufficientemente carica, rimuovi eventuali errori di batteria
                else if (voltage > BATTERY_LOW_VOLTAGE + 0.5f) { // Isteresi
                    ErrorManager::removeError(ErrorCode::BATTERY_LOW);
                    ErrorManager::removeError(ErrorCode::BATTERY_CRITICAL);
                }
                
                #ifdef DEBUG_MODE
                    SERIAL_DEBUG.print(F("Batteria: "));
                    SERIAL_DEBUG.print(voltage, 2);
                    SERIAL_DEBUG.print(F("V, "));
                    SERIAL_DEBUG.print(current, 0);
                    SERIAL_DEBUG.println(F("mA"));
                #endif
            // } else {
            //     ErrorManager::addError(ErrorCode::BATTERY_ERROR, "Errore lettura batteria");
            //     
            //     #ifdef DEBUG_MODE
            //         SERIAL_DEBUG.print(F("Errore: "));
            //         SERIAL_DEBUG.print(ErrorManager::getLastError());
            //         SERIAL_DEBUG.print(F(" - "));
            //         SERIAL_DEBUG.println(ErrorManager::getLastErrorDescription());
            //     #endif
            // }
        #endif
    }

    // 4. Aggiornamento della macchina a stati
    mowerStateMachine.update();
    
    // 5. Gestione telemetria (se abilitata)
    #ifdef ENABLE_WIFI
        // Aggiorna la telemetria se abilitata nel CommandHandler
        commandHandler.updateTelemetry();
    #endif
    
    // 6. Gestione del watchdog
    #ifdef ENABLE_WATCHDOG
        // Riavvia il watchdog per evitare il reset
        wdt_reset();
    #endif
}
