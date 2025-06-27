#include "setup.h"
#include "loop.h"
#include "../../config.h"
#include "../../src/error/ErrorManager.h"

// Dichiarazione esterna di commandHandler
#ifdef ENABLE_WIFI
#include "../../src/communications/WiFiSerialBridge.h"
#include "../../src/communications/CommandHandler.h"
extern CommandHandler commandHandler;
#endif

// Includi i moduli necessari
#ifdef ENABLE_GPS
#include "../../src/sensors/GPS.h"
#endif

#ifdef ENABLE_BATTERY_MONITOR
#include "../battery/BatteryMonitor.h"
extern BatteryMonitor batteryMonitor;
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
unsigned long lastSensorUpdate = 0;
unsigned long lastTelemetryUpdate = 0;
unsigned long lastNavigationUpdate = 0;

#ifdef ENABLE_BATTERY_MONITOR
static unsigned long lastBatteryUpdate = 0;
#endif

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
    #ifdef ENABLE_BATTERY_MONITOR
    if ((currentTime - lastBatteryUpdate) >= BATTERY_CHECK_INTERVAL) {
        lastBatteryUpdate = currentTime;

        // Aggiorna le letture della batteria
        batteryMonitor.update();
        
        // Leggi lo stato della batteria
        float voltage = batteryMonitor.getVoltage();
        float current = batteryMonitor.getCurrent();
        float soc = batteryMonitor.getSOC();

        // Gestione transizioni quando la batteria è in carica (sulla base)
        if (batteryMonitor.isCharging()) {
            // Se siamo in MOWING, andiamo in RETURN_TO_BASE
            if (mowerStateMachine.getCurrentState() == MowerState::MOWING) {
                mowerStateMachine.sendEvent(MowerEvent::LOW_BATTERY);
            } 
            // Se siamo già in RETURN_TO_BASE e la tensione è stabile, andiamo in CHARGING
            else if (mowerStateMachine.getCurrentState() == MowerState::RETURN_TO_BASE) {
                // Aggiungi un piccolo ritardo per assicurarti che il robot sia effettivamente sulla base
                static unsigned long chargingStartTime = 0;
                if (chargingStartTime == 0) {
                    chargingStartTime = currentTime;
                } else if (currentTime - chargingStartTime > 2000) { // Aspetta 2 secondi
                    mowerStateMachine.sendEvent(MowerEvent::CHARGING_COMPLETE);
                    chargingStartTime = 0; // Reset per il prossimo ciclo
                }
            }
            // Se siamo in IDLE e la tensione è alta, andiamo in CHARGING
            else if (mowerStateMachine.getCurrentState() == MowerState::IDLE) {
                mowerStateMachine.sendEvent(MowerEvent::CHARGING_COMPLETE);
            }
        } else {
            // Reset del timer se la tensione scende
            static unsigned long chargingStartTime = 0;
            chargingStartTime = 0;
        }

        // Gestione degli eventi di batteria critica e bassa
        if (batteryMonitor.isCritical()) {
            mowerStateMachine.sendEvent(MowerEvent::EMERGENCY_STOP);
            ErrorManager::addError(ErrorCode::BATTERY_CRITICAL,
                               String("Tensione critica: ") + String(voltage, 2) + "V");
        } else if (batteryMonitor.isLow()) {
            if (mowerStateMachine.getCurrentState() != MowerState::RETURN_TO_BASE) {
                mowerStateMachine.sendEvent(MowerEvent::LOW_BATTERY);
                ErrorManager::addError(ErrorCode::BATTERY_LOW,
                                   String("Batteria scarica: ") + String(soc, 1) + "% (" + 
                                   String(voltage, 2) + "V)");
            }
        }
        // }
    }
    #endif

    // 4. Aggiornamento della macchina a stati
    mowerStateMachine.update();
    
    // 4.1 Aggiornamento accelerazione/decelerazione motori
    mowerManeuver.updateAcceleration();
    
    // 5. Gestione telemetria (se abilitata)
    #ifdef ENABLE_WIFI
        // Process incoming WiFi commands continuously
        if (wifiBridge.available()) {
            WiFiCommand cmd = wifiBridge.processIncoming();
            #ifdef SERIAL_DEBUG
                if (cmd.isValid) {
                    SERIAL_DEBUG.print(F("[Loop] Received cmd: "));
                    SERIAL_DEBUG.println(cmd.command);
                }
            #endif
        }

        // Send periodic telemetry
        if (currentTime - lastTelemetryUpdate >= TELEMETRY_INTERVAL_MS) {
            lastTelemetryUpdate = currentTime;
            commandHandler.updateTelemetry();
        }

        // Send periodic status (isMowing, isCharging, battery, state)
        static unsigned long lastStatusUpdate = 0;
        const uint32_t STATUS_INTERVAL_MS = 1000;
        if (currentTime - lastStatusUpdate >= STATUS_INTERVAL_MS) {
            lastStatusUpdate = currentTime;
            StaticJsonDocument<128> statusDoc;
            statusDoc["isMowing"] = mowerStateMachine.isMowing();
            bool isChargingFlag = (mowerStateMachine.getCurrentState() == MowerState::CHARGING);
            #ifdef ENABLE_BATTERY_MONITOR
                // Aggiorna le letture della batteria
                batteryMonitor.update();
                float battVolt = batteryMonitor.getVoltage();
                isChargingFlag = batteryMonitor.isCharging();
                uint8_t battLevel = static_cast<uint8_t>(batteryMonitor.getSOC());
                JsonObject batt = statusDoc.createNestedObject("battery");
                batt["voltage"] = battVolt;
                batt["level"] = battLevel;
                batt["current"] = batteryMonitor.getCurrent();
                batt["power"] = batteryMonitor.getPower();
                batt["capacity"] = batteryMonitor.getCapacity();
            #endif
            statusDoc["isCharging"] = isChargingFlag;
            statusDoc["state"] = static_cast<int>(mowerStateMachine.getCurrentState());
            wifiBridge.sendTelemetry(statusDoc);
        }
    #endif
    
    // 6. Gestione del watchdog
    #ifdef ENABLE_WATCHDOG
        // Riavvia il watchdog per evitare il reset
        wdt_reset();
    #endif
}
