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
#include <INA226_WE.h>
extern INA226_WE batteryMonitor;
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

        // Leggi lo stato della batteria con gestione degli errori
        float voltage = 0.0f;
        float current = 0.0f;

        // Lettura batteria
        voltage = batteryMonitor.getBusVoltage_V();
        current = batteryMonitor.getCurrent_mA();

        const float BATTERY_CRITICAL_VOLTAGE = BATTERY_EMPTY_VOLTAGE * BATTERY_CELLS; // soglia pack batteria scarica
        const float BATTERY_LOW_VOLTAGE = BATTERY_LOW_THRESHOLD * BATTERY_CELLS; // soglia pack batteria bassa

        // Informa la state-machine se collegato alla base di ricarica
        if (voltage >= BATTERY_RECHARGING && mowerStateMachine.getCurrentState() != MowerState::CHARGING) {
            mowerStateMachine.sendEvent(MowerEvent::CHARGING_COMPLETE);
        }
        // Se è in carica e la tensione scende a FULL_BATTERY_VOLTAGE, passa in docking
        if (mowerStateMachine.getCurrentState() == MowerState::CHARGING && voltage <= FULL_BATTERY_VOLTAGE) {
            mowerStateMachine.sendEvent(MowerEvent::RETURN_TO_BASE);
        }

        // Ignores invalid reading (0V) that sometimes appears at start
        if (voltage > 0.1f && voltage < BATTERY_CRITICAL_VOLTAGE) {
            mowerStateMachine.sendEvent(MowerEvent::EMERGENCY_STOP);
            ErrorManager::addError(ErrorCode::BATTERY_CRITICAL,
                                   String("Tensione critica: ") + String(voltage, 2) + "V");
        } else if (voltage > 0.1f && voltage < BATTERY_LOW_VOLTAGE) {
            if (mowerStateMachine.getCurrentState() != MowerState::RETURN_TO_BASE) {
                mowerStateMachine.sendEvent(MowerEvent::LOW_BATTERY);
                ErrorManager::addError(ErrorCode::BATTERY_LOW,
                                       String("Batteria scarica: ") + String(voltage, 2) + "V");
            }
        }
        // }
    }
    #endif

    // 4. Aggiornamento della macchina a stati
    mowerStateMachine.update();
    
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
                float battVolt = batteryMonitor.getBusVoltage_V();
                isChargingFlag = (battVolt >= BATTERY_RECHARGING);
                const float PACK_MIN_VOLT = BATTERY_EMPTY_VOLTAGE * BATTERY_CELLS;   // e.g. 3.2*3 = 9.6V
                const float PACK_MAX_VOLT = BATTERY_FULL_VOLTAGE * BATTERY_CELLS;   // e.g. 4.16*3 ≈ 12.48V
                uint8_t battLevel = (uint8_t)constrain(map(battVolt * 1000, PACK_MIN_VOLT * 1000, PACK_MAX_VOLT * 1000, 0, 100), 0, 100);
                JsonObject batt = statusDoc.createNestedObject("battery");
                batt["voltage"] = battVolt;
                batt["level"] = battLevel;
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
