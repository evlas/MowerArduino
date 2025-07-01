#include "EmergencyStopState.h"
#include "../functions/Mower.h"
#include "IdleState.h"
#include "ErrorState.h"
#include <Arduino.h>

// Intervallo tra i segnali acustici di emergenza (in millisecondi)
const unsigned long EMERGENCY_BEEP_INTERVAL = 2000;

void EmergencyStopState::enter(Mower& mower) {
DEBUG_PRINTLN(F("EMERGENCY: Entering state"));
    
    // Azioni di emergenza: ferma tutto
    mower.emergencyStop();
    
    // Inizializza le variabili di stato
    lastBeepTime_ = 0;
    isBeeping_ = false;
    
    // Display updates are now handled by the LCDMenu class
    
    // Play emergency sound sequence
    mower.playBuzzerTone(2000, 500);
    lastBeepTime_ = millis();
    isBeeping_ = true;
    
    // Segnale acustico di emergenza
    mower.playBuzzerTone(2000, 200);
    delay(100);
    mower.playBuzzerTone(1500, 200);
    delay(100);
    mower.playBuzzerTone(1000, 500);
}

void EmergencyStopState::update(Mower& mower) {
    unsigned long currentTime = millis();
    
    // Se è passato abbastanza tempo dall'ultimo segnale acustico
    if (currentTime - lastBeepTime_ > EMERGENCY_BEEP_INTERVAL) {
        if (isBeeping_) {
            // Spegni il buzzer
            mower.stopBuzzer();
        } else {
            // Emetti un altro segnale acustico
            mower.playBuzzerTone(2000, 500);
            lastBeepTime_ = currentTime;
        }
        isBeeping_ = !isBeeping_;
    }
    
    // Nessuna azione specifica durante l'update
    // L'utente deve risolvere manualmente l'errore
}

void EmergencyStopState::exit(Mower& mower) {
DEBUG_PRINTLN(F("EMERGENCY: Exiting state"));
    
    // Ferma il segnale acustico
    mower.stopBuzzer();
    
    // Reset emergency state
    mower.resetEmergencyStop();
    
    // Play confirmation sound
    mower.playBuzzerTone(2000, 100);
    delay(100);
    mower.playBuzzerTone(2500, 100);
    
    // Piccola pausa per mostrare il messaggio
    delay(1000);
}

void EmergencyStopState::handleEvent(Mower& mower, Event event) {
    static unsigned long lastUnknownEventTime = 0;
    const unsigned long MIN_TIME_BETWEEN_UNKNOWN_EVENTS = 1000; // 1 second
    
    unsigned long currentTime = millis();
    bool shouldLog = true;
    
    // Log unknown events, but throttle the logging
    if (event != Event::ERROR_CLEARED && event != Event::RESUME && event != Event::RESET) {
        if (currentTime - lastUnknownEventTime < MIN_TIME_BETWEEN_UNKNOWN_EVENTS) {
            shouldLog = false;
        } else {
            lastUnknownEventTime = currentTime;
        }
    }
    
    if (shouldLog) {
        DEBUG_PRINT(F("EMERGENCY: Handling event "));
        DEBUG_PRINT(mower.eventToString(event));
        DEBUG_PRINT(F(" at "));
        DEBUG_PRINT(currentTime);
        DEBUG_PRINT(F("ms, Battery: "));
        DEBUG_PRINT(mower.getBatteryPercentage());
        DEBUG_PRINTLN(F("%"));
    }

    switch (event) {
        case Event::ERROR_CLEARED:
            // Se l'errore è stato risolto, torna allo stato IDLE
            DEBUG_PRINTLN(F("EMERGENCY: Error cleared, returning to IDLE state"));
            mower.changeState(mower.getIdleState());
            break;
            
        case Event::RESUME:
            // Se viene richiesto il ripristino, verifica le condizioni
            DEBUG_PRINTLN(F("EMERGENCY: Resume requested, returning to IDLE state"));
            mower.changeState(mower.getIdleState());
            break;
            
        case Event::RESET:
            // Se l'utente ha premuto il pulsante di reset
            DEBUG_PRINT(F("EMERGENCY: Reset requested, error resolved: "));
            DEBUG_PRINTLN(mower.isErrorResolved() ? "YES" : "NO");
            
            if (mower.isErrorResolved()) {
                DEBUG_PRINTLN(F("EMERGENCY: Error resolved, clearing"));
                mower.handleEvent(Event::ERROR_CLEARED);
            } else {
                // Se l'errore persiste, emetti un segnale di errore
                DEBUG_PRINTLN(F("EMERGENCY: Error not resolved, beeping"));
                mower.playBuzzerTone(1000, 100);
                delay(100);
                mower.playBuzzerTone(1000, 100);
                
                // Mostra il messaggio di errore
                mower.setLcdCursor(0, 1);
                mower.printToLcd("ERRORE NON RISOLTO!");
            }
            break;
            
        // Ignora tutti gli altri eventi durante l'emergenza
        default:
            if (shouldLog) {
                DEBUG_PRINT(F("EMERGENCY: Ignoring event "));
                DEBUG_PRINT(mower.eventToString(event));
                DEBUG_PRINT(F(" in state EMERGENCY_STOP"));
                DEBUG_PRINT(F(", Battery: "));
                DEBUG_PRINT(mower.getBatteryPercentage());
                DEBUG_PRINTLN(F("%"));
            }
            break;
    }
}
