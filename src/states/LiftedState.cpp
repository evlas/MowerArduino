#include "LiftedState.h"
#include "../functions/Mower.h"
#include "IdleState.h"
#include "EmergencyStopState.h"
#include "../functions/MowerTypes.h"
#include <Arduino.h>

// Tempo minimo di sollevamento per conferma (in millisecondi)
const unsigned long LIFT_CONFIRMATION_TIME = 2000;  // 2 secondi
// Tempo massimo nello stato di sollevamento prima di andare in emergenza (5 minuti)
const unsigned long MAX_LIFTED_TIME = 5 * 60 * 1000UL;

void LiftedState::enter(Mower& mower) {
#ifdef DEBUG
    SERIAL_DEBUG.println(F("LIFTED: Entering state"));
#endif
    
    // Inizializza le variabili di stato
    liftStartTime_ = millis();
    liftConfirmed_ = false;
    
    // Ferma tutti i motori
    mower.stopMotors();
    
    // Disattiva le lame se erano accese
    mower.stopBlades();
    
    // Aggiorna il display
    mower.clearLcdDisplay();
    mower.setLcdCursor(0, 0);
    mower.printToLcd("LIFTED!");
    mower.setLcdCursor(0, 1);
    mower.printToLcd("Place me down!");
    
    // Segnale acustico di avviso
    mower.playBuzzerTone(1000, 500);  // 1000Hz for 500ms
}

void LiftedState::update(Mower& mower) {
    unsigned long currentTime = millis();
    
    // Verifica se il tempo massimo di sollevamento è stato superato
    if (currentTime - liftStartTime_ > MAX_LIFTED_TIME) {
#ifdef DEBUG
        SERIAL_DEBUG.println(F("LIFTED: Maximum lifted time exceeded"));
#endif
        mower.handleEvent(Event::ERROR_DETECTED);
        return;
    }
    
    // Verifica se il sollevamento è stato confermato
    if (!liftConfirmed_ && (currentTime - liftStartTime_ > LIFT_CONFIRMATION_TIME)) {
        liftConfirmed_ = true;
        
        // Aggiorna il display
        mower.clearLcdDisplay();
        mower.setLcdCursor(0, 0);
        mower.printToLcd("LIFTED!");
        mower.setLcdCursor(0, 1);
        mower.printToLcd("Press button...");
        
        // Emetti un altro segnale acustico
        mower.playBuzzerTone(1000, 500);  // 1000Hz for 500ms
    }
    
    // Verifica se il tosaerba è stato riposizionato a terra
    if (!mower.isLifted() && liftConfirmed_) {
        // Se è passato abbastanza tempo per confermare il riposizionamento
        if (currentTime - liftStartTime_ > LIFT_CONFIRMATION_TIME * 2) {
            mower.handleEvent(Event::PLACED_ON_GROUND);
        }
    }
}

void LiftedState::exit(Mower& mower) {
#ifdef SERIAL_DEBUG
    Serial.println(F("LIFTED: Exiting state"));
#endif
    
    // Disattiva eventuali segnali acustici
    mower.stopBuzzer();
    
    // Pulisci il display
    mower.clearLcdDisplay();
}

void LiftedState::handleEvent(Mower& mower, Event event) {
#ifdef SERIAL_DEBUG
    Serial.print(F("LIFTED: Handling event "));
    Serial.println(mower.eventToString(event));
#endif

    switch (event) {
        case Event::LIFT_RESOLVED:
            // Quando il sollevamento viene risolto, torna allo stato IDLE
            mower.changeState(mower.getIdleState());
            break;
            
        case Event::EMERGENCY_STOP:
            mower.changeState(mower.getEmergencyStopState());
            break;
            
        case Event::BUTTON_PRESSED:
            // Se viene premuto il pulsante mentre siamo in stato di sollevamento
            // confermato, possiamo forzare l'uscita dallo stato
            if (liftConfirmed_) {
                mower.handleEvent(Event::LIFT_RESOLVED);
                mower.setState(mower.getIdleState());
            }
            break;
            
        case Event::ERROR_DETECTED:
            mower.setState(mower.getEmergencyStopState());
            break;
            
        // Ignora altri eventi durante lo stato di sollevamento
        default:
#ifdef SERIAL_DEBUG
            Serial.print(F("LIFTED: Ignoring event "));
            Serial.println(mower.eventToString(event));
#endif
            break;
    }
}
