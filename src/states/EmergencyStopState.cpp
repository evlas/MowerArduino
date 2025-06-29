#include "EmergencyStopState.h"
#include "../functions/Mower.h"
#include "IdleState.h"
#include "ErrorState.h"
#include <Arduino.h>

// Intervallo tra i segnali acustici di emergenza (in millisecondi)
const unsigned long EMERGENCY_BEEP_INTERVAL = 2000;

void EmergencyStopState::enter(Mower& mower) {
#ifdef DEBUG
    SERIAL_DEBUG.println(F("EMERGENCY: Entering state"));
#endif
    
    // Azioni di emergenza: ferma tutto
    mower.emergencyStop();
    
    // Inizializza le variabili di stato
    lastBeepTime_ = 0;
    isBeeping_ = false;
    
    // Mostra il messaggio di emergenza sul display
    mower.clearLcdDisplay();
    mower.setLcdCursor(0, 0);
    mower.printToLcd("!!! EMERGENCY STOP !!!");
    mower.setLcdCursor(0, 1);
    
    // Mostra il messaggio di errore
    mower.clearLcdDisplay();
    mower.setLcdCursor(0, 0);
    mower.printToLcd("EMERGENCY STOP!");
    mower.setLcdCursor(0, 1);
    mower.printToLcd("Check and reset");
    
    // Emetti il primo segnale acustico
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
#ifdef DEBUG
    SERIAL_DEBUG.println(F("EMERGENCY: Exiting state"));
#endif
    
    // Ferma il segnale acustico
    mower.stopBuzzer();
    
    // Pulisci il display
    mower.clearLcdDisplay();
    
    // Ripristina lo stato di emergenza
    mower.resetEmergencyStop();
    
    // Mostra un messaggio di ripristino
    mower.setLcdCursor(0, 0);
    mower.printToLcd("RIPRISTINO");
    mower.setLcdCursor(0, 1);
    mower.printToLcd("IN CORSO...");
    
    // Breve segnale acustico di conferma
    mower.playBuzzerTone(2000, 100);
    delay(100);
    mower.playBuzzerTone(2500, 100);
    
    // Piccola pausa per mostrare il messaggio
    delay(1000);
}

void EmergencyStopState::handleEvent(Mower& mower, Event event) {
#ifdef DEBUG
    SERIAL_DEBUG.print(F("EMERGENCY: Handling event "));
    SERIAL_DEBUG.println(mower.eventToString(event));
#endif

    switch (event) {
        case Event::ERROR_CLEARED:
            // Se l'errore è stato risolto, torna allo stato IDLE
            mower.changeState(mower.getIdleState());
            break;
            
        case Event::RESUME:
            // Se viene richiesto il ripristino, verifica le condizioni
            mower.changeState(mower.getIdleState());
            break;
            
        case Event::RESET:
            // Se l'utente ha premuto il pulsante di reset
            if (mower.isErrorResolved()) {
                mower.handleEvent(Event::ERROR_CLEARED);
            } else {
                // Se l'errore persiste, emetti un segnale di errore
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
#ifdef DEBUG
            SERIAL_DEBUG.print(F("EMERGENCY: Ignoring event "));
            SERIAL_DEBUG.println(mower.eventToString(event));
#endif
            break;
    }
}
