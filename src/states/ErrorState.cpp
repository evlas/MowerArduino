#include "ErrorState.h"
#include "../functions/Mower.h"
#include "../functions/MowerTypes.h"

void ErrorState::enter(Mower& mower) {
#ifdef SERIAL_DEBUG
    Serial.println(F("Entering ErrorState"));
#endif
    // Ferma tutti i motori
    mower.stopMotors();
    
    // Disattiva i motori
    mower.stopDriveMotors();
    
    // Disattiva le lame
    mower.stopBlades();
    
    // Emetti un segnale acustico per indicare l'errore
    // mower.getBuzzer().beep(1000, 500); // Esempio: beep di 1 secondo con frequenza 500Hz
}

void ErrorState::update(Mower& mower) {
    // In caso di errore, non fare nulla fino a quando non viene risolto
}

void ErrorState::exit(Mower& mower) {
#ifdef SERIAL_DEBUG
    Serial.println(F("Exiting ErrorState"));
#endif
    // Pulizia delle risorse se necessario
}

void ErrorState::handleEvent(Mower& mower, Event event) {
    switch (event) {
        case Event::ERROR_CLEARED:
            // Quando l'errore viene risolto, torna allo stato IDLE
            mower.changeState(mower.getIdleState());
            break;
            
        case Event::EMERGENCY_STOP:
            // Se già in stato di errore e ricevuto un altro EMERGENCY_STOP, non fare nulla
            break;
            
        default:
            // Ignora altri eventi in stato di errore
            break;
    }
}
