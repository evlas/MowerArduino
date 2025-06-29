#include "ManualControlState.h"
#include "../functions/Mower.h"
#include "../functions/MowerTypes.h"

void ManualControlState::enter(Mower& mower) {
#ifdef DEBUG_MODE
    Serial.println(F("Entering ManualControlState"));
#endif
    // Inizializza lo stato di controllo manuale
    mower.setNavigationMode(Mower::NavigationMode::MANUAL);
}

void ManualControlState::update(Mower& mower) {
    // Aggiorna lo stato del controllo manuale
    // Qui verranno gestiti gli input dell'utente
}

void ManualControlState::exit(Mower& mower) {
#ifdef DEBUG_MODE
    Serial.println(F("Exiting ManualControlState"));
#endif
    // Pulizia delle risorse se necessario
}

void ManualControlState::handleEvent(Mower& mower, Event event) {
    switch (event) {
        case Event::MANUAL_CONTROL_DISABLED:
            // Torna allo stato IDLE quando il controllo manuale viene disabilitato
            mower.changeState(mower.getIdleState());
            break;
            
        case Event::EMERGENCY_STOP:
            // Passa allo stato di emergenza
            mower.changeState(mower.getEmergencyStopState());
            break;
            
        default:
            // Gestisci altri eventi se necessario
            break;
    }
}
