#include "IdleState.h"
#include "../functions/Mower.h"
#include "MowingState.h"
#include "EmergencyStopState.h"
#include "../functions/MowerTypes.h"

void IdleState::enter(Mower& mower) {
#ifdef DEBUG
    SERIAL_DEBUG.println(F("IDLE: Entering state"));
#endif
    
    // Azioni di entrata
    mower.stopBlades();
    mower.stopMotors();
}

void IdleState::update(Mower& mower) {
    // Nessuna azione in idle
}

void IdleState::exit(Mower& mower) {
#ifdef DEBUG
    SERIAL_DEBUG.println(F("IDLE: Exiting state"));
#endif
}

void IdleState::handleEvent(Mower& mower, Event event) {
    switch (event) {
        case Event::START_MOWING:
            mower.setState(mower.getMowingState());
            break;
            
        case Event::EMERGENCY_STOP:
            mower.setState(mower.getEmergencyStopState());
            break;
            
        // Gestisci altri eventi...
            
        default:
            // Ignora altri eventi
            break;
    }
}
