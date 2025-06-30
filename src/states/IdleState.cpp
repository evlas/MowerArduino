#include "IdleState.h"
#include "../functions/Mower.h"
#include "MowingState.h"
#include "EmergencyStopState.h"
#include "../functions/MowerTypes.h"

void IdleState::enter(Mower& mower) {
#ifdef DEBUG_MODE
    DEBUG_PRINTLN(F("IDLE: Entering state"));
#endif
    
    // Azioni di entrata
    mower.stopBlades();
    mower.stopMotors();
}

void IdleState::update(Mower& mower) {
    // Nessuna azione in idle
}

void IdleState::exit(Mower& mower) {
#ifdef DEBUG_MODE
    DEBUG_PRINTLN(F("IDLE: Exiting state"));
#endif
}

void IdleState::handleEvent(Mower& mower, Event event) {
    switch (event) {
        case Event::START_MOWING: {
            DEBUG_PRINTLN(F("IDLE: Received START_MOWING event"));
            MowerState& nextState = mower.getMowingState();
            DEBUG_PRINT(F("IDLE: Transitioning to MowingState at address: "));
            DEBUG_PRINTLN((uintptr_t)&nextState, HEX);
            mower.setState(nextState);
            DEBUG_PRINTLN(F("IDLE: State transition completed"));
            break;
        }
            
        case Event::EMERGENCY_STOP:
            mower.setState(mower.getEmergencyStopState());
            break;
            
        // Gestisci altri eventi...
            
        default:
            // Ignora altri eventi
            break;
    }
}
