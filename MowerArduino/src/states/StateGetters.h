#ifndef STATE_GETTERS_H
#define STATE_GETTERS_H

// File incluso in Mower.h per i metodi getter degli stati
// Includi i file di intestazione degli stati implementati
#include "IdleState.h"
#include "MowingState.h"
#include "DockingState.h"
#include "UndockingState.h"
#include "ChargingState.h"
#include "EmergencyStopState.h"
#include "ErrorState.h"
#include "LiftedState.h"

// Implementazione dei metodi getter per gli stati implementati
inline MowerState& Mower::getIdleState() { 
    return IdleState::getInstance(); 
}

inline MowerState& Mower::getMowingState() { 
    return MowingState::getInstance(); 
}

inline MowerState& Mower::getDockingState() { 
    return DockingState::getInstance(); 
}

inline MowerState& Mower::getUndockingState() { 
    return UndockingState::getInstance(); 
}

inline MowerState& Mower::getChargingState() { 
    return ChargingState::getInstance(); 
}

inline MowerState& Mower::getEmergencyStopState() { 
    return EmergencyStopState::getInstance(); 
}

inline MowerState& Mower::getLiftedState() { 
    return LiftedState::getInstance();
}

inline MowerState& Mower::getErrorState() { 
    return ErrorState::getInstance(); 
}

#endif // STATE_GETTERS_H
