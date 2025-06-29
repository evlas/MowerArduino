#ifndef UNDOCKING_STATE_H
#define UNDOCKING_STATE_H

#include "MowerState.h"

class UndockingState : public MowerState {
public:
    static UndockingState& getInstance() {
        static UndockingState instance;
        return instance;
    }
    
    // Implementazioni MowerState
    void enter(Mower& mower) override;
    void update(Mower& mower) override;
    void exit(Mower& mower) override;
    const char* getName() const override { return "UNDOCKING"; }
    State getStateType() const override { return State::UNDOCKING; }
    
    // Gestione eventi
    void handleEvent(Mower& mower, Event event) override;

    // Elimina costruttore di copia e operatore di assegnamento
    UndockingState(const UndockingState&) = delete;
    void operator=(const UndockingState&) = delete;

private:
    UndockingState() = default;  // Costruttore privato per singleton
    
    // Variabili di stato specifiche per l'undocking
    unsigned long undockingStartTime_;
    bool isReversing_;
    bool isRotating_;
};

#endif // UNDOCKING_STATE_H
