#ifndef MANUAL_CONTROL_STATE_H
#define MANUAL_CONTROL_STATE_H

#include "MowerState.h"

class ManualControlState : public MowerState {
public:
    static ManualControlState& getInstance() {
        static ManualControlState instance;
        return instance;
    }
    
    // Implementazioni MowerState
    void enter(Mower& mower) override;
    void update(Mower& mower) override;
    void exit(Mower& mower) override;
    const char* getName() const override { return "MANUAL_CONTROL"; }
    State getStateType() const override { return State::MANUAL_CONTROL; }
    
    // Gestione eventi
    void handleEvent(Mower& mower, Event event) override;

    // Elimina costruttore di copia e operatore di assegnamento
    ManualControlState(const ManualControlState&) = delete;
    void operator=(const ManualControlState&) = delete;

    // Metodi specifici per il controllo manuale
    void setLeftMotorSpeed(float speed);
    void setRightMotorSpeed(float speed);
    void setBladeSpeed(float speed);
    void stopAllMotors();

private:
    ManualControlState() = default;  // Costruttore privato per singleton
    
    // Variabili di stato
    float leftMotorSpeed_ = 0.0f;
    float rightMotorSpeed_ = 0.0f;
    float bladeSpeed_ = 0.0f;
};

#endif // MANUAL_CONTROL_STATE_H
