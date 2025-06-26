#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#include <Arduino.h>
#include "config.h"  // Include il file di configurazione

// Enum per la modalità di funzionamento del motore
enum class MotorMode {
    SINGLE_PIN,     // Usa un solo pin (PWM)
    DUAL_PIN,       // Usa due pin (PWM + direzione)
    H_BRIDGE        // Usa due pin PWM (ponte H)
};

class MotorBase {
protected:
    uint8_t pin1;           // Primo pin di controllo (PWM o direzione 1)
    uint8_t pin2;           // Secondo pin di controllo (direzione 2, opzionale)
    MotorMode mode;         // Modalità di funzionamento
    int currentSpeed_;       // Velocità corrente (-255 a 255)
    int targetSpeed_;        // Velocità obiettivo (-255 a 255)
    bool isEnabled;          // Stato di accensione
    bool isReversed;         // Se il motore è invertito
    unsigned long lastUpdateTime_;  // Ultimo aggiornamento velocità
    
    // Acceleration parameters (in PWM units per second)
    // Scaled for 0-799 range (20kHz PWM with ICR = 799)
    float maxAcceleration_ = (799.0f / MAX_LINEAR_SPEED) * MAX_LINEAR_ACCEL;  // [PWM units/s]
    float maxDeceleration_ = (799.0f / MAX_LINEAR_SPEED) * MAX_LINEAR_DECEL;  // [PWM units/s]
    
    // Applica la velocità ai pin del motore
    virtual void applySpeed();
    
public:
    MotorBase(uint8_t pin1, uint8_t pin2 = 255, bool reversed = false);
    virtual ~MotorBase() = default;
    
    virtual void begin();
    
    // Imposta la velocità target con accelerazione controllata
    virtual void setSpeed(float speed);
    
    // Imposta la velocità target direttamente (senza accelerazione)
    virtual void setSpeedImmediate(float speed);
    
    // Aggiorna la velocità in base all'accelerazione
    virtual void updateSpeed();
    
    virtual void stop();
    virtual void brake();
    virtual bool isMoving() const;
    
    // Restituisce la velocità corrente in percentuale (-100..100)
    virtual float getSpeed() const { 
        return speed_;
    }
    virtual void setEnabled(bool enabled);
    virtual void setReversed(bool reversed);
    
    // Imposta i parametri di accelerazione
    virtual void setAcceleration(float accel, float decel = -1) {
        maxAcceleration_ = (255.0f / (MAX_LINEAR_SPEED / accel)) * 1000.0f;
        if (decel > 0) {
            maxDeceleration_ = (255.0f / (MAX_LINEAR_SPEED / decel)) * 1000.0f;
        }
    }
    
    // Controlla se il motore ha raggiunto la velocità target
    virtual bool isAtTargetSpeed() const { 
        return abs(targetSpeed_ - currentSpeed_) < 2; 
    }
};

#endif // MOTOR_BASE_H
