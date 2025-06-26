#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#include <Arduino.h>
#include "../../config.h"  // Include il file di configurazione

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
    
    // Parametri di accelerazione (in unità PWM/secondo)
    // Scaled for 0-799 range (20kHz PWM with ICR = 799)
    float maxAcceleration_ = (799.0f / (MAX_LINEAR_SPEED / MAX_LINEAR_ACCEL)) * 1000.0f;
    float maxDeceleration_ = (799.0f / (MAX_LINEAR_SPEED / (MAX_LINEAR_ACCEL * 1.5f))) * 1000.0f;
    
    // Applica la velocità ai pin del motore
    virtual void applySpeed();
    
public:
    MotorBase(uint8_t pin1, uint8_t pin2 = 255, bool reversed = false);
    virtual ~MotorBase() = default;
    
    virtual void begin();
    
    // Imposta la velocità target con accelerazione controllata
    virtual void setSpeed(int speed);
    
    // Imposta la velocità target direttamente (senza accelerazione)
    virtual void setSpeedImmediate(int speed);
    
    // ===== Nuova API controllo percentuale (-100 .. 100) =====
    inline void setPowerPercent(int8_t percent) {
        // Converte la percentuale (-100..100) in PWM usando i limiti definiti in config.h
        int pwm = map(constrain(percent, -100, 100), -100, 100, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
        setSpeed(pwm);
    }
    inline void setPowerPercentImmediate(int8_t percent) {
        int pwm = map(constrain(percent, -100, 100), -100, 100, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
        setSpeedImmediate(pwm);
    }
    
    // Aggiorna la velocità in base all'accelerazione
    virtual void updateSpeed();
    
    virtual void stop();
    virtual void brake();
    virtual bool isMoving() const;
    // Restituisce la velocità corrente in PWM (-799..799)
    virtual int getCurrentSpeed() const { return currentSpeed_; }
    
    // Restituisce la velocità corrente in percentuale (-100..100)
    virtual int8_t getCurrentPowerPercent() const { 
        return map(currentSpeed_, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED, -100, 100);
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
