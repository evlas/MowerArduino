#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#include <Arduino.h>
#include "config.h"  // Include il file di configurazione

class MotorBase {
protected:
    uint8_t pwmPin;         // Pin PWM per il controllo della velocità
    uint8_t dirPin;         // Pin per il controllo della direzione
    bool enabled_;         // Stato di accensione
    bool reversed_;        // Se il motore è invertito
    int currentSpeed_;      // Velocità corrente (-100 a 100)
    int targetSpeed_;       // Velocità obiettivo (-100 a 100)
    unsigned long lastUpdateTime_;  // Ultimo aggiornamento velocità
    
    // Parametri di accelerazione (in unità di velocità al secondo)
    float maxAcceleration_ = MAX_LINEAR_ACCEL;
    float maxDeceleration_ = MAX_LINEAR_DECEL;
    
    // Applica la velocità ai pin del motore
    virtual void applySpeed();
    
public:
    /**
     * @brief Costruttore della classe MotorBase
     * @param pwmPin Pin per il controllo PWM della velocità
     * @param dirPin Pin per il controllo della direzione
     */
    MotorBase(uint8_t pwmPin, uint8_t dirPin);
    
    virtual ~MotorBase() = default;
    
    /**
     * @brief Inizializza il motore
     */
    virtual void begin();
    
    /**
     * @brief Imposta la velocità target con accelerazione controllata
     * @param speed Velocità in percentuale (-100 a 100)
     */
    virtual void setSpeed(float speed);
    
    /**
     * @brief Imposta la velocità target senza accelerazione
     * @param speed Velocità in percentuale (-100 a 100)
     */
    virtual void setSpeedImmediate(float speed);
    
    /**
     * @brief Aggiorna la velocità in base all'accelerazione
     */
    virtual void updateSpeed();
    
    /**
     * @brief Ferma il motore con decelerazione controllata
     */
    virtual void stop();
    
    /**
     * @brief Ferma il motore immediatamente (frenata)
     */
    virtual void brake();
    
    /**
     * @brief Verifica se il motore è in movimento
     * @return true se il motore sta girando
     */
    virtual bool isMoving() const;
    
    /**
     * @brief Abilita/disabilita il motore
     * @param enabled true per abilitare, false per disabilitare
     */
    virtual void setEnabled(bool enabled) { enabled_ = enabled; }
    
    /**
     * @brief Imposta l'inversione del motore
     * @param reversed true per invertire la direzione
     */
    virtual void setReversed(bool reversed) { reversed_ = reversed; }
    
    // Metodi di accesso
    bool isEnabled() const { return enabled_; }
    bool isReversed() const { return reversed_; }
    float getSpeed() const { return currentSpeed_; }
    float getTargetSpeed() const { return targetSpeed_; }
    uint8_t getPwmPin() const { return pwmPin; }
    uint8_t getDirPin() const { return dirPin; }
};

#endif // MOTOR_BASE_H
