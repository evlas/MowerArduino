#include "DriveMotor.h"
#include "config.h"
#include "pin_config.h"
#include <Arduino.h>

// Define the ISR handlers in the global namespace
namespace DriveMotorISR {
    DriveMotor* leftMotorInstance = nullptr;
    DriveMotor* rightMotorInstance = nullptr;
    
    void handleLeftEncoderISR() {
        if (leftMotorInstance) {
            leftMotorInstance->handleEncoderISR();
        }
    }
    
    void handleRightEncoderISR() {
        if (rightMotorInstance) {
            rightMotorInstance->handleEncoderISR();
        }
    }
}

DriveMotor::DriveMotor(uint8_t pwmPin, uint8_t dirPin, uint8_t encoderPin,
                     float wheelDiameter, int pulsesPerRevolution, bool isLeft)
    : MotorBase(pwmPin, dirPin),
      wheelDiameter_(wheelDiameter),
      wheelCircumference_(PI * wheelDiameter),
      pulsesPerRevolution_(pulsesPerRevolution),
      encoderPin_(encoderPin),
      isLeftMotor_(isLeft) {
    
    // Imposta la direzione invertita per il motore destro
    if (!isLeftMotor_) {
        setReversed(true);
    }
    
    // Registra questa istanza nel gestore ISR
    if (isLeftMotor_) {
        DriveMotorISR::leftMotorInstance = this;
    } else {
        DriveMotorISR::rightMotorInstance = this;
    }
}

void DriveMotor::begin() {
    // Inizializza il motore base
    MotorBase::begin();
    
    // Configura il pin dell'encoder come input con pullup
    pinMode(encoderPin_, INPUT_PULLUP);
    
    // Attiva l'interrupt sull'encoder usando la funzione built-in
    if (isLeftMotor_) {
        attachInterrupt(digitalPinToInterrupt(encoderPin_), DriveMotorISR::handleLeftEncoderISR, CHANGE);
    } else {
        attachInterrupt(digitalPinToInterrupt(encoderPin_), DriveMotorISR::handleRightEncoderISR, CHANGE);
    }
    
    // Inizializza il tempo dell'ultimo aggiornamento
    lastSpeedUpdate_ = millis();
    
    // Imposta l'accelerazione di default basata sulla configurazione
    maxAcceleration_ = MAX_LINEAR_ACCEL;
    maxDeceleration_ = MAX_LINEAR_DECEL;
}

void DriveMotor::update() {
    // Aggiorna la velocità con accelerazione
    updateSpeed();
    
    // Aggiorna la stima della velocità lineare
    updateSpeedEstimate();
    
    // Aggiorna la posizione
    unsigned long currentTime = millis();
    if (lastUpdateTime_ > 0) {
        float dt = (currentTime - lastUpdateTime_) / 1000.0f;  // in secondi
        updatePosition(dt);
        
        // Aggiorna la stima della velocità ogni 50ms (20Hz)
        if ((currentTime - lastSpeedUpdate_) >= 50) {
            // Calcola la velocità in m/s
            float distancePerPulse = wheelCircumference_ / pulsesPerRevolution_;
            long deltaCount = encoderCount_ - lastEncoderCount_;
            
            // Aggiorna la distanza percorsa
            distanceTraveled_ += deltaCount * distancePerPulse;
            
            // Calcola la velocità lineare (m/s)
            if (dt > 0) {
                currentLinearSpeed_ = (deltaCount * distancePerPulse) / (dt / 1000.0f);
            }
            
            // Aggiorna i contatori per il prossimo ciclo
            lastEncoderCount_ = encoderCount_;
            lastSpeedUpdate_ = currentTime;
        }
    }
    lastUpdateTime_ = currentTime;
}

void DriveMotor::resetEncoder() {
    noInterrupts();
    encoderCount_ = 0;
    lastEncoderCount_ = 0;
    distanceTraveled_ = 0.0f;
    currentLinearSpeed_ = 0.0f;
    interrupts();
}

void DriveMotor::setLinearSpeed(float speed) {
    // Limita la velocità al massimo consentito
    speed = constrain(speed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    
    // Converti da m/s a percentuale (-100% a 100%)
    float percent = (speed / MAX_LINEAR_SPEED) * 100.0f;
    
    // Imposta la velocità target
    setSpeed(percent);
}

// Metodo per la gestione dell'interrupt
void DriveMotor::handleEncoderISR() {
    // Incrementa il contatore degli impulsi
    encoderCount_++;
}

void DriveMotor::updatePosition(float dt) {
    if (dt <= 0) return;  // Evita divisioni per zero o valori negativi
    
    // Calcola la percentuale di potenza in base alla velocità massima consentita
    float percent = (currentLinearSpeed_ / MAX_LINEAR_SPEED) * 100.0f;
    setSpeed(percent);
    
    // Calcola lo spostamento lineare in base alla velocità corrente
    float distance = currentLinearSpeed_ * dt;
    
    // Aggiorna la posizione (in un sistema di riferimento locale al motore)
    // La direzione dipende dal segno della velocità e dalla direzione del motore
    float direction = (currentLinearSpeed_ >= 0) ? 1.0f : -1.0f;
    if (reversed_) direction *= -1.0f;
    
    // Aggiorna le coordinate (in un sistema di riferimento locale)
    // Nota: Queste coordinate sono relative alla posizione iniziale del motore
    // e andranno trasformate nel sistema di riferimento globale dal PositionManager
    x_ += distance * cos(theta_);
    y_ += distance * sin(theta_);
    
    // Aggiorna la distanza percorsa (sempre positiva)
    distanceTraveled_ += abs(distance);
    
    // Aggiorna il timestamp
    lastUpdateTime_ = millis() / 1000.0f;
}

void DriveMotor::updateSpeedEstimate() {
    unsigned long currentTime = millis();
    
    // Calcola il tempo trascorso dall'ultimo aggiornamento in secondi
    float dt = (currentTime - lastSpeedUpdate_) / 1000.0f;
    
    if (dt > 0 && lastSpeedUpdate_ > 0) {
        // Calcola la velocità in m/s
        float distancePerPulse = wheelCircumference_ / pulsesPerRevolution_;
        long deltaCount = encoderCount_ - lastEncoderCount_;
        
        // Calcola la velocità lineare (m/s)
        currentLinearSpeed_ = (deltaCount * distancePerPulse) / dt;
        
        // Aggiorna la distanza percorsa
        distanceTraveled_ += deltaCount * distancePerPulse;
        
        // Aggiorna il conteggio precedente
        lastEncoderCount_ = encoderCount_;
    }
    
    // Aggiorna il tempo dell'ultimo aggiornamento
    lastSpeedUpdate_ = currentTime;
}