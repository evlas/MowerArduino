#include "DriveMotor.h"
#include "../../config.h"
#include "../../pin_config.h"
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
                     float wheelDiameter, int pulsesPerRevolution, bool reversed)
    : MotorBase(pwmPin, dirPin, reversed),
      wheelDiameter_(wheelDiameter),
      wheelCircumference_(PI * wheelDiameter),
      pulsesPerRevolution_(pulsesPerRevolution),
      encoderPin_(encoderPin),
      isLeftMotor_(pwmPin == MOTOR_LEFT_PWM_PIN) {
    
    // Register this instance in the ISR handler
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
    
    // Imposta l'accelerazione di default
    setAcceleration(MAX_LINEAR_ACCEL, MAX_LINEAR_ACCEL * 1.5f);
}

void DriveMotor::update() {
    static unsigned long lastUpdateTime = millis();
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;  // Converti in secondi
    
    // Aggiorna la velocità in base all'accelerazione
    MotorBase::updateSpeed();
    
    // Aggiorna la stima della velocità basata sull'encoder
    updateSpeedEstimate();
    
    // Aggiorna la posizione in base alla velocità
    if (dt > 0) {
        updatePosition(dt);
    }
    
    lastUpdateTime = currentTime;
}

void DriveMotor::updateSpeedEstimate() {
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - lastSpeedUpdate_;
    
    // Aggiorna la stima della velocità ogni 50ms (20Hz)
    if (dt >= 50) {
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
    
    // Converti da m/s a valore PWM (-799 a 799 per 20kHz PWM con ICR = 799)
    int percent = map(fabs(speed) * 1000, 0, MAX_LINEAR_SPEED * 1000, 0, 100);
    
    // Applica la direzione
    if (speed < 0) {
        percent = -percent;
    }
    
    // Imposta la velocità target
    setPowerPercent(percent);
}

// Metodo per la gestione dell'interrupt
void DriveMotor::handleEncoderISR() {
    // Incrementa il contatore degli impulsi
    encoderCount_++;
}

void DriveMotor::updatePosition(float dt) {
    if (dt <= 0) return;  // Evita divisioni per zero o valori negativi
    
    // Calcola lo spostamento lineare in base alla velocità corrente
    float distance = currentLinearSpeed_ * dt;
    
    // Aggiorna la posizione (in un sistema di riferimento locale al motore)
    // La direzione dipende dal segno della velocità e dalla direzione del motore
    float direction = (currentLinearSpeed_ >= 0) ? 1.0f : -1.0f;
    if (isReversed) direction *= -1.0f;
    
    // Aggiorna le coordinate (in un sistema di riferimento locale)
    // Nota: Queste coordinate sono relative alla posizione iniziale del motore
    // e andranno trasformate nel sistema di riferimento globale dal PositionManager
    x_ += distance * direction;
    
    // Aggiorna la distanza percorsa (sempre positiva)
    distanceTraveled_ += abs(distance);
    
    // Aggiorna il timestamp
    lastUpdateTime_ = millis() / 1000.0f;
}