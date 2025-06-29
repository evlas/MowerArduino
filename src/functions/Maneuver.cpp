#include "Maneuver.h"
#include "../config.h"  // Per le costanti di velocità
#include "../motors/DriveMotor/DriveMotor.h"
#include "PositionManager.h"  // Includi l'header di PositionManager


Maneuver::~Maneuver() {
    stop();
}

float Maneuver::normalizeAngle(float angle) {
    // Normalizza l'angolo tra -PI e PI
    while (angle > PI) {
        angle -= TWO_PI;
    }
    while (angle < -PI) {
        angle += TWO_PI;
    }
    return angle;
}

// Basic movements
void Maneuver::forward(int duration, float speedPercent) {
    // Limita la velocità tra 0 e 100%
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    
    // Se la distanza è 0, movimento continuo alla velocità specificata
    if (duration == 0) {
        setSpeed(speedPercent);
        moving_ = true;
        return;
    }
    
    // Altrimenti, movimento per una certa durata
    targetLeftSpeed_ = speedPercent;
    targetRightSpeed_ = speedPercent;
    currentLeftSpeed_ = 0;
    currentRightSpeed_ = 0;
    moving_ = true;
    useTimerForDistance_ = true;
    movementEndTime_ = millis() + duration;
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Forward at speed: "));
    SERIAL_DEBUG.println(speedPercent);
    #endif
}

void Maneuver::backward(int duration, float speedPercent) {
    // Limita la velocità tra 0 e 100%
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    
    // Se la distanza è 0, movimento continuo alla velocità specificata
    if (duration == 0) {
        setSpeed(-speedPercent);
        moving_ = true;
        return;
    }
    
    // Altrimenti, movimento per una certa durata
    targetLeftSpeed_ = -speedPercent;
    targetRightSpeed_ = -speedPercent;
    currentLeftSpeed_ = 0;
    currentRightSpeed_ = 0;
    moving_ = true;
    useTimerForDistance_ = true;
    movementEndTime_ = millis() + duration;
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Backward at speed: "));
    SERIAL_DEBUG.println(speedPercent);
    #endif
}

void Maneuver::turnLeft(int angle, float speedPercent) {
    // Limita la velocità tra 0 e 100%
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    
    // Imposta velocità opposte per i due motori per la rotazione
    targetLeftSpeed_ = -speedPercent;
    targetRightSpeed_ = speedPercent;
    moving_ = true;
    turning_ = true;
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Turn left at speed: "));
    SERIAL_DEBUG.println(speedPercent);
    #endif
}

void Maneuver::turnRight(int angle, float speedPercent) {
    // Limita la velocità tra 0 e 100%
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    
    // Imposta velocità opposte per i due motori per la rotazione
    targetLeftSpeed_ = speedPercent;
    targetRightSpeed_ = -speedPercent;
    moving_ = true;
    turning_ = true;
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Turn right at speed: "));
    SERIAL_DEBUG.println(speedPercent);
    #endif
}

void Maneuver::rotateLeft(float speedPercent) {
    // Limita la velocità tra 0 e 100%
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    
    // Imposta velocità opposte per i due motori per la rotazione
    targetLeftSpeed_ = -speedPercent;
    targetRightSpeed_ = speedPercent;
    moving_ = false;
    turning_ = true;
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Rotate left at speed: "));
    SERIAL_DEBUG.println(speedPercent);
    #endif
}

void Maneuver::rotateRight(float speedPercent) {
    // Limita la velocità tra 0 e 100%
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    
    // Imposta velocità opposte per i due motori per la rotazione
    targetLeftSpeed_ = speedPercent;
    targetRightSpeed_ = -speedPercent;
    moving_ = false;
    turning_ = true;
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Rotate right at speed: "));
    SERIAL_DEBUG.println(speedPercent);
    #endif
}

void Maneuver::moveStraight(int distance, float speedPercent) {
    // Limita la velocità tra 0 e 100%
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    
    // Se la distanza è 0, movimento continuo alla velocità specificata
    if (distance == 0) {
        setSpeed(speedPercent);
        moving_ = true;
        return;
    }
    
    // Altrimenti, movimento per una certa distanza
    targetLeftSpeed_ = (distance > 0) ? speedPercent : -speedPercent;
    targetRightSpeed_ = (distance > 0) ? speedPercent : -speedPercent;
    currentLeftSpeed_ = 0;
    currentRightSpeed_ = 0;
    moving_ = true;
    useTimerForDistance_ = false;
    targetDistance_ = abs(distance) / 100.0f; // Converti cm in metri
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Move straight at speed: "));
    SERIAL_DEBUG.println(speedPercent);
    #endif
}

// Complex maneuvers
void Maneuver::rotate(int degrees, float speedPercent) {
    // Limita la velocità tra 0 e 100%
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    
    // Imposta velocità opposte per i due motori per la rotazione
    if (degrees > 0) {
        targetLeftSpeed_ = -speedPercent;
        targetRightSpeed_ = speedPercent;
    } else {
        targetLeftSpeed_ = speedPercent;
        targetRightSpeed_ = -speedPercent;
    }
    moving_ = true;
    turning_ = true;
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Rotate at speed: "));
    SERIAL_DEBUG.println(speedPercent);
    #endif
}

void Maneuver::spiral(float maxRadius, float speedPercent) {
    // Limita la velocità tra 0 e 100%
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    
    // Imposta velocità opposte per i due motori per la rotazione
    setSpeed(-speedPercent, speedPercent);
    moving_ = true;
    float currentAngle = 0.0f;  // Angolo in radianti
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Avvio spirale - Raggio max: "));
    SERIAL_DEBUG.print(maxRadius);
    SERIAL_DEBUG.print(F(" m, Velocità: "));
    SERIAL_DEBUG.println(speedPercent);
    #endif
}

// Private helper methods
void Maneuver::setSpeed(float leftSpeedPercent, float rightSpeedPercent, bool isTrajectoryCorrection) {
    // Limita le velocità tra -100% e 100%
    leftSpeedPercent = constrain(leftSpeedPercent, -100.0f, 100.0f);
    rightSpeedPercent = constrain(rightSpeedPercent, -100.0f, 100.0f);
    
    // Se è una correzione di traiettoria, aggiorna direttamente i motori
    if (isTrajectoryCorrection) {
        if (leftMotor_) leftMotor_->setSpeed(leftSpeedPercent);
        if (rightMotor_) rightMotor_->setSpeed(rightSpeedPercent);
        return;
    }
    
    // Altrimenti, aggiorna le velocità target per l'accelerazione
    targetLeftSpeed_ = leftSpeedPercent;
    targetRightSpeed_ = rightSpeedPercent;
    isTrajectoryCorrection_ = isTrajectoryCorrection;
    
    // Se la velocità è zero, ferma immediatamente i motori
    if (leftSpeedPercent == 0 && rightSpeedPercent == 0) {
        stop();
        return;
    }
}

void Maneuver::setSpeed(float speedPercent, bool isTrajectoryCorrection) {
    setSpeed(speedPercent, speedPercent, isTrajectoryCorrection);
}

void Maneuver::setDirection(bool leftForward, bool rightForward) {
    // For MotorBase, we'll set negative speed for reverse
    // The actual direction will be handled by the motor driver
    // Range 0-100% mappato su 0-799 (20kHz PWM con ICR = 799)
    if (leftMotor_) {
        // Otteniamo la velocità in percentuale e invertiamo la direzione se necessario
        int8_t speed = leftMotor_->getSpeed();
        leftMotor_->setSpeedImmediate(leftForward ? abs(speed) : -abs(speed));
    }
    if (rightMotor_) {
        // Otteniamo la velocità in percentuale e invertiamo la direzione se necessario
        int8_t speed = rightMotor_->getSpeed();
        rightMotor_->setSpeedImmediate(rightForward ? abs(speed) : -abs(speed));
    }
}

void Maneuver::updateAcceleration() {
    if (!isAccelerating_ && !isDecelerating_) {
        return;  // Nessun aggiornamento necessario
    }
    
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - lastAccelUpdate_;
    
    if (dt < 10) {
        return;  // Esegui l'aggiornamento massimo a 100Hz
    }
    
    // Calcola l'accelerazione per ogni motore
    float leftOutput = leftPid_.compute(currentLeftSpeed_, targetLeftSpeed_);
    float rightOutput = rightPid_.compute(currentRightSpeed_, targetRightSpeed_);
    
    // Applica l'output ai motori
    if (leftMotor_) {
        leftMotor_->setSpeed(static_cast<int8_t>(leftOutput));
    }
    if (rightMotor_) {
        rightMotor_->setSpeed(static_cast<int8_t>(rightOutput));
    }
    
    // Aggiorna le velocità correnti
    currentLeftSpeed_ = leftOutput;
    currentRightSpeed_ = rightOutput;
    
    // Verifica se abbiamo raggiunto i target
    bool leftReached = abs(targetLeftSpeed_ - currentLeftSpeed_) < 1.0f;
    bool rightReached = abs(targetRightSpeed_ - currentRightSpeed_) < 1.0f;
    
    // Se entrambi i motori hanno raggiunto la velocità target
    if (leftReached && rightReached) {
        isAccelerating_ = false;
        isDecelerating_ = false;
        
        // Resetta i termini integrali per evitare sovraccarichi
        leftPid_.reset();
        rightPid_.reset();
    }
    
    lastAccelUpdate_ = currentTime;
}

// Avvia il taglio con la larghezza corretta della lama
void Maneuver::startMowing(float bladeWidth, float speed) {
    // Imposta la larghezza della lama
    bladeWidth_ = bladeWidth;
    
    // Avvia il movimento in avanti con la velocità specificata
    forward(0, speed);  // 0 = durata illimitata
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Avvio taglio - Larghezza lama: "));
    SERIAL_DEBUG.print(bladeWidth_);
    SERIAL_DEBUG.print(F(" m, Velocità: "));
    SERIAL_DEBUG.println(speed);
    #endif
}

// getLinearVelocity() and getAngularVelocity() are defined in the header file
