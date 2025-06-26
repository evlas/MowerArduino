#include "Maneuver.h"
#include "../../config.h"  // Per le costanti di velocità
#include "../motors/DriveMotor.h"


Maneuver::~Maneuver() {
    stop();
}

// Basic movements
void Maneuver::forward(int speed) {
    moving_ = true;
    turning_ = false;
    
    // Imposta i target di velocità
    targetLeftSpeed_ = constrain(speed, -100, 100);
    targetRightSpeed_ = constrain(speed, -100, 100);
    
    // Imposta la direzione in base al segno della velocità
    bool forward = (speed >= 0);
    setDirection(forward, forward);
    
    // Avvia l'accelerazione
    isAccelerating_ = true;
    isDecelerating_ = false;
    accelStep_ = 0.0f;
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Forward at speed: "));
    SERIAL_DEBUG.println(speed);
    #endif
    
    lastAccelUpdate_ = millis();
}

void Maneuver::backward(int speed) {
    moving_ = true;
    turning_ = false;
    setDirection(false, false);
    
    // Imposta i target di velocità (negativi per andare indietro)
    targetLeftSpeed_ = constrain(-speed, -100, 100);
    targetRightSpeed_ = constrain(-speed, -100, 100);
    
    // Avvia l'accelerazione
    isAccelerating_ = true;
    isDecelerating_ = false;
    
    // Resetta i controllori PID
    leftPid_.reset();
    rightPid_.reset();
    
    lastAccelUpdate_ = millis();
}

void Maneuver::turnLeft(int speed) {
    moving_ = false;
    turning_ = true;
    setDirection(false, true);
    
    // Imposta i target di velocità per la rotazione a sinistra
    targetLeftSpeed_ = constrain(-speed, -100, 100);
    targetRightSpeed_ = constrain(speed, -100, 100);
    
    // Avvia l'accelerazione
    isAccelerating_ = true;
    isDecelerating_ = false;
    
    // Resetta i controllori PID
    leftPid_.reset();
    rightPid_.reset();
    
    lastAccelUpdate_ = millis();
}

void Maneuver::turnRight(int speed) {
    moving_ = false;
    turning_ = true;
    setDirection(true, false);
    
    // Imposta i target di velocità per la rotazione a destra
    targetLeftSpeed_ = constrain(speed, -100, 100);
    targetRightSpeed_ = constrain(-speed, -100, 100);
    
    // Avvia l'accelerazione
    isAccelerating_ = true;
    isDecelerating_ = false;
    
    // Resetta i controllori PID
    leftPid_.reset();
    rightPid_.reset();
    
    lastAccelUpdate_ = millis();
}

void Maneuver::stop() {
    moving_ = false;
    turning_ = false;
    
    // Azzera le velocità target
    targetLeftSpeed_ = 0;
    targetRightSpeed_ = 0;
    currentLeftSpeed_ = 0;
    currentRightSpeed_ = 0;
    
    // Ferma i motori
    if (leftMotor_) {
        leftMotor_->setSpeed(0);
        leftMotor_->stop();
    }
    if (rightMotor_) {
        rightMotor_->setSpeed(0);
        rightMotor_->stop();
    }
}

// Complex maneuvers
void Maneuver::rotate(int degrees, int speed) {
    // TODO: Implement rotation logic using encoders
    // This is a placeholder implementation
    if (degrees > 0) {
        turnRight(speed);
    } else {
        turnLeft(speed);
    }
    // Add delay or encoder-based rotation here
}

void Maneuver::moveStraight(int distance, int speed) {
    // TODO: Implement distance-based movement using encoders
    forward(speed);
    // Add encoder-based distance measurement here
}

void Maneuver::zigzag(int distance, int width, int speed) {
    // TODO: Implement zigzag pattern
    forward(speed);
    // Add zigzag logic using distance and width parameters
}

void Maneuver::spiral(float maxRadius, float speed) {
    // Usa la larghezza di taglio per calcolare il raggio iniziale e l'incremento
    const float safetyMargin = 0.05f;      // [m] Margine di sicurezza per sovrapposizione
    const float minTurnRadius = 0.3f;      // [m] Raggio minimo di sterzata
    
    // Calcola il raggio iniziale (metà della larghezza di taglio)
    float currentRadius = bladeWidth_ / 2.0f;
    
    // Converti la velocità da m/s a un valore adatto ai motori (0-100)
    int motorSpeed = static_cast<int>((speed / maxLinearSpeed_) * 100.0f);
    motorSpeed = constrain(motorSpeed, 0, 100);
    
    // Variabili per il controllo della posizione
    unsigned long lastUpdate = millis();
    
    // Inizia la spirale
    while (currentRadius < maxRadius) {
        // Calcola la circonferenza a questo raggio
        float circumference = 2.0f * PI * currentRadius;
        
        // Calcola la differenza di velocità per mantenere il raggio di curvatura
        float circumferenceRatio = (currentRadius + wheelBase_/2.0f) / (currentRadius - wheelBase_/2.0f);
        int speedDiff = static_cast<int>((motorSpeed * (circumferenceRatio - 1.0f)) / (1.0f + circumferenceRatio));
        
        // Applica la velocità con correzione di traiettoria
        setSpeed(motorSpeed + speedDiff, motorSpeed - speedDiff, true);
        
        // Calcola il tempo necessario per completare il giro
        float timeForCircle = circumference / speed;  // in secondi
        unsigned long startTime = millis();
        
        // Esegui il cerchio con controllo continuo
        while ((millis() - startTime) < (timeForCircle * 1000)) {
            // Qui potresti aggiungere controlli per condizioni di uscita
            if (!moving_) {
                return;  // Esci se il movimento viene interrotto
            }
            
            // Aggiornamento periodico
            if ((millis() - lastUpdate) > 100) {
                lastUpdate = millis();
            }
            
            delay(10);  // Piccola pausa per non sovraccaricare il processore
        }
        
        // Aumenta il raggio per il prossimo giro
        currentRadius += (bladeWidth_ - safetyMargin);
        
        // Assicurati di non scendere sotto il raggio minimo
        if (currentRadius < minTurnRadius) {
            currentRadius = minTurnRadius;
        }
    }
    
    // Ferma il robot alla fine della spirale
    stop();
}

// Manteniamo il vecchio metodo per retrocompatibilità
void Maneuver::spiral(int speed) {
    // Converti la velocità da 0-100 a m/s
    float speedMps = (speed / 100.0f) * maxLinearSpeed_;
    spiral(5.0f, speedMps);  // Usa il raggio massimo di default di 5m
}

void Maneuver::begin() {
    // Resetta lo stato
    moving_ = false;
    turning_ = false;
    
    // Ferma i motori
    stop();
    
    // Reset motor positions if needed
    // Note: This is a placeholder. The DriveMotor class should implement position tracking
    if (leftMotor_) leftMotor_->setPowerPercentImmediate(0);
    if (rightMotor_) rightMotor_->setPowerPercentImmediate(0);
}

// Private helper methods
void Maneuver::setSpeed(int leftSpeed, int rightSpeed, bool isTrajectoryCorrection) {
    // Se è una correzione di traiettoria, applica solo una piccola variazione
    if (isTrajectoryCorrection) {
        targetLeftSpeed_ = constrain(currentLeftSpeed_ + leftSpeed, -100, 100);
        targetRightSpeed_ = constrain(currentRightSpeed_ + rightSpeed, -100, 100);
    } else {
        // Altrimenti, imposta direttamente le velocità target
        targetLeftSpeed_ = constrain(leftSpeed, -100, 100);
        targetRightSpeed_ = constrain(rightSpeed, -100, 100);
        
        // Imposta la velocità direttamente (la direzione è gestita dal segno della velocità)
        // Usa la nuova API percentuale
        if (leftMotor_) {
            leftMotor_->setPowerPercent(targetLeftSpeed_);
        }
        if (rightMotor_) {
            rightMotor_->setPowerPercent(targetRightSpeed_);
        }
    }
    
    // Aggiorna lo stato
    moving_ = (abs(targetLeftSpeed_) > 0 || abs(targetRightSpeed_) > 0);
    turning_ = ((targetLeftSpeed_ >= 0) != (targetRightSpeed_ >= 0));
    
    #ifdef DEBUG_MODE
    if (!isTrajectoryCorrection) {
        SERIAL_DEBUG.print(F("Set speed - Left: "));
        SERIAL_DEBUG.print(targetLeftSpeed_);
        SERIAL_DEBUG.print(F("%, Right: "));
        SERIAL_DEBUG.print(targetRightSpeed_);
        SERIAL_DEBUG.println(F("%"));
    }
    #endif
}

void Maneuver::setSpeed(int speed, bool isTrajectoryCorrection) {
    setSpeed(speed, speed, isTrajectoryCorrection);
}

void Maneuver::setDirection(bool leftForward, bool rightForward) {
    // For MotorBase, we'll set negative speed for reverse
    // The actual direction will be handled by the motor driver
    // Range 0-100% mappato su 0-799 (20kHz PWM con ICR = 799)
    if (leftMotor_) {
        // Otteniamo la velocità in percentuale e invertiamo la direzione se necessario
        int8_t speed = leftMotor_->getCurrentPowerPercent();
        leftMotor_->setPowerPercentImmediate(leftForward ? abs(speed) : -abs(speed));
    }
    if (rightMotor_) {
        // Otteniamo la velocità in percentuale e invertiamo la direzione se necessario
        int8_t speed = rightMotor_->getCurrentPowerPercent();
        rightMotor_->setPowerPercentImmediate(rightForward ? abs(speed) : -abs(speed));
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
    
    // Applica l'output ai motori con la nuova API percentuale
    if (leftMotor_) {
        leftMotor_->setPowerPercent(static_cast<int8_t>(leftOutput));
    }
    if (rightMotor_) {
        rightMotor_->setPowerPercent(static_cast<int8_t>(rightOutput));
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

// getLinearVelocity() and getAngularVelocity() are defined in the header file
