#include "Maneuver.h"
#include "config.h"  // Per le costanti di velocità
#include "DriveMotor.h"
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
void Maneuver::forward(int distanceMeters, float speed) {
    // Assicura che la velocità sia nel range corretto
    speed = constrain(speed, -100.0f, 100.0f);
    
    // Imposta la direzione in base al segno della velocità
    bool isForward = (speed >= 0);
    setDirection(isForward, isForward);
    
    // Se la distanza è 0, movimento continuo
    if (distanceMeters == 0) {
        targetLeftSpeed_ = speed;
        targetRightSpeed_ = speed;
        moving_ = (speed != 0);
        turning_ = false;
        
        if (speed == 0) {
            stop();
        } else {
            setSpeed(speed, speed);
        }
        
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.print(F("Forward continuous at speed: "));
        SERIAL_DEBUG.println(speed);
        #endif
        
        return;
    }
    
    // Se c'è una distanza specificata, calcola il tempo necessario
    if (distanceMeters > 0) {
        if (positionManager_ != nullptr && positionManager_->isOdometryEnabled()) {
            // Salva la posizione di partenza
            RobotPosition pos = positionManager_->getPosition();
            startX_ = pos.x / 100.0f;  // Converti da cm a metri
            startY_ = pos.y / 100.0f;  // Converti da cm a metri
            targetDistance_ = distanceMeters;  // Memorizza la distanza target
            
            #ifdef DEBUG_MODE
            SERIAL_DEBUG.print(F("Starting from X: "));
            SERIAL_DEBUG.print(startX_);
            SERIAL_DEBUG.print(F(", Y: "));
            SERIAL_DEBUG.print(startY_);
            SERIAL_DEBUG.print(F(" with target distance: "));
            SERIAL_DEBUG.println(targetDistance_);
            #endif
        } else {
            // Se non c'è un PositionManager attivo, usa il timer come fallback
            // Calcola il tempo necessario in millisecondi (velocità in m/s, convertita da %)
            float speedMps = (speed / 100.0f) * maxLinearSpeed_;
            unsigned long durationMs = (distanceMeters / speedMps) * 1000.0f;
            movementEndTime_ = millis() + durationMs;
            useTimerForDistance_ = true;
            
            #ifdef DEBUG_MODE
            SERIAL_DEBUG.print(F("Position not available, using timer for "));
            SERIAL_DEBUG.print(durationMs);
            SERIAL_DEBUG.println(F(" ms"));
            #endif
        }
        
        // Imposta la velocità target
        targetLeftSpeed_ = speed;
        targetRightSpeed_ = speed;
        
        // Aggiorna lo stato
        moving_ = true;
        turning_ = false;
        
        // Avvia il movimento
        setSpeed(speed, speed);
        
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.print(F("Forward for "));
        SERIAL_DEBUG.print(distanceMeters);
        SERIAL_DEBUG.print(F(" meters at speed: "));
        SERIAL_DEBUG.println(speed);
        #endif
        
        // Avvia il timer di movimento
        lastAccelUpdate_ = millis();
    } else {
        // Distanza negativa non valida
        stop();
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println(F("Errore: distanza negativa non consentita"));
        #endif
    }
}

// Metodo da chiamare nel loop principale per aggiornare il movimento
void Maneuver::updateMovement() {
    if (!moving_ || (targetLeftSpeed_ == 0 && targetRightSpeed_ == 0)) {
        return;
    }
    
    bool shouldStop = false;
    
    if (useTimerForDistance_ && millis() >= movementEndTime_) {
        // Timer scaduto
        stop();
        moving_ = false;
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println(F("Tempo di movimento scaduto"));
        #endif
        return;
    } else if (positionManager_ != nullptr && !useTimerForDistance_ && positionManager_->isOdometryEnabled()) {
        // Se stiamo usando PositionManager per il controllo della distanza
        // Ottieni la posizione attuale
        RobotPosition pos = positionManager_->getPosition();
        // Calcola la distanza percorsa (converti da cm a metri)
        float dx = (pos.x / 100.0f) - startX_;
        float dy = (pos.y / 100.0f) - startY_;
        float distanceTraveled = sqrt(dx*dx + dy*dy);
        
        #ifdef DEBUG_MODE
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 1000) {  // Stampa ogni secondo
            SERIAL_DEBUG.print(F("Percorso: "));
            SERIAL_DEBUG.print(distanceTraveled);
            SERIAL_DEBUG.print(F(" / "));
            SERIAL_DEBUG.print(targetDistance_);
            SERIAL_DEBUG.println(F(" metri"));
            lastPrint = millis();
        }
        #endif
        
        // Se abbiamo raggiunto o superato la distanza target, fermati
        if (distanceTraveled >= targetDistance_) {
            shouldStop = true;
            #ifdef DEBUG_MODE
            SERIAL_DEBUG.print(F("Raggiunta la distanza target: "));
            SERIAL_DEBUG.print(distanceTraveled);
            SERIAL_DEBUG.println(F(" metri"));
            #endif
        }
    }
    
    // Gestione della rotazione con PositionManager
    if (turning_ && positionManager_ != nullptr && positionManager_->isIMUEnabled()) {
        // Usa PositionManager per controllare l'angolo
        RobotPosition pos = positionManager_->getPosition();
        float currentAngle = pos.theta;
        float angleDiff = normalizeAngle(targetAngle_ - currentAngle);
        
        #ifdef DEBUG_MODE
        static unsigned long lastAnglePrint = 0;
        if (millis() - lastAnglePrint > 100) {  // Stampa ogni 100ms
            SERIAL_DEBUG.print(F("Rotazione: "));
            SERIAL_DEBUG.print(currentAngle * RAD_TO_DEG);
            SERIAL_DEBUG.print(F("° / "));
            SERIAL_DEBUG.print(targetAngle_ * RAD_TO_DEG);
            SERIAL_DEBUG.print(F("° (diff: "));
            SERIAL_DEBUG.print(angleDiff * RAD_TO_DEG);
            SERIAL_DEBUG.println(F("°)"));
            lastAnglePrint = millis();
        }
        #endif
        
        // Soglia di tolleranza (circa 2 gradi)
        if (fabs(angleDiff) < 0.035) {
            shouldStop = true;
            #ifdef DEBUG_MODE
            SERIAL_DEBUG.println(F("Angolo target raggiunto"));
            #endif
        }
    }
    
    if (shouldStop) {
        stop();
        // Resetta le variabili di stato
        startX_ = 0;
        startY_ = 0;
        useTimerForDistance_ = false;
        turning_ = false;
    }
}

void Maneuver::backward(int duration, float speed) {
    // Imposta la velocità target per entrambi i motori (valori negativi per andare indietro)
    targetLeftSpeed_ = constrain(-speed, -100.0f, 100.0f);
    targetRightSpeed_ = constrain(-speed, -100.0f, 100.0f);
    
    // Imposta la direzione all'indietro
    setDirection(false, false);
    
    // Aggiorna lo stato
    moving_ = (speed != 0);
    turning_ = false;
    
    // Se la velocità è zero, ferma il robot
    if (speed == 0) {
        stop();
    } else {
        // Altrimenti, imposta la velocità
        setSpeed(-speed, -speed);
    }
    
    // TODO: Implementare gestione durata
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Backward at speed: "));
    SERIAL_DEBUG.println(speed);
    #endif
    
    lastAccelUpdate_ = millis();
}

void Maneuver::turnLeft(int angle, float speed) {
    // Se abbiamo PositionManager con IMU abilitato, usiamo l'angolo reale
    if (positionManager_ != nullptr && positionManager_->isIMUEnabled()) {
        // Converti l'angolo in radianti
        float targetAngle = normalizeAngle(angle * DEG_TO_RAD);
        
        // Ottieni l'angolo corrente
        RobotPosition pos = positionManager_->getPosition();
        float currentAngle = pos.theta;
        
        // Calcola l'angolo target assoluto
        targetAngle_ = normalizeAngle(currentAngle + targetAngle);
        
        // Imposta la velocità dei motori per girare a sinistra
        targetLeftSpeed_ = constrain(-speed, -100.0f, 100.0f);
        targetRightSpeed_ = constrain(speed, -100.0f, 100.0f);
        
        // Imposta la direzione
        setDirection(false, true);
        
        // Aggiorna lo stato
        moving_ = (speed != 0);
        turning_ = (speed != 0);
        
        // Imposta la velocità
        setSpeed(-speed, speed);
        
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.print(F("Turning left from: "));
        SERIAL_DEBUG.print(currentAngle * RAD_TO_DEG);
        SERIAL_DEBUG.print(F(" to: "));
        SERIAL_DEBUG.print(targetAngle_ * RAD_TO_DEG);
        SERIAL_DEBUG.print(F(" degrees at speed: "));
        SERIAL_DEBUG.println(speed);
        #endif
    } else {
        // Fallback al comportamento originale se PositionManager non è disponibile
        targetLeftSpeed_ = constrain(-speed, -100.0f, 100.0f);
        targetRightSpeed_ = constrain(speed, -100.0f, 100.0f);
        
        setDirection(false, true);
        moving_ = (speed != 0);
        turning_ = (speed != 0);
        
        if (speed == 0) {
            stop();
        } else {
            setSpeed(-speed, speed);
        }
        
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.print(F("Turn left (no IMU) at speed: "));
        SERIAL_DEBUG.println(speed);
        #endif
    }
}

void Maneuver::turnRight(int angle, float speed) {
    // Se abbiamo PositionManager con IMU abilitato, usiamo l'angolo reale
    if (positionManager_ != nullptr && positionManager_->isIMUEnabled()) {
        // Converti l'angolo in radianti (negativo per girare a destra)
        float targetAngle = normalizeAngle(-angle * DEG_TO_RAD);
        
        // Ottieni l'angolo corrente
        RobotPosition pos = positionManager_->getPosition();
        float currentAngle = pos.theta;
        
        // Calcola l'angolo target assoluto
        targetAngle_ = normalizeAngle(currentAngle + targetAngle);
        
        // Imposta la velocità dei motori per girare a destra
        targetLeftSpeed_ = constrain(speed, -100.0f, 100.0f);
        targetRightSpeed_ = constrain(-speed, -100.0f, 100.0f);
        
        // Imposta la direzione
        setDirection(true, false);
        
        // Aggiorna lo stato
        moving_ = (speed != 0);
        turning_ = (speed != 0);
        
        // Imposta la velocità
        setSpeed(speed, -speed);
        
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.print(F("Turning right from: "));
        SERIAL_DEBUG.print(currentAngle * RAD_TO_DEG);
        SERIAL_DEBUG.print(F(" to: "));
        SERIAL_DEBUG.print(targetAngle_ * RAD_TO_DEG);
        SERIAL_DEBUG.print(F(" degrees at speed: "));
        SERIAL_DEBUG.println(speed);
        #endif
    } else {
        // Fallback al comportamento originale se PositionManager non è disponibile
        targetLeftSpeed_ = constrain(speed, -100.0f, 100.0f);
        targetRightSpeed_ = constrain(-speed, -100.0f, 100.0f);
        
        setDirection(true, false);
        moving_ = (speed != 0);
        turning_ = (speed != 0);
        
        if (speed == 0) {
            stop();
        } else {
            setSpeed(speed, -speed);
        }
        
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.print(F("Turn right (no IMU) at speed: "));
        SERIAL_DEBUG.println(speed);
        #endif
    }
}

void Maneuver::rotateLeft(float speed) {
    // Imposta i target di velocità per ruotare a sinistra (sul posto)
    targetLeftSpeed_ = constrain(-speed, -100.0f, 100.0f);
    targetRightSpeed_ = constrain(speed, -100.0f, 100.0f);
    
    // Imposta la direzione appropriata per ogni motore
    setDirection(false, true);
    
    // Aggiorna lo stato
    moving_ = false;
    turning_ = (speed != 0);
    
    // Se la velocità è zero, ferma il robot
    if (speed == 0) {
        stop();
    } else {
        // Altrimenti, imposta la velocità
        setSpeed(-speed, speed);
    }
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Rotate left at speed: "));
    SERIAL_DEBUG.println(speed);
    #endif
}

void Maneuver::rotateRight(float speed) {
    // Imposta i target di velocità per ruotare a destra (sul posto)
    targetLeftSpeed_ = constrain(speed, -100.0f, 100.0f);
    targetRightSpeed_ = constrain(-speed, -100.0f, 100.0f);
    
    // Imposta la direzione appropriata per ogni motore
    setDirection(true, false);
    
    // Aggiorna lo stato
    moving_ = false;
    turning_ = (speed != 0);
    
    // Se la velocità è zero, ferma il robot
    if (speed == 0) {
        stop();
    } else {
        // Altrimenti, imposta la velocità
        setSpeed(speed, -speed);
    }
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Rotate right at speed: "));
    SERIAL_DEBUG.println(speed);
    #endif
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
void Maneuver::rotate(int degrees, float speed) {
    // Normalizza l'angolo tra -180 e 180 gradi
    while (degrees > 180) degrees -= 360;
    while (degrees < -180) degrees += 360;
    
    // Determina la direzione di rotazione
    if (degrees > 0) {
        rotateRight(speed);
    } else {
        rotateLeft(-speed);
    }
    
    // TODO: Implementare la rotazione per l'angolo specificato
    // usando l'odometria o un timer
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Rotazione di "));
    SERIAL_DEBUG.print(degrees);
    SERIAL_DEBUG.print(F(" gradi a velocità "));
    SERIAL_DEBUG.println(speed);
    #endif
}

void Maneuver::moveStraight(int distance, float speed) {
    // Imposta la direzione in base alla distanza
    bool isForward = (distance >= 0);
    
    // Imposta la velocità
    if (isForward) {
        forward(abs(distance), speed);
    } else {
        backward(abs(distance), speed);
    }
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Movimento rettilineo di "));
    SERIAL_DEBUG.print(distance);
    SERIAL_DEBUG.print(F(" cm a velocità "));
    SERIAL_DEBUG.println(speed);
    #endif
}

void Maneuver::zigzag(int distance, int width, float speed) {
    // Salva i parametri
    zigzagDistance_ = abs(distance);  // Distanza in cm
    zigzagWidth_ = abs(width);       // Larghezza in cm
    zigzagSpeed_ = constrain(speed, 0.0f, 100.0f);
    
    // Inizializza lo stato
    isZigzagging_ = true;
    zigzagState_ = ZIGZAG_FORWARD;
    zigzagDirection_ = 1;  // Inizia con una svolta a destra
    
    // Resetta i contatori
    zigzagStartTime_ = millis();
    
    // Inizia ad andare dritto
    forward(0, zigzagSpeed_);  // 0 = durata illimitata
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Avvio zigzag - Distanza: "));
    SERIAL_DEBUG.print(zigzagDistance_);
    SERIAL_DEBUG.print(F(" cm, Larghezza: "));
    SERIAL_DEBUG.print(zigzagWidth_);
    SERIAL_DEBUG.print(F(" cm, Velocità: "));
    SERIAL_DEBUG.println(zigzagSpeed_);
    #endif
}

// Aggiorna il movimento a zigzag (da chiamare nel loop principale)
void Maneuver::updateZigzag() {
    if (!isZigzagging_) return;
    
    if (leftMotor_ == nullptr || rightMotor_ == nullptr) {
        stop();
        isZigzagging_ = false;
        return;
    }
    
    float currentX = (leftMotor_->getX() + rightMotor_->getX()) / 2.0f;
    float currentY = (leftMotor_->getY() + rightMotor_->getY()) / 2.0f;
    
    if (zigzagState_ == ZIGZAG_TURNING) {
        // Siamo in fase di svolta
        float currentAngle = atan2(leftMotor_->getY() - rightMotor_->getY(), 
                                 leftMotor_->getX() - rightMotor_->getX());
        float angleDiff = atan2(sin(zigzagTargetAngle_ - currentAngle), 
                              cos(zigzagTargetAngle_ - currentAngle));
        
        // Se abbiamo raggiunto l'angolo target, riprendi il movimento rettilineo
        if (abs(angleDiff) < 0.1f) {  // ~5.7 gradi
            zigzagState_ = ZIGZAG_FORWARD;
            forward(0, zigzagSpeed_);  // 0 = durata illimitata
            
            // Aggiorna l'ultima posizione di svolta
            lastTurnX_ = currentX;
            lastTurnY_ = currentY;
            
            #ifdef DEBUG_MODE
            SERIAL_DEBUG.println(F("Fine svolta, riprendo avanti dritto"));
            #endif
        }
    } else {
        // Siamo in fase di movimento rettilineo
        // Calcola la distanza percorsa dall'ultima svolta
        float dx = currentX - lastTurnX_;
        float dy = currentY - lastTurnY_;
        float distance = sqrt(dx*dx + dy*dy);
        
        // Se abbiamo raggiunto la distanza desiderata, inizia la svolta
        if (distance >= zigzagDistance_) {
            zigzagState_ = ZIGZAG_TURNING;
            
            // Calcola l'angolo corrente
            float currentAngle = atan2(leftMotor_->getY() - rightMotor_->getY(), 
                                     leftMotor_->getX() - rightMotor_->getX());
            
            // Imposta l'angolo target (90 gradi nella direzione appropriata)
            zigzagTargetAngle_ = currentAngle + (zigzagDirection_ * HALF_PI);
            
            // Esegui la svolta
            if (zigzagDirection_ > 0) {
                turnRight(90, zigzagSpeed_ * 0.7f);  // Ruota di 90° a destra
            } else {
                turnLeft(90, zigzagSpeed_ * 0.7f);   // Ruota di 90° a sinistra
            }
            
            #ifdef DEBUG_MODE
            SERIAL_DEBUG.print(F("Inizio svolta a "));
            SERIAL_DEBUG.print(zigzagDirection_ > 0 ? "destra" : "sinistra");
            SERIAL_DEBUG.print(F(" fino a "));
            SERIAL_DEBUG.print(zigzagTargetAngle_ * RAD_TO_DEG);
            SERIAL_DEBUG.println(F(" gradi"));
            #endif
        }
    }
}

void Maneuver::updateObstacleAvoidance() {
    if (!isAvoidingObstacle_) return;
    
    unsigned long currentTime = millis();
    unsigned long maneuverTime = currentTime - avoidanceStartTime_;
    
    switch (avoidanceManeuver_) {
        case 1: // Retromarcia
            backward(1000, savedTargetLeftSpeed_ * 0.5f);
            if (maneuverTime > 1000) { // Retrocede per 1 secondo
                avoidanceManeuver_ = 2; // Prossimo passo: gira a destra
                avoidanceStartTime_ = currentTime;
            }
            break;
            
        case 2: // Gira a destra
            turnRight(90, savedTargetLeftSpeed_ * 0.7f);
            if (maneuverTime > 1500) { // Gira per 1.5 secondi
                avoidanceManeuver_ = 3; // Prossimo passo: avanti
                avoidanceStartTime_ = currentTime;
            }
            break;
            
        case 3: // Avanti
            forward(2000, savedTargetLeftSpeed_);
            if (maneuverTime > 2000 || !obstacleDetected_) { // Avanti per 2 secondi o fino a quando non c'è più l'ostacolo
                resumeAfterObstacle();
            }
            break;
    }
}

void Maneuver::resumeAfterObstacle() {
    // Ripristina lo stato precedente
    isAvoidingObstacle_ = false;
    obstaclePhase1Done_ = false;
    obstaclePhase2Done_ = false;
    obstaclePhase3Done_ = false;
    
    // Riprendi il movimento precedente
    if (isZigzagging_) {
        // Se stava facendo zigzag, riprendi lo zigzag
        forward(0, zigzagSpeed_);  // 0 = durata illimitata
    } else if (turning_) {
        // Se stava girando, riprendi la rotazione
        if (targetLeftSpeed_ > targetRightSpeed_) {
            turnRight(90, targetLeftSpeed_);  // 90° è un valore predefinito
        } else {
            turnLeft(90, targetRightSpeed_);   // 90° è un valore predefinito
        }
    } else {
        // Altrimenti riprendi il movimento rettilineo
        float avgSpeed = (savedTargetLeftSpeed_ + savedTargetRightSpeed_) / 2.0f;
        if (avgSpeed >= 0) {
            forward(0, avgSpeed);  // 0 = durata illimitata
        } else {
            backward(0, -avgSpeed); // 0 = durata illimitata
        }
    }
    
    // Resetta i flag di stato
    obstaclePhase1Done_ = false;
    obstaclePhase2Done_ = false;
    obstaclePhase3Done_ = false;
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.println(F("Ripresa movimento dopo ostacolo"));
    #endif
}

void Maneuver::spiral(float maxRadius, float speed) {
    // Variabili per il controllo della posizione
    unsigned long lastUpdate = millis();
    float currentRadius = 0.0f;
    const float bladeWidth = bladeWidth_ > 0 ? bladeWidth_ : 0.2f;  // Larghezza del taglio in metri
    const float safetyMargin = 0.05f; // Margine di sicurezza in metri
    const float minTurnRadius = 0.3f; // Raggio minimo di sterzata in metri
    
    // Assicurati che il robot sia fermo all'inizio
    stop();
    
    // Imposta la velocità iniziale
    float motorSpeed = constrain(speed, 0.0f, 100.0f);
    
    // Variabili per il controllo della posizione
    float currentX = 0.0f, currentY = 0.0f;
    float currentAngle = 0.0f;  // Angolo in radianti
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Avvio spirale - Raggio max: "));
    SERIAL_DEBUG.print(maxRadius);
    SERIAL_DEBUG.print(F(" m, Velocità: "));
    SERIAL_DEBUG.println(motorSpeed);
    #endif
    
    // Continua fino a raggiungere il raggio massimo
    while (currentRadius < maxRadius) {
        if (currentRadius < minTurnRadius) {
            // All'inizio, vai dritto per evitare curve troppo strette
            forward(100, motorSpeed);  // Avanza per 100ms
            currentX += 0.1f * cos(currentAngle);  // Stima la nuova posizione
            currentY += 0.1f * sin(currentAngle);
        } else {
            // Calcola la differenza di velocità per mantenere la traiettoria circolare
            float circumferenceRatio = (currentRadius + bladeWidth) / currentRadius;
            float speedDiff = (motorSpeed * (circumferenceRatio - 1.0f)) / (1.0f + circumferenceRatio);
            
            // Imposta velocità diverse per i due motori per creare la curva
            setSpeed(
                constrain(motorSpeed + speedDiff, 0.0f, 100.0f),
                constrain(motorSpeed - speedDiff, 0.0f, 100.0f)
            );
            
            // Stima il nuovo angolo
            float deltaAngle = (0.1f / currentRadius);  // Angolo percorso in questo passo
            currentAngle += deltaAngle;
            
            // Stima la nuova posizione
            currentX = currentRadius * cos(currentAngle);
            currentY = currentRadius * sin(currentAngle);
        }
        
        // Aspetta un po' prima del prossimo aggiornamento
        delay(100);
        
        // Aggiorna il raggio corrente
        currentRadius = sqrt(currentX * currentX + currentY * currentY);
        
        // Aumenta gradualmente il raggio di sterzata
        currentRadius += (bladeWidth - safetyMargin);
        
        // Non scendere sotto il raggio minimo di sterzata
        if (currentRadius < minTurnRadius) {
            currentRadius = minTurnRadius;
        }
        
        // Controlla se è stato rilevato un ostacolo
        if (obstacleDetected_) {
            // Ferma il movimento
            stop();
            // Esegui la manovra di evitamento
            setObstacleDetected(true);
            // Riprendi la spirale
            currentRadius = sqrt(currentX * currentX + currentY * currentY);
        }
    }
    
    // Alla fine della spirale, ferma il robot
    stop();
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.print(F("Spirale completata fino a raggio "));
    SERIAL_DEBUG.print(currentRadius);
    SERIAL_DEBUG.println(F(" metri"));
    #endif
}

// Manteniamo il vecchio metodo per retrocompatibilità
void Maneuver::startSpiral(float speed) {
    // Avvia una spirale con raggio massimo predefinito e la velocità specificata
    spiral(5.0f, speed);  // Raggio massimo di 5 metri, velocità specificata
}

void Maneuver::begin() {
    // Resetta lo stato
    moving_ = false;
    turning_ = false;
    
    // Ferma i motori
    stop();
    
    // Reset motor positions if needed
    // Note: This is a placeholder. The DriveMotor class should implement position tracking
    if (leftMotor_) leftMotor_->setSpeedImmediate(0);
    if (rightMotor_) rightMotor_->setSpeedImmediate(0);
}

// Private helper methods
void Maneuver::setSpeed(float leftSpeed, float rightSpeed, bool isTrajectoryCorrection) {
    // Se è una correzione di traiettoria, applica solo una piccola variazione
    if (isTrajectoryCorrection) {
        targetLeftSpeed_ = constrain(currentLeftSpeed_ + leftSpeed, -100.0f, 100.0f);
        targetRightSpeed_ = constrain(currentRightSpeed_ + rightSpeed, -100.0f, 100.0f);
    } else {
        // Altrimenti, imposta direttamente le velocità target
        targetLeftSpeed_ = constrain(leftSpeed, -100.0f, 100.0f);
        targetRightSpeed_ = constrain(rightSpeed, -100.0f, 100.0f);
    }
    
    // Imposta la velocità direttamente (la direzione è gestita dal segno della velocità)
    // Usa la nuova API percentuale
    if (leftMotor_) {
        leftMotor_->setSpeed(targetLeftSpeed_);
    }
    if (rightMotor_) {
        rightMotor_->setSpeed(targetRightSpeed_);
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

void Maneuver::setSpeed(float speed, bool isTrajectoryCorrection) {
    setSpeed(speed, speed, isTrajectoryCorrection);
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
