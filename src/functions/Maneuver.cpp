#include "Maneuver.h"
#include "../motors/MotorController.h"

Maneuver::Maneuver(MotorController* motorController, 
                   float bladeWidth, 
                   float wheelBase,
                   float maxLinearSpeed) :
    motorController(motorController),
    moving(false),
    turning(false),
    _bladeWidth(bladeWidth),
    _wheelBase(wheelBase),
    _maxLinearSpeed(maxLinearSpeed),
    _currentLeftSpeed(0.0f),
    _currentRightSpeed(0.0f),
    _targetLeftSpeed(0.0f),
    _targetRightSpeed(0.0f),
    _accelStep(0.0f),
    _isAccelerating(false),
    _isDecelerating(false) {
    
    // Inizializza i controllori PID
    // Valori iniziali (da tarare in base al robot)
    float kp = 2.0f;  // Guadagno proporzionale
    float ki = 0.5f;  // Guadagno integrale
    float kd = 0.1f;  // Guadagno derivativo
    
    // Inizializza i controllori per ogni lato
    _leftPid.init(kp, ki, kd, -100.0f, 100.0f);  // Range -100% a 100%
    _rightPid.init(kp, ki, kd, -100.0f, 100.0f);
    
    _lastAccelUpdate = millis();
}

Maneuver::~Maneuver() {
    stop();
}

// Basic movements
void Maneuver::forward(int speed) {
    moving = true;
    turning = false;
    setDirection(true, true);
    
    // Imposta i target di velocità
    _targetLeftSpeed = constrain(speed, -100, 100);
    _targetRightSpeed = constrain(speed, -100, 100);
    
    // Avvia l'accelerazione
    _isAccelerating = true;
    _isDecelerating = false;
    
    // Resetta i controllori PID
    _leftPid.reset();
    _rightPid.reset();
    
    _lastAccelUpdate = millis();
}

void Maneuver::backward(int speed) {
    moving = true;
    turning = false;
    setDirection(false, false);
    
    // Imposta i target di velocità (negativi per andare indietro)
    _targetLeftSpeed = constrain(-speed, -100, 100);
    _targetRightSpeed = constrain(-speed, -100, 100);
    
    // Avvia l'accelerazione
    _isAccelerating = true;
    _isDecelerating = false;
    
    // Resetta i controllori PID
    _leftPid.reset();
    _rightPid.reset();
    
    _lastAccelUpdate = millis();
}

void Maneuver::turnLeft(int speed) {
    moving = false;
    turning = true;
    setDirection(true, false);
    _targetLeftSpeed = speed * 0.5f;  // Riduci la velocità in curva
    setDirection(false, true);
    
    // Imposta i target di velocità per la rotazione a sinistra
    _targetLeftSpeed = constrain(-speed, -100, 100);
    _targetRightSpeed = constrain(speed, -100, 100);
    
    // Avvia l'accelerazione
    _isAccelerating = true;
    _isDecelerating = false;
    
    // Resetta i controllori PID
    _leftPid.reset();
    _rightPid.reset();
    
    _lastAccelUpdate = millis();
}

void Maneuver::turnRight(int speed) {
    moving = false;
    turning = true;
    setDirection(true, false);
    
    // Imposta i target di velocità per la rotazione a destra
    _targetLeftSpeed = constrain(speed, -100, 100);
    _targetRightSpeed = constrain(-speed, -100, 100);
    
    // Avvia l'accelerazione
    _isAccelerating = true;
    _isDecelerating = false;
    
    // Resetta i controllori PID
    _leftPid.reset();
    _rightPid.reset();
    
    _lastAccelUpdate = millis();
}

void Maneuver::stop() {
    moving = false;
    turning = false;
    _targetLeftSpeed = 0;
    _targetRightSpeed = 0;
    _isAccelerating = false;
    _isDecelerating = true;
    _lastAccelUpdate = millis();
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
    float currentRadius = _bladeWidth / 2.0f;
    
    // Converti la velocità da m/s a un valore adatto ai motori (0-100)
    int motorSpeed = static_cast<int>((speed / _maxLinearSpeed) * 100.0f);
    motorSpeed = constrain(motorSpeed, 0, 100);
    
    // Variabili per il controllo della posizione
    unsigned long lastUpdate = millis();
    
    // Inizia la spirale
    while (currentRadius < maxRadius) {
        // Calcola la circonferenza a questo raggio
        float circumference = 2.0f * PI * currentRadius;
        
        // Calcola la differenza di velocità per mantenere il raggio di curvatura
        float circumferenceRatio = (currentRadius + _wheelBase/2.0f) / (currentRadius - _wheelBase/2.0f);
        int speedDiff = static_cast<int>((motorSpeed * (circumferenceRatio - 1.0f)) / (1.0f + circumferenceRatio));
        
        // Applica la velocità con correzione di traiettoria
        setSpeed(motorSpeed + speedDiff, motorSpeed - speedDiff, true);
        
        // Calcola il tempo necessario per completare il giro
        float timeForCircle = circumference / speed;  // in secondi
        unsigned long startTime = millis();
        
        // Esegui il cerchio con controllo continuo
        while ((millis() - startTime) < (timeForCircle * 1000)) {
            // Qui potresti aggiungere controlli per condizioni di uscita
            if (!isMoving()) {
                return;  // Esci se il movimento viene interrotto
            }
            
            // Aggiornamento periodico
            if ((millis() - lastUpdate) > 100) {
                lastUpdate = millis();
            }
            
            delay(10);  // Piccola pausa per non sovraccaricare il processore
        }
        
        // Aumenta il raggio per il prossimo giro
        currentRadius += (_bladeWidth - safetyMargin);
        
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
    float speedMps = (speed / 100.0f) * MAX_LINEAR_SPEED;
    spiral(5.0f, speedMps);  // Usa il raggio massimo di default di 5m
}

void Maneuver::begin() {
    // Resetta lo stato
    moving = false;
    turning = false;
    
    // Ferma i motori
    stop();
    
    // Resetta la posizione
    motorController->resetPositionLeft();
    motorController->resetPositionRight();
    
    // Resetta l'odometria
    motorController->resetOdometry();
}

// Status
bool Maneuver::isMoving() {
    return moving;
}

bool Maneuver::isTurning() {
    return turning;
}

// Private helper methods
void Maneuver::setSpeed(int leftSpeed, int rightSpeed, bool isTrajectoryCorrection) {
    _isTrajectoryCorrection = isTrajectoryCorrection;
    
    if (!_isTrajectoryCorrection) {
        // Se non è una correzione di traiettoria, aggiorniamo i target di velocità
        _targetLeftSpeed = leftSpeed;
        _targetRightSpeed = rightSpeed;
        _isAccelerating = true;
        _isDecelerating = false;
        _lastAccelUpdate = millis();
    } else {
        // Se è una correzione di traiettoria, applichiamo direttamente le velocità
        // senza passare attraverso il sistema di accelerazione
        motorController->setLeftMotorSpeed(leftSpeed);
        motorController->setRightMotorSpeed(rightSpeed);
        _currentLeftSpeed = leftSpeed;
        _currentRightSpeed = rightSpeed;
    }
}

void Maneuver::setSpeed(int speed, bool isTrajectoryCorrection) {
    setSpeed(speed, speed, isTrajectoryCorrection);
}

void Maneuver::setDirection(bool leftForward, bool rightForward) {
    motorController->setLeftMotorDirection(leftForward);
    motorController->setRightMotorDirection(rightForward);
}

void Maneuver::updateAcceleration() {
    // Se non stiamo né accelerando né decelerando, o se stiamo correggendo la traiettoria, usciamo
    if ((!_isAccelerating && !_isDecelerating) || _isTrajectoryCorrection) {
        return;
    }

    // Calcola il tempo trascorso dall'ultimo aggiornamento in secondi
    unsigned long now = millis();
    float dt = (now - _lastAccelUpdate) / 1000.0f;
    
    // Se il tempo trascorso è troppo piccolo, usciamo per evitare calcoli inutili
    if (dt <= 0) {
        return;
    }
    
    _lastAccelUpdate = now;

    // Usa i controllori PID per calcolare le nuove velocità
    if (_isAccelerating || _isDecelerating) {
        // Calcola le nuove velocità usando i PID
        float leftOutput = _leftPid.compute(_currentLeftSpeed, _targetLeftSpeed);
        float rightOutput = _rightPid.compute(_currentRightSpeed, _targetRightSpeed);
        
        // Applica le correzioni di velocità
        _currentLeftSpeed = constrain(_currentLeftSpeed + leftOutput * dt, -100.0f, 100.0f);
        _currentRightSpeed = constrain(_currentRightSpeed + rightOutput * dt, -100.0f, 100.0f);
        
        // Imposta le nuove velocità
        motorController->setLeftMotorSpeed(static_cast<int>(_currentLeftSpeed));
        motorController->setRightMotorSpeed(static_cast<int>(_currentRightSpeed));
        
        // Verifica se abbiamo raggiunto la velocità target
        bool leftReached = abs(_currentLeftSpeed - _targetLeftSpeed) < 1.0f;
        bool rightReached = abs(_currentRightSpeed - _targetRightSpeed) < 1.0f;
        
        // Se entrambi i motori hanno raggiunto la velocità target
        if (leftReached && rightReached) {
            _isAccelerating = false;
            _isDecelerating = false;
            
            // Resetta i termini integrali per evitare sovraccarichi
            _leftPid.reset();
            _rightPid.reset();
        }
    }
}

int Maneuver::getSpeed() const {
    // Get average speed of both motors
    int leftSpeed = motorController->getLeftMotorSpeed();
    int rightSpeed = motorController->getRightMotorSpeed();
    return (leftSpeed + rightSpeed) / 2;
}

int Maneuver::getLeftSpeed() const {
    return motorController->getLeftMotorSpeed();
}

int Maneuver::getRightSpeed() const {
    return motorController->getRightMotorSpeed();
}

float Maneuver::getLinearVelocity() const {
    return motorController->getLinearVelocity();
}

float Maneuver::getAngularVelocity() const {
    return motorController->getAngularVelocity();
}
