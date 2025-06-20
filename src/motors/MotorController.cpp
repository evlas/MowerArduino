#include "MotorController.h"
#include "MotorTipo1.h"
#include "MotorTipo2.h"

MotorController::MotorController(float diameter, float base, float ticks) {
    _leftMotor = nullptr;
    _rightMotor = nullptr;
    _hasLeftMotorError = false;
    _hasRightMotorError = false;
    
    // Inizializza l'odometria con i valori forniti
    initializeOdometry(diameter, base, ticks);

    // Imposta variabili odometriche a valori iniziali; i motori non sono
    // ancora creati, quindi non possiamo leggere le loro posizioni.
    _x = _y = _theta = 0.0f;
    _lastLeftPos = 0.0f;
    _lastRightPos = 0.0f;
    _lastUpdate = 0;
}

MotorController::~MotorController() {
    if (_leftMotor != nullptr) {
        delete _leftMotor;
    }
    if (_rightMotor != nullptr) {
        delete _rightMotor;
    }
}

bool MotorController::begin() {
    // Crea e configura i motori
    initializeMotors();
    
    // Inizializza i motori
    if (_leftMotor == nullptr || _rightMotor == nullptr) {
        return false;
    }
    
    if (!_leftMotor->initialize() || !_rightMotor->initialize()) {
        return false;
    }
    
    // Resetta l'odometria
    resetOdometry();
    
    // Resetta gli errori
    clearErrors();
    
    return true;
}

// Gestione errori
bool MotorController::hasLeftMotorError() const {
    return _hasLeftMotorError;
}

bool MotorController::hasRightMotorError() const {
    return _hasRightMotorError;
}

void MotorController::clearErrors() {
    _hasLeftMotorError = false;
    _hasRightMotorError = false;
    
    if (_leftMotor != nullptr) {
        _leftMotor->clearFault();
    }
    if (_rightMotor != nullptr) {
        _rightMotor->clearFault();
    }
}

// Reset posizione
void MotorController::initializeMotors() {
    // Crea una nuova istanza di motore in base alla configurazione
    Motor* motorInstance = createMotorInstance();
    
    if (motorInstance != nullptr) {
        // Configura i pin per il motore sinistro
        motorInstance->configurePins(MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_PPM_PIN);
        _leftMotor = motorInstance;
        
            // Crea una nuova istanza per il motore destro
        motorInstance = createMotorInstance();
        if (motorInstance != nullptr) {
            motorInstance->configurePins(MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_PPM_PIN);
            _rightMotor = motorInstance;
        }
    }
}

void MotorController::initializeOdometry(float diameter, float base, float ticks) {
    _wheelDiameter = diameter;
    _wheelBase = base;
    _ticksPerRevolution = ticks;
}

void MotorController::resetOdometry() {
    _x = 0.0f;
    _y = 0.0f;
    _theta = 0.0f;
    _lastLeftPos = _leftMotor->getPosition();
    _lastRightPos = _rightMotor->getPosition();
    _lastUpdate = millis();
}

void MotorController::updateOdometry() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - _lastUpdate) / 1000.0f; // in secondi
    
    // Leggi le nuove posizioni
    float currentLeftPos = _leftMotor->getPosition();
    float currentRightPos = _rightMotor->getPosition();
    
    // Calcola le distanze percorse
    float leftDistance = calculateDistance(currentLeftPos - _lastLeftPos);
    float rightDistance = calculateDistance(currentRightPos - _lastRightPos);
    
    // Aggiorna le posizioni precedenti
    _lastLeftPos = currentLeftPos;
    _lastRightPos = currentRightPos;
    _lastUpdate = currentTime;
    
    // Calcola l'orientamento
    _theta += calculateTheta(leftDistance, rightDistance);
    
    // Calcola la posizione
    float avgDistance = (leftDistance + rightDistance) / 2.0f;
    _x += avgDistance * cos(_theta);
    _y += avgDistance * sin(_theta);
}

float MotorController::calculateDistance(float ticks) const {
    return (ticks / _ticksPerRevolution) * PI * _wheelDiameter;
}

float MotorController::calculateTheta(float leftDistance, float rightDistance) const {
    return (rightDistance - leftDistance) / _wheelBase;
}

float MotorController::getX() const {
    return _x;
}

float MotorController::getY() const {
    return _y;
}

float MotorController::getTheta() const {
    return _theta;
}

float MotorController::getLinearVelocity() const {
    float currentLeftPos = _leftMotor->getPosition();
    float currentRightPos = _rightMotor->getPosition();
    
    float leftDistance = calculateDistance(currentLeftPos - _lastLeftPos);
    float rightDistance = calculateDistance(currentRightPos - _lastRightPos);
    
    float avgDistance = (leftDistance + rightDistance) / 2.0f;
    float deltaTime = (millis() - _lastUpdate) / 1000.0f;
    
    return avgDistance / deltaTime;
}

float MotorController::getAngularVelocity() const {
    float currentLeftPos = _leftMotor->getPosition();
    float currentRightPos = _rightMotor->getPosition();
    
    float leftDistance = calculateDistance(currentLeftPos - _lastLeftPos);
    float rightDistance = calculateDistance(currentRightPos - _lastRightPos);
    
    float thetaChange = calculateTheta(leftDistance, rightDistance);
    float deltaTime = (millis() - _lastUpdate) / 1000.0f;
    
    return thetaChange / deltaTime;
}

Motor* MotorController::createMotorInstance() {
    #ifdef DRIVE_MOTOR_TIPO1
    return new MotorTipo1();
    #elif defined(DRIVE_MOTOR_TIPO2)
    return new MotorTipo2();
    #else
    return nullptr;
    #endif
}

// Controllo motore sinistro
void MotorController::setLeftMotorSpeed(int speed) {
    if (_leftMotor != nullptr) {
        _leftMotor->setSpeed(speed);
    }
}

void MotorController::setLeftMotorDirection(bool forward) {
    if (_leftMotor != nullptr) {
        _leftMotor->setDirection(forward);
    }
}

// Controllo motore destro
void MotorController::setRightMotorSpeed(int speed) {
    if (_rightMotor != nullptr) {
        _rightMotor->setSpeed(speed);
    }
}

void MotorController::setRightMotorDirection(bool forward) {
    if (_rightMotor != nullptr) {
        _rightMotor->setDirection(forward);
    }
}

// Stato dei motori
bool MotorController::isLeftMotorRunning() const {
    return _leftMotor != nullptr && _leftMotor->isRunning();
}

bool MotorController::isRightMotorRunning() const {
    return _rightMotor != nullptr && _rightMotor->isRunning();
}

bool MotorController::isLeftMotorFault() const {
    return _leftMotor != nullptr && _leftMotor->isFault();
}

bool MotorController::isRightMotorFault() const {
    return _rightMotor != nullptr && _rightMotor->isFault();
}

// Controllo tensione
float MotorController::getLeftMotorVoltage() const {
    return _leftMotor != nullptr ? _leftMotor->getVoltage() : 0.0;
}

float MotorController::getRightMotorVoltage() const {
    return _rightMotor != nullptr ? _rightMotor->getVoltage() : 0.0;
}

// Controllo temperatura
float MotorController::getLeftMotorTemperature() const {
    return _leftMotor->getTemperature();
}

float MotorController::getRightMotorTemperature() const {
    return _rightMotor->getTemperature();
}

// Velocità dei motori
int MotorController::getLeftMotorSpeed() const {
    return _leftMotor->getSpeed();
}

int MotorController::getRightMotorSpeed() const {
    return _rightMotor->getSpeed();
}

// Gestione errori
void MotorController::clearLeftMotorFault() {
    if (_leftMotor != nullptr) {
        _leftMotor->clearFault();
    }
}

void MotorController::clearRightMotorFault() {
    if (_rightMotor != nullptr) {
        _rightMotor->clearFault();
    }
}

// Reset posizione
void MotorController::resetPositionLeft() {
    if (_leftMotor != nullptr) {
        _leftMotor->resetPosition();
    }
}

void MotorController::resetPositionRight() {
    if (_rightMotor != nullptr) {
        _rightMotor->resetPosition();
    }
}
