#include "MotorController.h"
#include "MotorTipo1.h"
#include "MotorTipo2.h"

MotorController::MotorController() {
    _leftMotor = nullptr;
    _rightMotor = nullptr;
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
    
    return true;
}

void MotorController::initializeMotors() {
    // Crea una nuova istanza di motore in base alla configurazione
    Motor* motorInstance = createMotorInstance();
    
    if (motorInstance != nullptr) {
        // Configura i pin per il motore sinistro
        motorInstance->configurePins(MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_PPM_PIN);
        _leftMotor = motorInstance;
        
        // Crea una nuova istanza per il motore destro
        motorInstance = createMotorInstance();
        motorInstance->configurePins(MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_PPM_PIN);
        _rightMotor = motorInstance;
    }
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
    return _leftMotor != nullptr ? _leftMotor->getTemperature() : 0.0;
}

float MotorController::getRightMotorTemperature() const {
    return _rightMotor != nullptr ? _rightMotor->getTemperature() : 0.0;
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
