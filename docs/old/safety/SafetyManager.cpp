#include "SafetyManager.h"

SafetyManager::SafetyManager() :
    _currentState(SAFE),
    _lastDebounceTime(0),
    _lastButtonState(false),
    _buttonState(false),
    _stopButtonPressed(false),
    _imu(nullptr),
    _tiltThreshold(TILT_THRESHOLD),
    _liftThreshold(LIFT_THRESHOLD)
{
    // Inizializza il pin del pulsante
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
}

SafetyManager::~SafetyManager() {
    // Null
}

void SafetyManager::begin() {
    // Inizializza lo stato
    _currentState = SAFE;
    _lastButtonState = digitalRead(STOP_BUTTON_PIN);
    _lastDebounceTime = millis();
}

void SafetyManager::update() {
    // Verifica tutte le condizioni di sicurezza
    bool isTilted = checkTilt();
    bool isLifted = checkLift();
    bool isStopPressed = checkStopButton();

    // Determina lo stato di sicurezza
    if (isEmergencyStop()) {
        setState(EMERGENCY_STOP);
    } else if (isStopPressed) {
        setState(STOP_BUTTON_PRESSED);
    } else if (isTilted) {
        setState(TILTED);
    } else if (isLifted) {
        setState(LIFTED);
    } else {
        setState(SAFE);
    }
}

bool SafetyManager::checkTilt() {
    if (_imu == nullptr) return false;
    
    // Ottieni l'angolo di inclinazione dall'IMU
    float pitch = _imu->getTheta();
    
    // Verifica se l'inclinazione supera la soglia
    return abs(pitch) > _tiltThreshold;
}

bool SafetyManager::checkLift() {
    if (_imu == nullptr) return false;
    
    // Ottieni l'angolo di roll dall'IMU
    float roll = _imu->getAngularVelocity();
    
    // Verifica se il roll supera la soglia
    return abs(roll) > _liftThreshold;
}

bool SafetyManager::checkStopButton() {
    // Leggi lo stato del pulsante
    int reading = digitalRead(STOP_BUTTON_PIN);
    
    // Verifica il debounce
    unsigned long currentTime = millis();
    if (currentTime - _lastDebounceTime > STOP_BUTTON_DEBOUNCE) {
        // Se il pin ha cambiato stato
        if (reading != _lastButtonState) {
            _lastDebounceTime = currentTime;
            _buttonState = reading;
        }
        
        // Se il pulsante è stato premuto
        if (_buttonState == LOW) {
            _stopButtonPressed = true;
        }
    }
    
    _lastButtonState = reading;
    return _stopButtonPressed;
}

void SafetyManager::setState(SafetyState newState) {
    if (_currentState != newState) {
        _currentState = newState;
        
        // Se c'è un'emergency stop, disabilita tutti i motori
        if (newState == EMERGENCY_STOP) {
            // Qui potresti aggiungere il codice per disabilitare i motori
        }
    }
}

void SafetyManager::emergencyStop() {
    setState(EMERGENCY_STOP);
}

void SafetyManager::resetEmergencyStop() {
    if (_currentState == EMERGENCY_STOP) {
        setState(SAFE);
    }
}
