#include "SafetyManager.h"
#include "../../config.h"
#include "../../pin_config.h"

SafetyManager::SafetyManager() :
    _currentState(SAFE),
    _stopButtonDebounceTime(0),
    _lastStopButtonState(false),
    _stopButtonState(false),
    _stopButtonPressed(false),
    _liftDebounceTime(0),
    _lastLiftState(false),
    _liftState(false),
    _liftPressed(false),
    _imu(nullptr),
    _tiltThreshold(TILT_THRESHOLD)  // Soglia di inclinazione massima da config.h
{
    // Inizializza i pin
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LIFT_PIN, INPUT_PULLUP);
}

SafetyManager::~SafetyManager() {
    // Null
}

void SafetyManager::begin() {
    // Inizializza lo stato
    _currentState = SAFE;
    _lastStopButtonState = digitalRead(STOP_BUTTON_PIN);
    _lastLiftState = digitalRead(LIFT_PIN);
    _stopButtonDebounceTime = millis();
    _liftDebounceTime = millis();
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
    int reading = digitalRead(LIFT_PIN);
    unsigned long currentTime = millis();
    
    if (currentTime - _liftDebounceTime > BUTTON_DEBOUNCE) {
        if (reading != _lastLiftState) {
            _liftDebounceTime = currentTime;
            _liftState = reading;
        }
        
        // Il sensore è attivo basso (LOW = sollevato)
        _liftPressed = (_liftState == LOW);
    }
    
    _lastLiftState = reading;
    return _liftPressed;
}

bool SafetyManager::checkStopButton() {
    int reading = digitalRead(STOP_BUTTON_PIN);
    unsigned long currentTime = millis();
    
    if (currentTime - _stopButtonDebounceTime > BUTTON_DEBOUNCE) {
        if (reading != _lastStopButtonState) {
            _stopButtonDebounceTime = currentTime;
            _stopButtonState = reading;
        }
        
        _stopButtonPressed = (_stopButtonState == LOW);
    }
    
    _lastStopButtonState = reading;
    return _stopButtonPressed;
}

void SafetyManager::setState(SafetyState newState) {
    if (_currentState != newState) {
        _currentState = newState;
        // Qui puoi aggiungere logica aggiuntiva quando cambia lo stato
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

SafetyManager safety;
