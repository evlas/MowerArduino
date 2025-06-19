#include "Arduino.h"
#include "MotorTipo1.h"

// Inizializzazione del membro statico
MotorTipo1* MotorTipo1::_instance = nullptr;

MotorTipo1::MotorTipo1() :
    _currentSpeed(0),
    _forward(true),
    _position(0),
    _isRunning(false),
    _isFault(false),
    _voltage(0.0),
    _temperature(0.0),
    _encoderPosition(0),
    _encoderPin(-1),
    _lastState(0)
{
    _instance = this; // Imposta l'istanza corrente per l'ISR
}

MotorTipo1::~MotorTipo1() {
    setSpeed(0);
}

bool MotorTipo1::initialize() {
    pinMode(_directionPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    
    if (_encoderPin >= 0) {
        pinMode(_encoderPin, INPUT_PULLUP);
        _lastState = digitalRead(_encoderPin);
        
        // Configura l'interrupt per il pin dell'encoder
        if (_encoderPin == MOTOR_LEFT_PPM_PIN || _encoderPin == MOTOR_RIGHT_PPM_PIN) {
            attachInterrupt(digitalPinToInterrupt(_encoderPin), handleEncoderISR, CHANGE);
        }
    }
    
    _encoderPosition = 0;
    _position = 0;
    _isRunning = true;
    
    return true;
}

void MotorTipo1::configurePins(int directionPin, int pwmPin, int encoderPin) {
    _directionPin = directionPin;
    _pwmPin = pwmPin;
    _encoderPin = encoderPin;
    
    // Imposta l'istanza corrente per l'ISR
    _instance = this;
}

void MotorTipo1::handleEncoderISR() {
    if (_instance) {
        _instance->updateEncoder();
    }
}

void MotorTipo1::updateEncoder() {
    uint8_t newState = digitalRead(_encoderPin);
    
    // Aggiorna la posizione solo se c'è un cambiamento di stato
    if (newState != _lastState) {
        _encoderPosition += (_forward ? 1 : -1);
        _position = _encoderPosition;
        _lastState = newState;
    }
}

void MotorTipo1::setSpeed(int speed) {
    _currentSpeed = constrain(speed, MIN_SPEED, MAX_SPEED);
    analogWrite(_pwmPin, abs(_currentSpeed));
}

int MotorTipo1::getSpeed() const {
    return _currentSpeed;
}

void MotorTipo1::setDirection(bool forward) {
    _forward = forward;
    digitalWrite(_directionPin, _forward ? HIGH : LOW);
}

bool MotorTipo1::getDirection() const {
    return _forward;
}

long MotorTipo1::getPosition() const {
    return _position;
}

void MotorTipo1::resetPosition() {
    _encoderPosition = 0;
    _position = 0;
}

bool MotorTipo1::isRunning() const {
    return _isRunning;
}

bool MotorTipo1::isFault() const {
    return _isFault;
}

float MotorTipo1::getVoltage() const {
    return _voltage;
}

float MotorTipo1::getTemperature() const {
    return _temperature;
}

void MotorTipo1::clearFault() {
    _isFault = false;
}