#include "Arduino.h"
#include "MotorTipo1.h"

MotorTipo1::MotorTipo1() :
    _currentSpeed(0),
    _forward(true),
    _position(0),
    _isRunning(false),
    _isFault(false),
    _voltage(0.0),
    _temperature(0.0),
    _encoderPosition(0)
{
}

MotorTipo1::~MotorTipo1() {
    setSpeed(0);
}

bool MotorTipo1::initialize() {
    pinMode(_directionPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    pinMode(_encoderPin, INPUT_PULLUP);
    
//    analogWriteResolution(PWM_RESOLUTION);
//    analogWriteFrequency(_pwmPin, PWM_FREQUENCY);
    
    _encoderPosition = 0;
    _position = 0;
    
    _isRunning = true;
    
    return true;
}

void MotorTipo1::configurePins(int directionPin, int pwmPin, int encoderPin) {
    _directionPin = directionPin;
    _pwmPin = pwmPin;
    _encoderPin = encoderPin;
}

void MotorTipo1::updateEncoderPosition() {
    static bool lastState = digitalRead(_encoderPin);
    bool currentState = digitalRead(_encoderPin);
    
    if (currentState != lastState) {
        _encoderPosition += (_forward ? 1 : -1);
        _position = _encoderPosition;
    }
    lastState = currentState;
}

void MotorTipo1::setSpeed(int speed) {
    _currentSpeed = constrain(speed, -255, 255);
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