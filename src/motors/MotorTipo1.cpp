/**
 * @file MotorTipo1.cpp
 * @brief Implementation of the MotorTipo1 class
 */

#include "Arduino.h"
#include "MotorTipo1.h"

// Initialize static member
MotorTipo1* MotorTipo1::_instance = nullptr;

/**
 * @brief Construct a new MotorTipo1 object
 * 
 * Initializes all member variables to their default values and sets up the
 * instance pointer for ISR handling.
 */
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
    _instance = this; // Set the instance pointer for ISR handling
}

/**
 * @brief Destroy the MotorTipo1 object
 * 
 * Ensures the motor is stopped before destruction.
 */
MotorTipo1::~MotorTipo1() {
    setSpeed(0);
}

/**
 * @brief Initialize the motor hardware
 * 
 * Sets up the direction and PWM pins as outputs and configures the encoder
 * pin with interrupt if specified.
 * 
 * @return true if initialization was successful
 * @return false if initialization failed
 */
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

/**
 * @brief Configure the motor control pins
 * 
 * @param directionPin Pin number for direction control
 * @param pwmPin Pin number for PWM speed control
 * @param encoderPin Pin number for encoder input (-1 if not used)
 */
void MotorTipo1::configurePins(int directionPin, int pwmPin, int encoderPin) {
    _directionPin = directionPin;
    _pwmPin = pwmPin;
    _encoderPin = encoderPin;
    
    // Imposta l'istanza corrente per l'ISR
    _instance = this;
}

/**
 * @brief Static ISR handler for encoder interrupts
 * 
 * This static method forwards the interrupt to the instance method
 * to handle the encoder update.
 */
void MotorTipo1::handleEncoderISR() {
    if (_instance) {
        _instance->updateEncoder();
    }
}

/**
 * @brief Update encoder position based on pin state changes
 * 
 * This method is called from the ISR when an encoder pin state change is detected.
 * It updates the encoder position based on the current direction.
 */
void MotorTipo1::updateEncoder() {
    uint8_t newState = digitalRead(_encoderPin);
    
    // Aggiorna la posizione solo se c'è un cambiamento di stato
    if (newState != _lastState) {
        _encoderPosition += (_forward ? 1 : -1);
        _position = _encoderPosition;
        _lastState = newState;
    }
}

/**
 * @brief Set the motor speed
 * 
 * @param speed Desired speed (0-255 for PWM control)
 * The speed is constrained to the defined MIN_MOTOR_SPEED and MAX_MOTOR_SPEED values.
 */
void MotorTipo1::setSpeed(int speed) {
    _currentSpeed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    analogWrite(_pwmPin, abs(_currentSpeed));
}

/**
 * @brief Get the current motor speed
 * 
 * @return int Current speed (0-255)
 */
int MotorTipo1::getSpeed() const {
    return _currentSpeed;
}

/**
 * @brief Set the motor direction
 * 
 * @param forward True for forward direction, false for reverse
 */
void MotorTipo1::setDirection(bool forward) {
    _forward = forward;
    digitalWrite(_directionPin, _forward ? HIGH : LOW);
}

/**
 * @brief Get the current motor direction
 * 
 * @return true if motor is set to forward
 * @return false if motor is set to reverse
 */
bool MotorTipo1::getDirection() const {
    return _forward;
}

/**
 * @brief Get the current encoder position
 * 
 * @return long Current position in encoder counts
 */
long MotorTipo1::getPosition() const {
    return _position;
}

/**
 * @brief Reset the encoder position counter
 * 
 * Sets both the encoder position and internal position counter to zero.
 */
void MotorTipo1::resetPosition() {
    _encoderPosition = 0;
    _position = 0;
}

/**
 * @brief Check if the motor is currently running
 * 
 * @return true if the motor is running
 * @return false if the motor is stopped
 */
bool MotorTipo1::isRunning() const {
    return _isRunning;
}

/**
 * @brief Check if there is a fault condition
 * 
 * @return true if there is a fault
 * @return false if there is no fault
 */
bool MotorTipo1::isFault() const {
    return _isFault;
}

/**
 * @brief Get the last measured motor voltage
 * 
 * @return float Voltage in volts
 */
float MotorTipo1::getVoltage() const {
    return _voltage;
}

/**
 * @brief Get the last measured motor temperature
 * 
 * @return float Temperature in degrees Celsius
 */
float MotorTipo1::getTemperature() const {
    return _temperature;
}

/**
 * @brief Clear any fault conditions
 * 
 * Resets the fault flag to allow motor operation to continue.
 */
void MotorTipo1::clearFault() {
    _isFault = false;
}