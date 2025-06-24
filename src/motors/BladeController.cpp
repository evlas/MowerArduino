/**
 * @file BladeController.cpp
 * @brief Implementation of the BladeController class
 */

#include "BladeController.h"
#include "MotorTipo1.h"
#include "MotorTipo2.h"
#include "Motor.h"

/// Global instance of the BladeController
BladeController bladeController;

/**
 * @brief Construct a new BladeController::BladeController object
 * 
 * Initializes the blade motor pointer to nullptr.
 */
BladeController::BladeController() {
    _Blade = nullptr;
}

/**
 * @brief Destroy the BladeController::BladeController object
 * 
 * Cleans up the blade motor instance if it was created.
 */
BladeController::~BladeController() {
    if (_Blade != nullptr) {
        delete _Blade;
    }
}

/**
 * @brief Initialize the blade controller
 * 
 * @return true if initialization was successful
 * @return false if initialization failed
 */
bool BladeController::begin() {
    return initializeBlade();
}

/**
 * @brief Set the speed of the blade motor
 * 
 * @param speed The desired speed (0-255 for PWM control)
 */
void BladeController::setBladeSpeed(int speed) {
    if (_Blade != nullptr) {
        _Blade->setSpeed(speed);
    }
}

/**
 * @brief Set the direction of the blade motor
 * 
 * @param forward True for forward rotation, false for reverse
 */
void BladeController::setBladeDirection(bool forward) {
    if (_Blade != nullptr) {
        _Blade->setDirection(forward);
    }
}

/**
 * @brief Check if the blade motor is currently running
 * 
 * @return true if the blade motor is running
 * @return false if the blade motor is stopped or not initialized
 */
bool BladeController::isBladeRunning() const {
    return _Blade != nullptr && _Blade->isRunning();
}

/**
 * @brief Check if there is a fault with the blade motor
 * 
 * @return true if there is a fault condition
 * @return false if there is no fault or the motor is not initialized
 */
bool BladeController::isBladeFault() const {
    return _Blade != nullptr && _Blade->isFault();
}

/**
 * @brief Get the current voltage of the blade motor
 * 
 * @return float The voltage in volts, or 0.0 if not initialized
 */
float BladeController::getBladeVoltage() const {
    return _Blade != nullptr ? _Blade->getVoltage() : 0.0f;
}

/**
 * @brief Get the temperature of the blade motor
 * 
 * @return float The temperature in degrees Celsius, or 0.0 if not initialized
 */
float BladeController::getBladeTemperature() const {
    return _Blade != nullptr ? _Blade->getTemperature() : 0.0f;
}

/**
 * @brief Clear any fault conditions on the blade motor
 * 
 * This will attempt to clear any fault conditions that have been detected.
 * The motor should be stopped before calling this method.
 */
void BladeController::clearBladeFault() {
    if (_Blade != nullptr) {
        _Blade->clearFault();
    }
}

/**
 * @brief Initialize the blade motor
 * 
 * Creates and configures the appropriate motor instance based on the configuration.
 * 
 * @return true if initialization was successful
 * @return false if initialization failed
 */
bool BladeController::initializeBlade() {
    if (_Blade != nullptr) {
        delete _Blade;
    }
    _Blade = createBladeInstance();
    if (_Blade != nullptr) {
        // Per i motori della lama, l'encoder è opzionale e può essere NULL
        _Blade->configurePins(BLADE_MOTOR_DIR_PIN, BLADE_MOTOR1_PWM_PIN, NULL);
        return true;
    }
    return false;
}

/**
 * @brief Create an instance of the appropriate motor type for the blade
 * 
 * Uses compile-time configuration to determine which motor type to instantiate.
 * 
 * @return Motor* Pointer to the created motor instance, or nullptr if creation failed
 */
Motor* BladeController::createBladeInstance() {
    Motor* motor = nullptr;
    #ifdef BLADE_MOTOR_TIPO1
    motor = new MotorTipo1();
    #elif defined(BLADE_MOTOR_TIPO2)
    motor = new MotorTipo2();
    #endif
    return motor;
}