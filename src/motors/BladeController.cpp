#include "BladeController.h"
#include "MotorTipo1.h"
#include "MotorTipo2.h"
#include "Motor.h"

BladeController::BladeController() {
    _Blade = nullptr;
}

BladeController::~BladeController() {
    if (_Blade != nullptr) {
        delete _Blade;
    }
}

bool BladeController::begin() {
    return initializeBlade();
}

void BladeController::setBladeSpeed(int speed) {
    if (_Blade != nullptr) {
        _Blade->setSpeed(speed);
    }
}

void BladeController::setBladeDirection(bool forward) {
    if (_Blade != nullptr) {
        _Blade->setDirection(forward);
    }
}

bool BladeController::isBladeRunning() const {
    return _Blade != nullptr && _Blade->isRunning();
}

bool BladeController::isBladeFault() const {
    return _Blade != nullptr && _Blade->isFault();
}

float BladeController::getBladeVoltage() const {
    return _Blade != nullptr ? _Blade->getVoltage() : 0.0f;
}

float BladeController::getBladeTemperature() const {
    return _Blade != nullptr ? _Blade->getTemperature() : 0.0f;
}

void BladeController::clearBladeFault() {
    if (_Blade != nullptr) {
        _Blade->clearFault();
    }
}

bool BladeController::initializeBlade() {
    if (_Blade != nullptr) {
        delete _Blade;
    }
    _Blade = createBladeInstance();
    if (_Blade != nullptr) {
        // Per i motori della lama, l'encoder è opzionale e può essere NULL
        _Blade->configurePins(BLADE_MOTOR_DIR_PIN, BLADE_MOTOR_PWM_PIN, NULL);
        return true;
    }
    return false;
}

Motor* BladeController::createBladeInstance() {
    Motor* motor = nullptr;
    #ifdef BLADE_MOTOR_TIPO1
    motor = new MotorTipo1();
    #elif defined(BLADE_MOTOR_TIPO2)
    motor = new MotorTipo2();
    #endif
    return motor;
}