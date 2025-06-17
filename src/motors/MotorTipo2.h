#ifndef MOTOR_TIPO2_H
#define MOTOR_TIPO2_H

#include "Motor.h"
#include "Arduino.h"
#include "../../config.h"

class MotorTipo2 : public Motor {
public:
    MotorTipo2();
    ~MotorTipo2();

    // Implementazione specifica per i motori brushed
    bool initialize() override;
    void setSpeed(int speed) override;
    int getSpeed() const override;
    void setDirection(bool forward) override;
    bool getDirection() const override;
    long getPosition() const override;
    void resetPosition() override;
    bool isRunning() const override;
    bool isFault() const override;
    float getVoltage() const override;
    float getTemperature() const override;
    void clearFault() override;

    // Configurazione specifica
    void configurePins(int pwmPin, int directionPin);

private:
    // Pin configuration
    int _pwmPin;
    int _directionPin;
};

#endif // MOTOR_TIPO2_H
