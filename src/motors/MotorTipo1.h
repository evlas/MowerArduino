#ifndef MOTOR_TIPO1_H
#define MOTOR_TIPO1_H

#include "Arduino.h"
#include "Motor.h"
#include "../../config.h"

class MotorTipo1 : public Motor {
public:
    MotorTipo1();
    ~MotorTipo1();

    // Implementazione specifica per i motori brushless
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
    void configurePins(int directionPin, int pwmPin, int encoderPin);

private:
    // Pin configuration
    int _directionPin;
    int _pwmPin;
    int _encoderPin;

    // Stato del motore
    int _currentSpeed;
    bool _forward;
    long _position;
    bool _isRunning;
    bool _isFault;
    float _voltage;
    float _temperature;

    // Encoder
    volatile long _encoderPosition;
    void updateEncoderPosition();

    // PWM frequency
//    static const uint32_t PWM_FREQUENCY = 20000;
//    static const uint8_t PWM_RESOLUTION = 8;
};

#endif // MOTOR_TIPO1_H
