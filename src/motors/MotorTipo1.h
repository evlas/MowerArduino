#ifndef MOTOR_TIPO1_H
#define MOTOR_TIPO1_H

#include "Arduino.h"
#include "Motor.h"
#include "../../config.h"
#include "../../pin_config.h"

/**
 * @class MotorTipo1
 * @brief Implementation of Motor interface for Type 1 brushless motors
 * 
 * This class provides specific implementation for controlling Type 1 brushless motors
 * with support for direction control, speed control via PWM, and optional encoder feedback.
 */
class MotorTipo1 : public Motor {
public:
    /**
     * @brief Construct a new MotorTipo1 object
     */
    MotorTipo1();
    
    /**
     * @brief Destroy the MotorTipo1 object
     */
    ~MotorTipo1();

    // Motor interface implementation
    bool initialize() override;
    
    /**
     * @copydoc Motor::setSpeed
     */
    void setSpeed(int speed) override;
    
    /**
     * @copydoc Motor::getSpeed
     */
    int getSpeed() const override;
    
    /**
     * @copydoc Motor::setDirection
     */
    void setDirection(bool forward) override;
    
    /**
     * @copydoc Motor::getDirection
     */
    bool getDirection() const override;
    
    /**
     * @copydoc Motor::getPosition
     */
    long getPosition() const override;
    
    /**
     * @copydoc Motor::resetPosition
     */
    void resetPosition() override;
    
    /**
     * @copydoc Motor::isRunning
     */
    bool isRunning() const override;
    
    /**
     * @copydoc Motor::isFault
     */
    bool isFault() const override;
    
    /**
     * @copydoc Motor::getVoltage
     */
    float getVoltage() const override;
    
    /**
     * @copydoc Motor::getTemperature
     */
    float getTemperature() const override;
    
    /**
     * @copydoc Motor::clearFault
     */
    void clearFault() override;

    /**
     * @brief Configure the motor control pins
     * 
     * @param directionPin Pin number for direction control
     * @param pwmPin Pin number for PWM speed control
     * @param encoderPin Pin number for encoder input (optional, -1 if not used)
     */
    void configurePins(int directionPin, int pwmPin, int encoderPin);

private:
    // Pin configuration
    int _directionPin;  ///< Pin number for direction control
    int _pwmPin;        ///< Pin number for PWM speed control
    int _encoderPin;    ///< Pin number for encoder input (-1 if not used)


    // Motor state
    int _currentSpeed;  ///< Current motor speed (0-255)
    bool _forward;      ///< Current direction (true = forward, false = reverse)
    long _position;     ///< Current position in encoder counts
    bool _isRunning;    ///< True if motor is currently running
    bool _isFault;      ///< True if a fault condition exists
    float _voltage;     ///< Last measured voltage
    float _temperature; ///< Last measured temperature

    // Encoder related members
    volatile long _encoderPosition;  ///< Current encoder position
    
    /**
     * @brief Static ISR handler for encoder interrupts
     */
    static void handleEncoderISR();
    
    /**
     * @brief Update encoder position based on pin states
     */
    void updateEncoder();
    
    static MotorTipo1* _instance;  ///< Static instance pointer for ISR handling
    volatile uint8_t _lastState;    ///< Last state of encoder pins for decoding
    
    // PWM configuration (commented out as it's not currently used)
    // static const uint32_t PWM_FREQUENCY = 20000;
    // static const uint8_t PWM_RESOLUTION = 8;
};

#endif // MOTOR_TIPO1_H
