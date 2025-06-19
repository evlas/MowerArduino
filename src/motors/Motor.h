#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

/**
 * @class Motor
 * @brief Abstract base class for motor control
 * 
 * This class defines the interface for all motor types in the system.
 * Concrete implementations should inherit from this class and implement
 * all pure virtual methods to provide specific motor control functionality.
 */
class Motor {
public:
    /**
     * @brief Default constructor
     */
    Motor() {};
    
    /**
     * @brief Virtual destructor
     */
    virtual ~Motor() = default;

    /**
     * @brief Initialize the motor
     * 
     * @return true if initialization was successful
     * @return false if initialization failed
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief Configure the motor control pins
     * 
     * @param directionPin Pin number for direction control
     * @param pwmPin Pin number for PWM speed control
     * @param encoderPin Pin number for encoder input (optional, can be -1 if not used)
     */
    virtual void configurePins(int directionPin, int pwmPin, int encoderPin) = 0;
    
    /**
     * @brief Set the motor speed
     * 
     * @param speed Desired speed (0-255 for PWM control)
     */
    virtual void setSpeed(int speed) = 0;
    
    /**
     * @brief Get the current motor speed
     * 
     * @return int Current speed (0-255)
     */
    virtual int getSpeed() const = 0;
    
    /**
     * @brief Set the motor direction
     * 
     * @param forward True for forward, false for reverse
     */
    virtual void setDirection(bool forward) = 0;
    
    /**
     * @brief Get the current motor direction
     * 
     * @return true if motor is set to forward
     * @return false if motor is set to reverse
     */
    virtual bool getDirection() const = 0;
    
    /**
     * @brief Get the current motor position
     * 
     * @return long Current position in encoder counts
     */
    virtual long getPosition() const = 0;
    
    /**
     * @brief Reset the motor position counter
     */
    virtual void resetPosition() = 0;
    
    /**
     * @brief Check if the motor is currently running
     * 
     * @return true if the motor is running
     * @return false if the motor is stopped
     */
    virtual bool isRunning() const = 0;
    
    /**
     * @brief Check if there is a fault condition
     * 
     * @return true if there is a fault
     * @return false if there is no fault
     */
    virtual bool isFault() const = 0;
    
    /**
     * @brief Get the current motor voltage
     * 
     * @return float Voltage in volts
     */
    virtual float getVoltage() const = 0;
    
    /**
     * @brief Get the current motor temperature
     * 
     * @return float Temperature in degrees Celsius
     */
    virtual float getTemperature() const = 0;
    
    /**
     * @brief Clear any fault conditions
     * 
     * This should be called after a fault condition has been resolved
     * to allow the motor to operate again.
     */
    virtual void clearFault() = 0;

protected:
    int _speed;         ///< Current motor speed (0-255)
    bool _forward;      ///< Current direction (true = forward, false = reverse)
    long _position;     ///< Current position in encoder counts
    bool _isRunning;    ///< True if motor is currently running
    bool _isFault;      ///< True if a fault condition exists
    float _voltage;     ///< Last measured voltage
    float _temperature; ///< Last measured temperature
};

#endif // MOTOR_H
