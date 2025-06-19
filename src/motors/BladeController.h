#ifndef BLADE_CONTROLLER_H
#define BLADE_CONTROLLER_H

#include "Arduino.h"
#include "Motor.h"
#include "../../config.h"
#include "../../pin_config.h"

/**
 * @class BladeController
 * @brief Controls the cutting blade motor of the robotic mower
 * 
 * This class provides an interface to control the cutting blade motor,
 * including speed and direction control, as well as monitoring functions.
 */
class BladeController {
public:
    /**
     * @brief Construct a new BladeController object
     */
    BladeController();
    
    /**
     * @brief Destroy the BladeController object
     */
    ~BladeController();
    
    /**
     * @brief Initialize the blade controller
     * 
     * @return true if initialization was successful
     * @return false if initialization failed
     */
    bool begin();
    
    /**
     * @brief Set the blade motor speed
     * 
     * @param speed Desired speed (0-255 for PWM control)
     */
    void setBladeSpeed(int speed);
    
    /**
     * @brief Set the blade rotation direction
     * 
     * @param forward True for forward rotation, false for reverse
     */
    void setBladeDirection(bool forward);

    /**
     * @brief Check if the blade motor is currently running
     * 
     * @return true if the blade is running
     * @return false if the blade is stopped
     */
    bool isBladeRunning() const;
    
    /**
     * @brief Check if there is a fault with the blade motor
     * 
     * @return true if there is a fault
     * @return false if there is no fault
     */
    bool isBladeFault() const;
    
    /**
     * @brief Get the current voltage of the blade motor
     * 
     * @return float The voltage in volts
     */
    float getBladeVoltage() const;
    
    /**
     * @brief Get the temperature of the blade motor
     * 
     * @return float The temperature in degrees Celsius
     */
    float getBladeTemperature() const;
    
    /**
     * @brief Clear any fault conditions on the blade motor
     */
    void clearBladeFault();

private:
    Motor* _Blade;  ///< Pointer to the blade motor instance
    
    /**
     * @brief Initialize the blade motor
     * 
     * @return true if initialization was successful
     * @return false if initialization failed
     */
    bool initializeBlade();
    
    /**
     * @brief Create an instance of the appropriate motor type for the blade
     * 
     * @return Motor* Pointer to the created motor instance
     */
    Motor* createBladeInstance();
};

extern BladeController bladeController;

#endif // BLADE_CONTROLLER_H
