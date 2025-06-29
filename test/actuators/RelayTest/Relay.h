#ifndef RELAY_H
#define RELAY_H

#include "Arduino.h"
#include "pin_config.h"

/**
 * @class Relay
 * @brief Controls a relay module
 * 
 * This class provides methods to control a relay, which can be used
 * to switch higher voltage/current devices on and off.
 */
class Relay {
  public:
    /**
     * @brief Construct a new Relay object
     */
    Relay();
    
    /**
     * @brief Initialize the relay
     * 
     * Sets up the relay pin as an output and turns it off.
     */
    void begin(int pin);
    
    /**
     * @brief Turn the relay on
     */
    void on();
    
    /**
     * @brief Turn the relay off
     */
    void off();
    
    /**
     * @brief Check if the relay is currently on
     * 
     * @return true if the relay is on
     * @return false if the relay is off
     */
    bool isOn();
    
    /**
     * @brief Toggle the relay state
     * 
     * If the relay is on, turns it off, and vice versa.
     */
    void toggle();
  private:
    bool state;
    int pin_;
};

extern Relay relay;

#endif // RELAY_H
