#ifndef BUZZER_H
#define BUZZER_H

#include "Arduino.h"
#include "../../pin_config.h"

/**
 * @class Buzzer
 * @brief Controls the buzzer for audio feedback
 * 
 * This class provides methods to control a piezo buzzer for various
 * audio feedback purposes like beeps, alarms, and status indicators.
 */
class Buzzer {
  public:
    /**
     * @brief Construct a new Buzzer object
     */
    Buzzer();
    
    /**
     * @brief Initialize the buzzer
     * 
     * Sets up the buzzer pin as an output.
     */
    void begin();
    
    /**
     * @brief Generate a beep sound
     * 
     * @param frequency Frequency of the beep in Hz
     * @param duration Duration of the beep in milliseconds
     */
    void beep(unsigned int frequency, unsigned long duration);
    
    /**
     * @brief Stop any ongoing sound
     */
    void stop();
    
    /**
     * @brief Play an alarm sound
     * 
     * Plays a repeating pattern to indicate an alarm condition.
     */
    void alarm();
    
    /**
     * @brief Play a success sound
     * 
     * Plays a short, high-pitched beep to indicate success.
     */
    void success();
    
    /**
     * @brief Play an error sound
     * 
     * Plays a low-pitched beep to indicate an error condition.
     */
    void error();
  private:
    bool isBeeping;
};

extern Buzzer buzzer;

#endif // BUZZER_H
