/**
 * @file Buzzer.cpp
 * @brief Implementation of the Buzzer class
 */

#include "Buzzer.h"

/**
 * @brief Construct a new Buzzer object
 * 
 * Initializes the buzzer state to not beeping.
 */
Buzzer::Buzzer() {
    isBeeping = false;
}

/**
 * @brief Initialize the buzzer hardware
 * 
 * Sets up the buzzer pin as an output and initializes it to off.
 */
void Buzzer::begin() {
    pinMode(BUZZER_PIN, OUTPUT);
}

/**
 * @brief Generate a beep sound with specified frequency and duration
 * 
 * @param frequency The frequency of the beep in Hz (31-65535)
 * @param duration The duration of the beep in milliseconds
 */
void Buzzer::beep(unsigned int frequency, unsigned long duration) {
    isBeeping = true;
    tone(BUZZER_PIN, frequency, duration);
}

/**
 * @brief Stop any currently playing sound
 * 
 * Turns off the buzzer and updates the beeping state.
 */
void Buzzer::stop() {
    noTone(BUZZER_PIN);
    isBeeping = false;
}

/**
 * @brief Play a standard alarm sound
 * 
 * Plays an A5 note (880Hz) for 500ms to indicate an alarm condition.
 */
void Buzzer::alarm() {
    beep(880, 500); // A5 note
}

/**
 * @brief Play a success sound
 * 
 * Plays a short E6 note (1318Hz) for 200ms to indicate success.
 */
void Buzzer::success() {
    beep(1318, 200); // E6 note
}

/**
 * @brief Play an error sound
 * 
 * Plays a low A2 note (220Hz) for 500ms to indicate an error condition.
 */
void Buzzer::error() {
    beep(220, 500); // A2 note
}
