/**
 * @file Relay.cpp
 * @brief Implementation of the Relay class
 */

#include "Relay.h"

/**
 * @brief Construct a new Relay object
 * 
 * Initializes the relay state to off.
 */
Relay::Relay() {
    state = false;
}

/**
 * @brief Initialize the relay hardware
 * 
 * Sets up the relay pin as an output and ensures it starts in the off state.
 */
void Relay::begin() {
    pinMode(RELAY_MOTORS_PIN, OUTPUT);
    off();  // Start with relay off
}

/**
 * @brief Turn the relay on
 * 
 * Activates the relay by setting its control pin HIGH.
 */
void Relay::on() {
    digitalWrite(RELAY_MOTORS_PIN, HIGH);
    state = true;
}

/**
 * @brief Turn the relay off
 * 
 * Deactivates the relay by setting its control pin LOW.
 */
void Relay::off() {
    digitalWrite(RELAY_MOTORS_PIN, LOW);
    state = false;
}

/**
 * @brief Check the current state of the relay
 * 
 * @return true if the relay is on
 * @return false if the relay is off
 */
bool Relay::isOn() {
    return state;
}

/**
 * @brief Toggle the relay state
 * 
 * If the relay is on, turns it off, and if it's off, turns it on.
 * Updates the internal state accordingly.
 */
void Relay::toggle() {
    if (state) {
        off();
    } else {
        on();
    }
}
