#include "Relay.h"

Relay::Relay() {
    state = false;
}

void Relay::begin() {
    pinMode(RELAY_MOTORS_PIN, OUTPUT);
    off();  // Start with relay off
}

void Relay::on() {
    digitalWrite(RELAY_MOTORS_PIN, HIGH);
    state = true;
}

void Relay::off() {
    digitalWrite(RELAY_MOTORS_PIN, LOW);
    state = false;
}

bool Relay::isOn() {
    return state;
}

void Relay::toggle() {
    if (state) {
        off();
    } else {
        on();
    }
}
