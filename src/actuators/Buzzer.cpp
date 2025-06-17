#include "Buzzer.h"

Buzzer::Buzzer() {
    isBeeping = false;
}

void Buzzer::begin() {
    pinMode(BUZZER_PIN, OUTPUT);
}

void Buzzer::beep(unsigned int frequency, unsigned long duration) {
    isBeeping = true;
    tone(BUZZER_PIN, frequency, duration);
}

void Buzzer::stop() {
    noTone(BUZZER_PIN);
    isBeeping = false;
}

void Buzzer::alarm() {
    beep(880, 500); // A5 note
}

void Buzzer::success() {
    beep(1318, 200); // E6 note
}

void Buzzer::error() {
    beep(220, 500); // A2 note
}
