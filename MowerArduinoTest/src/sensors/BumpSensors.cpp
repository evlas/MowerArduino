#include "BumpSensors.h"

BumpSensors::BumpSensors() {
    // Costruttore
}

BumpSensors::~BumpSensors() {
    // Cleanup pin configuration if needed
}

void BumpSensors::begin() {
    pinMode(FRONT_LEFT_BUMP_PIN, INPUT_PULLUP);
    pinMode(FRONT_RIGHT_BUMP_PIN, INPUT_PULLUP);
    pinMode(REAR_LEFT_BUMP_PIN, INPUT_PULLUP);
    pinMode(REAR_RIGHT_BUMP_PIN, INPUT_PULLUP);
}

bool BumpSensors::readBumpPin(int pin) {
    return digitalRead(pin) == LOW;  // Active low
}

bool BumpSensors::isLeftBump() {
    return readBumpPin(FRONT_LEFT_BUMP_PIN);
}

bool BumpSensors::isRightBump() {
    return readBumpPin(FRONT_RIGHT_BUMP_PIN);
}

bool BumpSensors::isCenterBump() {
    return isLeftBump() && isRightBump();
}

void BumpSensors::getAllBumpStatus(bool& left, bool& center, bool& right) {
    left = isLeftBump();
    center = isCenterBump();
    right = isRightBump();
}

int BumpSensors::getBumpDirection() {
    bool left = isLeftBump();
    bool right = isRightBump();
    
    if (!left && !right) return -1;  // No bump
    if (left && right) return 2;      // Center bump
    if (left) return 0;               // Left bump
    return 1;                         // Right bump
}
