// RandomNavigator.h - Simple random navigation helper
#pragma once

#include <Arduino.h>
#include "../functions/MowerTypes.h"

class Mower;               // Forward declaration
class RandomNavigator {
public:
    explicit RandomNavigator(Mower& mower);

    // (Re)start the random navigation state machine
    void begin();

    // Call every cycle while RANDOM navigation mode is active
    void update();

private:
    enum class State { DRIVING, REVERSING, TURNING };

    void driveForward();

    Mower& mower_;
    State state_;
};
