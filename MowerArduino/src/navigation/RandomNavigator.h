// RandomNavigator.h - Simple random navigation helper
#ifndef RANDOM_NAVIGATOR_H
#define RANDOM_NAVIGATOR_H

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

#endif // RANDOM_NAVIGATOR_H
