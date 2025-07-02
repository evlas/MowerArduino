// LawnMowerNavigator.h - Parallel stripe (boustrophedon) navigation
#pragma once

#include <Arduino.h>
#include "../functions/MowerTypes.h"

class Mower;

class LawnMowerNavigator {
public:
    explicit LawnMowerNavigator(Mower& mower);

    void begin();
    void update();

private:
    enum class State { STRAIGHT, TURN1, OFFSET, TURN2, CORNER_TURN };

    void driveForward();

    Mower& mower_;
    State state_;
};
