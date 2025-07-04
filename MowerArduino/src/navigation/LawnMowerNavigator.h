// LawnMowerNavigator.h - Parallel stripe (boustrophedon) navigation
#ifndef LAWN_MOWER_NAVIGATOR_H
#define LAWN_MOWER_NAVIGATOR_H

#include <Arduino.h>
#include "../functions/MowerTypes.h"
#include "NavigatorBase.h"

// Forward declarations
class Mower;

class LawnMowerNavigator : public NavigatorBase {
public:
    explicit LawnMowerNavigator(Mower& mower);

    // NavigatorBase interface implementation
    void init(Mower& mower) override;
    void update(Mower& mower) override;
    bool handleEvent(Mower& mower, Event event) override;
    const char* getName() const override { return "LAWN_MOWER"; }
    void start(Mower& mower) override;
    void stop(Mower& mower) override;

private:
    enum class State { STRAIGHT, TURN1, OFFSET, TURN2, CORNER_TURN };

    void driveForward();

    Mower& mower_;
    State state_;
};

#endif // LAWN_MOWER_NAVIGATOR_H
