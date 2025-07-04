// RandomNavigator.h - Simple random navigation helper
#ifndef RANDOM_NAVIGATOR_H
#define RANDOM_NAVIGATOR_H

#include <Arduino.h>
#include "../functions/MowerTypes.h"
#include "NavigatorBase.h"

// Forward declarations
class Mower;
class RandomNavigator : public NavigatorBase {
public:
    explicit RandomNavigator(Mower& mower);

    // NavigatorBase interface implementation
    void init(Mower& mower) override;
    void update(Mower& mower) override;
    bool handleEvent(Mower& mower, Event event) override;
    const char* getName() const override { return "RANDOM"; }
    void start(Mower& mower) override;
    void stop(Mower& mower) override;

private:
    enum class State { DRIVING, REVERSING, TURNING };

    void driveForward();

    Mower& mower_;
    State state_;
};

#endif // RANDOM_NAVIGATOR_H
