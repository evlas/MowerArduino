// BorderNavigator.h - Border following navigation
#ifndef BORDER_NAVIGATOR_H
#define BORDER_NAVIGATOR_H

#include <Arduino.h>
#include "../functions/MowerTypes.h"
#include "NavigatorBase.h"

// Forward declarations
class Mower;

/**
 * @class BorderNavigator
 * @brief Implementa la navigazione lungo il perimetro
 */
class BorderNavigator : public NavigatorBase {
public:
    /**
     * @brief Construct a new Border Navigator object
     */
    BorderNavigator();
    void init(Mower& mower) override;
    void update(Mower& mower) override;
    bool handleEvent(Mower& mower, Event event) override;
    const char* getName() const override { return "BORDER"; }
    void start(Mower& mower) override;
    void stop(Mower& mower) override;
    
private:
    enum class State {
        FIND_BORDER,
        FOLLOW_BORDER,
        TURN_AROUND
    };
    
    State state_ = State::FIND_BORDER;
    unsigned long lastStateChange_ = 0;
};

#endif // BORDER_NAVIGATOR_H
