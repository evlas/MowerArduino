#ifndef RELAY_H
#define RELAY_H

#include "Arduino.h"
#include "../../pin_config.h"

class Relay {
  public:
    Relay();
    void begin();
    void on();
    void off();
    bool isOn();
    void toggle();
  private:
    bool state;
};

extern Relay relay;

#endif // RELAY_H
