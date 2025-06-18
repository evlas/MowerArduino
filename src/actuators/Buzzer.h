#ifndef BUZZER_H
#define BUZZER_H

#include "Arduino.h"
#include "../../pin_config.h"

class Buzzer {
  public:
    Buzzer();
    void begin();
    void beep(unsigned int frequency, unsigned long duration);
    void stop();
    void alarm();
    void success();
    void error();
  private:
    bool isBeeping;
};

extern Buzzer buzzer;

#endif // BUZZER_H
