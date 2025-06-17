#ifndef SETUP_H
#define SETUP_H

#include <Arduino.h>
#include "../../config.h"

// Variabili globali
extern unsigned long lastSensorUpdate;
extern unsigned long lastNavigationUpdate;
extern unsigned long lastSafetyCheck;

// Enum per stati del robot
enum MowerState {
  IDLE,
  MOWING,
  RETURNING_HOME,
  CHARGING,
  ERROR,
  MANUAL_CONTROL
};

extern MowerState currentState;

void my_setup();

#endif
