#ifndef SETUP_H
#define SETUP_H

#include <Arduino.h>
#include "../../config.h"
#include "../functions/StateMachine.h"

// Dichiarazione della macchina a stati globale
extern StateMachine mowerStateMachine;

// Variabili globali
extern unsigned long lastSensorUpdate;
extern unsigned long lastNavigationUpdate;
extern unsigned long lastSafetyCheck;

void my_setup();

#endif
