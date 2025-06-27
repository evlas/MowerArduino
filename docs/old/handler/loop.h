#ifndef LOOP_H
#define LOOP_H

#include <Arduino.h>
#include "../../config.h"
#include "../functions/StateMachine.h"
#include "../functions/Maneuver.h"

// Dichiarazione della macchina a stati globale
extern StateMachine mowerStateMachine;

// Dichiarazione dell'istanza di Maneuver
extern Maneuver mowerManeuver;

// Variabili per il controllo dei tempi
extern unsigned long lastBatteryUpdate;
extern unsigned long lastSensorUpdate;
extern unsigned long lastTelemetryUpdate;

// Include GPS
#ifdef ENABLE_GPS
#include "../../src/sensors/GPS.h"
extern GPSModule gps;
#endif

void loopMower();

#endif
