#ifndef LOOP_H
#define LOOP_H

#include <Arduino.h>
#include "../../config.h"

// Include GPS
#ifdef ENABLE_GPS
#include "../../src/sensors/GPS.h"
extern GPSModule gps;
#endif

void my_loop();

#endif
