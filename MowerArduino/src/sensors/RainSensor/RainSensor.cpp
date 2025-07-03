#include "RainSensor.h"
#include <Arduino.h>
#include "../../config.h"
#include "../../pin_config.h"

RainSensor rainSensor;

RainSensor::RainSensor() {
    // Constructor - initialization moved to begin()
}

void RainSensor::begin() {
    // Initialize the rain sensor pin
    pinMode(RAIN_SENSOR_PIN, INPUT);
}

bool RainSensor::isRaining() {
//    return analogRead(RAIN_SENSOR_PIN) < RAIN_THRESHOLD;
    return false;
}
