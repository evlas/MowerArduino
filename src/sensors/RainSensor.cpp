#ifdef ENABLE_RAIN_SENSOR
#include "RainSensor.h"

RainSensor::RainSensor() {
    // Initialize rain detection system
}

bool RainSensor::isRaining() {
    return analogRead(RAIN_SENSOR_PIN) < RAIN_THRESHOLD;
}
#endif // ENABLE_RAIN_SENSOR
