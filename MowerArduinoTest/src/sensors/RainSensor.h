#ifndef RAIN_SENSOR_H
#define RAIN_SENSOR_H

class RainSensor {
public:
    RainSensor();
    bool isRaining();
};

extern RainSensor rainSensor;

#endif
