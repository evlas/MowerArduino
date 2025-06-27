#ifndef RAIN_SENSOR_H
#define RAIN_SENSOR_H

class RainSensor {
public:
    RainSensor();
    void begin();
    bool isRaining();
};

// Create a single instance of RainSensor that can be used throughout the program
extern RainSensor rainSensor;

#endif
