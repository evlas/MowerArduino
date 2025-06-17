#ifndef ULTRASONIC_SENSORS_H
#define ULTRASONIC_SENSORS_H

#include "Arduino.h"
#include "../../pin_config.h"

class UltrasonicSensors {
public:
    UltrasonicSensors();
    ~UltrasonicSensors();
    
    // Initialize sensors
    void begin();
    
    // Get distance measurements
    float getFrontLeftDistance();
    float getFrontCenterDistance();
    float getFrontRightDistance();
    
    // Get all distances in one call
    void getAllDistances(float& left, float& center, float& right);
    
    // Check if obstacle is detected
    bool isObstacleDetected(float minDistance = 30.0f);  // Default 30cm
    
    // Get the direction with the most space
    int getDirectionWithMostSpace();  // Returns: 0=left, 1=center, 2=right
    
private:
    // Helper methods
    float measureDistance(int trigPin, int echoPin);
    
    // Constants
    static const float MAX_DISTANCE = 400.0f;  // Maximum measurable distance in cm
};

#endif // ULTRASONIC_SENSORS_H
