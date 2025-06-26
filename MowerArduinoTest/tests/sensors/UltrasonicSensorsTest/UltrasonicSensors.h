#ifndef ULTRASONIC_SENSORS_H
#define ULTRASONIC_SENSORS_H

#include "Arduino.h"
#include "pin_config.h"
#include "config.h"

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
    bool isObstacleDetected(float minDistance = OBSTACLE_CLEARANCE);  // Default distance from config.h
    
    // Get the direction with the most space
    int getDirectionWithMostSpace();  // Returns: 0=left, 1=center, 2=right
    
private:
    // Helper methods
    float measureDistance(int trigPin, int echoPin);
    
    // Utilizza MAX_ULTRASONIC_DISTANCE definita in config.h
};

extern UltrasonicSensors ultrasonicSensors;

#endif // ULTRASONIC_SENSORS_H
