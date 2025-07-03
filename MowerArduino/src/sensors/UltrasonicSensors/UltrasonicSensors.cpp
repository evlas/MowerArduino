#include "UltrasonicSensors.h"

UltrasonicSensors::UltrasonicSensors() {
    // Constructor
}

UltrasonicSensors::~UltrasonicSensors() {
    // Destructor
}

void UltrasonicSensors::begin() {
    // Initialize pins using the correct pin definitions from pin_config.h
    pinMode(ULTRASONIC_FRONT_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_FRONT_ECHO_PIN, INPUT);
    pinMode(ULTRASONIC_LEFT_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_LEFT_ECHO_PIN, INPUT);
    pinMode(ULTRASONIC_RIGHT_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_RIGHT_ECHO_PIN, INPUT);
}

float UltrasonicSensors::measureDistance(int trigPin, int echoPin) {
    // Clear the trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Set the trigger pin HIGH for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Read the echo pin
    long duration = pulseIn(echoPin, HIGH, 30000);  // Timeout after 30ms
    
    // Calculate the distance (in cm)
    float distance = duration * 0.034 / 2;  // Speed of sound is 340m/s
    
    // If the distance is out of range, return MAX_DISTANCE
    if (distance <= 0 || distance > MAX_ULTRASONIC_DISTANCE) {
        return MAX_ULTRASONIC_DISTANCE;
    }
    
    return distance;
}

float UltrasonicSensors::getFrontLeftDistance() {
    return measureDistance(ULTRASONIC_LEFT_TRIG_PIN, ULTRASONIC_LEFT_ECHO_PIN);
}

float UltrasonicSensors::getFrontCenterDistance() {
    return measureDistance(ULTRASONIC_FRONT_TRIG_PIN, ULTRASONIC_FRONT_ECHO_PIN);
}

float UltrasonicSensors::getFrontRightDistance() {
    return measureDistance(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN);
}

void UltrasonicSensors::getAllDistances(float& left, float& center, float& right) {
    left = getFrontLeftDistance();
    center = getFrontCenterDistance();
    right = getFrontRightDistance();
}

bool UltrasonicSensors::isObstacleDetected(float minDistance) {
    float left = getFrontLeftDistance();
    float center = getFrontCenterDistance();
    float right = getFrontRightDistance();
    
    return (left < minDistance || center < minDistance || right < minDistance);
}

int UltrasonicSensors::getDirectionWithMostSpace() {
    float left = getFrontLeftDistance();
    float center = getFrontCenterDistance();
    float right = getFrontRightDistance();
    
    // Return the direction with the maximum distance
    if (left >= center && left >= right) {
        return 0;  // Left
    } else if (center >= right) {
        return 1;  // Center
    } else {
        return 2;  // Right
    }
}
