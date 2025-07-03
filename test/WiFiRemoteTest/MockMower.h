// Minimal stub of Mower class for WiFiRemote unit-test on Arduino IDE
#pragma once
#include <Arduino.h>
#include "src/functions/MowerTypes.h"

class MockMower {
public:
    /* --- getters used by WiFiRemote --- */
    float getBatteryPercentage() const { return 80.0f; } // dummy value
    float getLatitude() const { return 45.123456f; }
    float getLongitude() const { return 9.123456f; }
    uint8_t getSatellites() const { return 10; }

    // returns number of distances filled
    int getUltrasonicDistances(float *buf, int n) const {
        if (n>=3){ buf[0]=150; buf[1]=160; buf[2]=155; }
        return 3;
    }

    bool isObstacleDetected() const { return false; }
    bool isRaining() const { return false; }

    /* --- actuators used by WiFiRemote --- */
    void startMowing() { Serial.println(F("Stub: startMowing")); }
    void stopMotors()  { Serial.println(F("Stub: stopMotors")); }
    void stopBlades()  { Serial.println(F("Stub: stopBlades")); }
    void stopDriveMotors(){ Serial.println(F("Stub: stopDriveMotors")); }

    void setLeftMotorSpeed(float s){ Serial.print(F("Stub: L=")); Serial.println(s); }
    void setRightMotorSpeed(float s){ Serial.print(F("Stub: R=")); Serial.println(s); }

    void emergencyStop(){ Serial.println(F("Stub: emergencyStop")); }
};
