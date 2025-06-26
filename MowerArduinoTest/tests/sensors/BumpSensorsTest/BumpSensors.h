#ifndef BUMP_SENSORS_H
#define BUMP_SENSORS_H

#include "Arduino.h"
#include "pin_config.h"

class BumpSensors {
public:
    BumpSensors();
    ~BumpSensors();

    void begin();
    
   // Get bump status
    bool isLeftBump();
    bool isRightBump();
    bool isCenterBump();
    
    // Get all bump status in one call
    void getAllBumpStatus(bool& left, bool& center, bool& right);
    
    // Get bump direction
    int getBumpDirection();  // Returns: 0=left, 1=right, 2=center, -1=no bump
    
private:
    // Helper methods
    bool readBumpPin(int pin);
};

extern BumpSensors bumpSensors;

#endif // BUMP_SENSORS_H
