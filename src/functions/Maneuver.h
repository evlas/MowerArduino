#ifndef MANEUVER_H
#define MANEUVER_H

#include "../motors/MotorController.h"

class Maneuver {
private:
    MotorController* motorController;
public:
    Maneuver(MotorController* motorController);
    // Add methods here
};

#endif
