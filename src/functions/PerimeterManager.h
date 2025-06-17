#ifndef PERIMETER_MANAGER_H
#define PERIMETER_MANAGER_H

#include "../motors/MotorController.h"
#include "../actuators/Buzzer.h"
#include "../LCD/LCDManager.h"

class PerimeterManager {
private:
    MotorController* motorController;
    Buzzer* buzzer;
    LCDManager* lcdManager;
public:
    PerimeterManager(MotorController* motorController, Buzzer* buzzer, LCDManager* lcdManager);
    // Add methods here
};

#endif
