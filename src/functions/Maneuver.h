#ifndef MANEUVER_H
#define MANEUVER_H

#include "Arduino.h"
#include "../motors/MotorController.h"

class Maneuver {
  public:
    Maneuver(MotorController* motorController);
    ~Maneuver();
    
    // Basic movements

    void forward(int speed = 100);
    void backward(int speed = 100);
    void turnLeft(int speed = 100);
    void turnRight(int speed = 100);
    void stop();
    
    // Complex maneuvers
    void rotate(int degrees, int speed = 100);
    void moveStraight(int distance, int speed = 100);
    void zigzag(int distance, int width, int speed = 100);
    void spiral(int radius, int speed = 100);
    
    // Status
    bool isMoving();
    bool isTurning();
    
    // Speed control
    void setSpeed(int speed);
    void setSpeed(int leftSpeed, int rightSpeed);
  
  private:
    MotorController* motorController;
    bool moving;
    bool turning;
    
    void setDirection(bool leftForward, bool rightForward);
};

#endif // MANEUVER_H
