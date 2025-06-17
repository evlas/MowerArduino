#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Arduino.h"
#include "Motor.h"
#include "../../config.h"
#include "../../pin_config.h"

class MotorController {
public:
    MotorController();
    ~MotorController();
    
    // Inizializzazione
    bool begin();
    
    // Controllo motori
    void setLeftMotorSpeed(int speed);
    void setRightMotorSpeed(int speed);
    void setLeftMotorDirection(bool forward);
    void setRightMotorDirection(bool forward);
    
    // Reset posizione
    void resetPositionLeft();
    void resetPositionRight();
    
    // Stato dei motori
    bool isLeftMotorRunning() const;
    bool isRightMotorRunning() const;
    bool isLeftMotorFault() const;
    bool isRightMotorFault() const;
    
    // Controllo tensione
    float getLeftMotorVoltage() const;
    float getRightMotorVoltage() const;
    
    // Controllo temperatura
    float getLeftMotorTemperature() const;
    float getRightMotorTemperature() const;
    
    // Gestione errori
    void clearLeftMotorFault();
    void clearRightMotorFault();

private:
    // Motori
    Motor* _leftMotor;
    Motor* _rightMotor;
    
    // Inizializzazione dei motori
    void initializeMotors();
    
    // Creazione dei motori in base alla configurazione
    Motor* createMotorInstance();
};

#endif // MOTOR_CONTROLLER_H
