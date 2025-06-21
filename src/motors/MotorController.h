#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Arduino.h"
#include "Motor.h"
#include "../../config.h"
#include "../../pin_config.h"
#include <math.h>

class MotorController {
public:
    MotorController(float diameter = 6.0f, float base = 20.0f, float ticks = 1000.0f);
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
    
    // Parametri geometrici
    float getWheelBase() const { return _wheelBase; }
    void resetOdometry();
    
    // Gestione errori
    bool hasLeftMotorError() const;
    bool hasRightMotorError() const;
    void clearErrors();
    
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
    
    // Velocità dei motori
    int getLeftMotorSpeed() const;
    int getRightMotorSpeed() const;
    
    // Gestione errori
    void clearLeftMotorFault();
    void clearRightMotorFault();
    
    // Odometria
    void updateOdometry();
    float getX() const;
    float getY() const;
    float getTheta() const;
    float getLinearVelocity() const;
    float getAngularVelocity() const;

private:
    // Motori
    Motor* _leftMotor;
    Motor* _rightMotor;
    
    // Stato di errore
    bool _hasLeftMotorError;
    bool _hasRightMotorError;
    
    // Parametri di odometria
    float _wheelDiameter;      // Diametro delle ruote in metri
    float _wheelBase;          // Distanza tra le ruote in metri
    float _ticksPerRevolution; // Numero di tick per giro di ruota
    
    // Stato odometria
    float _x;                  // Posizione X in metri
    float _y;                  // Posizione Y in metri
    float _theta;              // Orientamento in radianti (-PI a PI)
    float _lastLeftPos;        // Ultima posizione sinistra in tick
    float _lastRightPos;       // Ultima posizione destra in tick
    unsigned long _lastUpdate; // Timestamp dell'ultimo aggiornamento
    float _distancePerTick;    // Distanza per tick in metri/tick
    float _linearVelocity;     // Velocità lineare in m/s
    float _angularVelocity;    // Velocità angolare in rad/s
    
    // Inizializzazione dei motori
    void initializeMotors();
    
    // Creazione dei motori in base alla configurazione
    Motor* createMotorInstance();
    
    // Inizializzazione dell'odometria
    void initializeOdometry(float diameter, float base, float ticks);
    
    // Metodi privati
    float calculateDistance(float ticks) const;
    float calculateTheta(float leftDistance, float rightDistance) const;
};

extern MotorController motorController;

#endif // MOTOR_CONTROLLER_H
