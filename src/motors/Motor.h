#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor {
public:
    Motor() {};
    virtual ~Motor() = default;

    // Inizializzazione
    virtual bool initialize() = 0;
    
    // Configurazione pin
    virtual void configurePins(int directionPin, int pwmPin, int encoderPin) = 0;
    
    // Controllo velocità
    virtual void setSpeed(int speed) = 0;
    virtual int getSpeed() const = 0;
    
    // Controllo direzione
    virtual void setDirection(bool forward) = 0;
    virtual bool getDirection() const = 0;
    
    // Gestione posizione
    virtual long getPosition() const = 0;
    virtual void resetPosition() = 0;
    
    // Stato del motore
    virtual bool isRunning() const = 0;
    virtual bool isFault() const = 0;
    
    // Controllo tensione
    virtual float getVoltage() const = 0;
    
    // Controllo temperatura
    virtual float getTemperature() const = 0;
    
    // Gestione errori
    virtual void clearFault() = 0;

protected:
    int _speed;
    bool _forward;
    long _position;
    bool _isRunning;
    bool _isFault;
    float _voltage;
    float _temperature;
};

#endif // MOTOR_H
