#ifndef BLADE_CONTROLLER_H
#define BLADE_CONTROLLER_H

#include "Arduino.h"
#include "Motor.h"
#include "../../config.h"
#include "../../pin_config.h"

class BladeController {
public:
    BladeController();
    ~BladeController();
    
    // Inizializzazione
    bool begin();
    
    // Controllo motori
    void setBladeSpeed(int speed);
    void setBladeDirection(bool forward);

    // Stato dei motori
    bool isBladeRunning() const;
    bool isBladeFault() const;
    
    // Controllo tensione
    float getBladeVoltage() const;
    
    // Controllo temperatura
    float getBladeTemperature() const;
    
    // Gestione errori
    void clearBladeFault();

private:
    // Motori
    Motor* _Blade;
    
    // Inizializzazione dei motori
    bool initializeBlade();
    
    // Creazione dei motori in base alla configurazione
    Motor* createBladeInstance();
};

#endif // BLADE_CONTROLLER_H
