#ifndef PERIMETER_SENSORS_H
#define PERIMETER_SENSORS_H

#include "Arduino.h"
#include "../../pin_config.h"

class PerimeterSensors {
public:
    PerimeterSensors();
    ~PerimeterSensors();
    
    // Inizializzazione
    void begin();
    
    // Aggiornamento stato
    void update();
    
    // Verifica se il perimetro è stato rilevato
    bool isDetected() const;
    
    // Getters per il segnale
    int getSignalStrength() const;
    
private:
    int _signalStrength;
};

extern PerimeterSensors perimeterSensors;

#endif // PERIMETER_SENSORS_H
