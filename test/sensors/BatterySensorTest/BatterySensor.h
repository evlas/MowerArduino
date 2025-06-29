#ifndef BATTERY_SENSOR_H
#define BATTERY_SENSOR_H

#include <Wire.h>
#include <INA226_WE.h>

class BatterySensor {
public:
    // Inizializzazione
    bool begin();
    
    // Letture dirette dal sensore
    float readVoltage();    // Volt
    float readCurrent();    // Ampere (positivo = scarica, negativo = carica)
    float readPower();      // Watt
    
    // Stato del sensore
    bool isConnected() const { return connected; }

private:
    INA226_WE ina226;
    bool connected = false;
};

// Istanza globale
extern BatterySensor batterySensor;

#endif // BATTERY_SENSOR_H
