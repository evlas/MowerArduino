#include "BatterySensor.h"
#include "config.h"
#include <INA226_WE.h>

BatterySensor batterySensor;

bool BatterySensor::begin() {
    connected = ina226.init();
    if (!connected) {
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println(F("BatterySensor: INA226 init failed!"));
        #endif
        return false;
    }

    // Configurazione INA226
    ina226.setResistorRange(0.10, 10.0);  // 2mΩ shunt, 10A massimi
    ina226.setMeasureMode(CONTINUOUS);      // Modalità continua
    ina226.setAverage(AVERAGE_16);          // Media su 16 campioni
    ina226.setConversionTime(CONV_TIME_1100);  // Tempo di conversione 1.1ms
    ina226.setCorrectionFactor(1.0);        // Fattore di correzione

    #ifdef DEBUG_MODE
    SERIAL_DEBUG.println(F("BatterySensor: INA226 initialized"));
    #endif
    
    return true;
}

float BatterySensor::readVoltage() {
    return connected ? ina226.getBusVoltage_V() : 0.0f;
}

float BatterySensor::readCurrent() {
    return connected ? ina226.getCurrent_mA()/1000.0 : 0.0f;
}

float BatterySensor::readPower() {
    return connected ? ina226.getBusVoltage_V() * ina226.getCurrent_mA() / 1000.0f : 0.0f;
}
