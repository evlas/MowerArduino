#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include "../sensors/BatterySensor.h"
#include "../../config.h"

// Costanti di default se non definite in config.h
#ifndef BATTERY_CELLS
#define BATTERY_CELLS 6
#endif

#ifndef BATTERY_VOLTAGE_MAX
#define BATTERY_VOLTAGE_MAX (4.2f * BATTERY_CELLS)  // 25.2V per 6S
#endif

#ifndef BATTERY_VOLTAGE_MIN
#define BATTERY_VOLTAGE_MIN (3.0f * BATTERY_CELLS)   // 18.0V per 6S
#endif

#ifndef BATTERY_VOLTAGE_CRITICAL
#define BATTERY_VOLTAGE_CRITICAL (3.3f * BATTERY_CELLS) // 19.8V per 6S
#endif

#ifndef BATTERY_LOW_LEVEL
#define BATTERY_LOW_LEVEL 20.0f  // Soglia di batteria bassa (20%)
#endif

class BatteryMonitor {
public:
    // Inizializzazione
    void begin();
    
    // Aggiornamento letture
    void update();
    
    // Letture
    float getVoltage() const { return voltage; }          // Volt
    float getCurrent() const { return current; }          // Ampere (positivo = scarica, negativo = carica)
    float getPower() const { return power; }              // Watt
    float getCapacity() const { return capacity; }        // Ampere-ora
    float getSOC() const { return soc; }                  // Percentuale (0-100)
    bool isCharging() const { return current < -0.1f; }   // Soglia per rilevamento carica
    bool isCritical() const { return voltage < BATTERY_VOLTAGE_CRITICAL; }
    bool isLow() const { return soc < BATTERY_LOW_LEVEL; }  // Soglia di batteria bassa
    bool isFullyCharged() const { return voltage >= BATTERY_VOLTAGE_MAX * 0.99f && current > -0.1f; }  // 99% della tensione massima e corrente di carica quasi nulla
    
    // Get battery status based on current flow
    enum BatteryStatus {
        BATTERY_DISCHARGING,  // Current > 0: Battery is powering the motors
        BATTERY_CHARGING,     // Current < 0: Battery is being charged
        BATTERY_STANDBY       // Current ≈ 0: System in standby
    };
    
    BatteryStatus getBatteryStatus() const { 
        if (current > 0.1f) return BATTERY_DISCHARGING;
        if (current < -0.1f) return BATTERY_CHARGING;
        return BATTERY_STANDBY;
    }
    
    // Stampa stato su seriale (debug)
    void printStatus() const;

private:
    float voltage = 0.0f;     // Volt
    float current = 0.0f;     // Ampere
    float power = 0.0f;       // Watt
    float capacity = 0.0f;    // Ampere-ora
    float soc = 100.0f;       // Percentuale (0-100)
    unsigned long lastUpdate = 0;
    
    // Stima della capacità residua
    void updateCapacity();
};

// Istanza globale
extern BatteryMonitor batteryMonitor;

#endif // BATTERY_MONITOR_H
