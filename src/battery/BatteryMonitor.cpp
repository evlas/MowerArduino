#include "BatteryMonitor.h"
#include "../../config.h"

// Configurazione batteria LiPo 6S
#ifndef BATTERY_CAPACITY
#define BATTERY_CAPACITY 5.0f  // Ampere-ora (modificare in config.h se diverso)
#endif

// Istanza globale
BatteryMonitor batteryMonitor;

void BatteryMonitor::begin() {
    // Inizializza il sensore
    batterySensor.begin();
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.println(F("BatteryMonitor: Initialized"));
    #endif
}

void BatteryMonitor::update() {
    static unsigned long lastCapacityUpdate = 0;
    unsigned long currentMillis = millis();
    
    // Leggi i valori dal sensore
    voltage = batterySensor.readVoltage();
    current = batterySensor.readCurrent();
    power = batterySensor.readPower();
    
    // Stima SOC (State of Charge)
    soc = constrain(
        (voltage - BATTERY_VOLTAGE_MIN) / 
        (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) * 100.0f,
        0.0f, 100.0f
    );
    
    // Aggiorna capacità residua ogni secondo
    if (currentMillis - lastCapacityUpdate >= 1000) {
        updateCapacity();
        lastCapacityUpdate = currentMillis;
    }
    
    lastUpdate = currentMillis;
}

void BatteryMonitor::updateCapacity() {
    static float remainingCapacity = BATTERY_CAPACITY;  // Capacità residua in Ah
    static unsigned long lastTime = millis();
    
    unsigned long currentTime = millis();
    float deltaHours = (currentTime - lastTime) / 3600000.0f;  // ms -> ore
    
    // Integrazione della corrente nel tempo per stimare la capacità
    remainingCapacity -= current * deltaHours;
    remainingCapacity = constrain(remainingCapacity, 0, BATTERY_CAPACITY);
    
    // Reset capacità se la batteria è carica
    if (voltage >= BATTERY_VOLTAGE_MAX * 0.99f && current < -0.1f) {
        remainingCapacity = BATTERY_CAPACITY;
    }
    
    capacity = remainingCapacity;
    lastTime = currentTime;
}

void BatteryMonitor::printStatus() const {
    const char* statusText;
    switch(getBatteryStatus()) {
        case BATTERY_DISCHARGING: statusText = "DISCHARGING"; break;
        case BATTERY_CHARGING: statusText = "CHARGING"; break;
        case BATTERY_STANDBY: statusText = "STANDBY"; break;
        default: statusText = "UNKNOWN";
    }
    
    SERIAL_DEBUG.print(F("Battery: "));
    SERIAL_DEBUG.print(voltage, 2);
    SERIAL_DEBUG.print(F("V, "));
    SERIAL_DEBUG.print(current, 2);
    SERIAL_DEBUG.print(F("A, "));
    SERIAL_DEBUG.print(power, 1);
    SERIAL_DEBUG.print(F("W, "));
    SERIAL_DEBUG.print(soc, 1);
    SERIAL_DEBUG.print(F("%, "));
    SERIAL_DEBUG.print(capacity, 2);
    SERIAL_DEBUG.print(F("Ah, Status: "));
    SERIAL_DEBUG.println(statusText);
}
