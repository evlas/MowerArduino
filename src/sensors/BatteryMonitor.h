#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Wire.h>
#include <INA226_WE.h>

// Costanti per la configurazione dell'INA226
#define BATTERY_MONITOR_ADDRESS 0x40  // Indirizzo I2C predefinito
#define BATTERY_UPDATE_INTERVAL 1000  // Aggiornamento ogni 1 secondo

// Istanza globale
extern INA226_WE batteryMonitor;

#endif // BATTERY_MONITOR_H
