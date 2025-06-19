#ifndef WIFI_COMMANDS_H
#define WIFI_COMMANDS_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "CommandHandler.h"

// Forward declaration
class WiFiSerialBridge;

// Include battery monitor header if enabled
#ifdef ENABLE_BATTERY_MONITOR
#include "../../src/sensors/BatteryMonitor.h"
#endif

// Dichiarazione della funzione di gestione comandi
void handleWiFiCommand(const String& command, const JsonVariant& params);

// Inizializza il gestore dei comandi
void initWiFiCommands(WiFiSerialBridge& wifiBridge, CommandHandler& cmdHandler);

#endif // WIFI_COMMANDS_H
