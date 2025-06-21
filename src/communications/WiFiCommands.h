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

// Forward declaration for EEPROM settings
#ifdef ENABLE_EEPROM
struct EEPROMSettings;  // Defined in EEPROMConfig.h
#endif

// Dichiarazione della funzione di gestione comandi
void handleWiFiCommand(const String& command, const JsonVariant& params);

// Inizializza il gestore dei comandi
void initWiFiCommands(WiFiSerialBridge& wifiBridge, CommandHandler& cmdHandler);

// Funzioni per la gestione EEPROM via WiFi
#ifdef ENABLE_EEPROM
void sendEepromSettings();
void handleSetEepromSettings(const JsonVariant& params);
void handleResetEepromToDefault();
#endif

#endif // WIFI_COMMANDS_H
