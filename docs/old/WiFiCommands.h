#ifndef WIFI_COMMANDS_H
#define WIFI_COMMANDS_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "CommandHandler.h"
#include "../eeprom/EEPROMConfig.h"  // For PIDParams

// Forward declarations
class WiFiSerialBridge;

// Include battery monitor header if enabled
#ifdef ENABLE_BATTERY_MONITOR
#include "../battery/BatteryMonitor.h"
#endif

// Forward declaration for EEPROM settings
#ifdef ENABLE_EEPROM
#include "../eeprom/EEPROMManager.h"
#endif

// PIDParams is defined in EEPROMConfig.h

// Dichiarazione della funzione di gestione comandi
void handleWiFiCommand(const String& command, const JsonVariant& params);

// Inizializza il gestore dei comandi
void initWiFiCommands(WiFiSerialBridge& wifiBridge, CommandHandler& cmdHandler);

class WiFiCommands {
public:
    // Static initialization
    static void initWiFiCommands(WiFiSerialBridge& wifiBridge, CommandHandler& handler);
    
    // Command handlers
    static void handleWiFiCommand(const String& command, const JsonVariant& params);
    static String handleGetPID(const String& args);
    static String handleSetPID(const String& args);
    static void registerPIDCommands();

    // EEPROM related methods (if ENABLE_EEPROM is defined)
#ifdef ENABLE_EEPROM
    static void sendEepromSettings();
    static void handleSetEepromSettings(const JsonVariant& params);
    static void handleResetEepromToDefault();
#endif

private:
    // Static members
    static WiFiSerialBridge* wifiBridgeInstance;
    static CommandHandler* cmdHandler;
    
    // Helper methods
    static String formatPID(const PIDParams& p);
    static bool parsePIDParams(const String& args, PIDParams& params);
};

#endif // WIFI_COMMANDS_H
