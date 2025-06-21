#ifndef EEPROM_MANAGER_H
#define EEPROM_MANAGER_H

#include <Arduino.h>
#include <EEPROM.h>
#include <stddef.h>  // Per size_t e offsetof

// Includi il file di configurazione EEPROM che contiene le definizioni delle strutture
#include "EEPROMConfig.h"

// Sezioni di configurazione
enum ConfigSection {
    CONFIG_SYSTEM,      // Impostazioni di sistema
    CONFIG_MOWER,       // Impostazioni taglio
    CONFIG_NAVIGATION,  // Impostazioni navigazione
    CONFIG_MAINTENANCE, // Manutenzione
    CONFIG_PID,         // Parametri PID (carica solo leftPID, rightPID viene gestito separatamente)
    CONFIG_HOME_POS     // Posizione home
};

class EEPROMManager {
public:
    // Inizializza la EEPROM
    static void begin();
    
    // Carica le impostazioni dalla EEPROM
    static bool loadSettings(EEPROMSettings& settings);
    
    // Salva le impostazioni nella EEPROM
    static void saveSettings(EEPROMSettings& settings);
    
    // Carica le impostazioni di default
    static void loadDefaultSettings(EEPROMSettings& settings);
    
    // Carica una sezione di configurazione
    static bool loadConfigSection(ConfigSection section, void* data, size_t size);
    
    // Salva una sezione di configurazione
    static bool saveConfigSection(ConfigSection section, const void* data, size_t size);
    
    // Verifica se la EEPROM è vuota
    static bool isEEmpty();
    
    // Cancella il contenuto della EEPROM
    static void clearEEPROM();
    
    // Ottieni l'offset di una sezione
    static size_t getSectionOffset(ConfigSection section);
    
    // Funzione per il calcolo del checksum
    static uint8_t calculateChecksum(const void* data, size_t len);
    
    // Funzioni per aggiornare e validare i checksum per ogni tipo di struttura
    
    // SystemSettings
    static void updateChecksum(SystemSettings& data);
    static bool validateChecksum(const SystemSettings& data);
    
    // MowerSettings
    static void updateChecksum(MowerSettings& data);
    static bool validateChecksum(const MowerSettings& data);
    
    // NavigationSettings
    static void updateChecksum(NavigationSettings& data);
    static bool validateChecksum(const NavigationSettings& data);
    
    // MaintenanceSettings
    static void updateChecksum(MaintenanceSettings& data);
    static bool validateChecksum(const MaintenanceSettings& data);
    
    // PIDParams
    static void updateChecksum(PIDParams& data);
    static bool validateChecksum(const PIDParams& data);
    
    // HomePosition (struttura speciale)
    static void updateChecksum(HomePosition& data);
    static bool validateChecksum(const HomePosition& data);
    
    // Funzioni di gestione delle impostazioni
    static bool loadSystemSettings(SystemSettings& settings);
    static bool saveSystemSettings(const SystemSettings& settings);
    
    static bool loadMowerSettings(MowerSettings& settings);
    static bool saveMowerSettings(const MowerSettings& settings);
    
    static bool loadNavigationSettings(NavigationSettings& settings);
    static bool saveNavigationSettings(const NavigationSettings& settings);
    
    static bool loadMaintenanceSettings(MaintenanceSettings& settings);
    static bool saveMaintenanceSettings(const MaintenanceSettings& settings);
    
    static bool loadPIDParams(PIDParams& left, PIDParams& right);
    static void savePIDParams(const PIDParams& left, const PIDParams& right);
    
    static bool loadHomePosition(HomePosition& homePos);
    static void saveHomePosition(const HomePosition& homePos);
    static bool isHomePositionSet();
    static void clearHomePosition();
    
    // Funzioni per la migrazione delle impostazioni
    static bool migrateSettings(EEPROMSettings& settings, uint8_t fromVersion);
    
    // Funzioni helper private per la validazione delle posizioni
    static bool validateHomePosition(const HomePosition& homePos);
    static void updateHomePositionChecksum(HomePosition& homePos);
    
private:
    // Costruttore privato per impedire l'istanziazione
    EEPROMManager() {}
    
    // Previene la copia e l'assegnamento
    EEPROMManager(const EEPROMManager&);
    EEPROMManager& operator=(const EEPROMManager&);
};

// Le specializzazioni dei template sono dichiarate all'interno della classe

#endif // EEPROM_MANAGER_H
