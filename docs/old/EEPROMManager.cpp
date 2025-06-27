#include "EEPROMManager.h"
#include <EEPROM.h>
#include <string.h>

// Includi EEPROMConfig.h per le definizioni delle strutture
#include "EEPROMConfig.h"

// =============================================
// Implementazione dei metodi pubblici
// =============================================

void EEPROMManager::begin() {
    // Niente di specifico da fare per AVR
}

bool EEPROMManager::loadSettings(EEPROMSettings& settings) {
    // Leggi la struttura completa
    EEPROM.get(EEPROMSettings::EEPROM_BASE, settings);
    
    // Verifica la versione
    if (settings.version > EEPROMSettings::CURRENT_VERSION) {
        return false;  // Versione non supportata
    }
    
    // Se la versione è più vecchia, esegui la migrazione
    if (settings.version < EEPROMSettings::CURRENT_VERSION) {
        if (!migrateSettings(settings, settings.version)) {
            return false;  // Migrazione fallita
        }
    }
    
    // Verifica il checksum totale
    uint8_t savedChecksum = settings.checksum;
    settings.checksum = 0;  // Azzera il checksum per il calcolo
    uint8_t calcChecksum = calculateChecksum(&settings, sizeof(settings));
    settings.checksum = savedChecksum;  // Ripristina il checksum
    
    if (calcChecksum != savedChecksum) {
        return false;  // Checksum non valido
    }
    
    // Verifica i checksum delle singole sezioni
    if (!validateChecksum(settings.system) ||
        !validateChecksum(settings.mower) ||
        !validateChecksum(settings.navigation) ||
        !validateChecksum(settings.maintenance) ||
        !validateChecksum(settings.leftPID) ||
        !validateChecksum(settings.rightPID) ||
        !validateHomePosition(settings.homePosition)) {
        return false;
    }
    
    return true;
}

void EEPROMManager::saveSettings(EEPROMSettings& settings) {
    // Imposta la versione corrente
    settings.version = EEPROMSettings::CURRENT_VERSION;
    
    // Calcola i checksum delle singole sezioni
    updateChecksum(settings.system);
    updateChecksum(settings.mower);
    updateChecksum(settings.navigation);
    updateChecksum(settings.maintenance);
    updateChecksum(settings.leftPID);
    updateChecksum(settings.rightPID);
    updateHomePositionChecksum(settings.homePosition);
    
    // Calcola il checksum totale
    settings.checksum = 0;
    settings.checksum = calculateChecksum(&settings, sizeof(settings));
    
    // Salva in EEPROM
    EEPROM.put(EEPROMSettings::EEPROM_BASE, settings);
    
    #ifdef ESP32
        EEPROM.commit();  // Per ESP32 è necessario fare il commit esplicito
    #endif
}

void EEPROMManager::loadDefaultSettings(EEPROMSettings& settings) {
    // Azzera la struttura
    memset(&settings, 0, sizeof(settings));
    
    // Imposta la versione
    settings.version = EEPROMSettings::CURRENT_VERSION;
    
    // System Settings
    settings.system.language = 0; // Inglese
    settings.system.use24HourFormat = true;
    settings.system.useMetric = true;
    settings.system.brightness = 70;
    settings.system.volume = 80;
    updateChecksum(settings.system);
    
    // Mower Settings
    settings.mower.cuttingHeight = 5;
    settings.mower.bladeSpeed = 80;
    settings.mower.mowingPattern = 0; // Random
    settings.mower.rainDelay = false;
    settings.mower.rainDelayUntil = 0;
    updateChecksum(settings.mower);
    
    // Navigation Settings
    settings.navigation.perimeterMode = 0; // Wire
    settings.navigation.obstacleSensitivity = 3; // Media
    settings.navigation.gpsEnabled = true;
    settings.navigation.gpsUpdateInterval = 1.0f;
    updateChecksum(settings.navigation);
    
    // Maintenance Settings
    settings.maintenance.totalMowingTime = 0;
    settings.maintenance.bladeHours = 0;
    settings.maintenance.lastMaintenance = 0;
    settings.maintenance.nextMaintenance = 0;
    updateChecksum(settings.maintenance);
    
    // PID Settings
    settings.leftPID.kp = 2.0f;
    settings.leftPID.ki = 0.5f;
    settings.leftPID.kd = 0.1f;
    updateChecksum(settings.leftPID);
    
    settings.rightPID.kp = 2.0f;
    settings.rightPID.ki = 0.5f;
    settings.rightPID.kd = 0.1f;
    updateChecksum(settings.rightPID);
    
    // Home Position
    settings.homePosition.x = 0.0f;
    settings.homePosition.y = 0.0f;
    settings.homePosition.theta = 0.0f;
    settings.homePosition.isSet = false;
    updateHomePositionChecksum(settings.homePosition);
    
    // Calcola checksum totale
    settings.checksum = 0;
    settings.checksum = calculateChecksum(&settings, sizeof(settings));
}

bool EEPROMManager::loadConfigSection(ConfigSection section, void* data, size_t size) {
    if (!data || size == 0) return false;
    
    size_t offset = getSectionOffset(section);
    if (offset >= EEPROMSettings::EEPROM_SIZE) return false;
    
    // Leggi i dati dalla EEPROM
    uint8_t* dest = static_cast<uint8_t*>(data);
    for (size_t i = 0; i < size; i++) {
        dest[i] = EEPROM.read(EEPROMSettings::EEPROM_BASE + offset + i);
    }
    
    return true;
}

bool EEPROMManager::saveConfigSection(ConfigSection section, const void* data, size_t size) {
    if (!data || size == 0) return false;
    
    size_t offset = getSectionOffset(section);
    if (offset + size > EEPROMSettings::EEPROM_SIZE) return false;
    
    // Scrivi i dati nella EEPROM
    const uint8_t* src = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; i++) {
        EEPROM.write(EEPROMSettings::EEPROM_BASE + offset + i, src[i]);
    }
    
    #ifdef ESP32
        EEPROM.commit();  // Per ESP32 è necessario fare il commit esplicito
    #endif
    
    return true;
}

// Metodi per le impostazioni di sistema
bool EEPROMManager::loadSystemSettings(SystemSettings& settings) {
    return loadConfigSection(CONFIG_SYSTEM, &settings, sizeof(settings)) && 
           validateChecksum(settings);
}

bool EEPROMManager::saveSystemSettings(const SystemSettings& settings) {
    SystemSettings toSave = settings;
    updateChecksum(toSave);
    return saveConfigSection(CONFIG_SYSTEM, &toSave, sizeof(toSave));
}

// Metodi per le impostazioni del tagliaerba
bool EEPROMManager::loadMowerSettings(MowerSettings& settings) {
    return loadConfigSection(CONFIG_MOWER, &settings, sizeof(settings)) && 
           validateChecksum(settings);
}

bool EEPROMManager::saveMowerSettings(const MowerSettings& settings) {
    MowerSettings toSave = settings;
    updateChecksum(toSave);
    return saveConfigSection(CONFIG_MOWER, &toSave, sizeof(toSave));
}

// Metodi per le impostazioni di navigazione
bool EEPROMManager::loadNavigationSettings(NavigationSettings& settings) {
    return loadConfigSection(CONFIG_NAVIGATION, &settings, sizeof(settings)) && 
           validateChecksum(settings);
}

bool EEPROMManager::saveNavigationSettings(const NavigationSettings& settings) {
    NavigationSettings toSave = settings;
    updateChecksum(toSave);
    return saveConfigSection(CONFIG_NAVIGATION, &toSave, sizeof(toSave));
}

// Metodi per le impostazioni di manutenzione
bool EEPROMManager::loadMaintenanceSettings(MaintenanceSettings& settings) {
    return loadConfigSection(CONFIG_MAINTENANCE, &settings, sizeof(settings)) && 
           validateChecksum(settings);
}

bool EEPROMManager::saveMaintenanceSettings(const MaintenanceSettings& settings) {
    MaintenanceSettings toSave = settings;
    updateChecksum(toSave);
    return saveConfigSection(CONFIG_MAINTENANCE, &toSave, sizeof(toSave));
}

// Metodi per i parametri PID
bool EEPROMManager::loadPIDParams(PIDParams& left, PIDParams& right) {
    EEPROMSettings settings;
    if (loadSettings(settings)) {
        if (validateChecksum(settings.leftPID) && validateChecksum(settings.rightPID)) {
            left = settings.leftPID;
            right = settings.rightPID;
            return true;
        }
    }
    return false;
}

void EEPROMManager::savePIDParams(const PIDParams& left, const PIDParams& right) {
    EEPROMSettings settings;
    if (loadSettings(settings) || isEEmpty()) {
        settings.leftPID = left;
        settings.rightPID = right;
        updateChecksum(settings.leftPID);
        updateChecksum(settings.rightPID);
        saveSettings(settings);
    }
}

// Metodi per la posizione home
bool EEPROMManager::loadHomePosition(HomePosition& homePos) {
    EEPROMSettings settings;
    if (loadSettings(settings)) {
        if (settings.homePosition.isSet && validateHomePosition(settings.homePosition)) {
            homePos = settings.homePosition;
            return true;
        }
    }
    return false;
}

void EEPROMManager::saveHomePosition(const HomePosition& homePos) {
    EEPROMSettings settings;
    if (loadSettings(settings) || isEEmpty()) {
        settings.homePosition = homePos;
        updateChecksum(settings.homePosition);
        saveSettings(settings);
    }
}

bool EEPROMManager::isHomePositionSet() {
    EEPROMSettings settings;
    if (loadSettings(settings)) {
        return settings.homePosition.isSet && 
               validateHomePosition(settings.homePosition);
    }
    return false;
}

void EEPROMManager::clearHomePosition() {
    EEPROMSettings settings;
    if (loadSettings(settings)) {
        settings.homePosition.isSet = false;
        updateChecksum(settings.homePosition);
        saveSettings(settings);
    }
}

bool EEPROMManager::isEEmpty() {
    for (uint16_t i = 0; i < EEPROMSettings::EEPROM_SIZE; i++) {
        if (EEPROM.read(EEPROMSettings::EEPROM_BASE + i) != 0xFF) {
            return false;
        }
    }
    return true;
}

void EEPROMManager::clearEEPROM() {
    for (uint16_t i = 0; i < EEPROMSettings::EEPROM_SIZE; i++) {
        EEPROM.write(EEPROMSettings::EEPROM_BASE + i, 0xFF);
    }
    
    #ifdef ESP32
        EEPROM.commit();  // Per ESP32 è necessario fare il commit esplicito
    #endif
}

// =============================================
// Implementazione dei metodi privati
// =============================================

size_t EEPROMManager::getSectionOffset(ConfigSection section) {
    size_t offset = offsetof(EEPROMSettings, version) + sizeof(uint8_t); // Salta il byte di versione
    
    switch(section) {
        case CONFIG_SYSTEM: 
            return offsetof(EEPROMSettings, system);
        case CONFIG_MOWER: 
            return offsetof(EEPROMSettings, mower);
        case CONFIG_NAVIGATION: 
            return offsetof(EEPROMSettings, navigation);
        case CONFIG_MAINTENANCE: 
            return offsetof(EEPROMSettings, maintenance);
        case CONFIG_PID: 
            return offsetof(EEPROMSettings, leftPID);
        case CONFIG_HOME_POS: 
            return offsetof(EEPROMSettings, homePosition);
        default: 
            return 0;
    }
}

uint8_t EEPROMManager::calculateChecksum(const void* data, size_t size) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint8_t sum = 0;
    for (size_t i = 0; i < size; i++) {
        sum ^= bytes[i];  // XOR per il checksum
    }
    return sum;
}

// Implementazione delle funzioni di aggiornamento checksum per ogni tipo

void EEPROMManager::updateChecksum(SystemSettings& data) {
    data.checksum = 0;
    data.checksum = calculateChecksum(&data, sizeof(SystemSettings));
}

void EEPROMManager::updateChecksum(MowerSettings& data) {
    data.checksum = 0;
    data.checksum = calculateChecksum(&data, sizeof(MowerSettings));
}

void EEPROMManager::updateChecksum(NavigationSettings& data) {
    data.checksum = 0;
    data.checksum = calculateChecksum(&data, sizeof(NavigationSettings));
}

void EEPROMManager::updateChecksum(MaintenanceSettings& data) {
    data.checksum = 0;
    data.checksum = calculateChecksum(&data, sizeof(MaintenanceSettings));
}

void EEPROMManager::updateChecksum(PIDParams& data) {
    data.checksum = 0;
    data.checksum = calculateChecksum(&data, sizeof(PIDParams));
}

void EEPROMManager::updateChecksum(HomePosition& data) {
    if (!data.isSet) return;  // Non aggiornare se non è impostata
    data.checksum = 0;
    data.checksum = calculateChecksum(&data, sizeof(HomePosition));
}

// Implementazione delle funzioni di validazione checksum per ogni tipo

bool EEPROMManager::validateChecksum(const SystemSettings& data) {
    SystemSettings temp = data;
    uint8_t savedChecksum = temp.checksum;
    temp.checksum = 0;
    return (savedChecksum == calculateChecksum(&temp, sizeof(SystemSettings)));
}

bool EEPROMManager::validateChecksum(const MowerSettings& data) {
    MowerSettings temp = data;
    uint8_t savedChecksum = temp.checksum;
    temp.checksum = 0;
    return (savedChecksum == calculateChecksum(&temp, sizeof(MowerSettings)));
}

bool EEPROMManager::validateChecksum(const NavigationSettings& data) {
    NavigationSettings temp = data;
    uint8_t savedChecksum = temp.checksum;
    temp.checksum = 0;
    return (savedChecksum == calculateChecksum(&temp, sizeof(NavigationSettings)));
}

bool EEPROMManager::validateChecksum(const MaintenanceSettings& data) {
    MaintenanceSettings temp = data;
    uint8_t savedChecksum = temp.checksum;
    temp.checksum = 0;
    return (savedChecksum == calculateChecksum(&temp, sizeof(MaintenanceSettings)));
}

bool EEPROMManager::validateChecksum(const PIDParams& data) {
    PIDParams temp = data;
    uint8_t savedChecksum = temp.checksum;
    temp.checksum = 0;
    return (savedChecksum == calculateChecksum(&temp, sizeof(PIDParams)));
}

bool EEPROMManager::validateChecksum(const HomePosition& data) {
    if (!data.isSet) return true;  // Se non è impostata, è comunque valida
    HomePosition temp = data;
    uint8_t savedChecksum = temp.checksum;
    temp.checksum = 0;
    return (savedChecksum == calculateChecksum(&temp, sizeof(HomePosition)));
}

bool EEPROMManager::migrateSettings(EEPROMSettings& settings, uint8_t fromVersion) {
    bool migrated = false;
    
    // Se la versione è già aggiornata, non fare nulla
    if (fromVersion >= EEPROMSettings::CURRENT_VERSION) {
        return true;
    }
    
    // Se la versione è 2, aggiorna alla 3
    if (fromVersion == 2) {
        // Crea una nuova struttura con i valori di default
        EEPROMSettings defaultSettings;
        loadDefaultSettings(defaultSettings);
        
        // Copia i valori esistenti
        defaultSettings.leftPID = settings.leftPID;
        defaultSettings.rightPID = settings.rightPID;
        defaultSettings.homePosition = settings.homePosition;
        
        // Aggiorna la struttura con le nuove impostazioni
        settings = defaultSettings;
        fromVersion = 3;  // Aggiorna la versione per il prossimo passo
        migrated = true;
    }
    
    // Se la versione è 3, aggiorna alla 4
    if (fromVersion == 3) {
        // Crea una nuova struttura con i valori di default
        EEPROMSettings defaultSettings;
        loadDefaultSettings(defaultSettings);
        
        // Copia i valori esistenti
        defaultSettings.leftPID = settings.leftPID;
        defaultSettings.rightPID = settings.rightPID;
        defaultSettings.homePosition = settings.homePosition;
        
        // Copia le altre impostazioni mantenute
        defaultSettings.system = settings.system;
        defaultSettings.mower = settings.mower;
        defaultSettings.navigation = settings.navigation;
        defaultSettings.maintenance = settings.maintenance;
        
        // Aggiorna la struttura con le nuove impostazioni
        settings = defaultSettings;
        fromVersion = 4;  // Aggiorna la versione per il prossimo passo
        migrated = true;
    }
    
    // Se la versione è 4, aggiorna alla 5 (rimozione impostazioni di rete)
    if (fromVersion == 4) {
        // Crea una nuova struttura con i valori di default
        EEPROMSettings defaultSettings;
        loadDefaultSettings(defaultSettings);
        
        // Copia i valori esistenti
        defaultSettings.system = settings.system;
        defaultSettings.mower = settings.mower;
        defaultSettings.navigation = settings.navigation;
        defaultSettings.maintenance = settings.maintenance;
        defaultSettings.leftPID = settings.leftPID;
        defaultSettings.rightPID = settings.rightPID;
        defaultSettings.homePosition = settings.homePosition;
        
        // Aggiorna la struttura con le nuove impostazioni
        settings = defaultSettings;
        migrated = true;
    }
    
    // Se sono state apportate modifiche, aggiorna il checksum
    if (migrated) {
        settings.version = EEPROMSettings::CURRENT_VERSION;
        updateChecksum(settings.system);
        updateChecksum(settings.mower);
        updateChecksum(settings.navigation);
        updateChecksum(settings.maintenance);
        updateChecksum(settings.leftPID);
        updateChecksum(settings.rightPID);
        updateHomePositionChecksum(settings.homePosition);
        
        // Calcola il checksum totale
        settings.checksum = 0;
        settings.checksum = calculateChecksum(&settings, sizeof(settings));
    }
    
    return migrated;
}

// Funzioni per la gestione della posizione home
bool EEPROMManager::validateHomePosition(const HomePosition& data) {
    if (!data.isSet) return true;  // Se non è impostata, è comunque valida
    
    HomePosition temp = data;
    uint8_t savedChecksum = temp.checksum;
    temp.checksum = 0;
    return (savedChecksum == calculateChecksum(&temp, sizeof(HomePosition)));
}

void EEPROMManager::updateHomePositionChecksum(HomePosition& data) {
    if (!data.isSet) return;  // Non aggiornare se non è impostata
    data.checksum = 0;
    data.checksum = calculateChecksum(&data, sizeof(HomePosition));
}
