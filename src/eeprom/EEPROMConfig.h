#ifndef EEPROM_CONFIG_H
#define EEPROM_CONFIG_H

#include <Arduino.h>

// Struttura per i parametri PID
struct PIDParams {
    float kp;
    float ki;
    float kd;
    uint8_t checksum;  // Per validazione
};

// Struttura per la posizione del docking
struct HomePosition {
    float x;            // Coordinate X in metri
    float y;            // Coordinate Y in metri
    float theta;        // Orientamento in radianti
    bool isSet;         // Se true, la posizione è stata impostata
    uint8_t checksum;   // Per validazione
};

// Struttura per le impostazioni di sistema
struct SystemSettings {
    uint8_t language;         // 0=Inglese, 1=Italiano, ecc.
    bool use24HourFormat;     // true=24h, false=12h
    bool useMetric;           // true=metrico, false=imperiale
    uint8_t brightness;       // Livello luminosità display (0-100)
    uint8_t volume;           // Volume audio (0-100)
    uint8_t checksum;
};

// Struttura per le impostazioni del tagliaerba
struct MowerSettings {
    uint8_t cuttingHeight;    // Altezza di taglio (1-10)
    uint8_t bladeSpeed;       // Velocità lama (0-100%)
    uint8_t mowingPattern;    // 0=Random, 1=Parallelo, 2=Spirale
    bool rainDelay;           // Ritardo pioggia attivo
    uint32_t rainDelayUntil;  // Timestamp fine ritardo
    uint8_t checksum;
};

// Struttura per le impostazioni di navigazione
struct NavigationSettings {
    uint8_t perimeterMode;    // 0=Wire, 1=Virtual
    uint8_t obstacleSensitivity; // 1-5
    bool gpsEnabled;          // GPS abilitato
    float gpsUpdateInterval;  // Intervallo aggiornamento GPS (sec)
    uint8_t checksum;
};

// Struttura per le impostazioni di manutenzione
struct MaintenanceSettings {
    uint32_t totalMowingTime; // Minuti totali di taglio
    uint32_t bladeHours;      // Ore di utilizzo lama
    uint32_t lastMaintenance; // Timestamp ultima manutenzione
    uint32_t nextMaintenance; // Timestamp prossima manutenzione
    uint8_t checksum;
};

// Struttura per tutti i parametri salvati
struct EEPROMSettings {
    static const uint16_t EEPROM_BASE = 0;
    static const uint16_t EEPROM_SIZE = 1024;  // 1KB di spazio riservato
    
    // Versione della struttura (incrementare in caso di modifiche)
    static const uint8_t CURRENT_VERSION = 5;  // Incrementato per la rimozione di NetworkSettings
    
    uint8_t version;
    
    // Sezioni di configurazione
    SystemSettings system;
    MowerSettings mower;
    NavigationSettings navigation;
    MaintenanceSettings maintenance;
    
    // Sezioni preesistenti
    PIDParams leftPID;
    PIDParams rightPID;
    HomePosition homePosition;
    
    uint8_t checksum;  // Checksum totale
};

#endif // EEPROM_CONFIG_H
