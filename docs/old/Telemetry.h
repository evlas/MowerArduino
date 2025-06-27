#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

// Forward declarations
class StateMachine;
class Navigation;
class PositionManager;

// Importa solo gli enum necessari
#include "../functions/StateMachine.h"  // Per MowerState
#include "../functions/Navigation.h"    // Per NavigationMode

// Struttura per i dati di telemetria
struct TelemetryData {
    // Stato del robot
    MowerState currentState;
    NavigationMode::Mode navigationMode;
    bool isEmergency;
    bool isMowing;
    bool isCharging;
    
    // Posizione e orientamento
    float positionX;        // cm
    float positionY;        // cm
    float heading;          // radianti
    float speed;            // cm/s
    float angularSpeed;     // rad/s
    
    // Batteria
    float batteryVoltage;   // V
    float batteryCurrent;   // A
    uint8_t batteryLevel;   // %
    
    // Motori
    float leftMotorSpeed;   // %
    float rightMotorSpeed;  // %
    uint8_t bladeSpeed;     // %
    
    // Sensori
    float frontDistance;    // cm
    float rearDistance;     // cm
    bool isBumperPressed;   // true se un bumper è premuto
    bool isPerimeterSignal;  // true se il segnale perimetro è rilevato
    
    // Timestamp
    unsigned long timestamp; // ms dall'avvio
    
    // Flag di validità
    bool isPositionValid;
    bool isBatteryValid;
    bool isSensorsValid;
};

class Telemetry {
public:
    // Costruttore
    Telemetry(StateMachine* stateMachine, Navigation* navigation, PositionManager* positionManager);
    
    // Inizializzazione
    void begin();
    
    // Aggiorna i dati di telemetria
    void update();
    
    // Ottiene i dati di telemetria correnti
    const TelemetryData& getData() const;
    
    // Genera una stringa JSON con i dati di telemetria
    String toJSON() const;
    
    // Imposta i dati della batteria
    void setBatteryData(float voltage, float current, uint8_t level);
    
    // Imposta i dati dei sensori
    void setSensorsData(float frontDist, float rearDist, bool bumperPressed, bool perimeterSignal);
    
    // Imposta i dati dei motori
    void setMotorsData(float leftSpeed, float rightSpeed, uint8_t bladeSpeed);

private:
    // Riferimenti ai componenti
    StateMachine* _stateMachine;
    Navigation* _navigation;
    PositionManager* _positionManager;
    
    // Dati di telemetria
    TelemetryData _data;
    
    // Inizializza la struttura dei dati
    void _initData();
    
    // Aggiorna i dati di posizione
    void _updatePositionData();
    
    // Aggiorna lo stato del robot
    void _updateStateData();
};

#endif // TELEMETRY_H
