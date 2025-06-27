#ifndef MOWER_H
#define MOWER_H

#include <Arduino.h>
#include "MowerStateMachine.h"

// Forward declaration per evitare dipendenze circolari
class MowerStateMachine;

// Forward declaration of MowerState enum
class MowerStateMachine;

class Mower {
public:
    // Stati del tosaerba
    enum class State {
        IDLE,               // In attesa di comandi
        MOWING,             // Taglio dell'erba in corso
        DOCKING,            // Ritorno alla base di ricarica
        CHARGING,           // Ricarica in corso
        MANUAL_CONTROL,     // Controllo manuale attivo
        EMERGENCY_STOP,     // Arresto di emergenza attivo
        BORDER_DETECTED,    // Rilevato bordo dell'area di taglio
        LIFTED,             // Tosaerba sollevato da terra
        TESTING,            // Modalità test
        PAUSED,             // Pausa temporanea del taglio
        SLEEP,              // Modalità risparmio energetico
        RAIN_DELAY,         // Pausa per pioggia
        MAINTENANCE_NEEDED, // Manutenzione richiesta
        ROS_CONTROL,        // Controllo da ROS (Robot Operating System)
        ERROR               // Stato di errore
    };
    
    // Costruttore
    Mower();
    
    // Inizializzazione
    void begin();
    void update();

    // ===== Metodi motori =====
    void startBlades();
    void stopBlades();
    void setBladeSpeed(float speed);
    void setLeftMotorSpeed(float speed);
    void setRightMotorSpeed(float speed);
    void stopDriveMotors();
    void startDriveMotors();
    void stopMotors();
    
    // Modalità di navigazione
    enum class NavigationMode {
        STOPPED,        // Motori fermi, nessun movimento
        RANDOM,         // Movimento casuale nell'area di taglio
        MANUAL,         // Controllo manuale tramite comandi
        DOCKING,        // Navigazione verso la base di ricarica
        BORDER_FOLLOW,  // Seguimento del bordo dell'area di taglio
        SPIRAL,         // Modello a spirale per copertura efficiente
        LAWN_STRIPES,   // Modello a strisce per prato
        ZONE            // Taglio per zone definite
    };
    void setNavigationMode(NavigationMode mode);
    
    // ===== Metodi di sistema =====
    void enableCharging(bool enable);
    bool isCharging() const;
    bool isLifted() const;
    bool isBorderDetected() const;
    bool isCollisionDetected() const;
    bool isBatteryCritical() const;
    bool isBatteryLow() const;
    bool isBatteryCharged() const;
    bool isBatteryFull() const;
    bool isDocked() const;
    
    // Metodo per il logging dei cambiamenti di stato
    void logStateChange(MowerStateMachine::MowerState from, MowerStateMachine::MowerState to);

    // ===== Gestione sensori =====
    void setCharging(bool charging) { charging_ = charging; }
    void setLifted(bool lifted) { lifted_ = lifted; }
    void setBorderDetected(bool detected) { borderDetected_ = detected; }
    void setCollisionDetected(bool detected) { collisionDetected_ = detected; }
    void setDocked(bool docked) { docked_ = docked; }
    bool checkObstacles() const { return collisionDetected_; }

    // ===== Azioni =====
    void emergencyStop();
    void resetEmergencyStop();
    void handleBorder();
    void handleObstacle();
    void updateSensors();
    void startMowing();
    void startDocking();

    // ===== Gestione stato =====
    void setState(State newState);
    State getState() const { return currentState_; }
    void setStateMachine(MowerStateMachine* stateMachine);
    NavigationMode getNavigationMode() const { return navigationMode_; }
    const char* navigationModeToString(NavigationMode mode) const;

    // ===== Controllo da seriale =====
    void processSerialCommand(const String& command);

private:
    // Stato interno
    State currentState_;
    bool emergencyStopActive_;
    bool bladesRunning_;
    float leftMotorSpeed_;
    float rightMotorSpeed_;
    float bladeSpeed_;
    bool charging_;
    float batteryLevel_;
    NavigationMode navigationMode_;
    MowerStateMachine* stateMachine_;
    unsigned long lastSensorUpdate_;
    
    // Stati dei sensori
    bool lifted_;
    bool borderDetected_;
    bool collisionDetected_;
    bool docked_;
    
    // Metodi privati
    const char* stateToString(State state) const;
    void printPaddedNumber(int number) const;
    void checkSafety();
};

#endif // MOWER_H
