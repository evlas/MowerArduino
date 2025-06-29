#ifndef MOWER_STATE_MACHINE_H
#define MOWER_STATE_MACHINE_H

#include <Arduino.h>

// Forward declaration per evitare dipendenze circolari
class Mower;

/**
 * @brief Classe per la gestione della macchina a stati del tosaerba
 */
class MowerStateMachine {
public:
    /**
     * @brief Enumerazione degli stati del tosaerba
     */
    enum class MowerState {
        IDLE,               // In attesa di comandi
        MOWING,             // Taglio in corso
        DOCKING,            // Ritorno alla stazione di ricarica
        CHARGING,           // Ricarica della batteria
        MANUAL_CONTROL,     // Controllo manuale
        EMERGENCY_STOP,     // Arresto di emergenza
        BORDER_DETECTED,    // Rilevato bordo/ostacolo
        LIFTED,             // Tosaerba sollevato
        TESTING,            // Modalità test
        PAUSED,             // Pausa manuale
        SLEEP,              // Modalità a basso consumo
        RAIN_DELAY,         // Pausa per pioggia
        MAINTENANCE_NEEDED, // Manutenzione richiesta
        ROS_CONTROL,        // Controllo remoto ROS
        ERROR               // Stato di errore
    };

    // Metodi pubblici...
public:
    /**
     * @brief Costruttore
     * @param mower Riferimento alla classe Mower
     */
    explicit MowerStateMachine(Mower& mower);

    /**
     * @brief Inizializza la macchina a stati
     */
    void begin();

    /**
     * @brief Aggiorna la macchina a stati (da chiamare nel loop principale)
     */
    void update();

    /**
     * @brief Richiede un cambio di stato
     * @param newState Nuovo stato richiesto
     * @return true se la transizione è consentita, false altrimenti
     */
    bool requestStateChange(MowerState newState);

    /**
     * @brief Esegue la transizione allo stato specificato se consentita
     * @param newState Nuovo stato a cui passare
     * @return true se la transizione è avvenuta con successo, false altrimenti
     */
    bool transitionToState(MowerState newState);

    /**
     * @brief Restituisce lo stato corrente
     * @return Lo stato corrente
     */
    MowerState getCurrentState() const { return currentState_; }

    // Metodi per la gestione degli stati
    void startMowing();
    void stopMowing();
    void dock();
    void emergencyStop();
    void manualControl();
    void resetError();

private:
    // Riferimento alla classe Mower
    Mower& mower_;
    
    // Stato corrente e precedente
    MowerState currentState_;
    MowerState previousState_;
    
    // Timestamp di inizio stato
    unsigned long stateStartTime_;
    
    // Flag di emergenza
    bool emergencyStopActive_;

    // Metodi privati
    void onEnterState(MowerState newState);
    void onExitState(MowerState oldState);
    void onUpdateState();
    bool isTransitionAllowed(MowerState from, MowerState to) const;
    bool isEmergencyStopActive() const;
    
    // Gestori di stato
    void handleIdle();
    void handleMowing();
    void handleDocking();
    void handleCharging();
    void handleManualControl();
    void handlePaused();
    void handleSleep();
    void handleRainDelay();
    void handleMaintenanceNeeded();
    void handleROSControl();
    void handleEmergencyStop();
    void handleBorderDetected();
    void handleLifted();
    void handleTesting();
    void handleError();
};

#endif // MOWER_STATE_MACHINE_H
