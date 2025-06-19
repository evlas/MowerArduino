#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include "Navigation.h"

// Stati possibili del robot
enum class MowerState {
    IDLE,               // In attesa di comandi
    MANUAL_CONTROL,     // Controllo manuale
    MOWING,            // Taglio dell'erba
    RETURN_TO_BASE,     // Ritorno alla base di ricarica
    CHARGING,          // In carica
    ERROR,             // Stato di errore
    EMERGENCY_STOP,    // Arresto di emergenza
    BORDER_DETECTED,   // Rilevato bordo/ostacolo
    OBSTACLE_AVOIDANCE // Evitamento ostacolo
};

// Eventi che possono causare un cambiamento di stato
enum class MowerEvent {
    START_MOWING,       // Avvia il taglio
    STOP_MOWING,        // Ferma il taglio
    MANUAL_MODE,        // Passa alla modalità manuale
    RETURN_TO_BASE,     // Ritorna alla base di ricarica
    LOW_BATTERY,        // Batteria scarica
    BATTERY_CHARGED,    // Batteria carica
    OBSTACLE_DETECTED,  // Rilevato ostacolo
    BORDER_DETECTED,    // Rilevato bordo
    ERROR_DETECTED,     // Rilevato errore
    ERROR_RESOLVED,     // Errore risolto
    EMERGENCY_STOP,     // Arresto di emergenza
    EMERGENCY_CLEARED,  // Emergenza rientrata
    CHARGING_COMPLETE   // Carica completata
};

class StateMachine {
public:
    // Costruttore
    StateMachine();
    
    // Inizializzazione
    void begin();
    
    // Aggiorna lo stato in base agli eventi
    void update();
    
    // Invia un evento alla macchina a stati
    void sendEvent(MowerEvent event);
    
    // Ottieni lo stato corrente
    MowerState getCurrentState() const;
    
    // Imposta la modalità di navigazione
    void setNavigationMode(NavigationMode::Mode mode);
    
    // Ottieni la modalità di navigazione corrente
    NavigationMode::Mode getNavigationMode() const;
    
    // Verifica se il robot è in modalità di emergenza
    bool isEmergency() const;
    
    // Verifica se il robot sta tagliando
    bool isMowing() const;
    
    // Metodi per la gestione degli eventi
    void handleIdleState(MowerEvent event);
    void handleMowingState(MowerEvent event);
    void handleManualControlState(MowerEvent event);
    void handleReturnToBaseState(MowerEvent event);
    void handleEmergencyStopState(MowerEvent event);
    void handleRecoveryState(MowerEvent event);
    void handleDefaultState(MowerEvent event);
    void handleGlobalEvents(MowerEvent event);
    
    // Metodi di utilità per il debug
    const char* stateToString(MowerState state);
    const char* eventToString(MowerEvent event);
    
private:
    // Stato corrente
    MowerState _currentState;
    MowerState _previousState;  // Aggiunto per salvare lo stato precedente
    unsigned long _stateStartTime;
    NavigationMode::Mode _navigationMode;
    
    // Gestisci la transizione di stato
    void handleStateTransition(MowerState newState);
    
    // Esegui le azioni per lo stato corrente
    void executeStateActions();
    
    // Azioni eseguite all'ingresso di ogni stato
    void onEnterState(MowerState newState);
    
    // Azioni eseguite all'uscita da ogni stato
    void onExitState(MowerState oldState);
    
    // Gestisci un evento nello stato corrente
    void handleEvent(MowerEvent event);
    
    // Verifica le condizioni per le transizioni di stato
    void checkStateTransitions();
    
    // Azioni specifiche per ogni stato
    void idleActions();
    void manualControlActions();
    void mowingActions();
    void returnToBaseActions();
    void chargingActions();
    void errorActions();
    void emergencyStopActions();
    void borderDetectedActions();
    void obstacleAvoidanceActions();
};

// Dichiarazione esterna dell'istanza globale
extern StateMachine mowerStateMachine;

#endif // STATE_MACHINE_H
