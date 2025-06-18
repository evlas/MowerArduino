#include "StateMachine.h"
#include "Navigation.h"
#include "../../config.h"
#include <Arduino.h>

// Dichiarazione delle istanze esterne
extern Navigation navigation;
#ifdef ENABLE_BLADE_MOTORS
extern BladeController bladeController;
#endif

// Definizione dell'istanza globale della macchina a stati
// Questa è l'unica definizione valida nel progetto
StateMachine mowerStateMachine;

// Includi le dipendenze necessarie
#ifdef ENABLE_DRIVE_MOTORS
#include "../motors/MotorController.h"
extern MotorController motorController;
#endif

#ifdef ENABLE_BLADE_MOTORS
#include "../motors/BladeController.h"
extern BladeController bladeController;
#endif

// Costanti per i timeout (in millisecondi)
const unsigned long MOWING_MIN_DURATION = 30000;       // 30 secondi minimi di taglio
const unsigned long OBSTACLE_AVOIDANCE_TIMEOUT = 10000; // 10 secondi per evitare un ostacolo
const unsigned long BORDER_RECOVERY_TIME = 5000;        // 5 secondi per riprendersi da un bordo

StateMachine::StateMachine() : _currentState(MowerState::IDLE), _stateStartTime(0) {
    // Inizializza la navigazione con valori di default
    navigation.setMode(NavigationMode::STOPPED);
}

void StateMachine::begin() {
    _currentState = MowerState::IDLE;
    _stateStartTime = millis();
    onEnterState(_currentState);
}

void StateMachine::update() {
    // Esegui le azioni dello stato corrente
    executeStateActions();
    
    // Verifica le transizioni di stato
    checkStateTransitions();
}

void StateMachine::sendEvent(MowerEvent event) {
    handleEvent(event);
}

MowerState StateMachine::getCurrentState() const {
    return _currentState;
}

bool StateMachine::isEmergency() const {
    return _currentState == MowerState::EMERGENCY_STOP || 
           _currentState == MowerState::ERROR;
}

bool StateMachine::isMowing() const {
    return _currentState == MowerState::MOWING;
}

void StateMachine::handleStateTransition(MowerState newState) {
    if (newState == _currentState) {
        return; // Nessun cambiamento di stato
    }
    
    // Esegui le azioni di uscita dallo stato corrente
    onExitState(_currentState);
    
    // Cambia stato
    MowerState oldState = _currentState;
    _currentState = newState;
    _stateStartTime = millis();
    
    // Esegui le azioni di ingresso nel nuovo stato
    onEnterState(newState);
    
    // Log della transizione di stato
    Serial.print("Stato: ");
    Serial.print(static_cast<int>(oldState));
    Serial.print(" -> ");
    Serial.println(static_cast<int>(newState));
}

void StateMachine::executeStateActions() {
    // Aggiorna sempre la navigazione se il robot non è in stato di errore o di emergenza
    if (_currentState != MowerState::ERROR && _currentState != MowerState::EMERGENCY_STOP) {
        navigation.update();
    }
    
    switch (_currentState) {
        case MowerState::IDLE:
            idleActions();
            break;
        case MowerState::MANUAL_CONTROL:
            manualControlActions();
            break;
        case MowerState::MOWING:
            // La gestione di ostacoli e bordi è gestita internamente da Navigation::update()
            // Non è necessario fare nulla qui
            break;
        case MowerState::RETURN_TO_BASE:
            // TODO: Implementare la logica per verificare se la base è stata raggiunta
            // Se la base viene raggiunta, inviare l'evento CHARGING_COMPLETE
            break;
        case MowerState::BORDER_DETECTED:
            // Esegui la manovra per allontanarsi dal bordo
            borderDetectedActions();
            break;
        case MowerState::OBSTACLE_AVOIDANCE:
            // Esegui la manovra per evitare l'ostacolo
            obstacleAvoidanceActions();
            break;
        default:
            // Non fare nulla per gli altri stati
            break;
    }
}

void StateMachine::onEnterState(MowerState newState) {
    _stateStartTime = millis();
    
    switch (newState) {
        case MowerState::IDLE:
            // Ferma tutto
            navigation.setMode(NavigationMode::STOPPED);
            #ifdef ENABLE_BLADE_MOTORS
                bladeController.setBladeSpeed(0);
            #endif
            break;
        case MowerState::MOWING:
            // Imposta la modalità di navigazione casuale e avvia la lama
            navigation.setMode(NavigationMode::RANDOM);
            #ifdef ENABLE_BLADE_MOTORS
                bladeController.setBladeSpeed(DEFAULT_BLADE_SPEED);
            #endif
            break;
        case MowerState::RETURN_TO_BASE:
            // Imposta la navigazione verso la base e spegni la lama
            navigation.setMode(NavigationMode::STOPPED);  // Sostituire con la modalità di ritorno alla base
            #ifdef ENABLE_BLADE_MOTORS
                bladeController.setBladeSpeed(0);
            #endif
            // TODO: Implementare la logica di ritorno alla base
            break;
        case MowerState::CHARGING:
            // Ferma tutto
            navigation.setMode(NavigationMode::STOPPED);
            #ifdef ENABLE_BLADE_MOTORS
                bladeController.setBladeSpeed(0);
            #endif
            break;
        case MowerState::EMERGENCY_STOP:
            // Arresto di emergenza: ferma tutto immediatamente
            navigation.setMode(NavigationMode::STOPPED);
            #ifdef ENABLE_BLADE_MOTORS
                bladeController.setBladeSpeed(0);
            #endif
            break;
        case MowerState::BORDER_DETECTED:
            // Ferma la navigazione e prepara la manovra
            navigation.setMode(NavigationMode::STOPPED);
            break;
        case MowerState::OBSTACLE_AVOIDANCE:
            // Ferma la navigazione e prepara la manovra di evitamento
            navigation.setMode(NavigationMode::STOPPED);
            break;
    }        
}

void StateMachine::onExitState(MowerState oldState) {
    // Azioni da eseguire quando si esce da uno stato
    // (se necessario)
}

void StateMachine::handleEvent(MowerEvent event) {
    switch (_currentState) {
        case MowerState::IDLE:
            if (event == MowerEvent::START_MOWING) {
                handleStateTransition(MowerState::MOWING);
            } else if (event == MowerEvent::MANUAL_MODE) {
                handleStateTransition(MowerState::MANUAL_CONTROL);
            }
            break;
        case MowerState::MOWING:
            if (event == MowerEvent::STOP_MOWING) {
                handleStateTransition(MowerState::IDLE);
            } else if (event == MowerEvent::LOW_BATTERY) {
                handleStateTransition(MowerState::RETURN_TO_BASE);
            } else if (event == MowerEvent::OBSTACLE_DETECTED) {
                handleStateTransition(MowerState::OBSTACLE_AVOIDANCE);
            } else if (event == MowerEvent::BORDER_DETECTED) {
                handleStateTransition(MowerState::BORDER_DETECTED);
            } else if (event == MowerEvent::EMERGENCY_STOP) {
                handleStateTransition(MowerState::EMERGENCY_STOP);
            }
            break;
            
        case MowerState::RETURN_TO_BASE:
            if (event == MowerEvent::CHARGING_COMPLETE) {
                handleStateTransition(MowerState::CHARGING);
            } else if (event == MowerEvent::EMERGENCY_STOP) {
                handleStateTransition(MowerState::EMERGENCY_STOP);
            }
            break;
            
        // Gestisci altri stati...
        
        default:
            // Gestione di default per gli eventi
            if (event == MowerEvent::EMERGENCY_STOP) {
                handleStateTransition(MowerState::EMERGENCY_STOP);
            }
            break;
    }
}

void StateMachine::checkStateTransitions() {
    unsigned long currentTime = millis();
    unsigned long stateDuration = currentTime - _stateStartTime;
    
    switch (_currentState) {
        case MowerState::MOWING:
            // Verifica se è il momento di tornare alla base per la ricarica
            // (esempio: ogni 30 minuti di taglio)
            if (stateDuration > 30 * 60 * 1000) { // 30 minuti
                sendEvent(MowerEvent::LOW_BATTERY);
            }
            break;
            
        case MowerState::OBSTACLE_AVOIDANCE:
            // Se siamo in stato di evitamento ostacoli da troppo tempo, prova a cambiare strategia
            if (stateDuration > OBSTACLE_AVOIDANCE_TIMEOUT) {
                handleStateTransition(MowerState::MOWING);
            }
            break;
            
        case MowerState::BORDER_DETECTED:
            // Dopo un po' di tempo, riprova a muoverti
            if (stateDuration > BORDER_RECOVERY_TIME) {
                handleStateTransition(MowerState::MOWING);
            }
            break;
            
        default:
            break;
    }
}

// Implementazione delle azioni specifiche per ogni stato
void StateMachine::idleActions() {
    // Nessuna azione specifica nello stato di attesa
}

void StateMachine::manualControlActions() {
    // Le azioni di controllo manuale sono gestite da un controller esterno
}

void StateMachine::mowingActions() {
    // Esegui il pattern di taglio (es. spirale, linee parallele)
    // La logica specifica è gestita dal modulo di navigazione
    
    // Verifica la presenza di ostacoli o bordi
    // (da implementare con i sensori)
}

void StateMachine::returnToBaseActions() {
    // Naviga verso la stazione di ricarica
    // (da implementare con il sistema di navigazione)
    
    // Se la batteria è troppo bassa, fermati per evitare danni
    // (verifica con il gestore della batteria)
}

void StateMachine::chargingActions() {
    // Verifica lo stato della carica
    // (da implementare con il gestore della batteria)
    
    // Se la batteria è carica, torna in modalità di attesa
    // sendEvent(MowerEvent::BATTERY_CHARGED);
}

void StateMachine::errorActions() {
    // Mantieni il robot fermo e segnala l'errore
    // (da implementare con il sistema di notifica)
}

void StateMachine::emergencyStopActions() {
    // Mantieni il robot fermo finché l'emergenza non viene risolta
    // (richiede intervento manuale o reset)
}

void StateMachine::borderDetectedActions() {
    // La gestione del bordo è gestita internamente da Navigation
    // Se la navigazione è ancora in corso, continua a gestire il bordo
    if (navigation.isNavigating()) {
        // Continua a gestire il bordo
    } else {
        // Se la navigazione è terminata, torna a tagliare
        sendEvent(MowerEvent::START_MOWING);
    }
}

void StateMachine::obstacleAvoidanceActions() {
    // La gestione degli ostacoli è gestita internamente da Navigation
    // Se la navigazione è ancora in corso, continua a gestire l'ostacolo
    if (navigation.isNavigating()) {
        // Continua a gestire l'ostacolo
    } else {
        // Se la navigazione è terminata, torna a tagliare
        sendEvent(MowerEvent::START_MOWING);
    }
}
