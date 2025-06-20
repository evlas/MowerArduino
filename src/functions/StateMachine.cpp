#include "StateMachine.h"
#include "Navigation.h"
#include "../../config.h"
#include <Arduino.h>

#ifdef ENABLE_BATTERY_MONITOR
#include <INA226_WE.h>
extern INA226_WE batteryMonitor;
#endif

// Dichiarazione delle istanze esterne
extern Navigation navigation;
#ifdef ENABLE_BLADE_MOTORS
extern BladeController bladeController;
#endif

// Includi la gestione del relay
#ifdef ENABLE_RELAY
#include "../../src/actuators/Relay.h"
extern Relay relay;
#endif

// Includi la gestione del display
#ifdef ENABLE_DISPLAY
#include "../LCD/LCDManager.h"
extern LCDManager lcdManager;
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

StateMachine::StateMachine() : _currentState(MowerState::IDLE), _previousState(MowerState::IDLE), _stateStartTime(0) {
    // Inizializza la navigazione con valori di default
    navigation.setMode(NavigationMode::STOPPED);
    
    // Inizialmente il relay è disattivato (motori spenti)
    #ifdef ENABLE_RELAY
    relay.off();
    #endif
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
    if (_currentState == newState) {
        return; // Nessuna transizione necessaria
    }

    // Salva lo stato corrente come precedente prima di cambiare
    _previousState = _currentState;
    
    // Esegui le azioni di uscita dallo stato corrente
    onExitState(_currentState);
    
    // Aggiorna lo stato
    MowerState oldState = _currentState;
    _currentState = newState;
    _stateStartTime = millis();
    
    // Esegui le azioni di ingresso nel nuovo stato
    onEnterState(newState);
    
    #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.print("Transizione di stato: ");
        SERIAL_DEBUG.print(stateToString(oldState));
        SERIAL_DEBUG.print(" -> ");
        SERIAL_DEBUG.println(stateToString(newState));
    #endif
}

void StateMachine::handleEvent(MowerEvent event) {
    #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.print("Evento ricevuto: ");
        SERIAL_DEBUG.println(eventToString(event));
    #endif

    switch (_currentState) {
        case MowerState::IDLE:
            handleIdleState(event);
            break;
            
        case MowerState::MOWING:
            handleMowingState(event);
            break;
            
        case MowerState::MANUAL_CONTROL:
            handleManualControlState(event);
            break;
            
        case MowerState::RETURN_TO_BASE:
            handleReturnToBaseState(event);
            break;
            
        case MowerState::CHARGING:
            handleChargingState(event);
            break;
            
        case MowerState::EMERGENCY_STOP:
            handleEmergencyStopState(event);
            break;
            
        case MowerState::BORDER_DETECTED:
        case MowerState::OBSTACLE_AVOIDANCE:
            handleRecoveryState(event);
            break;
            
        default:
            handleDefaultState(event);
            break;
    }
    
    // Gestione eventi globali che possono verificarsi in qualsiasi stato
    handleGlobalEvents(event);
}

void StateMachine::handleIdleState(MowerEvent event) {
    switch (event) {
        case MowerEvent::START_MOWING:
            handleStateTransition(MowerState::MOWING);
            break;
            
        case MowerEvent::MANUAL_MODE:
            handleStateTransition(MowerState::MANUAL_CONTROL);
            break;
            
        case MowerEvent::RETURN_TO_BASE:
            handleStateTransition(MowerState::RETURN_TO_BASE);
            break;
            
        case MowerEvent::EMERGENCY_STOP:
        case MowerEvent::ERROR_DETECTED:
            handleStateTransition(MowerState::EMERGENCY_STOP);
            break;
            
        default:
            // Evento non gestito per questo stato
            break;
    }
}

void StateMachine::handleGlobalEvents(MowerEvent event) {
    switch (event) {
        case MowerEvent::EMERGENCY_STOP:
            handleStateTransition(MowerState::EMERGENCY_STOP);
            break;
            
        case MowerEvent::ERROR_DETECTED:
            // Se non siamo già in stato di errore
            if (_currentState != MowerState::EMERGENCY_STOP) {
                handleStateTransition(MowerState::EMERGENCY_STOP);
            }
            break;
            
        default:
            break;
    }
}

void StateMachine::handleMowingState(MowerEvent event) {
    switch (event) {
        case MowerEvent::STOP_MOWING:
            handleStateTransition(MowerState::IDLE);
            break;
            
        case MowerEvent::LOW_BATTERY:
        case MowerEvent::RETURN_TO_BASE:
            handleStateTransition(MowerState::RETURN_TO_BASE);
            break;
            
        case MowerEvent::OBSTACLE_DETECTED:
            handleStateTransition(MowerState::OBSTACLE_AVOIDANCE);
            break;
            
        case MowerEvent::BORDER_DETECTED:
            handleStateTransition(MowerState::BORDER_DETECTED);
            break;
            
        default:
            // Evento non gestito per questo stato
            break;
    }
}

void StateMachine::handleManualControlState(MowerEvent event) {
    switch (event) {
        case MowerEvent::RETURN_TO_BASE:
            handleStateTransition(MowerState::RETURN_TO_BASE);
            break;
            
        case MowerEvent::EMERGENCY_STOP:
            handleStateTransition(MowerState::EMERGENCY_STOP);
            break;
            
        default:
            // Evento non gestito per questo stato
            break;
    }
}

void StateMachine::handleReturnToBaseState(MowerEvent event) {
    switch (event) {
        case MowerEvent::CHARGING_COMPLETE:
            handleStateTransition(MowerState::CHARGING);
            break;
            
        case MowerEvent::EMERGENCY_STOP:
            handleStateTransition(MowerState::EMERGENCY_STOP);
            break;
            
        case MowerEvent::MANUAL_MODE:
            handleStateTransition(MowerState::MANUAL_CONTROL);
            break;
            
        default:
            // Evento non gestito per questo stato
            break;
    }
}

void StateMachine::handleChargingState(MowerEvent event) {
    switch (event) {
        case MowerEvent::BATTERY_CHARGED:
            // Batteria completamente carica, torna in stato IDLE
            handleStateTransition(MowerState::IDLE);
            break;
            
        case MowerEvent::EMERGENCY_STOP:
            handleStateTransition(MowerState::EMERGENCY_STOP);
            break;
            
        case MowerEvent::MANUAL_MODE:
            handleStateTransition(MowerState::MANUAL_CONTROL);
            break;
            
        default:
            // Ignora altri eventi
            break;
    }
}

void StateMachine::handleEmergencyStopState(MowerEvent event) {
    switch (event) {
        case MowerEvent::EMERGENCY_CLEARED:
            // Torna allo stato precedente se disponibile
            if (_previousState != MowerState::EMERGENCY_STOP) {
                handleStateTransition(_previousState);
            } else {
                handleStateTransition(MowerState::IDLE);
            }
            break;
            
        default:
            // Nessun altro evento è consentito in stato di emergenza
            break;
    }
}

void StateMachine::handleRecoveryState(MowerEvent event) {
    // Gestione comune per gli stati di recupero (BORDER_DETECTED e OBSTACLE_AVOIDANCE)
    unsigned long currentTime = millis();
    unsigned long stateDuration = currentTime - _stateStartTime;
    
    switch (event) {
        case MowerEvent::EMERGENCY_STOP:
            handleStateTransition(MowerState::EMERGENCY_STOP);
            break;
            
        case MowerEvent::ERROR_DETECTED:
            handleStateTransition(MowerState::EMERGENCY_STOP);
            break;
            
        default:
            // Timeout per tornare allo stato precedente
            if (stateDuration > BORDER_RECOVERY_TIME && _currentState == MowerState::BORDER_DETECTED) {
                handleStateTransition(_previousState);
            } else if (stateDuration > OBSTACLE_AVOIDANCE_TIMEOUT && _currentState == MowerState::OBSTACLE_AVOIDANCE) {
                handleStateTransition(_previousState);
            }
            break;
    }
}

void StateMachine::handleDefaultState(MowerEvent event) {
    // Gestione predefinita per stati non specificamente gestiti
    if (event == MowerEvent::EMERGENCY_STOP) {
        handleStateTransition(MowerState::EMERGENCY_STOP);
    } else if (event == MowerEvent::ERROR_DETECTED) {
        handleStateTransition(MowerState::EMERGENCY_STOP);
    }
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
            // Ferma la navigazione e prepara la manovra
            borderDetectedActions();
            break;
        case MowerState::OBSTACLE_AVOIDANCE:
            // Ferma la navigazione e prepara la manovra di evitamento
            obstacleAvoidanceActions();
            break;
        default:
            // Non fare nulla per gli altri stati
            break;
    }
}

void StateMachine::onEnterState(MowerState newState) {
    _stateStartTime = millis();
    
    // Gestione del relay in base allo stato
    #ifdef ENABLE_RELAY
    switch (newState) {
        // Stati in cui il relay deve essere SPENTO (motori disabilitati)
        case MowerState::IDLE:
        case MowerState::CHARGING:
        case MowerState::ERROR:
        case MowerState::EMERGENCY_STOP:
            relay.off();
            break;
            
        // Stati in cui il relay deve essere ACCESO (motori abilitati)
        case MowerState::MANUAL_CONTROL:
        case MowerState::MOWING:
        case MowerState::RETURN_TO_BASE:
        case MowerState::BORDER_DETECTED:
        case MowerState::OBSTACLE_AVOIDANCE:
            relay.on();
            break;
    }
    #endif
    
    // Aggiorna il display con lo stato corrente
    #ifdef ENABLE_DISPLAY
    switch (newState) {
        case MowerState::IDLE:
            lcdManager.setRobotState(RobotState::IDLE);
            break;
        case MowerState::MANUAL_CONTROL:
            lcdManager.setRobotState(RobotState::MANUAL);
            break;
        case MowerState::MOWING:
            lcdManager.setRobotState(RobotState::RUNNING);
            break;
        case MowerState::RETURN_TO_BASE:
            lcdManager.setRobotState(RobotState::RETURNING);
            break;
        case MowerState::CHARGING:
            lcdManager.setRobotState(RobotState::CHARGING);
            break;
        case MowerState::ERROR:
            lcdManager.setRobotState(RobotState::ERROR);
            break;
        case MowerState::EMERGENCY_STOP:
            lcdManager.setRobotState(RobotState::EMERGENCY);
            break;
        case MowerState::BORDER_DETECTED:
        case MowerState::OBSTACLE_AVOIDANCE:
            lcdManager.setRobotState(RobotState::RUNNING);
            break;
    }
    #endif

    // Gestione specifica per ogni stato
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
    #ifdef ENABLE_BATTERY_MONITOR
    float voltage = batteryMonitor.getBusVoltage_V();
    if (voltage >= FULL_BATTERY_VOLTAGE) {
        sendEvent(MowerEvent::BATTERY_CHARGED);
    }
    #endif
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

const char* StateMachine::stateToString(MowerState state) {
    switch (state) {
        case MowerState::IDLE: return "IDLE";
        case MowerState::MOWING: return "MOWING";
        case MowerState::MANUAL_CONTROL: return "MANUAL_CONTROL";
        case MowerState::RETURN_TO_BASE: return "RETURN_TO_BASE";
        case MowerState::CHARGING: return "CHARGING";
        case MowerState::ERROR: return "ERROR";
        case MowerState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case MowerState::BORDER_DETECTED: return "BORDER_DETECTED";
        case MowerState::OBSTACLE_AVOIDANCE: return "OBSTACLE_AVOIDANCE";
        default: return "UNKNOWN";
    }
}

const char* StateMachine::eventToString(MowerEvent event) {
    switch (event) {
        case MowerEvent::START_MOWING: return "START_MOWING";
        case MowerEvent::STOP_MOWING: return "STOP_MOWING";
        case MowerEvent::MANUAL_MODE: return "MANUAL_MODE";
        case MowerEvent::RETURN_TO_BASE: return "RETURN_TO_BASE";
        case MowerEvent::LOW_BATTERY: return "LOW_BATTERY";
        case MowerEvent::BATTERY_CHARGED: return "BATTERY_CHARGED";
        case MowerEvent::OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
        case MowerEvent::BORDER_DETECTED: return "BORDER_DETECTED";
        case MowerEvent::ERROR_DETECTED: return "ERROR_DETECTED";
        case MowerEvent::ERROR_RESOLVED: return "ERROR_RESOLVED";
        case MowerEvent::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case MowerEvent::EMERGENCY_CLEARED: return "EMERGENCY_CLEARED";
        case MowerEvent::CHARGING_COMPLETE: return "CHARGING_COMPLETE";
        default: return "UNKNOWN_EVENT";
    }
}
