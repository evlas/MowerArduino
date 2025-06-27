#include "MowerStateMachine.h"
#include "Mower.h"
#include <Arduino.h>

// Logging is done through the Mower class's logEvent method

MowerStateMachine::MowerStateMachine(Mower& mower)
    : mower_(mower),
      currentState_(MowerState::IDLE),
      previousState_(MowerState::IDLE),
      stateStartTime_(0),
      emergencyStopActive_(false) {
}

void MowerStateMachine::begin() {
    currentState_ = MowerState::IDLE;
    previousState_ = MowerState::IDLE;
    stateStartTime_ = millis();
    emergencyStopActive_ = false;
    
    // Chiama l'handler di ingresso per lo stato iniziale
    onEnterState(currentState_);
}

void MowerStateMachine::update() {
    // Controlla le condizioni di emergenza
    if (isEmergencyStopActive() && currentState_ != MowerState::EMERGENCY_STOP) {
        requestStateChange(MowerState::EMERGENCY_STOP);
        return;
    }

    // Aggiorna lo stato corrente
    onUpdateState();
}

bool MowerStateMachine::requestStateChange(MowerState newState) {
    if (currentState_ == newState) {
        return true; // Già nello stato richiesto
    }

    if (!isTransitionAllowed(currentState_, newState)) {
        Serial.print("Transizione non consentita: ");
        Serial.print(static_cast<int>(currentState_));
        Serial.print(" -> ");
        Serial.println(static_cast<int>(newState));
        return false;
    }

    // Uscita dallo stato corrente
    onExitState(currentState_);
    
    // Aggiornamento dello stato
    previousState_ = currentState_;
    currentState_ = newState;
    stateStartTime_ = millis();
    
    // Entrata nel nuovo stato
    onEnterState(currentState_);
    
    return true;
}

bool MowerStateMachine::transitionToState(MowerState newState) {
    return requestStateChange(newState);
}

void MowerStateMachine::startMowing() {
    if (!isEmergencyStopActive()) {
        requestStateChange(MowerState::MOWING);
    }
}

void MowerStateMachine::stopMowing() {
    if (currentState_ == MowerState::MOWING) {
        requestStateChange(MowerState::IDLE);
    }
}

void MowerStateMachine::dock() {
    if (!isEmergencyStopActive()) {
        requestStateChange(MowerState::DOCKING);
    }
}

void MowerStateMachine::emergencyStop() {
    requestStateChange(MowerState::EMERGENCY_STOP);
}

void MowerStateMachine::manualControl() {
    if (!isEmergencyStopActive()) {
        requestStateChange(MowerState::MANUAL_CONTROL);
    }
}

void MowerStateMachine::resetError() {
    if (currentState_ == MowerState::EMERGENCY_STOP) {
        requestStateChange(previousState_);
    }
}

void MowerStateMachine::onEnterState(MowerState newState) {
    Serial.print("Entrato nello stato: ");
    
    switch (newState) {
        case MowerState::IDLE:
            Serial.println("IDLE");
            mower_.stopBlades();
            mower_.stopDriveMotors();
            break;
            
        case MowerState::MOWING:
            Serial.println("MOWING");
            mower_.startBlades();
            mower_.startDriveMotors();
            mower_.setLeftMotorSpeed(150);
            mower_.setRightMotorSpeed(150);
            break;
            
        case MowerState::DOCKING:
            Serial.println("DOCKING");
            mower_.stopBlades();
            mower_.startDriveMotors();
            break;
            
        case MowerState::CHARGING:
            Serial.println("CHARGING");
            mower_.stopBlades();
            mower_.stopDriveMotors();
            mower_.setCharging(true);
            break;
            
        case MowerState::MANUAL_CONTROL:
            Serial.println("MANUAL_CONTROL");
            mower_.stopBlades();
            mower_.stopDriveMotors();
            break;
            
        case MowerState::PAUSED:
            Serial.println("PAUSED");
            mower_.stopBlades();
            mower_.stopDriveMotors();
            break;
            
        case MowerState::SLEEP:
            Serial.println("SLEEP");
            mower_.stopBlades();
            mower_.stopDriveMotors();
            // Aggiungi qui la logica per entrare in modalità a basso consumo
            break;
            
        case MowerState::RAIN_DELAY:
            Serial.println("RAIN_DELAY");
            mower_.stopBlades();
            mower_.stopDriveMotors();
            break;
            
        case MowerState::MAINTENANCE_NEEDED:
            Serial.println("MAINTENANCE_NEEDED");
            mower_.stopBlades();
            mower_.stopDriveMotors();
            // Attiva eventuali indicatori di manutenzione
            break;
            
        case MowerState::ROS_CONTROL:
            Serial.println("ROS_CONTROL");
            mower_.stopBlades();
            mower_.stopDriveMotors();
            // Inizializza la connessione ROS
            break;
            
        case MowerState::EMERGENCY_STOP:
            Serial.println("EMERGENCY_STOP");
            mower_.stopBlades();
            mower_.stopDriveMotors();
            mower_.emergencyStop();
            emergencyStopActive_ = true;
            break;
            
        case MowerState::BORDER_DETECTED:
            Serial.println("BORDER_DETECTED");
            mower_.handleBorder();
            break;
            
        case MowerState::LIFTED:
            Serial.println("LIFTED");
            mower_.emergencyStop();
            break;
            
        case MowerState::TESTING:
            Serial.println("TESTING");
            // Inizializza i test
            break;
            
        case MowerState::ERROR:
            Serial.println("ERROR");
            mower_.stopBlades();
            mower_.stopDriveMotors();
            // Attiva eventuali indicatori di errore
            break;
    }
}

void MowerStateMachine::onExitState(MowerState oldState) {
    // Azioni da eseguire all'uscita da uno stato
    Serial.print("Uscito dallo stato: ");
    
    switch (oldState) {
        case MowerState::CHARGING:
            Serial.println("CHARGING");
            mower_.setCharging(false);
            break;
            
        case MowerState::SLEEP:
            Serial.println("SLEEP");
            // Disattiva la modalità a basso consumo
            break;
            
        case MowerState::ROS_CONTROL:
            Serial.println("ROS_CONTROL");
            // Disconnessione da ROS
            break;
            
        case MowerState::TESTING:
            Serial.println("TESTING");
            // Pulizia dopo i test
            break;
            
        default:
            Serial.println(static_cast<int>(oldState));
            break;
    }
}

void MowerStateMachine::onUpdateState() {
    // Aggiorna i sensori del tosaerba
    mower_.updateSensors();
    
    // Controlla le condizioni di emergenza
    if (isEmergencyStopActive() && currentState_ != MowerState::EMERGENCY_STOP) {
        requestStateChange(MowerState::EMERGENCY_STOP);
        return;
    }
    
    // Gestione degli aggiornamenti di stato specifici
    switch (currentState_) {
        case MowerState::IDLE:
            handleIdle();
            break;
            
        case MowerState::MOWING:
            handleMowing();
            
            // Controlla se è necessario andare in carica
            if (mower_.isBatteryLow()) {
                requestStateChange(MowerState::DOCKING);
            }
            // Controlla se è stato rilevato un bordo
            else if (mower_.isBorderDetected()) {
                requestStateChange(MowerState::BORDER_DETECTED);
            }
            break;
            
        case MowerState::DOCKING:
            handleDocking();
            
            // Se la batteria è sufficientemente carica, torna a tagliare
            if (mower_.isBatteryCharged()) {
                requestStateChange(MowerState::MOWING);
            }
            // Se è collegato al caricabatterie, inizia la ricarica
            else if (mower_.isDocked()) {
                requestStateChange(MowerState::CHARGING);
            }
            break;
            
        case MowerState::CHARGING:
            handleCharging();
            
            // Se la batteria è completamente carica, torna in IDLE
            if (mower_.isBatteryFull()) {
                requestStateChange(MowerState::IDLE);
            }
            // Se viene rimosso dal caricabatterie, torna in DOCKING
            else if (!mower_.isDocked()) {
                requestStateChange(MowerState::DOCKING);
            }
            break;
            
        case MowerState::MANUAL_CONTROL:
            handleManualControl();
            break;
            
        case MowerState::PAUSED:
            handlePaused();
            break;
            
        case MowerState::SLEEP:
            handleSleep();
            
            // Esci dalla modalità sleep se la batteria è sufficientemente carica
            if (mower_.isBatteryCharged() && mower_.isDocked()) {
                requestStateChange(MowerState::CHARGING);
            }
            break;
            
        case MowerState::RAIN_DELAY:
            handleRainDelay();
            
            // Controlla se il ritardo per la pioggia è terminato
            if (millis() - stateStartTime_ > (24 * 60 * 60 * 1000UL)) { // 24 ore
                requestStateChange(MowerState::IDLE);
            }
            break;
            
        case MowerState::MAINTENANCE_NEEDED:
            handleMaintenanceNeeded();
            // Richiede intervento manuale per uscire da questo stato
            break;
            
        case MowerState::ROS_CONTROL:
            handleROSControl();
            // La logica di controllo è gestita da ROS
            break;
            
        case MowerState::EMERGENCY_STOP:
            handleEmergencyStop();
            // Richiede un reset manuale per uscire da questo stato
            break;
            
        case MowerState::BORDER_DETECTED:
            handleBorderDetected();
            
            // Dopo aver gestito il bordo, torna a tagliare
            if (!mower_.isBorderDetected()) {
                requestStateChange(MowerState::MOWING);
            }
            break;
            
        case MowerState::LIFTED:
            handleLifted();
            
            // Se il tosaerba viene riposizionato, torna in IDLE
            if (!mower_.isLifted()) {
                requestStateChange(MowerState::IDLE);
            }
            break;
            
        case MowerState::TESTING:
            handleTesting();
            // La logica di test è gestita internamente
            break;
            
        case MowerState::ERROR:
            handleError();
            // Richiede un reset manuale per uscire da questo stato
            break;
    }
}

bool MowerStateMachine::isTransitionAllowed(MowerState from, MowerState to) const {
    // Transizioni di emergenza hanno sempre la priorità
    if (to == MowerState::EMERGENCY_STOP) {
        return true;
    }
    
    // Transizioni dallo stato di emergenza
    if (from == MowerState::EMERGENCY_STOP) {
        return (to == MowerState::IDLE);
    }
    
    // Transizioni dallo stato di errore
    if (from == MowerState::ERROR) {
        return (to == MowerState::IDLE);
    }
    
    // Transizioni specifiche per ogni stato
    switch (from) {
        case MowerState::IDLE:
            return (to == MowerState::MOWING) ||
                   (to == MowerState::DOCKING) ||
                   (to == MowerState::MANUAL_CONTROL) ||
                   (to == MowerState::TESTING) ||
                   (to == MowerState::SLEEP) ||
                   (to == MowerState::PAUSED);
                   
        case MowerState::MOWING:
            return (to == MowerState::IDLE) ||
                   (to == MowerState::DOCKING) ||
                   (to == MowerState::BORDER_DETECTED) ||
                   (to == MowerState::LIFTED) ||
                   (to == MowerState::PAUSED) ||
                   (to == MowerState::MAINTENANCE_NEEDED) ||
                   (to == MowerState::RAIN_DELAY);
                   
        case MowerState::DOCKING:
            return (to == MowerState::CHARGING) ||
                   (to == MowerState::IDLE);
                   
        case MowerState::CHARGING:
            return (to == MowerState::IDLE) ||
                   (to == MowerState::MOWING);
                   
        case MowerState::MANUAL_CONTROL:
            return (to == MowerState::IDLE) ||
                   (to == MowerState::PAUSED);
                   
        case MowerState::BORDER_DETECTED:
            return (to == MowerState::MOWING) ||
                   (to == MowerState::IDLE) ||
                   (to == MowerState::DOCKING);
                   
        case MowerState::LIFTED:
            return (to == MowerState::IDLE);
                   
        case MowerState::TESTING:
            return (to == MowerState::IDLE);
            
        case MowerState::PAUSED:
            return (to == MowerState::IDLE) ||
                   (to == MowerState::MOWING) ||
                   (to == MowerState::MANUAL_CONTROL);
                   
        case MowerState::SLEEP:
            return (to == MowerState::IDLE);
            
        case MowerState::RAIN_DELAY:
            return (to == MowerState::IDLE) ||
                   (to == MowerState::MOWING);
                   
        case MowerState::MAINTENANCE_NEEDED:
            return (to == MowerState::IDLE);
            
        case MowerState::ROS_CONTROL:
            return (to == MowerState::IDLE) ||
                   (to == MowerState::MANUAL_CONTROL);
            
        default:
            return false;
    }
}
bool MowerStateMachine::isEmergencyStopActive() const {
    // Controlla le condizioni di emergenza
    if (mower_.isLifted()) {
        return true;
    }
    
    // Aggiungi qui altre condizioni di emergenza
    
    return emergencyStopActive_;
}

// Implementazione degli handler di stato
void MowerStateMachine::handleIdle() {
    // Nessuna azione specifica nello stato IDLE
}

void MowerStateMachine::handleMowing() {
    // Controlla se è stato rilevato un bordo
    if (mower_.isBorderDetected()) {
        requestStateChange(MowerState::BORDER_DETECTED);
        return;
    }
    
    // Controlla se il tosaerba è stato sollevato
    if (mower_.isLifted()) {
        requestStateChange(MowerState::LIFTED);
        return;
    }
    
    // Logica di taglio qui
}

void MowerStateMachine::handleDocking() {
    // Logica di aggancio alla stazione di ricarica
    // Per semplicità, dopo 5 secondi passiamo allo stato di ricarica
    if (millis() - stateStartTime_ > 5000) {
        requestStateChange(MowerState::CHARGING);
    }
}

void MowerStateMachine::handleCharging() {
    // Se non siamo più in carica, torniamo in IDLE
    if (!mower_.isCharging()) {
        requestStateChange(MowerState::IDLE);
    }
}

void MowerStateMachine::handleManualControl() {
    // Nessuna azione specifica, il controllo è manuale
}

void MowerStateMachine::handleEmergencyStop() {
    // Attendi il reset manuale
}

void MowerStateMachine::handleBorderDetected() {
    // Dopo aver gestito il bordo, torna a tagliare
    if (millis() - stateStartTime_ > 2000) { // Dopo 2 secondi
        requestStateChange(MowerState::MOWING);
    }
}

void MowerStateMachine::handleLifted() {
    // Attendi che il tosaerba venga riposizionato
    if (!mower_.isLifted()) {
        requestStateChange(MowerState::IDLE);
    }
}

void MowerStateMachine::handleTesting() {
    // Stato per i test
    static bool firstTime = true;
    if (firstTime) {
        mower_.logEvent("Modalità test attiva");
        firstTime = false;
    }
}

void MowerStateMachine::handlePaused() {
    // Nessuna azione specifica, il tosaerba è in pausa
    static bool firstTime = true;
    if (firstTime) {
        mower_.logEvent("Tosaerba in pausa");
        firstTime = false;
    }
}

void MowerStateMachine::handleSleep() {
    // Nessuna azione specifica, il tosaerba è in modalità risparmio energetico
    static bool firstTime = true;
    if (firstTime) {
        mower_.logEvent("Modalità risparmio energetico attiva");
        firstTime = false;
    }
}

void MowerStateMachine::handleRainDelay() {
    // Nessuna azione specifica, il tosaerba è in pausa per pioggia
    static bool firstTime = true;
    if (firstTime) {
        mower_.logEvent("Pausa per pioggia attiva");
        firstTime = false;
    }
}

void MowerStateMachine::handleMaintenanceNeeded() {
    // Nessuna azione specifica, il tosaerba richiede manutenzione
    static bool firstTime = true;
    if (firstTime) {
        mower_.logEvent("Manutenzione richiesta!");
        firstTime = false;
    }
}

void MowerStateMachine::handleROSControl() {
    // Nessuna azione specifica, il controllo è remoto via ROS
    static bool firstTime = true;
    if (firstTime) {
        mower_.logEvent("Controllo remoto ROS attivo");
        firstTime = false;
    }
}

void MowerStateMachine::handleError() {
    // Nessuna azione specifica, il tosaerba è in stato di errore
    static bool firstTime = true;
    if (firstTime) {
        mower_.logEvent("Errore rilevato!");
        firstTime = false;
    }
}
