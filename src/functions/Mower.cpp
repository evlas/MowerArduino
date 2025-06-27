#include "Mower.h"
#include "MowerStateMachine.h"
#include "../config.h"  // Per le costanti di configurazione

// ---------------------------------------------------------------------------
// Costruttore
Mower::Mower() : 
    currentState_(State::IDLE), 
    emergencyStopActive_(false), 
    bladesRunning_(false), 
    leftMotorSpeed_(0), 
    rightMotorSpeed_(0),
    bladeSpeed_(0.0f),
    charging_(false), 
    batteryLevel_(100.0f),
    navigationMode_(NavigationMode::STOPPED),
    stateMachine_(nullptr), 
    lastSensorUpdate_(0),
    lifted_(false),
    borderDetected_(false),
    collisionDetected_(false),
    docked_(false) {
    // L'inizializzazione del seriale e del generatore di numeri casuali 
    // è stata spostata nel metodo begin()
}

void Mower::begin() {
}

// ---------------------------------------------------------------------------
// Metodo update da chiamare nel loop principale
// ---------------------------------------------------------------------------
void Mower::update() {
    // Aggiorna i sensori con la frequenza desiderata
    updateSensors();

    // Aggiorna la macchina a stati se presente
    if (stateMachine_) {
        stateMachine_->update();
    }

    // Ulteriori controlli di sicurezza
    checkSafety();

    // Puoi aggiungere logging o debug qui se necessario
}



// ===== Implementazione metodi motori =====
void Mower::startBlades() {
    if (emergencyStopActive_) {
    
        return;
    }
    
    if (!bladesRunning_) {
        // Assicuriamoci che la velocità sia > 0
        if (bladeSpeed_ < 0.1f) {

            bladeSpeed_ = 95.0f;  // Imposta una velocità di default

        }
        
        bladesRunning_ = true;

        
        // Debug: stampa lo stato corrente
        Serial.print("[DEBUG] startBlades: bladeSpeed_=");
        Serial.print(bladeSpeed_);
        Serial.print("%, bladesRunning_=");
        Serial.println(bladesRunning_ ? "true" : "false");
    }
}

void Mower::setBladeSpeed(float speed) {
    if (emergencyStopActive_) {

        return;
    }
    
    float oldSpeed = bladeSpeed_;
    bladeSpeed_ = constrain(speed, 0.0f, 100.0f);
    
    // Debug: stampa i valori per il debug
    Serial.print("[DEBUG] setBladeSpeed: old=");
    Serial.print(oldSpeed);
    Serial.print("%, new=");
    Serial.print(bladeSpeed_);
    Serial.print("%, bladesRunning_=");
    Serial.println(bladesRunning_ ? "true" : "false");
    
    if (bladesRunning_ && (abs(oldSpeed - bladeSpeed_) > 0.1f)) {

    } else if (!bladesRunning_) {

    }
}

void Mower::stopBlades() {
    if (bladesRunning_) {
        bladesRunning_ = false;

    }
}

void Mower::setLeftMotorSpeed(float speed) {
    if (emergencyStopActive_) {
        leftMotorSpeed_ = 0;
        logEvent("Impossibile muovere i motori: EMERGENCY STOP attivo");
    } else {
        // Se la velocità richiesta è diversa da zero, usa DEFAULT_MOTOR_SPEED
        float effectiveSpeed = (speed != 0) ? 
            ((speed > 0) ? DEFAULT_MOTOR_SPEED : -DEFAULT_MOTOR_SPEED) : 0;
            
        leftMotorSpeed_ = constrain(effectiveSpeed, -100.0, 100.0);

    }
}

void Mower::setRightMotorSpeed(float speed) {
    if (emergencyStopActive_) {
        rightMotorSpeed_ = 0;
        logEvent("Impossibile muovere i motori: EMERGENCY STOP attivo");
    } else {
        // Se la velocità richiesta è diversa da zero, usa DEFAULT_MOTOR_SPEED
        float effectiveSpeed = (speed != 0) ? 
            ((speed > 0) ? DEFAULT_MOTOR_SPEED : -DEFAULT_MOTOR_SPEED) : 0;
            
        rightMotorSpeed_ = constrain(effectiveSpeed, -100.0, 100.0);

    }
}

void Mower::stopDriveMotors() {
    leftMotorSpeed_ = 0;
    rightMotorSpeed_ = 0;

}

void Mower::stopMotors() {
    stopBlades();
    stopDriveMotors();

}

void Mower::startDriveMotors() {

}

// ===== Metodi di navigazione =====
void Mower::setNavigationMode(NavigationMode mode) {
    navigationMode_ = mode;

    
    switch (mode) {
        case NavigationMode::STOPPED:
            stopMotors();
            break;
        case NavigationMode::RANDOM:
            startDriveMotors();
            break;
        case NavigationMode::MANUAL:
            stopMotors();
            break;
        case NavigationMode::DOCKING:
            startDriveMotors();
            break;
        case NavigationMode::BORDER_FOLLOW:
            startDriveMotors();
            break;
    }
}

const char* Mower::navigationModeToString(NavigationMode mode) const {
    switch (mode) {
        case NavigationMode::STOPPED: return "FERMO";
        case NavigationMode::RANDOM: return "CASUALE";
        case NavigationMode::MANUAL: return "MANUALE";
        case NavigationMode::DOCKING: return "AGGANCIO";
        case NavigationMode::BORDER_FOLLOW: return "SEGUI BORDO";
        case NavigationMode::SPIRAL: return "SPIRALE";
        case NavigationMode::LAWN_STRIPES: return "STRISCE";
        case NavigationMode::ZONE: return "ZONA";
        default: return "SCONOSCIUTO";
    }
}

// ===== Metodi di sistema =====
void Mower::enableCharging(bool enable) {
    charging_ = enable;

}

bool Mower::isCharging() const {
    return charging_;
}

bool Mower::isLifted() const {
    return lifted_;
}

bool Mower::isBorderDetected() const {
    return borderDetected_;
}

bool Mower::isCollisionDetected() const {
    return collisionDetected_;
}

bool Mower::isBatteryCritical() const {
    // Considera la batteria critica sotto il 10%
    return (batteryLevel_ < 10.0f);
}

bool Mower::isBatteryLow() const {
    // Considera la batteria bassa sotto il 20%
    return (batteryLevel_ < 20.0f);
}

bool Mower::isBatteryCharged() const {
    // Considera la batteria sufficientemente carica sopra l'80%
    return (batteryLevel_ > 80.0f);
}

bool Mower::isBatteryFull() const {
    // Considera la batteria completamente carica sopra il 95%
    return (batteryLevel_ > 95.0f);
}

bool Mower::isDocked() const {
    return docked_ && charging_;
}

void Mower::logStateChange(MowerStateMachine::MowerState from, MowerStateMachine::MowerState to) {
    String fromStr = "";
    String toStr = "";
    
    // Converti gli stati in stringhe leggibili
    switch (from) {
        case MowerStateMachine::MowerState::IDLE: fromStr = "IDLE"; break;
        case MowerStateMachine::MowerState::MOWING: fromStr = "MOWING"; break;
        case MowerStateMachine::MowerState::DOCKING: fromStr = "DOCKING"; break;
        case MowerStateMachine::MowerState::CHARGING: fromStr = "CHARGING"; break;
        case MowerStateMachine::MowerState::MANUAL_CONTROL: fromStr = "MANUAL_CONTROL"; break;
        case MowerStateMachine::MowerState::EMERGENCY_STOP: fromStr = "EMERGENCY_STOP"; break;
        case MowerStateMachine::MowerState::BORDER_DETECTED: fromStr = "BORDER_DETECTED"; break;
        case MowerStateMachine::MowerState::LIFTED: fromStr = "LIFTED"; break;
        case MowerStateMachine::MowerState::TESTING: fromStr = "TESTING"; break;
        case MowerStateMachine::MowerState::PAUSED: fromStr = "PAUSED"; break;
        case MowerStateMachine::MowerState::SLEEP: fromStr = "SLEEP"; break;
        case MowerStateMachine::MowerState::RAIN_DELAY: fromStr = "RAIN_DELAY"; break;
        case MowerStateMachine::MowerState::MAINTENANCE_NEEDED: fromStr = "MAINTENANCE_NEEDED"; break;
        case MowerStateMachine::MowerState::ROS_CONTROL: fromStr = "ROS_CONTROL"; break;
        case MowerStateMachine::MowerState::ERROR: fromStr = "ERROR"; break;
    }
    
    switch (to) {
        case MowerStateMachine::MowerState::IDLE: toStr = "IDLE"; break;
        case MowerStateMachine::MowerState::MOWING: toStr = "MOWING"; break;
        case MowerStateMachine::MowerState::DOCKING: toStr = "DOCKING"; break;
        case MowerStateMachine::MowerState::CHARGING: toStr = "CHARGING"; break;
        case MowerStateMachine::MowerState::MANUAL_CONTROL: toStr = "MANUAL_CONTROL"; break;
        case MowerStateMachine::MowerState::EMERGENCY_STOP: toStr = "EMERGENCY_STOP"; break;
        case MowerStateMachine::MowerState::BORDER_DETECTED: toStr = "BORDER_DETECTED"; break;
        case MowerStateMachine::MowerState::LIFTED: toStr = "LIFTED"; break;
        case MowerStateMachine::MowerState::TESTING: toStr = "TESTING"; break;
        case MowerStateMachine::MowerState::PAUSED: toStr = "PAUSED"; break;
        case MowerStateMachine::MowerState::SLEEP: toStr = "SLEEP"; break;
        case MowerStateMachine::MowerState::RAIN_DELAY: toStr = "RAIN_DELAY"; break;
        case MowerStateMachine::MowerState::MAINTENANCE_NEEDED: toStr = "MAINTENANCE_NEEDED"; break;
        case MowerStateMachine::MowerState::ROS_CONTROL: toStr = "ROS_CONTROL"; break;
        case MowerStateMachine::MowerState::ERROR: toStr = "ERROR"; break;
    }
    

}

// ===== Gestione stato =====
void Mower::setState(State newState) {
    if (currentState_ == newState) {
        return; // Nessun cambiamento di stato
    }
    
    State oldState = currentState_;
    currentState_ = newState;
    
    // Log del cambiamento di stato

    
    // Notifica la macchina a stati del cambiamento
    if (stateMachine_ != nullptr) {
        // Converti lo stato Mower::State in MowerStateMachine::MowerState
        MowerStateMachine::MowerState newMowerState = static_cast<MowerStateMachine::MowerState>(newState);
        stateMachine_->transitionToState(newMowerState);
    }
}

// ===== Azioni =====
void Mower::startMowing() {

    setState(State::MOWING);
    setNavigationMode(NavigationMode::RANDOM);
    // Prima impostiamo la velocità, poi avviamo le lame
    setBladeSpeed(95.0f);  // Imposta la velocità delle lame al 95%
    startBlades();
}

void Mower::startDocking() {

    setState(State::DOCKING);
    setNavigationMode(NavigationMode::DOCKING);
    stopBlades();
}

void Mower::updateSensors() {
    // Simula l'aggiornamento dei sensori
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdate_ > 1000) { // Aggiorna ogni secondo
        lastSensorUpdate_ = currentTime;
        
        // Simula il consumo della batteria in base all'attività
        if (bladesRunning_) {
            // Consumo delle lame
            batteryLevel_ -= 0.1f;
        }
        
        // Consumo dei motori di trazione (più veloce quando si muovono)
        if (leftMotorSpeed_ != 0 || rightMotorSpeed_ != 0) {
            float avgSpeed = (abs(leftMotorSpeed_) + abs(rightMotorSpeed_)) / 2.0f;
            batteryLevel_ -= (avgSpeed * 0.002f); // Consumo proporzionale alla velocità
        }
        
        // Simula la ricarica quando collegato alla stazione di ricarica
        if (isDocked() && charging_) {
            batteryLevel_ += 1.0f; // Ricarica veloce quando agganciato
        } else if (charging_) {
            batteryLevel_ += 0.2f; // Ricarica lenta quando solo collegato
        }
        
        // Mantieni la batteria tra 0 e 100%
        batteryLevel_ = constrain(batteryLevel_, 0.0f, 100.0f);
        
        // Log del livello della batteria ogni 10 secondi
        static unsigned long lastBatteryLog = 0;
        if (currentTime - lastBatteryLog > 10000) {
            lastBatteryLog = currentTime;

        }
        // Verifica la sicurezza
        checkSafety();
        lastSensorUpdate_ = currentTime;
    }
}

void Mower::checkSafety() {
    // Verifica le condizioni di sicurezza e agisci di conseguenza
    bool safetyTriggered = false;
    
    // 1. Verifica se il tosaerba è stato sollevato
    if (isLifted()) {
        if (currentState_ != State::LIFTED && currentState_ != State::EMERGENCY_STOP) {

            if (stateMachine_ != nullptr) {
                stateMachine_->requestStateChange(MowerStateMachine::MowerState::LIFTED);
            }
            safetyTriggered = true;
        }
    }
    
    // 2. Verifica la presenza di ostacoli
    if (isCollisionDetected()) {

        stopMotors();
        // Aggiungi qui la logica per evitare l'ostacolo
        delay(500); // Breve pausa
        safetyTriggered = true;
    }
    
    // 3. Verifica il livello critico della batteria
    if (isBatteryCritical() && 
        currentState_ != State::DOCKING &&
        currentState_ != State::CHARGING) {

        if (stateMachine_ != nullptr) {
            stateMachine_->requestStateChange(MowerStateMachine::MowerState::DOCKING);
        }
        safetyTriggered = true;
    }
    
    // 4. Verifica la temperatura (simulata)
    if (random(1000) < 2) { // 0.2% di probabilità di surriscaldamento

        // Riduci la velocità dei motori per raffreddamento
        leftMotorSpeed_ *= 0.8f;
        rightMotorSpeed_ *= 0.8f;
    }
    
    // 5. Se è stata attivata una condizione di sicurezza, registrala
    if (safetyTriggered) {
        // Aggiungi qui eventuali azioni aggiuntive da eseguire quando viene rilevato un problema di sicurezza
    }
}

// ===== Gestione sensori =====

// ===== Azioni =====
void Mower::emergencyStop() {
    if (!emergencyStopActive_) {
        emergencyStopActive_ = true;
        stopBlades();
        stopDriveMotors();
        currentState_ = State::EMERGENCY_STOP;

        
        // Notifica la macchina a stati del cambiamento
        if (stateMachine_ != nullptr) {
            stateMachine_->transitionToState(MowerStateMachine::MowerState::EMERGENCY_STOP);
        }
    }
}

void Mower::resetEmergencyStop() {
    if (emergencyStopActive_) {
        emergencyStopActive_ = false;

        
        // Ripristina lo stato precedente se disponibile
        if (stateMachine_ != nullptr) {
            // Passa allo stato IDLE dopo un emergency stop
            stateMachine_->transitionToState(MowerStateMachine::MowerState::IDLE);
        }
    }
}

void Mower::handleBorder() {

    // Simula una rotazione lontano dal bordo
    setLeftMotorSpeed(DEFAULT_MOTOR_SPEED);
    setRightMotorSpeed(DEFAULT_MOTOR_SPEED);

    delay(500);
    stopDriveMotors();
}

void Mower::handleObstacle() {

    
    // 1. Ferma i motori
    stopDriveMotors();
    delay(100);
    
    // 2. Vai indietro per 1 secondo
    setLeftMotorSpeed(-DEFAULT_MOTOR_SPEED);
    setRightMotorSpeed(-DEFAULT_MOTOR_SPEED);

    delay(1000);
    stopDriveMotors();
    
    // 3. Ruota a sinistra per evitare l'ostacolo
    setLeftMotorSpeed(-DEFAULT_MOTOR_SPEED);
    setRightMotorSpeed(DEFAULT_MOTOR_SPEED);

    delay(500); // Ruota per 500ms
    stopDriveMotors();
    
    // 4. Riprendi il movimento in avanti
    setLeftMotorSpeed(DEFAULT_MOTOR_SPEED);
    setRightMotorSpeed(DEFAULT_MOTOR_SPEED);

}

void Mower::printStatus() const {
    // Intestazione
    Serial.println("\n╔══════════════════════════════╗");
    Serial.println("║       STATO TOSAERBA        ║");
    Serial.println("╠══════════════════════════════╣");
    
    // Stato corrente
    Serial.print("║ Stato: ");
    Serial.print(stateToString(currentState_));
    // Allinea a destra
    for (int i = 0; i < 23 - strlen(stateToString(currentState_)); i++) {
        Serial.print(" ");
    }
    Serial.println("║");
    
    // Livello batteria con barra di avanzamento
    Serial.print("║ Batteria: ");
    int bars = map(constrain(batteryLevel_, 0, 100), 0, 100, 0, 20);
    for (int i = 0; i < 20; i++) {
        if (i < bars) {
            Serial.print("█");
        } else {
            Serial.print("░");
        }
    }
    Serial.print(" ");
    if (batteryLevel_ < 10) Serial.print(" ");
    if (batteryLevel_ < 100) Serial.print(" ");
    Serial.print(batteryLevel_, 1);
    Serial.println("% ║");
    
    // Stato lame
    Serial.print("║ Lame: ");
    Serial.print(bladesRunning_ ? "ACCESE " : "SPENTE ");
    if (bladesRunning_) {
        Serial.print("[");
        int bladeBars = map(bladeSpeed_, 0, 100, 0, 10);
        for (int i = 0; i < 10; i++) {
            Serial.print(i < bladeBars ? "≡" : " ");
        }
        Serial.print("] ");
        if (bladeSpeed_ < 100) Serial.print(" ");
        if (bladeSpeed_ < 10) Serial.print(" ");
        Serial.print(bladeSpeed_, 0);
        Serial.println("%   ║");
        
        // Debug: verifica incoerenza tra stato lame e velocità
        if (bladeSpeed_ < 1.0f) {
            Serial.println("║ [WARN] Lame accese ma velocità a 0%! ║");
        }
    } else {
        Serial.println("                    ║");
    }
    
    // Stato motori
    Serial.print("║ Motori: SX=");
    printPaddedNumber(leftMotorSpeed_);
    Serial.print("% DX=");
    printPaddedNumber(rightMotorSpeed_);
    Serial.println("%   ║");
    
    // Stato alimentazione
    Serial.print("║ Alimentazione: ");
    if (charging_) {
        if (isDocked()) {
            Serial.print("AGGANCIATO alla base");
        } else {
            Serial.print("COLLEGATO");
        }
    } else {
        Serial.print("A BATTERIA");
    }
    Serial.println("    ║");
    
    // Stato sicurezza
    Serial.print("║ Sicurezza: ");
    if (emergencyStopActive_) {
        Serial.print("EMERGENZA ATTIVA!");
    } else if (isLifted()) {
        Serial.print("SOLLEVATO!");
    } else if (isBorderDetected()) {
        Serial.print("BORDO RILEVATO");
    } else if (isCollisionDetected()) {
        Serial.print("OSTACOLO RILEVATO");
    } else {
        Serial.print("TUTTO OK");
    }
    
    // Allinea a destra
    int safetyLen = 0;
    if (emergencyStopActive_) safetyLen = 15;
    else if (isLifted()) safetyLen = 10;
    else if (isBorderDetected()) safetyLen = 14;
    else if (isCollisionDetected()) safetyLen = 16;
    else safetyLen = 8;
    
    for (int i = 0; i < 18 - safetyLen; i++) {
        Serial.print(" ");
    }
    Serial.println("║");
    
    // Piè di pagina
    Serial.println("╚══════════════════════════════╝\n");
}

// Funzione di supporto per stampare numeri con riempimento
void Mower::printPaddedNumber(int number) const {
    if (number >= 0) {
        if (number < 100) Serial.print(" ");
        if (number < 10) Serial.print(" ");
        Serial.print(number);
    } else {
        if (number > -100) Serial.print(" ");
        if (number > -10) Serial.print(" ");
        Serial.print(number);
    }
}

const char* Mower::stateToString(State s) const {
    switch (s) {
        case State::IDLE: return "IN ATTESA";
        case State::MOWING: return "TAGLIO";
        case State::DOCKING: return "RITORNO BASE";
        case State::CHARGING: return "IN CARICA";
        case State::MANUAL_CONTROL: return "CONTROLLO MANUALE";
        case State::EMERGENCY_STOP: return "EMERGENZA";
        case State::BORDER_DETECTED: return "BORDO RILEVATO";
        case State::LIFTED: return "SOLLEVATO";
        case State::TESTING: return "TEST";
        case State::PAUSED: return "IN PAUSA";
        case State::SLEEP: return "IN RIPOSO";
        case State::RAIN_DELAY: return "RITARDO PIOGGIA";
        case State::MAINTENANCE_NEEDED: return "MANUTENZIONE";
        case State::ROS_CONTROL: return "CONTROLLO ROS";
        case State::ERROR: return "ERRORE";
        default: return "SCONOSCIUTO";
    }
}

void Mower::setStateMachine(MowerStateMachine* stateMachine) {
    stateMachine_ = stateMachine;

}

void Mower::processSerialCommand(const String& command) {
    if (command.startsWith("LIFT ")) {
        setLifted(command.endsWith("1"));

    } 
    else if (command.startsWith("BORDER ")) {
        setBorderDetected(command.endsWith("1"));

    }
    else if (command.startsWith("COLLISION ")) {
        setCollisionDetected(command.endsWith("1"));

    }
    else if (command.startsWith("DOCK ")) {
        setDocked(command.endsWith("1"));

    }
    else if (command.startsWith("CHARGE ")) {
        setCharging(command.endsWith("1"));

    }
    else if (command == "STATUS") {
        printStatus();
    }
    else if (command == "HELP") {








    }
    else {

    }
}
