#include "Mower.h"
#include "../config.h"  // Per le costanti di configurazione
#include "../pin_config.h"
#include <Arduino.h>
#include "../LCD/LCDMenu.h"

// Includi tutti gli stati
#include "../states/IdleState.h"
#include "../states/MowingState.h"
#include "../states/DockingState.h"
#include "../states/ChargingState.h"
#include "../states/EmergencyStopState.h"
#include "../states/ManualControlState.h"
#include "../states/ErrorState.h"

// ---------------------------------------------------------------------------
// Costruttore
Mower::Mower(LCDMenu& lcdMenu) : 
    currentState_(nullptr),
    emergencyStopActive_(false), 
    bladesRunning_(false), 
    leftMotorSpeed_(0), 
    rightMotorSpeed_(0),
    bladeSpeed_(0.0f),
    charging_(false), 
    batteryLevel_(100.0f),
    navigationMode_(NavigationMode::RANDOM),
    lastSensorUpdate_(0),
    lifted_(false),
    borderDetected_(false),
    collisionDetected_(false),
    docked_(false),
    batteryCritical_(false),
    batteryLow_(false),
    batteryCharged_(false),
    batteryFull_(false),
    // Inizializza i motori con i pin corretti
    leftMotor(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_REVERSE),
    rightMotor(MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_REVERSE),
    bladeMotor(MOTOR_BLADE_PWM_PIN, MOTOR_BLADE_DIR_PIN, MOTOR_BLADE_REVERSE),
    // Inizializza il riferimento a LCDMenu
    lcdMenu(lcdMenu),
    // Inizializza i sensori con i costruttori predefiniti
    // I pin verranno impostati nel metodo initializeComponents()
    batterySensor(),
    bumpSensors(),
    ultrasonicSensors(),
    imu(),
    rainSensor()
{
    // Inizializza i relè
    motorRelay.begin(RELAY_MOTORS_PIN);
    chargingRelay.begin(RELAY_CHARGING_PIN);
}

void Mower::begin() {
    // Initialize serial communication for debug if enabled
    #ifdef DEBUG_MODE
        SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD);
//        DEBUG_PRINTLN("Debug serial initialized");
    #endif
    
    // Il display LCD è gestito dalla classe LCDMenu
    
    // Inizializza il sensore di batteria
    batterySensor.begin();
    DEBUG_PRINTLN("Battery sensor initialized");
    
    // Inizializza i sensori di urto
    bumpSensors.begin();
    
    // Inizializza i sensori ad ultrasuoni
    ultrasonicSensors.begin();
    
    // Inizializza l'IMU
    imu.begin();
    DEBUG_PRINTLN("IMU inizializzata");
    
    // Inizializza il sensore di pioggia
    rainSensor.begin();
    
    // Inizializza il cicalino
    buzzer.begin();
    
    // Imposta lo stato iniziale
    setState(State::IDLE);

    Wire.begin();

    // Initialize LCD display
    lcdMenu.print("Mower");
    lcdMenu.setCursor(0, 1);
    lcdMenu.print("Starting...");
    
    // Initialize actuators
    motorRelay.begin(RELAY_MOTORS_PIN);
    chargingRelay.begin(RELAY_CHARGING_PIN);
       
    // Initialize battery sensor
    batterySensor.begin();

    // Initialize bump sensors
    bumpSensors.begin();
    
    // Initialize ultrasonic sensors
    ultrasonicSensors.begin();
    
    // Initialize IMU
    imu.begin();

    #ifdef ENABLE_GPS
        gps.begin();
    #endif

    // Initialize rain sensor
    rainSensor.begin();
    
    // Initialize other components
    stopMotors();

    delay(5000);

    lcdMenu.clear();
    lcdMenu.setCursor(0, 0);
    lcdMenu.print("Mower");
    lcdMenu.setCursor(0, 1);
    lcdMenu.print("Started!");

    // Play startup sound
    buzzer.startupSound();
    
}

// ---------------------------------------------------------------------------
// Metodo update da chiamare nel loop principale
// ---------------------------------------------------------------------------
void Mower::update() {
    // Aggiorna i sensori
    updateSensors();
    
    // Controlla la sicurezza (ostacoli, sollevamento, ecc.)
    checkSafety();
    
    // Gestisci la transizione di stato se necessario
    if (nextState_ != nullptr && nextState_ != currentState_) {
        // Notifica l'uscita dallo stato corrente
        if (currentState_ != nullptr) {
            currentState_->exit(*this);
        }
        
        // Cambia stato
        MowerState* oldState = currentState_;
        currentState_ = nextState_;
        nextState_ = nullptr;
        
        // Notifica l'ingresso nel nuovo stato
        if (currentState_ != nullptr) {
            currentState_->enter(*this);
        }
    }
    
    // Aggiorna lo stato corrente
    if (currentState_ != nullptr) {
        currentState_->update(*this);
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
            bladeSpeed_ = 0.95f;  // Velocità di default (95%)
        }
        
        bladesRunning_ = true;
        
        // Avvia il motore e imposta la velocità
        int motorSpeed = bladeSpeed_ * 100;  // Converti in percentuale (0-100)
        bladeMotor.setSpeed(motorSpeed);
        
        // Debug: stampa lo stato corrente
        #ifdef DEBUG
            SERIAL_DEBUG.print("Blades started at speed: ");
            SERIAL_DEBUG.print(motorSpeed);
            SERIAL_DEBUG.println("%");
        #endif
    }
}

void Mower::setBladeSpeed(float speed) {
    // Assicurati che la velocità sia nel range corretto (0.0 - 100.0)
    bladeSpeed_ = constrain(speed, 0.0f, 100.0f);
    
    if (emergencyStopActive_) {
        bladeSpeed_ = 0;
        return;
    }
    
    float oldSpeed = bladeSpeed_;
    
    // Debug: stampa i valori per il debug
    #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.print("[DEBUG] setBladeSpeed: old=");
        SERIAL_DEBUG.print(oldSpeed);
        SERIAL_DEBUG.print("%, new=");
        SERIAL_DEBUG.print(bladeSpeed_ * 100);
        SERIAL_DEBUG.print("%, bladesRunning_=");
        SERIAL_DEBUG.println(bladesRunning_ ? "true" : "false");
    #endif
    
    if (bladesRunning_ && (abs(oldSpeed - bladeSpeed_) > 0.1f)) {
        // Se il motore è già in esecuzione e la velocità è cambiata in modo significativo
        int motorSpeed = bladeSpeed_ * 100;  // Converti in percentuale (0-100)
        bladeMotor.setSpeed(motorSpeed);
    } else if (!bladesRunning_ && bladeSpeed_ > 0.1f) {
        // Se il motore non è in esecuzione ma dovrebbe esserlo
        bladesRunning_ = true;
        int motorSpeed = bladeSpeed_ * 100;  // Converti in percentuale (0-100)
        bladeMotor.setSpeed(motorSpeed);
    } else if (bladeSpeed_ < 0.1f) {
        // Se la velocità è molto bassa, ferma le lame
        stopBlades();
    }
}

void Mower::stopBlades() {
    if (bladesRunning_) {
        bladesRunning_ = false;
        bladeMotor.setSpeed(0);  // Imposta la velocità a 0
        bladeSpeed_ = 0.0f;
        
        #ifdef DEBUG
            SERIAL_DEBUG.println("Blades stopped");
        #endif
    }
}

void Mower::setLeftMotorSpeed(float speed) {
    // Assicurati che la velocità sia nel range corretto (-100.0 - 100.0)
    leftMotorSpeed_ = constrain(speed, -100.0f, 100.0f);
    
    if (!emergencyStopActive_) {
        // Imposta la velocità del motore sinistro
        int motorSpeed = leftMotorSpeed_ * 100;  // Converti in percentuale (-100 a 100)
        leftMotor.setSpeed(motorSpeed);
        
        #ifdef DEBUG
        SERIAL_DEBUG.print("Left motor speed set to: ");
        SERIAL_DEBUG.println(motorSpeed);
        #endif
    }
}

void Mower::setRightMotorSpeed(float speed) {
    // Assicurati che la velocità sia nel range corretto (-100.0 - 100.0)
    rightMotorSpeed_ = constrain(speed, -100.0f, 100.0f);
    
    if (!emergencyStopActive_) {
        // Imposta la velocità del motore destro
        int motorSpeed = rightMotorSpeed_ * 100;  // Converti in percentuale (-100 a 100)
        rightMotor.setSpeed(motorSpeed);
        
        #ifdef DEBUG
        SERIAL_DEBUG.print("Right motor speed set to: ");
        SERIAL_DEBUG.println(motorSpeed);
        #endif
    }
}

void Mower::stopDriveMotors() {
    // Ferma entrambi i motori di trazione
    leftMotor.stop();
    rightMotor.stop();
    leftMotorSpeed_ = 0;
    rightMotorSpeed_ = 0;
    
    #ifdef DEBUG
    SERIAL_DEBUG.println("Drive motors stopped");
    #endif
}

void Mower::startDriveMotors() {
    // Riattiva i motori di trazione con le ultime velocità impostate
    if (!emergencyStopActive_) {
        setLeftMotorSpeed(leftMotorSpeed_);
        setRightMotorSpeed(rightMotorSpeed_);
        #ifdef DEBUG
        SERIAL_DEBUG.println("Drive motors started");
        #endif
    }
}

void Mower::stopMotors() {
    // Ferma tutti i motori
    stopBlades();
    stopDriveMotors();
    
    #ifdef DEBUG
    SERIAL_DEBUG.println("All motors stopped");
    #endif
}

void Mower::stopBuzzer() {
    buzzer.stop();
}

// Update method is already implemented above

void Mower::startMowing() {
    handleEvent(Event::START_MOWING);
}

void Mower::startDocking() {
    #ifdef DEBUG
    SERIAL_DEBUG.println("Starting docking procedure");
    #endif
    changeState(getDockingState());
}

void Mower::updateSensors() {
    // Aggiorna i sensori
    unsigned long currentTime = millis();
    
    // Aggiorna i sensori di base ogni ciclo
    // I sensi di urto e ultrasuoni non hanno bisogno di un metodo update esplicito
    // IMU update is handled internally by the IMU library
    
    // Aggiorna il sensore di pioggia (se presente)
    #ifdef RAIN_SENSOR_ENABLED
    rainSensor.update();
    
    // Gestisci lo stato di pioggia
    if (rainSensor.isRaining() && getState() != State::RAIN_DELAY) {
        handleEvent(Event::RAIN_DETECTED);
    }
    #endif
    
    // Aggiorna il GPS (se abilitato)
    #ifdef GPS_ENABLED
    gps.update();
    #endif
    
    // Aggiorna il sensore di batteria
    batterySensor.update();
    
    // Aggiorna lo stato della batteria
    batteryLevel_ = batterySensor.getBatteryPercentage();
    bool wasCharging = charging_;
    charging_ = batterySensor.isCharging();
    
    // Gestisci gli eventi di ricarica
    if (charging_ && !wasCharging) {
        handleEvent(Event::CHARGING_STARTED);
    } else if (!charging_ && wasCharging) {
        handleEvent(Event::CHARGING_STOPPED);
    }
    
    // Aggiorna gli stati della batteria
    bool wasBatteryLow = batteryLow_;
    bool wasBatteryCritical = batteryCritical_;
    bool wasBatteryCharged = batteryCharged_;
    bool wasBatteryFull = batteryFull_;
    
    batteryLow_ = (batteryLevel_ <= BATTERY_LOW_THRESHOLD);
    batteryCritical_ = (batteryLevel_ <= BATTERY_CRITICAL_THRESHOLD);
    batteryCharged_ = (batteryLevel_ >= BATTERY_CHARGED_THRESHOLD);
    batteryFull_ = batterySensor.isBatteryFull();
    
    // Gestisci gli eventi della batteria
    if (batteryLow_ && !wasBatteryLow) {
        handleEvent(Event::BATTERY_LOW);
    }
    
    if (batteryCritical_ && !wasBatteryCritical) {
        handleEvent(Event::BATTERY_CRITICAL);
    }
    
    if (batteryFull_ && !wasBatteryFull) {
        handleEvent(Event::BATTERY_FULL);
    }
    
    // Log del livello della batteria ogni 10 secondi
    static unsigned long lastBatteryLog = 0;
    if (currentTime - lastBatteryLog > 10000) { // Ogni 10 secondi
        lastBatteryLog = currentTime;
        #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.print(F("Battery: "));
        SERIAL_DEBUG.print(batterySensor.getBatteryPercentage());
        SERIAL_DEBUG.print(F("% "));
        SERIAL_DEBUG.print(batterySensor.getVoltage());
        SERIAL_DEBUG.print(F("V "));
        SERIAL_DEBUG.print(batterySensor.getCurrent());
        SERIAL_DEBUG.print(F("A "));
        SERIAL_DEBUG.println(charging_ ? "CHARGING" : "DISCHARGING");
        #endif
    }
    
    // Verifica se siamo agganciati alla stazione di ricarica
    bool wasDocked = docked_;
    #ifdef DOCK_DETECT_PIN
    docked_ = digitalRead(DOCK_DETECT_PIN) == LOW; // Assumi che il pin vada a LOW quando agganciato
    #else
    docked_ = false; // Se il pin non è definito, assumi che non siamo agganciati
    #endif
    
    if (docked_ && !wasDocked) {
        // Appena agganciato
        handleEvent(Event::DOCK_DETECTED);
    } else if (!docked_ && wasDocked) {
        // Appena sganciato
        handleEvent(Event::UNDOCK_DETECTED);
    }
    
    // Verifica i sensori di sollevamento
    bool wasLifted = lifted_;
    // Usa l'accelerometro per determinare se il robot è sollevato
    // Nota: implementa questa logica basandoti sulla tua IMU specifica
    // Esempio: 
    // float accelZ = imu.getAccelerationZ();
    // lifted_ = (accelZ < -9.0f); // Se l'accelerazione Z è verso l'alto
    
    if (lifted_ && !wasLifted) {
        handleEvent(Event::LIFT_DETECTED);
    }
    
    // Verifica gli urti
    bool leftBump = bumpSensors.isLeftBump();
    bool rightBump = bumpSensors.isRightBump();
    bool centerBump = bumpSensors.isCenterBump();
    
    // Considera una collisione se viene rilevato un urto da qualsiasi lato
    collisionDetected_ = (leftBump || rightBump || centerBump);
    
    // Verifica la presenza di ostacoli
    // Usa il metodo appropriato per ottenere la distanza dagli ostacoli
    // Esempio: 
    // float distance = ultrasonicSensors.getFrontDistance();
    // obstacleDetected_ = (distance > 0 && distance < OBSTACLE_DISTANCE_THRESHOLD);
    
    // Gestisci collisioni rilevate
    if (collisionDetected_) {
        handleEvent(Event::OBSTACLE_DETECTED);
        // Non inviare automaticamente l'evento di emergenza qui
        // L'evento di emergenza dovrebbe essere gestito dallo stato attuale
    }
}

// Gestione stato
void Mower::changeState(MowerState& newState) {
    // Se lo stato richiesto è lo stesso di quello corrente, non fare nulla
    if (currentState_ != nullptr && currentState_->getStateType() == newState.getStateType()) {
        return;
    }
    
    // Esci dallo stato corrente
    if (currentState_ != nullptr) {
        currentState_->exit(*this);
    }
    
    // Imposta il nuovo stato
    currentState_ = &newState;
    
    // Entra nel nuovo stato
    currentState_->enter(*this);
}

void Mower::handleEvent(Event event) {
    if (currentState_ != nullptr) {
        currentState_->handleEvent(*this, event);
    }
}

// Metodi di sistema
void Mower::emergencyStop() {
    if (!emergencyStopActive_) {
        emergencyStopActive_ = true;
        stopBlades();
        stopDriveMotors();
        
        // Usa il nuovo sistema di stati
        changeState(getEmergencyStopState());
        
        // Invia l'evento di emergenza
        handleEvent(Event::EMERGENCY_STOP);
    }
}

void Mower::resetEmergencyStop() {
    if (emergencyStopActive_) {
        emergencyStopActive_ = false;
        // Ripristina lo stato precedente o torna allo stato di idle
        changeState(getIdleState());
    }
}

void Mower::handleBorder() {
    // Invia l'evento di rilevamento del bordo
    handleEvent(Event::BORDER_DETECTED);
    
    #ifdef SERIAL_DEBUG
    Serial.println(F("Border detected - executing border handling"));
    #endif
    
    // Se abbiamo uno stato attivo, deleghiamo a lui la gestione
    if (currentState_) {
        currentState_->handleEvent(*this, Event::BORDER_DETECTED);
        return;
    }
    
    // Comportamento predefinito se non c'è uno stato attivo
    
    // 1. Ferma tutti i motori
    stopMotors();
    delay(100);
    
    // 2. Vai indietro per 30cm (valore da calibrare)
    setLeftMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.7f);
    setRightMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.7f);
    delay(1000); // Durata da calibrare in base alla velocità
    stopDriveMotors();
    
    // 3. Ruota di 135° a destra (per allontanarsi dal bordo)
    setLeftMotorSpeed(DEFAULT_MOTOR_SPEED * 0.5f);
    setRightMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.5f);
    delay(900); // Durata della rotazione da calibrare (circa 135°)
    stopDriveMotors();
    
    // 4. Riprendi il movimento in avanti
    setLeftMotorSpeed(DEFAULT_MOTOR_SPEED);
    setRightMotorSpeed(DEFAULT_MOTOR_SPEED);
    
    // 5. Se dopo 2 secondi siamo ancora sul bordo, fermati
    unsigned long borderCheckStart = millis();
    const unsigned long BORDER_CHECK_TIMEOUT = 2000; // 2 secondi
    
    while (isBorderDetected() && (millis() - borderCheckStart < BORDER_CHECK_TIMEOUT)) {
        // Se siamo ancora sul bordo, fermati e segnala l'errore
        stopMotors();
        delay(100);
        
        #ifdef SERIAL_DEBUG
        Serial.println(F("Still on border after maneuver, trying again..."));
        #endif
        
        // Prova a ruotare di più
        setLeftMotorSpeed(DEFAULT_MOTOR_SPEED * 0.5f);
        setRightMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.5f);
        delay(300);
        stopDriveMotors();
        
        // Riprova ad andare avanti
        setLeftMotorSpeed(DEFAULT_MOTOR_SPEED);
        setRightMotorSpeed(DEFAULT_MOTOR_SPEED);
        delay(500);
    }
    
    // Se siamo usciti dal ciclo a causa del timeout, fermiamo tutto
    if (millis() - borderCheckStart >= BORDER_CHECK_TIMEOUT) {
        stopMotors();
        #ifdef SERIAL_DEBUG
        Serial.println(F("ERROR: Could not get away from border after multiple attempts"));
        #endif
        handleEvent(Event::ERROR_DETECTED);
    }
    stopDriveMotors();
}

void Mower::handleObstacle() {
    // Invia l'evento di rilevamento ostacolo
    handleEvent(Event::OBSTACLE_DETECTED);
    
    // Se abbiamo uno stato attivo, deleghiamo a lui la gestione
    if (currentState_) {
        currentState_->handleEvent(*this, Event::OBSTACLE_DETECTED);
        return;
    }
    
    // Comportamento predefinito se non c'è uno stato attivo
    #ifdef SERIAL_DEBUG
    Serial.println(F("Obstacle detected - executing default avoidance"));
    #endif
    
    // 1. Ferma tutti i motori
    stopMotors();
    delay(100);
    
    // 2. Vai indietro per 500ms
    setLeftMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.7f);
    setRightMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.7f);
    delay(500);
    stopDriveMotors();
    
    // 3. Ruota a destra di 45° (valore da calibrare in base al robot)
    setLeftMotorSpeed(DEFAULT_MOTOR_SPEED * 0.5f);
    setRightMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.5f);
    delay(300); // Durata della rotazione da calibrare
    stopDriveMotors();
    
    // 4. Riprendi il movimento in avanti
    setLeftMotorSpeed(DEFAULT_MOTOR_SPEED);
    setRightMotorSpeed(DEFAULT_MOTOR_SPEED);
    
    // 5. Imposta un timer per verificare se l'ostacolo è stato superato
    unsigned long avoidanceStartTime = millis();
    const unsigned long MAX_AVOIDANCE_TIME = 5000; // 5 secondi massimo per evitare loop infiniti
    
    while (isCollisionDetected() && (millis() - avoidanceStartTime < MAX_AVOIDANCE_TIME)) {
        // Se l'ostacolo è ancora presente, ripeti la manovra
        stopMotors();
        delay(100);
        
        // Ruota a sinistra stavolta
        setLeftMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.5f);
        setRightMotorSpeed(DEFAULT_MOTOR_SPEED * 0.5f);
        delay(600); // Ruota un po' di più
        stopDriveMotors();
        
        // Riprova ad andare avanti
        setLeftMotorSpeed(DEFAULT_MOTOR_SPEED);
        setRightMotorSpeed(DEFAULT_MOTOR_SPEED);
        delay(500);
    }
    
    // Se siamo usciti dal ciclo a causa del timeout, fermiamo tutto
    if (millis() - avoidanceStartTime >= MAX_AVOIDANCE_TIME) {
        stopMotors();
        #ifdef SERIAL_DEBUG
        Serial.println(F("ERROR: Could not avoid obstacle after multiple attempts"));
        #endif
        handleEvent(Event::ERROR_DETECTED);
    }
}

// Registra lo stato del sistema
void Mower::logStatus() {
    #ifdef ENABLE_DEBUG
    static unsigned long lastLog = 0;
    
    // Logga lo stato solo ogni 5 secondi
    if (millis() - lastLog < 5000) {
        return;
    }
    
    lastLog = millis();
    
    SERIAL_DEBUG.print("Stato: ");
    SERIAL_DEBUG.print(stateToString(getState()));
    SERIAL_DEBUG.print(", Batteria: ");
    SERIAL_DEBUG.print(batteryLevel_);
    SERIAL_DEBUG.print("%, Carica: ");
    SERIAL_DEBUG.print(charging_ ? "SI" : "NO");
    SERIAL_DEBUG.print(", Bordo: ");
    SERIAL_DEBUG.print(borderDetected_ ? "RILEVATO" : "NO");
    SERIAL_DEBUG.print(", Urto: ");
    SERIAL_DEBUG.println(collisionDetected_ ? "RILEVATO" : "NO");
    #endif
}

const char* Mower::eventToString(Event event) const {
    switch (event) {
        // Comandi utente
        case Event::START_MOWING: return "START_MOWING";
        case Event::STOP_MOWING: return "STOP_MOWING";
        case Event::START_DOCKING: return "START_DOCKING";
        case Event::PAUSE: return "PAUSE";
        case Event::RESUME: return "RESUME";
        case Event::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case Event::MANUAL_CONTROL_ENABLED: return "MANUAL_CONTROL_ENABLED";
        case Event::MANUAL_CONTROL_DISABLED: return "MANUAL_CONTROL_DISABLED";
        
        // Eventi sensori
        case Event::BORDER_DETECTED: return "BORDER_DETECTED";
        case Event::LIFT_DETECTED: return "LIFT_DETECTED";
        case Event::RAIN_DETECTED: return "RAIN_DETECTED";
        case Event::BATTERY_LOW: return "BATTERY_LOW";
        case Event::BATTERY_FULL: return "BATTERY_FULL";
        case Event::CHARGING_STARTED: return "CHARGING_STARTED";
        case Event::CHARGING_STOPPED: return "CHARGING_STOPPED";
        case Event::DOCK_DETECTED: return "DOCK_DETECTED";
        case Event::UNDOCK_DETECTED: return "UNDOCK_DETECTED";
        case Event::OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
        case Event::UNDOCKING_COMPLETE: return "UNDOCKING_COMPLETE";
        case Event::ERROR_DETECTED: return "ERROR_DETECTED";
        case Event::ERROR_CLEARED: return "ERROR_CLEARED";
        
        default: return "UNKNOWN_EVENT";
    }
}

const char* Mower::navigationModeToString(NavigationMode mode) const {
    switch (mode) {
        case NavigationMode::RANDOM: return "RANDOM";
        case NavigationMode::SPIRAL: return "SPIRAL";
        case NavigationMode::LAWN_MOWER: return "LAWN_MOWER";
        case NavigationMode::BORDER: 
        case NavigationMode::FOLLOW_BORDER: 
            return "PERIMETER";
        case NavigationMode::ZONE: return "ZONE";
        case NavigationMode::DOCKING: return "DOCKING";
        case NavigationMode::UNDOCKING: return "UNDOCKING";
        case NavigationMode::MANUAL: return "MANUAL";
        case NavigationMode::SPOT: return "SPOT";
        case NavigationMode::POINT_TO_POINT: return "POINT_TO_POINT";
        default: return "UNKNOWN";
    }
}

// Implementazione del metodo getState()
State Mower::getState() const { 
    return currentState_ ? currentState_->getStateType() : State::ERROR; 
}

// Implementazione di setState(State)
void Mower::setState(State newState) {
    DEBUG_PRINT("Cambio stato da: ");
    DEBUG_PRINT(stateToString(getState()));
    DEBUG_PRINT(" a: ");
    DEBUG_PRINTLN(stateToString(newState));
    
    switch (newState) {
        case State::IDLE:
            changeState(getIdleState());
            break;
        case State::MOWING:
            changeState(getMowingState());
            break;
        case State::DOCKING:
            changeState(getDockingState());
            break;
        case State::UNDOCKING:
            changeState(getUndockingState());
            break;
        case State::CHARGING:
            changeState(getChargingState());
            break;
        case State::EMERGENCY_STOP:
            changeState(getEmergencyStopState());
            break;
        case State::LIFTED:
            changeState(getLiftedState());
            break;
        case State::ERROR:
            changeState(getErrorState());
            break;
        default:
            DEBUG_PRINTLN("Stato non valido");
            break;
    }
}

// Implementazione di stateToString
const char* Mower::stateToString(State state) const {
    switch (state) {
        case State::IDLE: return "IDLE";
        case State::MOWING: return "MOWING";
        case State::DOCKING: return "DOCKING";
        case State::CHARGING: return "CHARGING";
        case State::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case State::MANUAL_CONTROL: return "MANUAL_CONTROL";
        case State::ERROR: return "ERROR";
        case State::LIFTED: return "LIFTED";
        case State::PAUSED: return "PAUSED";
        case State::SLEEP: return "SLEEP";
        case State::RAIN_DELAY: return "RAIN_DELAY";
        case State::MAINTENANCE_NEEDED: return "MAINTENANCE_NEEDED";
        case State::ROS_CONTROL: return "ROS_CONTROL";
        default: return "UNKNOWN";
    }
}

// Il metodo setStateMachine è stato rimosso in favore del nuovo sistema a stati basato su MowerState

// I metodi getBatteryPercentage(), getUptime(), isRaining(), isObstacleDetected(),
// isAlignedWithDock() e calculateDockAlignmentCorrection() sono implementati inline nel file header

// Gestione errori
void Mower::logError(const String& error) {
    lastError_ = error;
    errorTimestamp_ = millis();
    // Puoi aggiungere qui la logica per salvare l'errore su EEPROM o SD card
}

String Mower::getLastError() const {
    return lastError_;
}

bool Mower::isErrorResolved() const {
    // Implementa la logica per verificare se l'errore è stato risolto
    // Questo è un esempio: verifica se è passato abbastanza tempo dall'ultimo errore
    return (millis() - errorTimestamp_) > ERROR_RESET_TIMEOUT;
}

// Controllo motori
void Mower::updateMotors() {
    // Aggiorna i motori in base alle velocità impostate
    if (emergencyStopActive_) {
        // In caso di emergenza, ferma tutti i motori
        stopMotors();
    } else {
        // Altrimenti, aggiorna i motori in base alle velocità impostate
        if (bladesRunning_) {
            setBladeSpeed(bladeSpeed_);
        }
        
        // Aggiorna i motori di trazione
        setLeftMotorSpeed(leftMotorSpeed_);
        setRightMotorSpeed(rightMotorSpeed_);
        
        // Aggiorna lo stato dei motori
        leftMotor.update();
        rightMotor.update();
        
        #ifdef DEBUG_MOTORS
        static unsigned long lastDebugTime = 0;
        if (millis() - lastDebugTime > 1000) {
            lastDebugTime = millis();
            SERIAL_DEBUG.print("Motors - L: ");
            SERIAL_DEBUG.print(leftMotorSpeed_);
            SERIAL_DEBUG.print(", R: ");
            SERIAL_DEBUG.println(rightMotorSpeed_);
        }
        #endif
    }
}

void Mower::checkSafety() {
    // Verifica le condizioni di sicurezza e applica l'arresto di emergenza se necessario
    if (isLifted()) {
        emergencyStop();
        handleEvent(Event::LIFT_DETECTED);
    } else if (isBatteryCritical()) {
        emergencyStop();
        handleEvent(Event::BATTERY_CRITICAL);
    } else if (isCollisionDetected()) {
        emergencyStop();
        handleEvent(Event::OBSTACLE_DETECTED);
    } else {
        // Ferma i motori per sicurezza se nessuna condizione di sicurezza è attiva
        stopDriveMotors();
    }
}

// Navigazione
void Mower::startRandomMovement() {
    setNavigationMode(NavigationMode::RANDOM);
    // Altre operazioni necessarie per avviare il movimento casuale
}

void Mower::stopRandomMovement() {
    if (navigationMode_ == NavigationMode::RANDOM) {
        stopDriveMotors();
    }
}

void Mower::followPerimeter() {
    setNavigationMode(NavigationMode::BORDER);
    // Altre operazioni necessarie per seguire il perimetro
}

// Metodi di debug
void Mower::printDebugInfo() const {
    Serial.println("=== Mower Debug Info ===");
    Serial.print("Battery: ");
    Serial.print(getBatteryPercentage());
    Serial.println("%");
    
    Serial.print("Left Motor: ");
    Serial.print(leftMotorSpeed_);
    Serial.print(", Right Motor: ");
    Serial.println(rightMotorSpeed_);
    
    Serial.print("Blade Speed: ");
    Serial.println(bladeSpeed_);
    
    Serial.print("Lifted: ");
    Serial.println(lifted_ ? "YES" : "NO");
    
    Serial.print("Docked: ");
    Serial.println(docked_ ? "YES" : "NO");
    
    Serial.print("Last Error: ");
    Serial.println(lastError_);
    
    Serial.println("=======================");
}

// Il display LCD è gestito dalla classe LCDMenu

// Implementazione del metodo per il buzzer
void Mower::playBuzzerTone(unsigned int frequency, unsigned long duration) {
    #ifdef BUZZER_ENABLED
    buzzer.playTone(frequency, duration);
    #endif
}

// Imposta la modalità di navigazione
void Mower::setNavigationMode(NavigationMode mode) {
    if (navigationMode_ != mode) {
        navigationMode_ = mode;
        #ifdef DEBUG
        SERIAL_DEBUG.print(F("Navigation mode set to: "));
        SERIAL_DEBUG.println(navigationModeToString(mode));
        #endif
    }
}

// Abilita o disabilita la ricarica
bool Mower::enableCharging(bool enable) {
    if (enable == charging_) {
        return true; // Già nello stato richiesto
    }
    
    if (enable) {
        // Attiva la ricarica
        chargingRelay.on();
        charging_ = true;
        #ifdef DEBUG
        SERIAL_DEBUG.println(F("Charging enabled"));
        #endif
    } else {
        // Disattiva la ricarica
        chargingRelay.off();
        charging_ = false;
        #ifdef DEBUG
        SERIAL_DEBUG.println(F("Charging disabled"));
        #endif
    }
    
    return true;
}

// ===== LCD Display Methods =====
void Mower::clearLcdDisplay() {
    // Forward to LCDMenu instance
    lcdMenu.clear();
}

void Mower::setLcdCursor(uint8_t col, uint8_t row) {
    // Forward to LCDMenu instance
    lcdMenu.setCursor(col, row);
}

void Mower::printToLcd(const String &text) {
    // Forward to LCDMenu instance
    lcdMenu.print(text);
}

void Mower::printToLcd(int number) {
    // Forward to LCDMenu instance
    lcdMenu.print(number);
}

void Mower::updateLcdDisplay(const String &line1, const String &line2) {
    // Update both lines of the LCD display
    lcdMenu.clear();
    lcdMenu.setCursor(0, 0);
    lcdMenu.print(line1);
    
    if (line2.length() > 0) {
        lcdMenu.setCursor(0, 1);
        lcdMenu.print(line2);
    }
}
