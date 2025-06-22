#include "CommandHandler.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include "../../config.h"

// Include per le costanti di velocità
#include "../../config.h"

// Include sensor headers
#ifdef ENABLE_ULTRASONIC
#include "../../src/sensors/UltrasonicSensors.h"
#endif

#ifdef ENABLE_BUMP_SENSORS
#include "../../src/sensors/BumpSensors.h"
#endif

CommandHandler::CommandHandler(StateMachine* stateMachine, 
                             Navigation* navigation
                             #ifdef ENABLE_ULTRASONIC
                             , UltrasonicSensors* ultrasonicSensors
                             #endif
                             #ifdef ENABLE_BUMP_SENSORS
                             , BumpSensors* bumpSensors
                             #endif
                             ) 
    : _stateMachine(stateMachine), 
      _navigation(navigation),
      _currentSpeed(0.0f),
      _currentTurnRate(0.0f),
      _currentAngle(0.0f),
      _currentBladeSpeed(0),
      _maxSpeed(static_cast<float>(MAX_MOTOR_SPEED))
      #ifdef ENABLE_ULTRASONIC
      , _ultrasonicSensors(ultrasonicSensors)
      #endif
      #ifdef ENABLE_BUMP_SENSORS
      , _bumpSensors(bumpSensors)
      #endif
      , _telemetry(nullptr)
      , _telemetryEnabled(false)
      , _telemetryInterval(1000)  // Intervallo predefinito: 1 secondo
      , _lastTelemetryUpdate(0) {}

// Metodi di movimento
void CommandHandler::moveForward(float speed) {
    moveAtSpeed(constrain(speed, static_cast<float>(MIN_MOTOR_SPEED), _maxSpeed));
}

void CommandHandler::moveBackward(float speed) {
    moveAtSpeed(-constrain(speed, static_cast<float>(MIN_MOTOR_SPEED), _maxSpeed));
}

void CommandHandler::moveAtSpeed(float speed) {
    if (validateSpeed(speed)) {
        _currentSpeed = speed;
        _currentTurnRate = 0.0f;  // Resetta la rotazione
        _navigation->setMode(NavigationMode::MANUAL);
        // Implementa la logica per muoversi alla velocità specificata
    }
}

// Metodi di rotazione
void CommandHandler::turnLeft(float angle) {
    turnByAngle(constrain(angle, MIN_ANGLE, MAX_ANGLE));
}

void CommandHandler::turnRight(float angle) {
    turnByAngle(-constrain(angle, MIN_ANGLE, MAX_ANGLE));
}

void CommandHandler::turnByAngle(float angle) {
    if (validateAngle(angle)) {
        _currentAngle = normalizeAngle(_currentAngle + angle);
        // Riduci la velocità durante le curve per maggiore controllo
        float turnSpeed = min(_currentSpeed, static_cast<float>(TURN_MOTOR_SPEED));
        // Implementa la logica per ruotare dell'angolo specificato
    }
}

// Controllo di movimento avanzato
void CommandHandler::moveAtSpeedAndTurn(float speed, float turnRate) {
    if (validateSpeed(speed) && validateTurnRate(turnRate)) {
        _currentSpeed = speed;
        _currentTurnRate = turnRate;
        _navigation->setMode(NavigationMode::MANUAL);
        // Implementa la logica per muoversi con velocità e rotazione combinate
    }
}

void CommandHandler::stop() {
    _currentSpeed = 0.0f;
    _currentTurnRate = 0.0f;
    _navigation->setMode(NavigationMode::STOPPED);
}

// Gestione modalità di funzionamento
void CommandHandler::startMowing(float speed) {
    if (validateSpeed(speed)) {
        _currentSpeed = speed;
        _stateMachine->sendEvent(MowerEvent::START_MOWING);
    }
}

void CommandHandler::startBlade() {
    setBladeSpeed(DEFAULT_BLADE_SPEED);
}

void CommandHandler::startBlade(uint8_t speed) {
    if (speed <= MAX_MOTOR_SPEED) {
        _currentBladeSpeed = speed;
        // Implementa la logica per avviare la lama alla velocità specificata
    }
}

// Metodi mancanti
void CommandHandler::stopMowing() {
    _stateMachine->sendEvent(MowerEvent::STOP_MOWING);
    stop();
}

void CommandHandler::returnToBase() {
    if (!_stateMachine) {
        #ifdef SERIAL_DEBUG
            SERIAL_DEBUG.println("Errore: StateMachine non inizializzata");
        #endif
        return;
    }
    
    // Ferma eventuali movimenti in corso
    stop();
    
    // Invia l'evento di ritorno alla base
    _stateMachine->sendEvent(MowerEvent::RETURN_TO_BASE);
    
    #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.println("Avvio ritorno alla base...");
    #endif
}

void CommandHandler::setMowingPattern(NavigationMode::Mode pattern) {
    if (_navigation) {
        _navigation->setMode(pattern);
    }
}

void CommandHandler::setBladeSpeed(uint8_t speed) {
    if (speed <= 100) {  // 0-100%
        _currentBladeSpeed = map(speed, 0, 100, 0, 255);
        // Implementa la logica per impostare la velocità della lama
    }
}

void CommandHandler::stopBlade() {
    _currentBladeSpeed = 0;
    // Implementa la logica per fermare la lama
}

// Metodi di validazione
bool CommandHandler::validateSpeed(float speed) const {
    return (speed >= -static_cast<float>(MAX_MOTOR_SPEED) && 
            speed <= static_cast<float>(MAX_MOTOR_SPEED));
}

bool CommandHandler::validateTurnRate(float turnRate) const {
    return (turnRate >= -1.0f && turnRate <= 1.0f);
}

bool CommandHandler::validateAngle(float angle) const {
    return (fabs(angle) >= MIN_ANGLE && fabs(angle) <= MAX_ANGLE);
}

bool CommandHandler::validateBladeSpeed(uint8_t speed) const {
    return (speed >= 0 && speed <= 100);
}

// Normalizzazione angolo tra 0 e 360 gradi
float CommandHandler::normalizeAngle(float angle) const {
    angle = fmod(angle, 360.0f);
    if (angle < 0) {
        angle += 360.0f;
    }
    return angle;
}

// Implementazione dei metodi per la telemetria
void CommandHandler::enableTelemetry(bool enable) {
    _telemetryEnabled = enable;
    if (_telemetry) {
        if (enable) {
            _telemetry->begin();
        }
    }
}

void CommandHandler::setTelemetryInterval(uint32_t intervalMs) {
    _telemetryInterval = max(static_cast<uint32_t>(100), intervalMs);  // Minimo 100ms
}

bool CommandHandler::isTelemetryEnabled() const {
    return _telemetryEnabled && (_telemetry != nullptr);
}

uint32_t CommandHandler::getTelemetryInterval() const {
    return _telemetryInterval;
}

// Popola il documento JSON con i dati dei sensori
void CommandHandler::getSensorData(DynamicJsonDocument& doc) {
    // Aggiungi dati dei sensori ultrasonici se disponibili
    #ifdef ENABLE_ULTRASONIC
    if (_ultrasonicSensors != nullptr) {
        JsonObject ultrasonic = doc.createNestedObject("ultrasonic");
        ultrasonic["left"] = _ultrasonicSensors->getFrontLeftDistance();
        ultrasonic["center"] = _ultrasonicSensors->getFrontCenterDistance();
        ultrasonic["right"] = _ultrasonicSensors->getFrontRightDistance();
    }
    #endif

    // Aggiungi dati dei sensori di urto se disponibili
    #ifdef ENABLE_BUMP_SENSORS
    if (_bumpSensors != nullptr) {
        JsonObject bumper = doc.createNestedObject("bumper");
        bool left, center, right;
        _bumpSensors->getAllBumpStatus(left, center, right);
        bumper["left"] = left;
        bumper["center"] = center;
        bumper["right"] = right;
    }
    #endif
}

// Metodo per aggiornare la telemetria (da chiamare nel loop principale)
void CommandHandler::updateTelemetry() {
    if (!_telemetryEnabled || !_telemetry) {
        return;
    }
    
    unsigned long currentTime = millis();
    if (currentTime - _lastTelemetryUpdate >= _telemetryInterval) {
        // Update the telemetry data
        _telemetry->update();
        
        // Debug: print telemetry data if debug is enabled
        #ifdef SERIAL_DEBUG
        String telemetryData = _telemetry->toJSON();
        SERIAL_DEBUG.println("Telemetry data:");
        SERIAL_DEBUG.println(telemetryData);
        #endif
        
        _lastTelemetryUpdate = currentTime;
    }
}

// Implementazione dei metodi getter
float CommandHandler::getCurrentSpeed() const {
    return _currentSpeed;
}

float CommandHandler::getCurrentAngle() const {
    return _currentAngle;
}
