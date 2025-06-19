#include "Telemetry.h"
#include <ArduinoJson.h>

Telemetry::Telemetry(StateMachine* stateMachine, Navigation* navigation, PositionManager* positionManager)
    : _stateMachine(stateMachine), _navigation(navigation), _positionManager(positionManager) {
    _initData();
}

void Telemetry::begin() {
    // Inizializza i dati
    _initData();
    
    // Inizializza i componenti se necessario
    if (_positionManager) {
        _positionManager->begin();
    }
}

void Telemetry::update() {
    // Aggiorna i timestamp
    _data.timestamp = millis();
    
    // Aggiorna i dati di stato
    _updateStateData();
    
    // Aggiorna i dati di posizione se disponibili
    if (_positionManager) {
        _updatePositionData();
    }
    
    // Nota: i dati di batteria e sensori vengono aggiornati tramite i metodi setter
    // da altri componenti del sistema
}

const TelemetryData& Telemetry::getData() const {
    return _data;
}

String Telemetry::toJSON() const {
    // Crea un documento JSON
    StaticJsonDocument<512> doc;
    JsonObject root = doc.to<JsonObject>();
    
    // Aggiungi i dati di stato
    root["state"] = static_cast<int>(_data.currentState);
    root["navigationMode"] = static_cast<int>(_data.navigationMode);
    root["isEmergency"] = _data.isEmergency;
    root["isMowing"] = _data.isMowing;
    root["isCharging"] = _data.isCharging;
    
    // Aggiungi i dati di posizione
    JsonObject position = root.createNestedObject("position");
    position["x"] = _data.positionX;
    position["y"] = _data.positionY;
    position["heading"] = _data.heading;
    position["speed"] = _data.speed;
    position["angularSpeed"] = _data.angularSpeed;
    position["isValid"] = _data.isPositionValid;
    
    // Aggiungi i dati della batteria
    JsonObject battery = root.createNestedObject("battery");
    battery["voltage"] = _data.batteryVoltage;
    battery["current"] = _data.batteryCurrent;
    battery["level"] = _data.batteryLevel;
    battery["isValid"] = _data.isBatteryValid;
    
    // Aggiungi i dati dei motori
    JsonObject motors = root.createNestedObject("motors");
    motors["leftSpeed"] = _data.leftMotorSpeed;
    motors["rightSpeed"] = _data.rightMotorSpeed;
    motors["bladeSpeed"] = _data.bladeSpeed;
    
    // Aggiungi i dati dei sensori
    JsonObject sensors = root.createNestedObject("sensors");
    sensors["frontDistance"] = _data.frontDistance;
    sensors["rearDistance"] = _data.rearDistance;
    sensors["bumperPressed"] = _data.isBumperPressed;
    sensors["perimeterSignal"] = _data.isPerimeterSignal;
    sensors["isValid"] = _data.isSensorsValid;
    
    // Aggiungi il timestamp
    root["timestamp"] = _data.timestamp;
    
    // Serializza in JSON
    String output;
    serializeJson(root, output);
    return output;
}

void Telemetry::setBatteryData(float voltage, float current, uint8_t level) {
    _data.batteryVoltage = voltage;
    _data.batteryCurrent = current;
    _data.batteryLevel = level;
    _data.isBatteryValid = true;
}

void Telemetry::setSensorsData(float frontDist, float rearDist, bool bumperPressed, bool perimeterSignal) {
    _data.frontDistance = frontDist;
    _data.rearDistance = rearDist;
    _data.isBumperPressed = bumperPressed;
    _data.isPerimeterSignal = perimeterSignal;
    _data.isSensorsValid = true;
}

void Telemetry::setMotorsData(float leftSpeed, float rightSpeed, uint8_t bladeSpeed) {
    _data.leftMotorSpeed = leftSpeed;
    _data.rightMotorSpeed = rightSpeed;
    _data.bladeSpeed = bladeSpeed;
}

void Telemetry::_initData() {
    // Inizializza tutti i campi a valori predefiniti
    memset(&_data, 0, sizeof(_data));
    
    // Imposta i flag di validità a false
    _data.isPositionValid = false;
    _data.isBatteryValid = false;
    _data.isSensorsValid = false;
    
    // Inizializza lo stato
    _data.currentState = MowerState::IDLE;
    _data.navigationMode = NavigationMode::STOPPED;
    _data.isEmergency = false;
    _data.isMowing = false;
    _data.isCharging = false;
    
    // Inizializza il timestamp
    _data.timestamp = millis();
}

void Telemetry::_updatePositionData() {
    if (!_positionManager) {
        _data.isPositionValid = false;
        return;
    }
    
    RobotPosition pos = _positionManager->getPosition();
    
    _data.positionX = pos.x;
    _data.positionY = pos.y;
    _data.heading = pos.theta;
    _data.speed = pos.speed;
    _data.angularSpeed = pos.omega;
    _data.isPositionValid = pos.isValid;
}

void Telemetry::_updateStateData() {
    if (_stateMachine) {
        _data.currentState = _stateMachine->getCurrentState();
        _data.isEmergency = _stateMachine->isEmergency();
        _data.isMowing = _stateMachine->isMowing();
        
        // Determina se il robot sta caricando
        _data.isCharging = (_data.currentState == MowerState::CHARGING);
    }
    
    if (_navigation) {
        _data.navigationMode = _navigation->getMode();
    }
}
