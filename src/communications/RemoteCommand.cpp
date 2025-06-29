#include "RemoteCommand.h"

RemoteCommand::RemoteCommand(Mower& mower) 
    : mower_(mower),
      remoteControlEnabled_(false),
      isPaused_(false),
      lastCommandTime_(0),
      pauseStartTime_(0),
      totalPauseTime_(0),
      customCommandHandler_(nullptr),
      statusUpdateHandler_(nullptr) {
}

void RemoteCommand::begin() {
    remoteControlEnabled_ = false;
    isPaused_ = false;
    lastCommandTime_ = 0;
    pauseStartTime_ = 0;
    totalPauseTime_ = 0;
}

bool RemoteCommand::processCommand(RemoteCommandType command, const CommandData& data) {
    // I comandi di emergenza sono sempre accettati
    if (command != RemoteCommandType::EMERGENCY_STOP && 
        (!remoteControlEnabled_ || isPaused_)) {
        return false;
    }

    // Aggiorna il timestamp dell'ultimo comando
    lastCommandTime_ = millis();
    bool result = true;

    switch (command) {
        case RemoteCommandType::START_MOWING:
            mower_.startBlades();
            break;
            
        case RemoteCommandType::STOP_MOWING:
            mower_.stopBlades();
            break;
            
        case RemoteCommandType::PAUSE_MOWING:
            result = pauseOperations(true);
            break;
            
        case RemoteCommandType::RESUME_MOWING:
            result = pauseOperations(false);
            break;
            
        case RemoteCommandType::MOVE_FORWARD:
            executeMovement(data.speed, data.speed, static_cast<unsigned long>(data.distance * 1000));
            break;
            
        case RemoteCommandType::MOVE_BACKWARD:
            executeMovement(-data.speed, -data.speed, static_cast<unsigned long>(data.distance * 1000));
            break;
            
        case RemoteCommandType::TURN_LEFT:
            executeMovement(-data.speed, data.speed, static_cast<unsigned long>(data.angle * 10));
            break;
            
        case RemoteCommandType::TURN_RIGHT:
            executeMovement(data.speed, -data.speed, static_cast<unsigned long>(data.angle * 10));
            break;
            
        case RemoteCommandType::ROTATE_LEFT:
            // Ruota sul posto a sinistra
            executeMovement(-data.speed, data.speed, static_cast<unsigned long>(data.angle * 10));
            break;
            
        case RemoteCommandType::ROTATE_RIGHT:
            // Ruota sul posto a destra
            executeMovement(data.speed, -data.speed, static_cast<unsigned long>(data.angle * 10));
            break;
            
        case RemoteCommandType::SET_SPEED:
            // Imposta la stessa velocità per entrambi i motori
            mower_.setLeftMotorSpeed(data.speed);
            mower_.setRightMotorSpeed(data.speed);
            break;
            
        case RemoteCommandType::SET_BLADE_SPEED:
            mower_.setBladeSpeed(data.speed);
            break;
            
        case RemoteCommandType::SET_NAV_MODE:
            mower_.setNavigationMode(static_cast<Mower::NavigationMode>(data.navMode));
            break;
            
        case RemoteCommandType::RETURN_TO_BASE:
            // Implementa la logica per tornare alla base
            // Nota: Aggiungere l'implementazione appropriata
            break;
            
        case RemoteCommandType::DOCK:
            // Implementa la logica per agganciarsi
            // Nota: Aggiungere l'implementazione appropriata
            break;
            
        case RemoteCommandType::UNDOCK:
            // Implementa la logica per sganciarsi
            // Nota: Aggiungere l'implementazione appropriata
            break;
            
        case RemoteCommandType::EMERGENCY_STOP:
            mower_.emergencyStop();
            remoteControlEnabled_ = false;
            isPaused_ = false;
            break;
            
        case RemoteCommandType::CUSTOM_COMMAND:
            if (data.stringValue) {
                result = handleCustomCommand(data.stringValue, 
                                           data.intValue ? String(data.intValue).c_str() : nullptr);
            }
            break;
            
        default:
            result = false;
    }
    
    // Notifica il cambio di stato
    if (result) {
        updateStatus();
    }
    
    return result;
}

bool RemoteCommand::sendCustomCommand(const char* command, const char* params) {
    if (!command) return false;
    
    CommandData data{};
    data.stringValue = command;
    data.intValue = params ? atoi(params) : 0;
    
    return processCommand(RemoteCommandType::CUSTOM_COMMAND, data);
}

bool RemoteCommand::pauseOperations(bool pause) {
    if (isPaused_ == pause) {
        return true; // Già nello stato richiesto
    }
    
    if (pause) {
        // Metti in pausa
        pauseStartTime_ = millis();
        mower_.stopMotors();
        mower_.stopBlades();
    } else {
        // Riprendi
        if (pauseStartTime_ > 0) {
            totalPauseTime_ += (millis() - pauseStartTime_);
            pauseStartTime_ = 0;
        }
        // Riprendi le lame solo se erano attive prima della pausa
        if (mower_.bladesRunning_) {
            mower_.startBlades();
        }
    }
    
    isPaused_ = pause;
    updateStatus();
    return true;
}

void RemoteCommand::update() {
    unsigned long currentTime = millis();
    
    // Gestione del timeout del comando
    if (remoteControlEnabled_ && !isPaused_ && 
        (currentTime - lastCommandTime_ > COMMAND_TIMEOUT_MS)) {
        mower_.stopMotors();
    }
    
    // Gestione del timeout della pausa
    if (isPaused_ && pauseStartTime_ > 0 && 
        (currentTime - pauseStartTime_ > MAX_PAUSE_TIME_MS)) {
        // Timeout della pausa, riprendi automaticamente
        pauseOperations(false);
    }
    
    // Aggiornamento periodico dello stato
    static unsigned long lastStatusUpdate = 0;
    if (currentTime - lastStatusUpdate > 1000) { // Ogni secondo
        updateStatus();
        lastStatusUpdate = currentTime;
    }
}

void RemoteCommand::setRemoteControlEnabled(bool enable) {
    if (remoteControlEnabled_ != enable) {
        remoteControlEnabled_ = enable;
        if (!enable) {
            mower_.stopMotors();
            isPaused_ = false;
        }
        lastCommandTime_ = millis();
        updateStatus();
    }
}

void RemoteCommand::getStatus(RemoteStatus& status) const {
    // Utilizziamo i membri diretti invece dei metodi getter inesistenti
    status.isMoving = (mower_.leftMotorSpeed_ != 0 || mower_.rightMotorSpeed_ != 0);
    status.isMowing = mower_.bladesRunning_;
    status.isPaused = isPaused_;
    status.isDocked = mower_.isDocked();
    status.isCharging = mower_.isCharging();
    status.batteryLevel = mower_.getBatteryPercentage();
    status.currentSpeed = (mower_.leftMotorSpeed_ + mower_.rightMotorSpeed_) / 2.0f;
    status.navMode = mower_.getNavigationMode();
    
    // Posizione e orientamento (da implementare)
    status.positionX = 0.0f;
    status.positionY = 0.0f;
    status.heading = 0.0f;
    
    // Se disponibile, ottieni la posizione
    // Nota: Rimuovi il commento se il metodo getPosition è implementato
    // float x, y;
    // if (mower_.getPosition(x, y)) {
    //     status.positionX = x;
    //     status.positionY = y;
    //     status.posX = x;
    //     status.posY = y;
    // }
    
    // Se disponibile, ottieni l'orientamento
    // Nota: Rimuovi il commento se il metodo getHeading è implementato
    // status.heading = mower_.getHeading();
}

void RemoteCommand::setCustomCommandHandler(bool (*callback)(const char*, const char*)) {
    customCommandHandler_ = callback;
}

void RemoteCommand::setStatusUpdateHandler(void (*callback)(const RemoteStatus&)) {
    statusUpdateHandler_ = callback;
}

void RemoteCommand::executeMovement(float leftSpeed, float rightSpeed, unsigned long duration) {
    if (isPaused_) return;
    
    // Imposta la velocità dei motori
    mower_.setLeftMotorSpeed(leftSpeed);
    mower_.setRightMotorSpeed(rightSpeed);
    
    // Se è specificata una durata, programma lo spegnimento
    if (duration > 0) {
        lastCommandTime_ = millis() - (min(duration, COMMAND_TIMEOUT_MS));
    }
}

void RemoteCommand::updateStatus() {
    RemoteStatus status;
    getStatus(status);
    notifyStatusChanged();
}

void RemoteCommand::notifyStatusChanged() {
    if (statusUpdateHandler_) {
        RemoteStatus status;
        getStatus(status);
        statusUpdateHandler_(status);
    }
}

bool RemoteCommand::handleCustomCommand(const char* command, const char* params) {
    if (customCommandHandler_) {
        return customCommandHandler_(command, params);
    }
    return false;
}
