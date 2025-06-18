#include "ErrorManager.h"

// Inizializza le variabili statiche
ErrorInfo ErrorManager::_activeErrors[MAX_ERRORS];
uint8_t ErrorManager::_errorCount = 0;
bool ErrorManager::_initialized = false;

// Definizione delle descrizioni degli errori
const ErrorManager::ErrorDescription ErrorManager::_errorDescriptions[9] = {
    {ErrorCode::NO_ERROR, "Nessun errore"},
    {ErrorCode::BATTERY_LOW, "Batteria scarica"},
    {ErrorCode::BATTERY_CRITICAL, "Batteria critica"},
    {ErrorCode::BATTERY_ERROR, "Errore lettura batteria"},
    {ErrorCode::GPS_ERROR, "Errore GPS"},
    {ErrorCode::MOTOR_ERROR, "Errore motore"},
    {ErrorCode::SENSOR_ERROR, "Errore sensore"},
    {ErrorCode::WIFI_ERROR, "Errore connessione WiFi"},
    {ErrorCode::UNKNOWN_ERROR, "Errore sconosciuto"}
};

const uint8_t ErrorManager::ERROR_DESCRIPTIONS_COUNT = 9;

void ErrorManager::begin() {
    if (!_initialized) {
        _initializeErrorDescriptions();
        _errorCount = 0;  // Azzera il contatore invece di usare clear()
        _initialized = true;
    }
}

void ErrorManager::_initializeErrorDescriptions() {
    // Le descrizioni sono inizializzate staticamente
    // Non è necessario fare nulla qui
}

void ErrorManager::addError(ErrorCode code, const String& message, bool isCritical) {
    if (!_initialized) begin();
    
    // Verifica se l'errore è già presente
    for (uint8_t i = 0; i < _errorCount; i++) {
        if (_activeErrors[i].code == code) {
            // Aggiorna l'errore esistente
            _activeErrors[i].message = (message.length() == 0) ? getErrorString(code) : message;
            _activeErrors[i].timestamp = millis();
            _activeErrors[i].isCritical = isCritical;
            return;
        }
    }
    
    // Aggiungi un nuovo errore se c'è spazio
    if (_errorCount < MAX_ERRORS) {
        _activeErrors[_errorCount].code = code;
        _activeErrors[_errorCount].message = (message.length() == 0) ? getErrorString(code) : message;
        _activeErrors[_errorCount].timestamp = millis();
        _activeErrors[_errorCount].isCritical = isCritical;
        _errorCount++;
        
        #ifdef SERIAL_DEBUG
            Serial.print(F("Errore: "));
            Serial.print(_activeErrors[_errorCount-1].message);
            Serial.print(F(" ("));
            Serial.print(static_cast<int>(code));
            Serial.print(F(") - "));
            Serial.println(isCritical ? F("Critico") : F("Non critico"));
        #endif
    } else {
        #ifdef SERIAL_DEBUG
            Serial.println(F("Errore: Troppi errori attivi!"));
        #endif
    }
}

bool ErrorManager::removeError(ErrorCode code) {
    if (!_initialized) begin();
    
    for (uint8_t i = 0; i < _errorCount; i++) {
        if (_activeErrors[i].code == code) {
            // Sposta tutti gli elementi successivi di una posizione indietro
            for (uint8_t j = i; j < _errorCount - 1; j++) {
                _activeErrors[j] = _activeErrors[j + 1];
            }
            _errorCount--;
            return true;
        }
    }
    return false;
}

bool ErrorManager::hasError(ErrorCode code) {
    if (!_initialized) begin();
    
    for (uint8_t i = 0; i < _errorCount; i++) {
        if (_activeErrors[i].code == code) {
            return true;
        }
    }
    return false;
}

bool ErrorManager::hasCriticalError() {
    if (!_initialized) begin();
    
    for (uint8_t i = 0; i < _errorCount; i++) {
        if (_activeErrors[i].isCritical) {
            return true;
        }
    }
    return false;
}

void ErrorManager::clearAllErrors() {
    if (!_initialized) begin();
    _errorCount = 0;
}

uint8_t ErrorManager::getErrorCount() {
    if (!_initialized) begin();
    return _errorCount;
}

ErrorInfo ErrorManager::getError(uint8_t index) {
    if (!_initialized) begin();
    
    static ErrorInfo invalidError = {ErrorCode::UNKNOWN_ERROR, "Indice non valido", 0, false};
    
    if (index < _errorCount) {
        return _activeErrors[index];
    }
    return invalidError;
}

String ErrorManager::getErrorString(ErrorCode code) {
    if (!_initialized) begin();
    
    for (uint8_t i = 0; i < ERROR_DESCRIPTIONS_COUNT; i++) {
        if (_errorDescriptions[i].code == code) {
            return String(_errorDescriptions[i].description);
        }
    }
    return String("Errore sconosciuto");
}

bool ErrorManager::clearErrors() {
    if (!_initialized) begin();
    
    bool allCleared = true;
    
    // Rimuovi tutti gli errori non critici
    for (int8_t i = _errorCount - 1; i >= 0; i--) {
        if (!_activeErrors[i].isCritical) {
            removeError(_activeErrors[i].code);
        } else {
            allCleared = false;
        }
    }
    
    return allCleared;
}
