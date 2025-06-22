#include "WiFiSerialBridge.h"
#include <Arduino.h>
#include <ArduinoJson.h>

// Definizione della variabile globale
WiFiSerialBridge wifiBridge(SERIAL_WIFI, 115200);

#ifdef ENABLE_WIFI

// Inizializzazione del membro statico
StaticJsonDocument<0> WiFiSerialBridge::emptyDoc;

// Struttura per l'header del protocollo binario
#pragma pack(push, 1)
struct CommandHeader {
    uint8_t startMarker = 0xAA;
    uint8_t commandId;
    uint16_t dataLength;
    uint8_t checksum;
};
#pragma pack(pop)

WiFiSerialBridge::WiFiSerialBridge(HardwareSerial& serial, unsigned long baudRate)
    : _serial(serial), _baudRate(baudRate), _commandReady(false), _commandHandler(nullptr) {
    // Inizializza il buffer circolare
    memset(_circularBuffer, 0, SERIAL_BUFFER_SIZE);
}

void WiFiSerialBridge::begin() {
    _serial.begin(_baudRate);
    _head = _tail = 0;
    _messageCount = 0;
    _nextMsgId = 0;
    _rxDoc.clear();
    _txDoc.clear();
    _txQueue.clear();
}

bool WiFiSerialBridge::available() const {
    return _serial.available() > 0;
}

void WiFiSerialBridge::_processBuffer() {
    while (_available() > 0) {
        char c = _circularBuffer[_tail];
        _tail = (_tail + 1) % SERIAL_BUFFER_SIZE;
        
        if (c == '\n' || c == '\r') {
            if (_parseIncoming() && _commandHandler) {
                _commandHandler(_rxDoc["command"].as<String>(), _rxDoc["params"]);
            }
        }
    }
}

String WiFiSerialBridge::_compressPayload(const String& payload) {
    String compressed;
    compressed.reserve(payload.length());
    
    for (size_t i = 0; i < payload.length(); i++) {
        if (payload[i] > ' ') {  // Rimuove spazi e caratteri di controllo
            compressed += payload[i];
        }
    }
    return compressed;
}

bool WiFiSerialBridge::_parseIncoming() {
    DeserializationError error = deserializeJson(_rxDoc, _circularBuffer + _tail);
    if (!error) {
        _tail = (_tail + measureJson(_rxDoc) + 1) % SERIAL_BUFFER_SIZE;
        return _rxDoc.containsKey("command");
    }
    return false;
}

WiFiCommand WiFiSerialBridge::processIncoming() {
    WiFiCommand cmd = {"", JsonVariant(), false};
    
    // Leggi tutti i dati disponibili dalla seriale
    while (_serial.available()) {
        if ((_head + 1) % SERIAL_BUFFER_SIZE != _tail) {  // Se c'è spazio nel buffer
            _circularBuffer[_head] = _serial.read();
            _head = (_head + 1) % SERIAL_BUFFER_SIZE;
        } else {
            _serial.read();  // Scarta il byte se il buffer è pieno
        }
    }
    
    // Elabora il buffer
    _processBuffer();
    
    // Restituisci l'ultimo comando elaborato
    if (_rxDoc.containsKey("command")) {
        cmd.command = _rxDoc["command"].as<String>();
        cmd.params = _rxDoc["params"];
        cmd.isValid = true;
    }
    
    return cmd;
}

WiFiCommand WiFiSerialBridge::_parseCommand(const String& jsonString) {
    WiFiCommand result = {"", JsonVariant(), false};
    
    // Alloca un buffer per il parsing JSON
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    
    if (error) {
        // Se il parsing fallisce, potremmo avere un comando semplice
        if (jsonString.length() < 50) {  // Se è abbastanza corto da essere un comando semplice
            result.command = jsonString;
            result.isValid = true;
        }
        return result;
    }
    
    // Estrai il comando e i parametri
    if (doc.containsKey("cmd")) {
        result.command = doc["cmd"].as<String>();
        result.params = doc["params"];
        result.isValid = true;
    }
    
    return result;
}

bool WiFiSerialBridge::sendCommand(const String& command, const JsonDocument& params) {
    if (!_serial) return false;
    
    // Usa il documento preallocato
    _txDoc.clear();
    _txDoc["command"] = command;
    
    if (!params.isNull()) {
        _txDoc["params"] = params;
    }
    
    // Aggiungi timestamp e ID messaggio
    _txDoc["ts"] = millis();
    _txDoc["id"] = _nextMsgId++;
    
    // Invia il JSON compresso
    String jsonStr;
    serializeJson(_txDoc, jsonStr);
    
    #ifdef COMPRESS_PAYLOADS
    jsonStr = _compressPayload(jsonStr);
    #endif
    
    _serial.println(jsonStr);
    
    return true;
}

bool WiFiSerialBridge::queueMessage(const String& command, const JsonObject& params) {
    if (_messageCount >= MAX_QUEUED_MESSAGES) {
        sendQueuedMessages(); // Invia la coda se piena
    }
    
    if (_messageCount < MAX_QUEUED_MESSAGES) {
        JsonObject msg = _txQueue.createNestedObject();
        msg["cmd"] = command;
        msg["params"] = params;
        msg["ts"] = millis();
        msg["id"] = _nextMsgId++;
        _messageCount++;
        return true;
    }
    return false;
}

void WiFiSerialBridge::sendQueuedMessages() {
    if (_messageCount > 0) {
        // Usa il documento preallocato per la serializzazione
        _txDoc.clear();
        _txDoc["batch"] = true;
        _txDoc["count"] = _messageCount;
        _txDoc["messages"] = _txQueue;
        
        String jsonStr;
        serializeJson(_txDoc, jsonStr);
        
        #ifdef COMPRESS_PAYLOADS
        jsonStr = _compressPayload(jsonStr);
        #endif
        
        _serial.println(jsonStr);
        
        // Pulisci la coda
        _txQueue.clear();
        _messageCount = 0;
    }
}

bool WiFiSerialBridge::sendResponse(const String& status, const String& message, const JsonDocument& data) {
    if (!_serial) return false;
    
    // Usa il documento preallocato
    _txDoc.clear();
    _txDoc["status"] = status;
    _txDoc["message"] = message;
    
    if (!data.isNull()) {
        _txDoc["data"] = data;
    }
    
    // Aggiungi timestamp e ID messaggio
    _txDoc["ts"] = millis();
    _txDoc["id"] = _nextMsgId++;
    
    // Invia il JSON compresso
    String jsonStr;
    serializeJson(_txDoc, jsonStr);
    
    #ifdef COMPRESS_PAYLOADS
    jsonStr = _compressPayload(jsonStr);
    #endif
    
    _serial.println(jsonStr);
    
    return true;
}

bool WiFiSerialBridge::sendTelemetry(const JsonDocument& data) {
    if (data.isNull()) return false;
    
    String output;
    serializeJson(data, output);
    _serial.println(output);
    
    #ifdef SERIAL_DEBUG
    SERIAL_DEBUG.print(F("[WiFiBridge] TX telem: "));
    SERIAL_DEBUG.println(output);
    #endif
    
    return true;
}

void WiFiSerialBridge::setCommandHandler(void (*callback)(const String&, const JsonVariant&)) {
    _commandHandler = callback;
}

#endif // ENABLE_WIFI
