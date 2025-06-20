#include "WiFiSerialBridge.h"
#include <Arduino.h>
#include <ArduinoJson.h>

// Definizione della variabile globale
WiFiSerialBridge wifiBridge(SERIAL_WIFI, 115200);

#ifdef ENABLE_WIFI

// Inizializzazione del membro statico
StaticJsonDocument<0> WiFiSerialBridge::emptyDoc;

WiFiSerialBridge::WiFiSerialBridge(HardwareSerial& serial, unsigned long baudRate)
    : _serial(serial), _baudRate(baudRate), _commandReady(false), _commandHandler(nullptr) {}

void WiFiSerialBridge::begin() {
    _serial.begin(_baudRate);
    _inputBuffer.reserve(256); // Alloca spazio per il buffer di input
    _inputBuffer = "";
}

bool WiFiSerialBridge::available() const {
    return _serial.available() > 0;
}

WiFiCommand WiFiSerialBridge::processIncoming() {
    WiFiCommand cmd = {"", JsonVariant(), false};
    
    while (_serial.available()) {
        char c = _serial.read();
        
        #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.write(c); // echo raw char for debugging
        #endif
        
        if (c == '\n' || c == '\r') {
            if (_inputBuffer.length() > 0) {
                #ifdef SERIAL_DEBUG
                SERIAL_DEBUG.print(F("\n[WiFiBridge] RX line: "));
                SERIAL_DEBUG.println(_inputBuffer);
                #endif
                cmd = _parseCommand(_inputBuffer);
                _inputBuffer = "";
                
                // Se è stato registrato un gestore di comandi, chiamalo
                if (_commandHandler && cmd.isValid) {
                    #ifdef SERIAL_DEBUG
                    SERIAL_DEBUG.print(F("[WiFiBridge] Dispatch cmd: "));
                    SERIAL_DEBUG.println(cmd.command);
                    #endif
                    _commandHandler(cmd.command, cmd.params);
                }
                
                return cmd;
            }
        } else if (isPrintable(c)) {
            _inputBuffer += c;
            // Preveniamo overflow del buffer
            if (_inputBuffer.length() >= 255) {
                _inputBuffer = "";
            }
        }
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
    if (command.length() == 0) return false;
    
    StaticJsonDocument<256> doc;
    doc["cmd"] = command;
    
    if (!params.isNull()) {
        doc["params"] = params;
    }
    
    String output;
    serializeJson(doc, output);
    _serial.println(output);
    
    #ifdef SERIAL_DEBUG
    SERIAL_DEBUG.print(F("[WiFiBridge] TX cmd: "));
    SERIAL_DEBUG.println(output);
    #endif
    
    return true;
}

bool WiFiSerialBridge::sendResponse(const String& status, const String& message, const JsonDocument& data) {
    StaticJsonDocument<512> doc;
    doc["status"] = status;
    doc["message"] = message;
    
    if (!data.isNull()) {
        doc["data"] = data;
    }
    
    String output;
    serializeJson(doc, output);
    _serial.println(output);
    
    #ifdef SERIAL_DEBUG
    SERIAL_DEBUG.print(F("[WiFiBridge] TX resp: "));
    SERIAL_DEBUG.println(output);
    #endif
    
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
