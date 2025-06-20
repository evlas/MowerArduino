#ifndef WIFI_SERIAL_BRIDGE_H
#define WIFI_SERIAL_BRIDGE_H

#include "../../config.h"  // Per ENABLE_WIFI

#ifdef ENABLE_WIFI

#include <Arduino.h>
#include <ArduinoJson.h>

// Struttura per i comandi ricevuti
struct WiFiCommand {
    String command;
    JsonVariant params;
    bool isValid;
};

// Forward declaration per la classe CommandHandler
class CommandHandler;

class WiFiSerialBridge {
public:
    /**
     * @brief Costruttore
     * @param serial Reference all'oggetto Serial (es. Serial1, Serial2, ecc.)
     * @param baudRate Baud rate per la comunicazione seriale
     */
    WiFiSerialBridge(HardwareSerial& serial, unsigned long baudRate = 115200);
    
    /**
     * @brief Inizializza la comunicazione seriale
     */
    void begin();
    
    /**
     * @brief Elabora i dati in arrivo dalla seriale
     * @return WiFiCommand struttura con il comando ricevuto e i parametri
     */
    WiFiCommand processIncoming();
    
    /**
     * @brief Invia un comando al modulo WiFi
     * @param command Nome del comando
     * @param params Parametri del comando (opzionale)
     * @return true se l'invio è riuscito, false altrimenti
     */
    bool sendCommand(const String& command, const JsonDocument& params = emptyDoc);
    
    /**
     * @brief Invia un messaggio di risposta
     * @param status Stato della risposta (es. "ok", "error")
     * @param message Messaggio di risposta
     * @param data Dati aggiuntivi (opzionali)
     * @return true se l'invio è riuscito, false altrimenti
     */
    bool sendResponse(const String& status, const String& message, const JsonDocument& data = emptyDoc);
    
    /**
     * @brief Invia dati di telemetria
     * @param data Dati di telemetria in formato JSON
     * @return true se l'invio è riuscito, false altrimenti
     */
    bool sendTelemetry(const JsonDocument& data);
    
    /**
     * @brief Verifica se ci sono dati disponibili in ricezione
     * @return true se ci sono dati disponibili, false altrimenti
     */
    bool available() const;
    
    /**
     * @brief Imposta la funzione di callback per i comandi non gestiti
     * @param callback Funzione di callback con firma: void callback(const String& command, const JsonVariant& params)
     */
    void setCommandHandler(void (*callback)(const String&, const JsonVariant&));
    
    // Documento JSON vuoto statico per i parametri opzionali
    static StaticJsonDocument<0> emptyDoc;

private:
    HardwareSerial& _serial;
    unsigned long _baudRate;
    String _inputBuffer;
    bool _commandReady;
    void (*_commandHandler)(const String&, const JsonVariant&);
    
    /**
     * @brief Elabora un comando ricevuto
     * @param jsonString Stringa JSON contenente il comando
     * @return WiFiCommand struttura con il comando elaborato
     */
    WiFiCommand _parseCommand(const String& jsonString);
};

// Rendi disponibile l'istanza globale definita nel file .cpp
extern WiFiSerialBridge wifiBridge;

#else

// Definizione stub quando il WiFi è disabilitato
class WiFiSerialBridge {
public:
    WiFiSerialBridge(HardwareSerial&, unsigned long = 115200) {}
    void begin() {}
    bool available() const { return false; }
    bool sendCommand(const String&, const JsonDocument& = emptyDoc) { return false; }
    bool sendResponse(const String&, const String&, const JsonDocument& = emptyDoc) { return false; }
    bool sendTelemetry(const JsonDocument&) { return false; }
    void setCommandHandler(void (*)(const String&, const JsonVariant&)) {}
    
    struct WiFiCommand {
        String command;
        JsonVariant params;
        bool isValid;
    };
    
    WiFiCommand processIncoming() { return {"", JsonVariant(), false}; }
    
    static StaticJsonDocument<0> emptyDoc;
};

#endif // ENABLE_WIFI

#endif // WIFI_SERIAL_BRIDGE_H
