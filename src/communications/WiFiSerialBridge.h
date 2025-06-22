#ifndef WIFI_SERIAL_BRIDGE_H
#define WIFI_SERIAL_BRIDGE_H

#include "../../config.h"  // Per ENABLE_WIFI

#ifdef ENABLE_WIFI

#include <Arduino.h>
#include <ArduinoJson.h>

// Dimensione massima del buffer circolare
#define SERIAL_BUFFER_SIZE 1024
// Dimensione massima per i documenti JSON
#define MAX_JSON_DOC_SIZE 512
// Numero massimo di messaggi in coda
#define MAX_QUEUED_MESSAGES 10

// Struttura per i comandi ricevuti
struct WiFiCommand {
    String command;
    JsonVariant params;
    bool isValid;
};

class WiFiSerialBridge {
private:
    // Buffer circolare per i dati in arrivo
    char _circularBuffer[SERIAL_BUFFER_SIZE];
    size_t _head = 0;
    size_t _tail = 0;
    
    // Documenti JSON preallocati
    StaticJsonDocument<MAX_JSON_DOC_SIZE> _rxDoc;
    StaticJsonDocument<MAX_JSON_DOC_SIZE * 2> _txDoc;
    
    // Coda messaggi per il batch
    StaticJsonDocument<MAX_JSON_DOC_SIZE * MAX_QUEUED_MESSAGES> _txQueue;
    uint8_t _messageCount = 0;
    uint8_t _nextMsgId = 0;
    
    // Metodi privati
    size_t _available() const { return (_head >= _tail) ? (_head - _tail) : (SERIAL_BUFFER_SIZE - _tail + _head); }
    void _processBuffer();
    String _compressPayload(const String& payload);
    bool _parseIncoming();
    
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
     * @brief Accoda un messaggio per l'invio in batch
     * @param command Nome del comando
     * @param params Parametri del comando
     * @return true se il messaggio è stato accodato, false se la coda è piena
     */
    bool queueMessage(const String& command, const JsonObject& params);
    
    /**
     * @brief Invia tutti i messaggi accodati
     */
    void sendQueuedMessages();
    
    /**
     * @brief Invia un comando in formato binario (più efficiente)
     * @param cmdId ID del comando
     * @param data Dati binari del comando
     * @param len Lunghezza dei dati
     * @return true se l'invio è riuscito, false altrimenti
     */
    bool sendBinaryCommand(uint8_t cmdId, const uint8_t* data, size_t len);
    
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
     * @brief Imposta la funzione di callback per la gestione dei comandi
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
