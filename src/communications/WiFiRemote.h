#ifndef WIFI_REMOTE_H
#define WIFI_REMOTE_H

#include "RemoteCommand.h"
#include <Arduino.h>
#include "../config.h"  // SERIAL_WIFI is defined here
#include <ArduinoJson.h>  // Per la gestione del JSON

// Note: SERIAL_WIFI should be defined in config.h

// Protocol constants
#define PROTOCOL_START_MARKER 0xAA
#define PROTOCOL_END_MARKER   0x55
#define MAX_PAYLOAD_SIZE      256  // Dimensione massima del payload
#define HEADER_SIZE           6    // start(1) + type(1) + length(2) + crc(2)

// Message types
enum class MessageType : uint8_t {
    CMD_MOVE = 0x01,
    CMD_ROTATE = 0x02,
    CMD_MOW = 0x03,
    CMD_DOCK = 0x04,
    CMD_EMERGENCY = 0x05,
    CMD_SET_SPEED = 0x06,
    CMD_SET_BLADE_SPEED = 0x07,
    CMD_SET_NAV_MODE = 0x08,
    CMD_CUSTOM = 0x09,
    RSP_STATUS = 0x81,
    RSP_ACK = 0x82,
    RSP_ERROR = 0x83,
    RSP_LOG = 0x84
};

// Navigation modes (matching RemoteCommand.h)
enum class NavMode : uint8_t {
    RANDOM = 0,
    PATTERN = 1,
    MANUAL = 2,
    AUTO = 3,
    EDGE = 4,
    SPIRAL = 5,
    ZIGZAG = 6
};

// Command structures
#pragma pack(push, 1)
typedef struct {
    uint8_t direction;  // 0=forward, 1=backward, 2=left, 3=right
    int8_t speed;       // -100 to 100%
    uint16_t distance;  // cm
} MoveCommand;

typedef struct {
    uint8_t direction;  // 0=left, 1=right
    int8_t speed;       // -100 to 100%
    uint16_t angle;     // degrees
} RotateCommand;

typedef struct {
    uint8_t action;     // 0=start, 1=stop, 2=pause, 3=resume
} MowCommand;

typedef struct {
    uint8_t action;     // 0=return, 1=dock, 2=undock
} DockCommand;

#pragma pack(pop)

/**
 * @brief Classe per la gestione del controllo remoto WiFi
 */
class WiFiRemote {
public:
    explicit WiFiRemote(RemoteCommand& remote);
    
    /**
     * @brief Inizializza la comunicazione seriale
     * @param baudRate Velocità di comunicazione (default: 115200)
     */
    void begin(unsigned long baudRate = 115200);
    
    /**
     * @brief Aggiorna lo stato della comunicazione
     * Da chiamare nel loop principale
     */
    void update();
    
    /**
     * @brief Invia lo stato del robot al client
     * @param force Se true, invia lo stato completo anche se non è cambiato
     */
    void sendStatus(bool force = false);
    
    /**
     * @brief Invia un messaggio di log al client
     * @param message Il messaggio da inviare
     * @param isError Se true, il messaggio viene marcato come errore
     */
    void sendLog(const String& message, bool isError = false);
    
    // Costanti
    static constexpr unsigned long STATUS_UPDATE_INTERVAL = 1000; // Intervallo di aggiornamento stato (ms)
    static constexpr unsigned long CONNECTION_TIMEOUT = 10000;    // Timeout connessione (ms)
    
    /**
     * @brief Check if connected to a remote client
     * @return true if connected, false otherwise
     */
    bool isConnected() const { return connected_; }
    
private:
    // Riferimento al gestore dei comandi remoti
    RemoteCommand& remote_;
    
    // Stato della connessione
    bool connected_;
    unsigned long lastStatusTime_;
    unsigned long lastCommandTime_;
    
    // Buffer per l'ultimo stato inviato
    uint8_t lastStatus_[32];
    
    // Buffer per la ricezione
    uint8_t rxBuffer_[MAX_PAYLOAD_SIZE + HEADER_SIZE];
    uint16_t rxIndex_;
    bool inPacket_;
    
    // Metodi privati
    void processIncomingData();
    void processPacket(const uint8_t* data, uint16_t length);
    void sendAck(MessageType cmdType);
    void sendError(uint8_t errorCode);
    bool sendPacket(MessageType type, const void* data = nullptr, uint16_t length = 0);
    uint16_t calculateCRC16(const uint8_t* data, uint16_t length);
    
    // Metodi per la gestione delle risposte JSON
    void sendJsonResponse(JsonDocument& json);
    void createStatusJson(JsonDocument& json);
};

#endif // WIFI_REMOTE_H
