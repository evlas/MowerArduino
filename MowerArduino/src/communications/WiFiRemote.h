#ifndef WIFI_REMOTE_H
#define WIFI_REMOTE_H

// RemoteCommand eliminated
#include <Arduino.h>
#include "../functions/MowerTypes.h"
#include "../config.h"  // SERIAL_WIFI is defined here
#include <ArduinoJson.h>  // Per la gestione del JSON
#include "../sensors/BatterySensor/BatterySensor.h"
#include "../sensors/IMUModule/IMUModule.h"
#include "../sensors/GPSModule/GPSModule.h"
#include "../sensors/UltrasonicSensors/UltrasonicSensors.h"
#include "../sensors/RainSensor/RainSensor.h"

// Note: SERIAL_WIFI should be defined in config.h

// Protocol constants
#define PROTOCOL_START_MARKER 0xAA
#define PROTOCOL_END_MARKER   0x55
#define MAX_PAYLOAD_SIZE      512  // Aumentata la dimensione massima del payload
#define HEADER_SIZE           6    // start(1) + type(1) + length(2) + crc(2)

// Message types
enum class MessageType : uint8_t {
    // Comandi dal client al robot
    CMD_MOVE = 0x01,           // Comando di movimento
    CMD_ROTATE = 0x02,         // Comando di rotazione
    CMD_MOW = 0x03,            // Controllo lama
    CMD_DOCK = 0x04,           // Torna alla base
    CMD_EMERGENCY = 0x05,      // Arresto di emergenza
    CMD_SET_SPEED = 0x06,      // Imposta velocità
    CMD_SET_BLADE_SPEED = 0x07,// Imposta velocità lama
    CMD_SET_NAV_MODE = 0x08,   // Imposta modalità navigazione
    CMD_JSON = 0x09,           // Comando JSON generico
    CMD_PING = 0x0A,           // Richiesta di risposta (keep-alive)
    
    // Risposte dal robot al client
    RSP_ACK = 0x81,            // Conferma generica
    RSP_STATUS = 0x82,         // Stato completo
    RSP_TELEMETRY = 0x83,      // Dati telemetria
    RSP_ERROR = 0x84,          // Messaggio di errore
    RSP_JSON = 0x85,           // Risposta JSON
    RSP_PONG = 0x86,           // Risposta a PING
    RSP_SENSOR_DATA = 0x87     // Dati dei sensori (compatibilità)
};

// Tipi di comando per il controllo manuale
enum class CommandType : uint8_t {
    NONE = 0,
    MANUAL_CONTROL,     // Controllo manuale (usa speed e turn)
    START_MOWING,       // Inizia il taglio
    STOP_MOWING,        // Ferma il taglio
    DOCK,               // Torna alla base
    PAUSE,              // Metti in pausa
    RESUME              // Riprendi
};

// Strutture dati per la comunicazione binaria
#pragma pack(push, 1)

// Pacchetto di stato (inviato dal robot)
typedef struct {
    uint16_t batteryVoltage;    // mV
    int16_t batteryCurrent;     // mA
    uint8_t batteryPercent;     // 0-100%
    uint8_t bumpers;            // Bitmask dei bumper (bit 0: frontale, bit 1: posteriore, ecc.)
    uint8_t rainDetected;       // 0-100% di umidità rilevata
    uint8_t ultrasonicDist;     // Distanza in cm (0-255 cm)
    int16_t imuAccelX;          // Accelerazione X (m/s² * 100)
    int16_t imuAccelY;          // Accelerazione Y (m/s² * 100)
    int16_t imuAccelZ;          // Accelerazione Z (m/s² * 100)
    int16_t imuGyroX;           // Giroscopio X (gradi/s * 100)
    int16_t imuGyroY;           // Giroscopio Y (gradi/s * 100)
    int16_t imuGyroZ;           // Giroscopio Z (gradi/s * 100)
    int32_t gpsLat;             // Latitudine * 1e7
    int32_t gpsLon;             // Longitudine * 1e7
    uint8_t gpsSats;            // Numero di satelliti
    uint8_t statusFlags;        // Bitmask di stato
} StatusPacket;

// Pacchetto di comando (ricevuto dal robot)
typedef struct {
    uint8_t command;            // Tipo di comando (CommandType)
    int8_t speed;               // Velocità (-100 a 100)
    int8_t turn;                // Sterzata (-100 a 100)
    uint8_t flags;              // Bitmask di opzioni
} CommandPacket;

#pragma pack(pop)

// Stati della connessione
enum class ConnectionState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR
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
// ------------------------------------------------------
// Inlined former RemoteCommand basic definitions
// ------------------------------------------------------

// Tipi comando remoti
enum class RemoteCommandType {
    NONE,
    START_MOWING,
    STOP_MOWING,
    PAUSE_MOWING,
    RESUME_MOWING,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    ROTATE_LEFT,
    ROTATE_RIGHT,
    SET_SPEED,
    SET_BLADE_SPEED,
    RETURN_TO_BASE,
    DOCK,
    UNDOCK,
    SET_NAV_MODE,
    EMERGENCY_STOP
};

struct CommandData {
    float speed = 0;
    float distance = 0;
    float angle = 0;
    NavigationMode navMode = NavigationMode::MANUAL;
};

struct RemoteStatus {
    bool isMoving = false;
    bool isMowing = false;
    bool isPaused = false;
    bool isDocked = false;
    bool isCharging = false;
    float batteryLevel = 0;
    float currentSpeed = 0;
    float batteryVoltage = 0;
    float batteryCurrent = 0;
    float leftMotorSpeed = 0;
    float rightMotorSpeed = 0;
    float bladeSpeed = 0;
    NavigationMode navMode = NavigationMode::MANUAL;
    float positionX = 0;
    float positionY = 0;
    float heading = 0;
    unsigned long timestamp = 0;
};

class WiFiRemote {
public:
    explicit WiFiRemote(Mower& mower);
    
    /**
     * @brief Inizializza la comunicazione seriale
     * @param baudRate Velocità di comunicazione (default: SERIAL_WIFI_BAUD)
     */
    void begin(unsigned long baudRate = SERIAL_WIFI_BAUD);
    
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
    
    // ---- Remote control helpers (inlined from RemoteCommand) ----
    void setRemoteControlEnabled(bool enable);
    bool isRemoteControlEnabled() const { return remoteControlEnabled_; }
    bool processRemoteCommand(RemoteCommandType cmd, const CommandData& data = {});
    void getRemoteStatus(RemoteStatus& status) const;
    
private:
    // Riferimento al mower
    Mower& mower_;

    // ---- Stato remoto integrato ----
    bool remoteControlEnabled_ = false;
    bool paused_ = false;
    unsigned long lastRemoteCmdTime_ = 0;
    
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
    
    // Definizione dei tipi di callback
    typedef void (*JsonCommandCallback)(const JsonDocument&, JsonDocument&);
    typedef void (*ConnectionStateCallback)(ConnectionState);
    
    // Metodi pubblici aggiuntivi
    /**
     * @brief Registra un gestore per un comando JSON personalizzato
     * @param command Nome del comando (deve essere una stringa costante)
     * @param callback Funzione da chiamare quando il comando viene ricevuto
     * @return true se registrato con successo, false se non c'è più spazio
     */
    bool registerCommand(const char* command, JsonCommandCallback callback);
    
    /**
     * @brief Rimuove un gestore di comando
     * @param command Nome del comando da rimuovere
     */
    void unregisterCommand(const char* command);
    
    /**
     * @brief Imposta il callback per i cambiamenti di stato della connessione
     * @param callback Funzione da chiamare quando lo stato della connessione cambia
     */
    void onConnectionStateChange(ConnectionStateCallback callback);
    
    // Metodi privati
    void processIncomingData();
    void processPacket(const uint8_t* data, uint16_t length);
    void sendAck(MessageType cmdType);
    void sendError(uint8_t errorCode, const String& message = "");
    bool sendPacket(MessageType type, const void* data = nullptr, uint16_t length = 0);
    uint16_t calculateCRC16(const uint8_t* data, uint16_t length) const;
    
    // Gestione comandi
    void handleJsonCommand(const String& jsonStr);
    void handleMoveCommand(const uint8_t* data, uint16_t length);
    void handleEmergencyCommand();
    
    // Gestione connessione
    void setConnectionState(ConnectionState state);
    void checkConnection();
    void resetConnection();
    
    // Invia lo stato completo del robot
    void sendRobotStatus();
    
    // Metodi per la gestione delle risposte JSON
    void sendJsonResponse(const JsonDocument& json);
    void createStatusJson(JsonDocument& json);
    
    // Gestione heartbeat
    void sendHeartbeat();
    
    // Struttura per i comandi personalizzati
    struct CommandHandler {
        const char* command;
        JsonCommandCallback callback;
    };
    
    static const int MAX_COMMANDS = 10;
    CommandHandler commandHandlers_[MAX_COMMANDS];
    int numCommands_ = 0;
    
    // Callback per lo stato della connessione
    ConnectionStateCallback connectionStateCallback_ = nullptr;
    
    // Stato della connessione
    ConnectionState connectionState_ = ConnectionState::DISCONNECTED;
    bool waitingForPong_ = false;
    unsigned long lastPingTime_ = 0;
};

#endif // WIFI_REMOTE_H
