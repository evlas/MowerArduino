#ifndef REMOTE_COMMAND_H
#define REMOTE_COMMAND_H

#include "../functions/Mower.h"
#include "../functions/MowerTypes.h"

/**
 * @brief Enumerazione dei comandi disponibili
 */
enum class RemoteCommandType {
    NONE,               // Nessun comando
    START_MOWING,       // Inizia il taglio
    STOP_MOWING,        // Ferma il taglio
    PAUSE_MOWING,       // Mette in pausa il taglio
    RESUME_MOWING,      // Riprende il taglio dalla pausa
    MOVE_FORWARD,       // Avanti
    MOVE_BACKWARD,      // Indietro
    TURN_LEFT,          // Gira a sinistra
    TURN_RIGHT,         // Gira a destra
    ROTATE_LEFT,        // Ruota sul posto a sinistra
    ROTATE_RIGHT,       // Ruota sul posto a destra
    SET_SPEED,          // Imposta velocità
    SET_BLADE_SPEED,    // Imposta velocità lama
    FOLLOW_PATH,        // Segui un percorso predefinito
    RETURN_TO_BASE,     // Torna alla base di ricarica
    DOCK,               // Aggancia alla stazione di ricarica
    UNDOCK,             // Sgancia dalla stazione di ricarica
    SET_NAV_MODE,       // Imposta modalità navigazione
    SET_WORKING_AREA,   // Imposta area di lavoro
    SET_SAFETY_MARGIN,  // Imposta margine di sicurezza
    GET_STATUS,         // Richiedi stato attuale
    GET_SENSOR_DATA,    // Richiedi dati dai sensori
    CALIBRATE_SENSORS,  // Calibra i sensori
    UPDATE_FIRMWARE,    // Aggiornamento firmware
    EMERGENCY_STOP,     // Arresto di emergenza
    CUSTOM_COMMAND      // Comando personalizzato
};


/**
 * @brief Struttura per i dati del comando
 */
struct CommandData {
    float speed;            // Velocità in percentuale (-100% a 100%)
    float distance;         // Distanza in metri (opzionale)
    float angle;            // Angolo in gradi (opzionale)
    float value;            // Valore generico
    int intValue;           // Valore intero generico
    bool boolValue;         // Valore booleano generico
    const char* stringValue; // Stringa generica
    NavigationMode navMode;  // Modalità di navigazione
};

/**
 * @brief Stato del comando remoto
 */
struct RemoteStatus {
    bool isMoving;              // Se il robot si sta muovendo
    bool isMowing;              // Se il taglio è attivo
    bool isPaused;              // Se il robot è in pausa
    bool isDocked;              // Se è agganciato alla base
    bool isCharging;            // Se sta caricando
    float batteryLevel;         // Livello batteria in percentuale
    float batteryVoltage;       // Tensione della batteria in volt
    float batteryCurrent;       // Corrente della batteria in ampere
    float currentSpeed;         // Velocità attuale media
    float leftMotorSpeed;       // Velocità motore sinistro (-100 a 100%)
    float rightMotorSpeed;      // Velocità motore destro (-100 a 100%)
    float bladeSpeed;           // Velocità lama (0-100%)
    NavigationMode navMode;     // Modalità di navigazione attuale
    float positionX;            // Posizione X in metri
    float positionY;            // Posizione Y in metri
    float heading;              // Orientamento in gradi (0-360)
    unsigned long timestamp;    // Timestamp dell'ultimo aggiornamento
    
    // Costruttore con valori di default
    RemoteStatus() : 
        isMoving(false),
        isMowing(false),
        isPaused(false),
        isDocked(false),
        isCharging(false),
        batteryLevel(0.0f),
        batteryVoltage(0.0f),
        batteryCurrent(0.0f),
        currentSpeed(0.0f),
        leftMotorSpeed(0.0f),
        rightMotorSpeed(0.0f),
        bladeSpeed(0.0f),
        navMode(NavigationMode::MANUAL),
        positionX(0.0f),
        positionY(0.0f),
        heading(0.0f),
        timestamp(0) {}
};

/**
 * @brief Classe per gestire i comandi remoti
 * 
 * Questa classe si occupa di ricevere e processare i comandi remoti,
 * delegando l'esecuzione alla classe Mower esistente.
 */
class RemoteCommand {
public:
    /**
     * @brief Costruttore
     * @param mower Riferimento all'istanza Mower esistente
     */
    explicit RemoteCommand(Mower& mower);
    
    /**
     * @brief Inizializza il gestore dei comandi
     */
    void begin();
    
    /**
     * @brief Elabora un comando ricevuto
     * @param command Tipo di comando
     * @param data Dati aggiuntivi del comando
     * @return true se il comando è stato accettato
     */
    bool processCommand(RemoteCommandType command, const CommandData& data = {});
    
    /**
     * @brief Aggiorna lo stato del controllo remoto
     * Da chiamare nel loop principale
     */
    void update();
    
    /**
     * @brief Imposta la modalità di controllo remoto
     * @param enable true per abilitare il controllo remoto
     */
    void setRemoteControlEnabled(bool enable);
    
    /**
     * @brief Verifica se il controllo remoto è abilitato
     * @return true se il controllo remoto è abilitato
     */
    bool isRemoteControlEnabled() const { return remoteControlEnabled_; }
    
    /**
     * @brief Invia un comando personalizzato
     * @param command Comando da inviare
     * @param params Parametri aggiuntivi
     * @return true se il comando è stato accettato
     */
    bool sendCustomCommand(const char* command, const char* params = nullptr);
    
    /**
     * @brief Imposta il gestore dei comandi personalizzati
     * @param callback Funzione di callback per gestire i comandi personalizzati
     */
    void setCustomCommandHandler(bool (*callback)(const char*, const char*));
    
    /**
     * @brief Imposta il gestore degli aggiornamenti di stato
     * @param callback Funzione di callback per gli aggiornamenti di stato
     */
    void setStatusUpdateHandler(void (*callback)(const RemoteStatus&));
    
    /**
     * @brief Ottiene lo stato corrente del robot
     * @param status Struttura in cui salvare lo stato
     */
    void getStatus(RemoteStatus& status) const;
    
private:
    // Riferimento al mower
    Mower& mower_;
    
    // Stato del controllo remoto
    bool remoteControlEnabled_;
    bool isPaused_;
    
    // Gestione del tempo
    unsigned long lastCommandTime_;
    unsigned long pauseStartTime_;
    unsigned long totalPauseTime_;
    
    // Gestione dei comandi personalizzati
    bool (*customCommandHandler_)(const char*, const char*);
    
    // Gestione degli aggiornamenti di stato
    void (*statusUpdateHandler_)(const RemoteStatus&);
    
    // Costanti
    static constexpr unsigned long COMMAND_TIMEOUT_MS = 30000; // Timeout comando in millisecondi
    static constexpr unsigned long MAX_PAUSE_TIME_MS = 3600000; // Massimo tempo di pausa (1 ora)
    
    // Metodi privati
    bool pauseOperations(bool pause);
    void updateStatus();
    void notifyStatusChanged();
    bool handleCustomCommand(const char* command, const char* params);
    void executeMovement(float leftSpeed, float rightSpeed, unsigned long duration = 0);
    
    // Metodo per notificare gli aggiornamenti di stato
    void notifyStatusUpdate() {
        if (statusUpdateHandler_) {
            RemoteStatus status;
            
            // Imposta lo stato del sistema
            // Nota: Assumiamo che questi membri esistano nella classe Mower
            // Se non esistono, sostituisci con i metodi getter appropriati
            status.isMoving = (mower_.leftMotorSpeed_ != 0 || mower_.rightMotorSpeed_ != 0);
            status.isMowing = mower_.bladesRunning_;
            status.isPaused = (mower_.getState() == State::PAUSED);
            status.isDocked = mower_.isDocked();
            status.isCharging = mower_.isCharging();
            
            // Dati della batteria
            status.batteryLevel = mower_.getBatteryPercentage();
            // Se questi metodi non esistono, impostali a 0 o a un valore predefinito
            status.batteryVoltage = 0.0f;  // Sostituisci con mower_.getBatteryVoltage() se disponibile
            status.batteryCurrent = 0.0f;  // Sostituisci con mower_.getBatteryCurrent() se disponibile
            
            // Dati di movimento
            status.leftMotorSpeed = mower_.leftMotorSpeed_;
            status.rightMotorSpeed = mower_.rightMotorSpeed_;
            status.bladeSpeed = mower_.bladeSpeed_;
            status.currentSpeed = (mower_.leftMotorSpeed_ + mower_.rightMotorSpeed_) / 2.0f;
            
            // Dati di navigazione
            status.navMode = mower_.getNavigationMode();
            
            // Posizione e orientamento
            // Se questi metodi non esistono, impostali a 0 o a un valore predefinito
            status.positionX = 0.0f;  // Sostituisci con mower_.getPositionX() se disponibile
            status.positionY = 0.0f;  // Sostituisci con mower_.getPositionY() se disponibile
            status.heading = 0.0f;    // Sostituisci con mower_.getHeading() se disponibile
            
            // Timestamp
            status.timestamp = millis();
            
            // Notifica il gestore
            statusUpdateHandler_(status);
        }
    }
};

#endif // REMOTE_COMMAND_H
