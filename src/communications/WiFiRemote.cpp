#include "WiFiRemote.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include "../config.h"  // SERIAL_WIFI is defined here

// Default baud rate for WiFi module if not defined in config.h
#ifndef SERIAL_WIFI_BAUD
  #define SERIAL_WIFI_BAUD 115200
#endif

// CRC-16-CCITT polynomial: x^16 + x^12 + x^5 + 1 (0x1021)
#define CRC16_POLY 0x1021

WiFiRemote::WiFiRemote(RemoteCommand& remote)
    : remote_(remote),
      connected_(false),
      lastStatusTime_(0),
      lastCommandTime_(0),
      rxIndex_(0),
      inPacket_(false) {
    memset(lastStatus_, 0, sizeof(lastStatus_));
    memset(rxBuffer_, 0, sizeof(rxBuffer_));
}

void WiFiRemote::begin(unsigned long baudRate) {
    // Use the configured baud rate if none specified
    if (baudRate == 0) {
        baudRate = SERIAL_WIFI_BAUD;
    }
    
    // Initialize the serial port for WiFi module
    SERIAL_WIFI.begin(baudRate);
    connected_ = false;
    lastCommandTime_ = millis();
    
    #ifdef DEBUG_MODE
    DEBUG_PRINT(F("WiFiRemote: Inizializzazione SERIAL_WIFI su baud rate: "));
    DEBUG_PRINTLN(baudRate);
    #endif
}

void WiFiRemote::update() {
    processIncomingData();
    
    // Invia aggiornamento di stato periodico
    unsigned long currentTime = millis();
    if (currentTime - lastStatusTime_ >= STATUS_UPDATE_INTERVAL) {
        sendStatus(false); // Invia solo se ci sono cambiamenti
        lastStatusTime_ = currentTime;
    }
    
    // Gestione timeout connessione
    if (connected_ && (currentTime - lastCommandTime_ > CONNECTION_TIMEOUT)) {
        connected_ = false;
        #ifdef DEBUG_MODE
        DEBUG_PRINTLN(F("WiFiRemote: Timeout connessione"));
        #endif
    }
}

void WiFiRemote::processIncomingData() {
    while (SERIAL_WIFI.available()) {
        uint8_t c = SERIAL_WIFI.read();
        
        if (!inPacket_) {
            if (c == PROTOCOL_START_MARKER) {
                inPacket_ = true;
                rxIndex_ = 0;
                rxBuffer_[rxIndex_++] = c;
            }
            continue;
        }
        
        // Se siamo in un pacchetto, aggiungi al buffer
        if (rxIndex_ < sizeof(rxBuffer_)) {
            rxBuffer_[rxIndex_++] = c;
        } else {
            // Buffer overflow, reset
            inPacket_ = false;
            sendLog("Errore: Pacchetto troppo grande", true);
            continue;
        }
        
        // Controlla se abbiamo ricevuto il marker di fine pacchetto
        if (c == PROTOCOL_END_MARKER) {
            // Processa il pacchetto completo
            processPacket(rxBuffer_, rxIndex_);
            inPacket_ = false;
        }
    }
}

/**
 * @brief Processa un pacchetto ricevuto
 * @param data Puntatore ai dati del pacchetto
 * @param length Lunghezza totale del pacchetto (inclusi header e terminatore)
 */
void WiFiRemote::processPacket(const uint8_t* data, uint16_t length) {
    // Verifica la lunghezza minima e i marker di inizio/fine
    if (length < HEADER_SIZE || data[0] != PROTOCOL_START_MARKER || data[length-1] != PROTOCOL_END_MARKER) {
        #ifdef DEBUG_MODE
        DEBUG_PRINT(F("WiFiRemote: Pacchetto non valido - "));
        DEBUG_PRINT(F("Lunghezza: ")); DEBUG_PRINT(length);
        DEBUG_PRINT(F(" Start: 0x")); DEBUG_PRINT(data[0], HEX);
        DEBUG_PRINT(F(" End: 0x")); DEBUG_PRINTLN(data[length-1], HEX);
        #endif
        sendLog("Pacchetto non valido", true);
        return;
    }
    
    // Estrai i campi dell'header
    MessageType type = static_cast<MessageType>(data[1]);
    uint16_t payloadLength = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    uint16_t receivedCrc = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    
    // Verifica la lunghezza del payload
    if (length != HEADER_SIZE + payloadLength) {
        #ifdef DEBUG_MODE
        DEBUG_PRINT(F("WiFiRemote: Lunghezza payload non valida - "));
        DEBUG_PRINT(payloadLength); DEBUG_PRINT(F(" vs "));
        DEBUG_PRINTLN(length - HEADER_SIZE);
        #endif
        sendLog("Lunghezza pacchetto non valida", true);
        return;
    }
    
    // Calcola e verifica il CRC
    uint16_t calculatedCrc = calculateCRC16(data + 6, payloadLength);
    if (calculatedCrc != receivedCrc) {
        #ifdef DEBUG_MODE
        DEBUG_PRINT(F("WiFiRemote: Errore CRC - "));
        DEBUG_PRINT(receivedCrc, HEX); DEBUG_PRINT(F(" vs "));
        DEBUG_PRINTLN(calculatedCrc, HEX);
        #endif
        sendLog("Errore CRC", true);
        return;
    }
    
    // Aggiorna il timestamp dell'ultimo comando
    lastCommandTime_ = millis();
    bool wasConnected = connected_;
    connected_ = true;
    
    if (!wasConnected) {
        #ifdef DEBUG_MODE
        DEBUG_PRINTLN(F("WiFiRemote: Connessione stabilita"));
        #endif
        sendLog("Connessione WiFi stabilita");
    }
    
    // Elabora il comando
    const uint8_t* payload = data + 6;
    CommandData cmdData;
    bool commandProcessed = false;
    
    #ifdef DEBUG_MODE
    DEBUG_PRINT(F("WiFiRemote: Elaborazione comando 0x"));
    DEBUG_PRINTLN(static_cast<uint8_t>(type), HEX);
    #endif
    
    switch (type) {
        case MessageType::CMD_MOVE: {
            if (payloadLength >= sizeof(MoveCommand)) {
                const MoveCommand* cmd = reinterpret_cast<const MoveCommand*>(payload);
                cmdData.speed = static_cast<float>(cmd->speed);  // Converti a float mantenendo il segno
                cmdData.distance = cmd->distance;
                
                #ifdef DEBUG_MODE
                DEBUG_PRINT(F("WiFiRemote: MOVE - Speed: "));
                DEBUG_PRINT(cmdData.speed);
                DEBUG_PRINT(F(" Distance: "));
                DEBUG_PRINT(cmdData.distance);
                DEBUG_PRINT(F(" Direction: "));
                DEBUG_PRINTLN(cmd->direction);
                #endif
                
                switch (cmd->direction) {
                    case 0: commandProcessed = remote_.processCommand(RemoteCommandType::MOVE_FORWARD, cmdData); break;
                    case 1: commandProcessed = remote_.processCommand(RemoteCommandType::MOVE_BACKWARD, cmdData); break;
                    case 2: commandProcessed = remote_.processCommand(RemoteCommandType::TURN_LEFT, cmdData); break;
                    case 3: commandProcessed = remote_.processCommand(RemoteCommandType::TURN_RIGHT, cmdData); break;
                    default: break;
                }
            }
            break;
        }
        
        case MessageType::CMD_ROTATE: {
            if (payloadLength >= sizeof(RotateCommand)) {
                const RotateCommand* cmd = reinterpret_cast<const RotateCommand*>(payload);
                cmdData.speed = static_cast<float>(cmd->speed);  // Converti a float mantenendo il segno
                cmdData.angle = cmd->angle;
                
                #ifdef DEBUG_MODE
                DEBUG_PRINT(F("WiFiRemote: ROTATE - Speed: "));
                DEBUG_PRINT(cmdData.speed);
                DEBUG_PRINT(F(" Angle: "));
                DEBUG_PRINT(cmdData.angle);
                DEBUG_PRINT(F(" Direction: "));
                DEBUG_PRINTLN(cmd->direction);
                #endif
                
                if (cmd->direction == 0) {
                    commandProcessed = remote_.processCommand(RemoteCommandType::ROTATE_LEFT, cmdData);
                } else if (cmd->direction == 1) {
                    commandProcessed = remote_.processCommand(RemoteCommandType::ROTATE_RIGHT, cmdData);
                }
            }
            break;
        }
        
        case MessageType::CMD_MOW: {
            if (payloadLength >= 1) {
                #ifdef DEBUG_MODE
                DEBUG_PRINT(F("WiFiRemote: MOW - Action: "));
                DEBUG_PRINTLN(payload[0]);
                #endif
                
                switch (payload[0]) {
                    case 0: commandProcessed = remote_.processCommand(RemoteCommandType::START_MOWING); break;
                    case 1: commandProcessed = remote_.processCommand(RemoteCommandType::STOP_MOWING); break;
                    case 2: commandProcessed = remote_.processCommand(RemoteCommandType::PAUSE_MOWING); break;
                    case 3: commandProcessed = remote_.processCommand(RemoteCommandType::RESUME_MOWING); break;
                    default: break;
                }
            }
            break;
        }
        
        case MessageType::CMD_DOCK: {
            if (payloadLength >= 1) {
                #ifdef DEBUG_MODE
                DEBUG_PRINT(F("WiFiRemote: DOCK - Action: "));
                DEBUG_PRINTLN(payload[0]);
                #endif
                
                switch (payload[0]) {
                    case 0: commandProcessed = remote_.processCommand(RemoteCommandType::RETURN_TO_BASE); break;
                    case 1: commandProcessed = remote_.processCommand(RemoteCommandType::DOCK); break;
                    case 2: commandProcessed = remote_.processCommand(RemoteCommandType::UNDOCK); break;
                    default: break;
                }
            }
            break;
        }
        
        case MessageType::CMD_EMERGENCY:
            #ifdef DEBUG_MODE
            DEBUG_PRINTLN(F("WiFiRemote: EMERGENCY_STOP"));
            #endif
            commandProcessed = remote_.processCommand(RemoteCommandType::EMERGENCY_STOP);
            break;
            
        case MessageType::CMD_SET_SPEED:
            if (payloadLength >= 1) {
                // Converti il valore firmato in float mantenendo il segno
                int8_t speed = static_cast<int8_t>(payload[0]);
                cmdData.speed = static_cast<float>(speed);
                
                #ifdef DEBUG_MODE
                DEBUG_PRINT(F("WiFiRemote: SET_SPEED - "));
                DEBUG_PRINTLN(cmdData.speed);
                #endif
                
                commandProcessed = remote_.processCommand(RemoteCommandType::SET_SPEED, cmdData);
            }
            break;
            
        case MessageType::CMD_SET_BLADE_SPEED:
            if (payloadLength >= 1) {
                // Per la velocità delle lame usiamo solo valori positivi (0-100%)
                cmdData.speed = static_cast<float>(payload[0] & 0x7F);  // Assicura valore positivo
                
                #ifdef DEBUG_MODE
                DEBUG_PRINT(F("WiFiRemote: SET_BLADE_SPEED - "));
                DEBUG_PRINTLN(cmdData.speed);
                #endif
                
                commandProcessed = remote_.processCommand(RemoteCommandType::SET_BLADE_SPEED, cmdData);
            }
            break;
            
        case MessageType::CMD_SET_NAV_MODE:
            if (payloadLength >= 1) {
                cmdData.navMode = static_cast<NavigationMode>(payload[0]);
                
                #ifdef DEBUG_MODE
                DEBUG_PRINT(F("WiFiRemote: SET_NAV_MODE - "));
                DEBUG_PRINTLN(static_cast<int>(cmdData.navMode));
                #endif
                
                commandProcessed = remote_.processCommand(RemoteCommandType::SET_NAV_MODE, cmdData);
            }
            break;
            
        case MessageType::CMD_CUSTOM:
            // Implementa la logica per i comandi personalizzati
            #ifdef DEBUG_MODE
            DEBUG_PRINTLN(F("WiFiRemote: Comando personalizzato ricevuto"));
            #endif
            if (payloadLength > 0) {
                // Estrai il comando e i parametri
                const char* cmd = reinterpret_cast<const char*>(payload);
                const char* params = (payloadLength > 1) ? reinterpret_cast<const char*>(payload + 1) : "";
                commandProcessed = remote_.sendCustomCommand(cmd, params);
            }
            break;
            
        case MessageType::RSP_STATUS:
            // Invia lo stato completo
            #ifdef DEBUG_MODE
            DEBUG_PRINTLN(F("WiFiRemote: Richiesta stato"));
            #endif
            sendStatus(true);
            commandProcessed = true;
            break;
            break;
            
        default:
            #ifdef DEBUG_MODE
            DEBUG_PRINT(F("WiFiRemote: Comando sconosciuto: 0x"));
            DEBUG_PRINTLN(static_cast<uint8_t>(type), HEX);
            #endif
            break;
    }
    
    if (commandProcessed) {
        sendAck(type);
    } else {
        sendLog("Comando non valido", true);
    }
}

/**
 * @brief Invia lo stato corrente al client
 * @param force Se true, forza l'invio anche se lo stato non è cambiato
 */
void WiFiRemote::sendStatus(bool force) {
    static uint8_t lastStatus[32] = {0};
    uint8_t currentStatus[32] = {0};
    bool statusChanged = force;
    
    // Ottieni lo stato corrente
    RemoteStatus status;
    remote_.getStatus(status);
    
    // Prepara il buffer di stato
    currentStatus[0] = static_cast<uint8_t>(status.batteryLevel);
    currentStatus[1] = static_cast<uint8_t>(status.currentSpeed + 128); // Converti in 0-255 (-128 a 127 -> 0-255)
    currentStatus[2] = (status.isMoving ? 0x01 : 0x00) |
                      (status.isMowing ? 0x02 : 0x00) |
                      (status.isPaused ? 0x04 : 0x00) |
                      (status.isDocked ? 0x08 : 0x00) |
                      (status.isCharging ? 0x10 : 0x00);
    currentStatus[3] = static_cast<uint8_t>(status.navMode);
    
    // Controlla se lo stato è cambiato
    if (!force) {
        for (int i = 0; i < 16; i++) {
            if (currentStatus[i] != lastStatus[i]) {
                statusChanged = true;
                break;
            }
        }
    }
    
    // Invia solo se lo stato è cambiato o se è forzato
    if (statusChanged) {
        #ifdef DEBUG_MODE
        DEBUG_PRINTLN(F("WiFiRemote: Invio stato"));
        #endif
        
        // Invia il pacchetto di stato
        sendPacket(MessageType::RSP_STATUS, currentStatus, 16);
        
        // Aggiorna l'ultimo stato inviato
        memcpy(lastStatus, currentStatus, sizeof(lastStatus));
    }
    
    // Invia anche una versione JSON dello stato
    DynamicJsonDocument jsonDoc(512);
    createStatusJson(jsonDoc);
    sendJsonResponse(jsonDoc);
}

void WiFiRemote::sendLog(const String& message, bool isError) {
    // Invia solo i primi 120 caratteri per evitare pacchetti troppo grandi
    uint8_t len = min(message.length(), 120);
    uint8_t buffer[len + 1];
    
    buffer[0] = isError ? 1 : 0;  // Flag di errore
    memcpy(buffer + 1, message.c_str(), len);
    
    sendPacket(MessageType::RSP_LOG, buffer, len + 1);
}

void WiFiRemote::sendAck(MessageType cmdType) {
    uint8_t ack = static_cast<uint8_t>(cmdType);
    sendPacket(MessageType::RSP_ACK, &ack, 1);
}

void WiFiRemote::sendError(uint8_t errorCode) {
    sendPacket(MessageType::RSP_ERROR, &errorCode, 1);
}

/**
 * @brief Invia un pacchetto dati al client
 * @param type Tipo di messaggio
 * @param data Puntatore ai dati da inviare (può essere nullptr)
 * @param length Lunghezza dei dati in byte
 * @return true se l'invio è riuscito, false altrimenti
 */
bool WiFiRemote::sendPacket(MessageType type, const void* data, uint16_t length) {
    // Verifica la lunghezza massima
    if (length > MAX_PAYLOAD_SIZE) {
        #ifdef DEBUG_MODE
        DEBUG_PRINT(F("WiFiRemote: Dati troppo grandi per il pacchetto: "));
        DEBUG_PRINT(length);
        DEBUG_PRINT(F(" > "));
        DEBUG_PRINTLN(MAX_PAYLOAD_SIZE);
        #endif
        sendLog("Dati troppo grandi per il pacchetto", true);
        return false;
    }
    
    // Alloca il buffer per il pacchetto completo
    uint8_t packet[HEADER_SIZE + MAX_PAYLOAD_SIZE + 1]; // +1 per il terminatore
    
    // Compila l'header
    packet[0] = PROTOCOL_START_MARKER;
    packet[1] = static_cast<uint8_t>(type);
    packet[2] = (length >> 8) & 0xFF;  // Lunghezza MSB
    packet[3] = length & 0xFF;         // Lunghezza LSB
    
    // Calcola il CRC
    uint16_t crc = 0xFFFF;
    if (data != nullptr && length > 0) {
        crc = calculateCRC16(static_cast<const uint8_t*>(data), length);
    }
    packet[4] = (crc >> 8) & 0xFF;     // CRC MSB
    packet[5] = crc & 0xFF;            // CRC LSB
    
    // Copia i dati se presenti
    if (data != nullptr && length > 0) {
        memcpy(packet + HEADER_SIZE, data, length);
    }
    
    // Aggiungi il terminatore
    packet[HEADER_SIZE + length] = PROTOCOL_END_MARKER;
    
    #ifdef DEBUG_MODE
    DEBUG_PRINT(F("WiFiRemote: Invio pacchetto - Tipo: 0x"));
    DEBUG_PRINT(static_cast<uint8_t>(type), HEX);
    DEBUG_PRINT(F(" Lunghezza: "));
    DEBUG_PRINT(length);
    DEBUG_PRINT(F(" CRC: 0x"));
    DEBUG_PRINTLN(crc, HEX);
    
    // Stampa i primi 16 byte del payload in esadecimale per il debug
    if (length > 0) {
        DEBUG_PRINT(F("Payload (max 16 byte):"));
        for (size_t i = 0; i < min(length, 16); i++) {
            if (i % 8 == 0) DEBUG_PRINTLN();
            DEBUG_PRINT(F("0x"));
            if (packet[HEADER_SIZE + i] < 0x10) DEBUG_PRINT('0');
            DEBUG_PRINT(packet[HEADER_SIZE + i], HEX);
            DEBUG_PRINT(' ');
        }
        DEBUG_PRINTLN();
    }
    #endif
    
    // Invia il pacchetto
    size_t bytesWritten = SERIAL_WIFI.write(packet, HEADER_SIZE + length + 1);
    SERIAL_WIFI.flush();
    
    // Verifica che tutti i byte siano stati scritti
    if (bytesWritten != static_cast<size_t>(HEADER_SIZE + length + 1)) {
        #ifdef DEBUG_MODE
        DEBUG_PRINT(F("WiFiRemote: Errore nell'invio del pacchetto - Scritti: "));
        DEBUG_PRINT(bytesWritten);
        DEBUG_PRINT(F(" / "));
        DEBUG_PRINTLN(HEADER_SIZE + length + 1);
        #endif
        return false;
    }
    
    return true;
}

/**
 * @brief Calcola il checksum CRC-16-CCITT
 * @param data Puntatore ai dati
 * @param length Lunghezza dei dati in byte
 * @return Il checksum CRC-16-CCITT calcolato
 */
uint16_t WiFiRemote::calculateCRC16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;  // Valore iniziale
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (static_cast<uint16_t>(data[i]) << 8);
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;  // Polinomio CRC-16-CCITT
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Crea un oggetto JSON con lo stato corrente del sistema
 * @param json Documento JSON da popolare con i dati di stato
 */
void WiFiRemote::createStatusJson(JsonDocument& json) {
    // Informazioni di connessione
    json["connected"] = connected_;
    json["lastCommandTime"] = lastCommandTime_;
    
    // Ottieni lo stato corrente dal RemoteCommand
    RemoteStatus status;
    remote_.getStatus(status);
    
    // Stato del sistema
    json["isMoving"] = status.isMoving;
    json["isMowing"] = status.isMowing;
    json["isPaused"] = status.isPaused;
    json["isDocked"] = status.isDocked;
    json["isCharging"] = status.isCharging;
    
    // Dati della batteria
    json["batteryLevel"] = status.batteryLevel;
    json["batteryVoltage"] = status.batteryVoltage;
    json["batteryCurrent"] = status.batteryCurrent;
    
    // Dati di movimento
    json["currentSpeed"] = status.currentSpeed;
    json["leftMotorSpeed"] = status.leftMotorSpeed;
    json["rightMotorSpeed"] = status.rightMotorSpeed;
    json["bladeSpeed"] = status.bladeSpeed;
    
    // Dati di navigazione
    json["navMode"] = static_cast<uint8_t>(status.navMode);
    
    // Posizione e orientamento
    JsonObject position = json.createNestedObject("position");
    position["x"] = status.positionX;
    position["y"] = status.positionY;
    position["heading"] = status.heading;
    
    // Mantieni anche i campi diretti per compatibilità
    json["posX"] = status.positionX;
    json["posY"] = status.positionY;
    json["heading"] = status.heading;
    
    // Timestamp
    json["timestamp"] = status.timestamp;
    
    // Aggiungi informazioni sul sistema
    json["uptime"] = millis() / 1000;  // Uptime in secondi
    // Note: WiFi signal strength and heap info not available on Arduino Mega 2560
    // without additional hardware/modules dall'avvio
}

/**
 * @brief Invia una risposta JSON al client
 * @param json Documento JSON da inviare
 */
void WiFiRemote::sendJsonResponse(JsonDocument& json) {
    // Aggiungi il timestamp se non è già presente
    if (!json.containsKey("timestamp")) {
        json["timestamp"] = millis();
    }
    
    // Serializza il JSON in una stringa
    String jsonStr;
    serializeJson(json, jsonStr);
    
    #ifdef DEBUG_MODE
    DEBUG_PRINTLN(F("WiFiRemote: Invio risposta JSON:"));
    serializeJsonPretty(json, SERIAL_DEBUG);
    DEBUG_PRINTLN();
    #endif
    
    // Invia il pacchetto JSON
    sendPacket(MessageType::RSP_LOG, jsonStr.c_str(), jsonStr.length());
}
