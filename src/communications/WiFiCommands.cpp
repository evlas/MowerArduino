#include <Arduino.h>
#include <ArduinoJson.h>
#include "WiFiCommands.h"

// Include EEPROM related headers if enabled
#ifdef ENABLE_EEPROM
#include "../eeprom/EEPROMConfig.h"
#include "../eeprom/EEPROMManager.h"

// Prototipi delle funzioni EEPROM
void sendEepromSettings();
void handleSetEepromSettings(const JsonVariant& params);
void handleResetEepromToDefault();
#endif

// Includi le dipendenze base dopo le strutture EEPROM
#include "WiFiSerialBridge.h"
#include "WiFiCommands.h"

// Riferimento all'istanza globale di commandHandler (definita in MowerArduino.ino)
extern CommandHandler commandHandler;

// Riferimento al bridge WiFi
static WiFiSerialBridge* wifiBridgeInstance = nullptr;

// Riferimento al monitor della batteria (definito in MowerArduino.ino)
#ifdef ENABLE_BATTERY_MONITOR
extern INA226_WE batteryMonitor;
#endif

// Variabile statica per il riferimento al gestore dei comandi
static CommandHandler* cmdHandler = nullptr;

void handleWiFiCommand(const String& command, const JsonVariant& params) {
    if (!cmdHandler) {
        #ifdef SERIAL_DEBUG
            SERIAL_DEBUG.println("ERRORE: CommandHandler non inizializzato!");
        #endif
        return;
    }

    #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.print("Comando ricevuto: ");
        SERIAL_DEBUG.println(command);
    #endif

    // Gestione dei comandi
    if (command == "startMowing") {
        float speed = DEFAULT_MOTOR_SPEED;
        if (params.containsKey("speed")) {
            speed = params["speed"].as<float>();
        }
        cmdHandler->startMowing(speed);
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Mowing started");
    }
    else if (command == "stopMowing") {
        cmdHandler->stopMowing();
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Mowing stopped");
    }
    else if (command == "moveForward") {
        float speed = DEFAULT_MOTOR_SPEED;
        if (params.containsKey("speed")) {
            speed = params["speed"].as<float>();
        }
        cmdHandler->moveForward(speed);
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Moving forward");
    }
    else if (command == "moveBackward") {
        float speed = DEFAULT_MOTOR_SPEED;
        if (params.containsKey("speed")) {
            speed = params["speed"].as<float>();
        }
        cmdHandler->moveBackward(speed);
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Moving backward");
    }
    else if (command == "stop") {
        cmdHandler->stop();
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Stopped");
    }
    else if (command == "turnLeft") {
        float angle = 90.0f;
        if (params.containsKey("angle")) {
            angle = params["angle"].as<float>();
        }
        cmdHandler->turnLeft(angle);
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Turning left");
    }
    else if (command == "turnRight") {
        float angle = 90.0f;
        if (params.containsKey("angle")) {
            angle = params["angle"].as<float>();
        }
        cmdHandler->turnRight(angle);
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Turning right");
    }
    else if (command == "setBladeSpeed") {
        if (params.containsKey("speed")) {
            uint8_t speed = params["speed"].as<uint8_t>();
            cmdHandler->setBladeSpeed(speed);
            if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Blade speed set");
        } else {
            if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("error", "Missing speed parameter");
        }
    }
    else if (command == "startBlade") {
        if (params.containsKey("speed")) {
            uint8_t speed = params["speed"].as<uint8_t>();
            cmdHandler->startBlade(speed);
        } else {
            cmdHandler->startBlade();
        }
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Blade started");
    }
    else if (command == "stopBlade") {
        cmdHandler->stopBlade();
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Blade stopped");
    }
    else if (command == "returnToBase") {
        cmdHandler->returnToBase();
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Returning to base");
    }
    else if (command == "getStatus") {
        // Invia lo stato attuale come risposta
        DynamicJsonDocument status(1024);
        status["state"] = static_cast<int>(mowerStateMachine.getCurrentState());
        #ifdef ENABLE_BATTERY_MONITOR
        status["batteryLevel"] = batteryMonitor.getBusVoltage_V();
        #else
        status["batteryLevel"] = 0.0f; // Battery monitoring not enabled
        #endif
        
        // Aggiungi i dati dei sensori
        cmdHandler->getSensorData(status);
        
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Current status", status);
    }
    else if (command == "getSensorData") {
        // Invia solo i dati dei sensori
        DynamicJsonDocument sensorData(1024);
        
        // Popola i dati dei sensori
        cmdHandler->getSensorData(sensorData);
        
        if (wifiBridgeInstance) {
            wifiBridgeInstance->sendResponse("ok", "Sensor data", sensorData);
        }
    }
    else if (command == "enableTelemetry") {
        uint32_t interval = 1000;
        if (params.containsKey("interval")) {
            interval = params["interval"].as<uint32_t>();
        }
        cmdHandler->enableTelemetry(true);
        cmdHandler->setTelemetryInterval(interval);
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Telemetry enabled");
    }
    else if (command == "disableTelemetry") {
        cmdHandler->enableTelemetry(false);
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Telemetry disabled");
    }
    #ifdef ENABLE_EEPROM
    else if (command == "getEepromSettings") {
        sendEepromSettings();
    }
    else if (command == "setEepromSettings") {
        handleSetEepromSettings(params);
    }
    else if (command == "resetEepromToDefault") {
        handleResetEepromToDefault();
    }
    else {
        // Comando non riconosciuto
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("error", "Comando non riconosciuto: " + command);
    }
}

#ifdef ENABLE_EEPROM
void sendEepromSettings() {
    if (!wifiBridgeInstance) return;
    
    EEPROMSettings settings;
    if (!EEPROMManager::loadSettings(settings)) {
        wifiBridgeInstance->sendResponse("error", "Impossibile caricare le impostazioni EEPROM");
        return;
    }
    
    // Crea un documento JSON per le impostazioni
    DynamicJsonDocument doc(4096);  // Aumentato a 4KB per contenere tutte le impostazioni
    JsonObject data = doc.to<JsonObject>();
    
    // Informazioni generali
    data["version"] = settings.version;
    
    // Impostazioni di sistema
    JsonObject system = data.createNestedObject("system");
    system["language"] = settings.system.language;
    system["use24HourFormat"] = settings.system.use24HourFormat;
    system["useMetric"] = settings.system.useMetric;
    system["brightness"] = settings.system.brightness;
    system["volume"] = settings.system.volume;
    
    // Impostazioni tagliaerba
    JsonObject mower = data.createNestedObject("mower");
    mower["cuttingHeight"] = settings.mower.cuttingHeight;
    mower["bladeSpeed"] = settings.mower.bladeSpeed;
    mower["mowingPattern"] = settings.mower.mowingPattern;
    mower["rainDelay"] = settings.mower.rainDelay;
    mower["rainDelayUntil"] = settings.mower.rainDelayUntil;
    
    // Impostazioni di navigazione
    JsonObject navigation = data.createNestedObject("navigation");
    navigation["perimeterMode"] = settings.navigation.perimeterMode;
    navigation["obstacleSensitivity"] = settings.navigation.obstacleSensitivity;
    navigation["gpsEnabled"] = settings.navigation.gpsEnabled;
    navigation["gpsUpdateInterval"] = settings.navigation.gpsUpdateInterval;
    
    // Impostazioni di manutenzione
    JsonObject maintenance = data.createNestedObject("maintenance");
    maintenance["totalMowingTime"] = settings.maintenance.totalMowingTime;
    maintenance["bladeHours"] = settings.maintenance.bladeHours;
    maintenance["lastMaintenance"] = settings.maintenance.lastMaintenance;
    maintenance["nextMaintenance"] = settings.maintenance.nextMaintenance;
    
    // Parametri PID
    JsonObject pid = data.createNestedObject("pid");
    
    JsonObject leftPid = pid.createNestedObject("left");
    leftPid["kp"] = settings.leftPID.kp;
    leftPid["ki"] = settings.leftPID.ki;
    leftPid["kd"] = settings.leftPID.kd;
    
    JsonObject rightPid = pid.createNestedObject("right");
    rightPid["kp"] = settings.rightPID.kp;
    rightPid["ki"] = settings.rightPID.ki;
    rightPid["kd"] = settings.rightPID.kd;
    
    // Posizione home
    JsonObject homePos = data.createNestedObject("homePosition");
    homePos["x"] = settings.homePosition.x;
    homePos["y"] = settings.homePosition.y;
    homePos["theta"] = settings.homePosition.theta;
    homePos["isSet"] = settings.homePosition.isSet;
    
    // Invia la risposta
    wifiBridgeInstance->sendResponse("ok", "eepromSettings", doc);
}

void handleSetEepromSettings(const JsonVariant& params) {
    if (!wifiBridgeInstance || !params.is<JsonObject>()) {
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("error", "Parametri non validi");
        return;
    }
    
    EEPROMSettings settings;
    if (!EEPROMManager::loadSettings(settings)) {
        wifiBridgeInstance->sendResponse("error", "Impossibile caricare le impostazioni correnti");
        return;
    }
    
    JsonObject json = params.as<JsonObject>();
    
    // Aggiorna solo i campi presenti nel JSON
    if (json.containsKey("system")) {
        JsonObject system = json["system"];
        if (system.containsKey("language")) settings.system.language = system["language"];
        if (system.containsKey("use24HourFormat")) settings.system.use24HourFormat = system["use24HourFormat"];
        if (system.containsKey("useMetric")) settings.system.useMetric = system["useMetric"];
        if (system.containsKey("brightness")) settings.system.brightness = system["brightness"];
        if (system.containsKey("volume")) settings.system.volume = system["volume"];
    }
    
    if (json.containsKey("mower")) {
        JsonObject mower = json["mower"];
        if (mower.containsKey("cuttingHeight")) settings.mower.cuttingHeight = mower["cuttingHeight"];
        if (mower.containsKey("bladeSpeed")) settings.mower.bladeSpeed = mower["bladeSpeed"];
        if (mower.containsKey("mowingPattern")) settings.mower.mowingPattern = mower["mowingPattern"];
        if (mower.containsKey("rainDelay")) settings.mower.rainDelay = mower["rainDelay"];
        if (mower.containsKey("rainDelayUntil")) settings.mower.rainDelayUntil = mower["rainDelayUntil"];
    }
    
    if (json.containsKey("navigation")) {
        JsonObject navigation = json["navigation"];
        if (navigation.containsKey("perimeterMode")) settings.navigation.perimeterMode = navigation["perimeterMode"];
        if (navigation.containsKey("obstacleSensitivity")) settings.navigation.obstacleSensitivity = navigation["obstacleSensitivity"];
        if (navigation.containsKey("gpsEnabled")) settings.navigation.gpsEnabled = navigation["gpsEnabled"];
        if (navigation.containsKey("gpsUpdateInterval")) settings.navigation.gpsUpdateInterval = navigation["gpsUpdateInterval"];
    }
    
    if (json.containsKey("maintenance")) {
        JsonObject maintenance = json["maintenance"];
        if (maintenance.containsKey("totalMowingTime")) settings.maintenance.totalMowingTime = maintenance["totalMowingTime"];
        if (maintenance.containsKey("bladeHours")) settings.maintenance.bladeHours = maintenance["bladeHours"];
        if (maintenance.containsKey("lastMaintenance")) settings.maintenance.lastMaintenance = maintenance["lastMaintenance"];
        if (maintenance.containsKey("nextMaintenance")) settings.maintenance.nextMaintenance = maintenance["nextMaintenance"];
    }
    
    if (json.containsKey("pid")) {
        JsonObject pid = json["pid"];
        
        if (pid.containsKey("left")) {
            JsonObject leftPid = pid["left"];
            if (leftPid.containsKey("kp")) settings.leftPID.kp = leftPid["kp"];
            if (leftPid.containsKey("ki")) settings.leftPID.ki = leftPid["ki"];
            if (leftPid.containsKey("kd")) settings.leftPID.kd = leftPid["kd"];
        }
        
        if (pid.containsKey("right")) {
            JsonObject rightPid = pid["right"];
            if (rightPid.containsKey("kp")) settings.rightPID.kp = rightPid["kp"];
            if (rightPid.containsKey("ki")) settings.rightPID.ki = rightPid["ki"];
            if (rightPid.containsKey("kd")) settings.rightPID.kd = rightPid["kd"];
        }
    }
    
    if (json.containsKey("homePosition")) {
        JsonObject homePos = json["homePosition"];
        if (homePos.containsKey("x")) settings.homePosition.x = homePos["x"];
        if (homePos.containsKey("y")) settings.homePosition.y = homePos["y"];
        if (homePos.containsKey("theta")) settings.homePosition.theta = homePos["theta"];
        if (homePos.containsKey("isSet")) settings.homePosition.isSet = homePos["isSet"];
    }
    
    // Aggiorna i checksum per ogni sezione
    EEPROMManager::updateChecksum(settings.system);
    EEPROMManager::updateChecksum(settings.mower);
    EEPROMManager::updateChecksum(settings.navigation);
    EEPROMManager::updateChecksum(settings.maintenance);
    EEPROMManager::updateChecksum(settings.leftPID);
    EEPROMManager::updateChecksum(settings.rightPID);
    EEPROMManager::updateChecksum(settings.homePosition);
    
    // Calcola il checksum totale
    settings.checksum = 0;
    uint8_t* data = reinterpret_cast<uint8_t*>(&settings);
    size_t dataSize = sizeof(settings) - 1;  // Escludi l'ultimo byte (checksum)
    settings.checksum = EEPROMManager::calculateChecksum(data, dataSize);

    // Salva le impostazioni
    EEPROMManager::saveSettings(settings);
    if (true) {
        wifiBridgeInstance->sendResponse("ok", "Impostazioni aggiornate con successo");
    } else {
        wifiBridgeInstance->sendResponse("error", "Errore nel salvataggio delle impostazioni");
    }
}

void handleResetEepromToDefault() {
    if (!wifiBridgeInstance) return;
    
    EEPROMSettings settings;
    // Carica e salva le impostazioni di default
    EEPROMManager::loadDefaultSettings(settings);
    EEPROMManager::saveSettings(settings);
    if (true) {
        wifiBridgeInstance->sendResponse("ok", "Impostazioni ripristinate ai valori di default");
    } else {
        wifiBridgeInstance->sendResponse("error", "Errore nel ripristino delle impostazioni");
    }
}
#endif

void initWiFiCommands(WiFiSerialBridge& wifiBridge, CommandHandler& handler) {
    cmdHandler = &handler;
    wifiBridgeInstance = &wifiBridge;
    
    // Imposta il gestore dei comandi
    wifiBridge.setCommandHandler(handleWiFiCommand);
    
    #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.println("WiFiCommands inizializzato");
    #endif
}

#endif // ENABLE_EEPROM
