#include "WiFiCommands.h"
#include "WiFiSerialBridge.h"
#include <ArduinoJson.h>

// Riferimento all'istanza globale di commandHandler (definita in MowerArduino.ino)
extern CommandHandler commandHandler;

// Riferimento al monitor della batteria (definito in MowerArduino.ino)
#ifdef ENABLE_BATTERY_MONITOR
extern INA226_WE batteryMonitor;
#endif

// Variabili statiche per i riferimenti
static CommandHandler* cmdHandler = nullptr;
static WiFiSerialBridge* wifiBridgeInstance = nullptr;

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
        StaticJsonDocument<256> status;
        status["state"] = static_cast<int>(mowerStateMachine.getCurrentState());
        #ifdef ENABLE_BATTERY_MONITOR
        status["batteryLevel"] = batteryMonitor.getBusVoltage_V();
        #else
        status["batteryLevel"] = 0.0f; // Battery monitoring not enabled
        #endif
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("ok", "Current status", status);
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
    else {
        // Comando non riconosciuto
        if (wifiBridgeInstance) wifiBridgeInstance->sendResponse("error", "Comando non riconosciuto: " + command);
    }
}

void initWiFiCommands(WiFiSerialBridge& wifiBridge, CommandHandler& handler) {
    cmdHandler = &handler;
    wifiBridgeInstance = &wifiBridge;
    wifiBridge.setCommandHandler(handleWiFiCommand);
    
    #ifdef SERIAL_DEBUG
        SERIAL_DEBUG.println("Gestore comandi WiFi inizializzato");
    #endif
}
