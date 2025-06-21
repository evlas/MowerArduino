#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "../../config.h"  // Include le costanti di velocità
#include <Arduino.h>
#include "Telemetry.h"

// Forward declarations
class StateMachine;
class Navigation;

#define MIN_ANGLE 1.0f
#define MAX_ANGLE 360.0f

class CommandHandler {
public:
    CommandHandler(StateMachine* stateMachine, Navigation* navigation);
    
    // Gestione comandi di movimento con controllo di velocità
    void moveForward(float speed = DEFAULT_MOTOR_SPEED);
    void moveBackward(float speed = DEFAULT_MOTOR_SPEED);
    void moveAtSpeed(float speed);  // Velocità tra -1.0 (indietro) e 1.0 (avanti)
    
    // Gestione rotazioni con angolo specifico
    void turnLeft(float angle = 90.0f);
    void turnRight(float angle = 90.0f);
    void turnByAngle(float angle);  // Angolo in gradi (positivo = orario, negativo = antiorario)
    
    // Controllo di movimento avanzato
    void moveAtSpeedAndTurn(float speed, float turnRate);  // turnRate tra -1.0 (sinistra) e 1.0 (destra)
    void stop();
    
    // Gestione modalità di funzionamento
    void startMowing(float speed = DEFAULT_MOTOR_SPEED);
    void stopMowing();
    void returnToBase();
    void setMowingPattern(NavigationMode::Mode pattern);
    
    // Controllo lama
    void setBladeSpeed(uint8_t speed);  // 0-100%
    void startBlade();
    void startBlade(uint8_t speed);  // Avvia lama con velocità specifica (0-255)
    void stopBlade();
    
    // Impostazioni e configurazione
    void setMaxSpeed(float maxSpeed);  // Imposta la velocità massima
    float getCurrentSpeed() const;
    float getCurrentAngle() const;
    void setCuttingHeight(uint8_t height);
    
    // Stato e diagnostica
    void requestStatus();
    void requestBatteryLevel();
    void requestErrorLog();
    
    // Gestione emergenza
    void emergencyStop();
    void resumeFromEmergency();
    
    // Gestione connessione
    void resetConnection();
    
    // Gestione telemetria
    void enableTelemetry(bool enable);
    void setTelemetryInterval(uint32_t intervalMs);
    bool isTelemetryEnabled() const;
    uint32_t getTelemetryInterval() const;
    void updateTelemetry();
    
private:
    StateMachine* _stateMachine;
    Navigation* _navigation;
    float _currentSpeed;         // -1.0 (indietro) a 1.0 (avanti)
    float _currentTurnRate;      // -1.0 (sinistra) a 1.0 (destra)
    float _currentAngle;         // Angolo corrente in gradi (0-360)
    uint8_t _currentBladeSpeed;   // 0-100%
    float _maxSpeed;             // Velocità massima consentita
    
    // Metodi di validazione
    bool validateSpeed(float speed) const;
    bool validateTurnRate(float turnRate) const;
    bool validateAngle(float angle) const;
    bool validateBladeSpeed(uint8_t speed) const;
    
    // Normalizzazione angolo tra 0 e 360 gradi
    float normalizeAngle(float angle) const;
    
    // Telemetria
    Telemetry* _telemetry;
    bool _telemetryEnabled;
    uint32_t _telemetryInterval;
    unsigned long _lastTelemetryUpdate;
};

#endif
