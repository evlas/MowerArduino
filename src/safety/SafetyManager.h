#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <Arduino.h>
#include "../sensors/IMU.h"

// Definizione dei parametri di sicurezza
#define TILT_THRESHOLD 45.0f     // Angolo massimo di inclinazione (gradi)
#define LIFT_THRESHOLD 15.0f     // Angolo massimo di sollevamento (gradi)
#define STOP_BUTTON_PIN 2        // Pin per il pulsante di stop
#define STOP_BUTTON_DEBOUNCE 50  // Debounce per il pulsante (ms)

class SafetyManager {
    public:
        // Stato di sicurezza
        enum SafetyState {
            SAFE,
            TILTED,
            LIFTED,
            STOP_BUTTON_PRESSED,
            EMERGENCY_STOP
        };

        SafetyManager();
        ~SafetyManager();

        // Inizializzazione
        void begin();

        // Aggiorna lo stato di sicurezza
        void update();

        // Ottiene lo stato di sicurezza
        SafetyState getState() const { return _currentState; }

        // Verifica se il robot è sicuro
        bool isSafe() const { return _currentState == SAFE; }

        // Verifica se c'è un'emergency stop
        bool isEmergencyStop() const { return _currentState == EMERGENCY_STOP; }

        // Gestione dell'emergency stop
        void emergencyStop();
        void resetEmergencyStop();

    private:
        // Stato corrente
        SafetyState _currentState;
        
        // Variabili per il debounce
        unsigned long _lastDebounceTime;
        bool _lastButtonState;
        bool _buttonState;
        bool _stopButtonPressed;

        // IMU per rilevare inclinazioni
        IMUModule* _imu;
        
        // Parametri di sicurezza
        float _tiltThreshold;
        float _liftThreshold;
        
        // Funzioni private
        bool checkTilt();
        bool checkLift();
        bool checkStopButton();
        void setState(SafetyState newState);
};

extern SafetyManager safety;

#endif
