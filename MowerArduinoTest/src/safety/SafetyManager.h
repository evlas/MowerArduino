#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <Arduino.h>
#include "../sensors/IMU.h"
#include "../../pin_config.h"
#include "../../config.h"

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
        
        // Stato del pulsante di stop di emergenza
        unsigned long _stopButtonDebounceTime;
        bool _lastStopButtonState;
        bool _stopButtonState;
        bool _stopButtonPressed;

        // Stato del sensore di sollevamento
        unsigned long _liftDebounceTime;
        bool _lastLiftState;
        bool _liftState;
        bool _liftPressed;

        // IMU per rilevare inclinazioni
        IMUModule* _imu;
        
        // Parametri di sicurezza
        float _tiltThreshold;
        
        // Funzioni private
        bool checkTilt();
        bool checkLift();
        bool checkStopButton();
        void setState(SafetyState newState);
};

extern SafetyManager safety;

#endif
