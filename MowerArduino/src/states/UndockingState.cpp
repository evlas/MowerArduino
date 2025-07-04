#include "UndockingState.h"
#include "../functions/Mower.h"
#include "MowingState.h"
#include "EmergencyStopState.h"
#include "DockingState.h"
#include "../functions/MowerTypes.h"

// Durata delle fasi di undocking (in millisecondi)
const unsigned long REVERSE_DURATION = 1500;    // 1.5 secondi di retromarcia
const unsigned long ROTATION_DURATION = 1000;   // 1 secondo di rotazione
const unsigned long MAX_UNDOCKING_TIME = 10000;  // 10 secondi massimi per l'undocking

void UndockingState::enter(Mower& mower) {
#ifdef DEBUG_MODE
    DEBUG_PRINTLN(F("UNDOCKING: Entering state"));
#endif
    
    // Inizializza le variabili di stato
    undockingStartTime_ = millis();
    isReversing_ = true;
    isRotating_ = false;
    
    // Assicurati che le lame siano spente
    mower.stopBlades();
    
    // Inizia la retromarcia per uscire dal dock al 50% della velocità predefinita
    mower.setLeftMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.5f);
    mower.setRightMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.5f);
    
    // Display updates are now handled by the LCDMenu class
    
    // Play undocking start sound
    mower.playBuzzerTone(1000, 200);  // 1000Hz for 200ms
    delay(100);
    mower.playBuzzerTone(1500, 200);  // 1500Hz for 200ms
}

void UndockingState::update(Mower& mower) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - undockingStartTime_;
    
    // Verifica il timeout di sicurezza
    if (elapsedTime > MAX_UNDOCKING_TIME) {
#ifdef DEBUG_MODE
        DEBUG_PRINTLN(F("UNDOCKING: Timeout during undocking"));
#endif
        mower.handleEvent(Event::ERROR_DETECTED);
        return;
    }
    
    // Fase 1: Retromarcia per uscire dal dock
    if (isReversing_ && elapsedTime > REVERSE_DURATION) {
        isReversing_ = false;
        isRotating_ = true;
        
        // Ferma i motori
        mower.stopDriveMotors();
        
        // Piccola pausa per stabilizzare
        delay(200);
        
        // Inizia la rotazione di 90° a destra per allontanarsi dal dock al 60% della velocità predefinita
        mower.setLeftMotorSpeed(DEFAULT_MOTOR_SPEED * 0.6f);
        mower.setRightMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.6f);
        
        // Update rotation start time
        undockingStartTime_ = currentTime;
        
#ifdef DEBUG_MODE
        DEBUG_PRINTLN(F("UNDOCKING: Starting rotation"));
#endif
    }
    // Fase 2: Rotazione completata, passa allo stato di taglio
    else if (isRotating_ && (currentTime - undockingStartTime_ > ROTATION_DURATION)) {
        // Ferma i motori
        mower.stopDriveMotors();
        
        // Short pause before completing
        delay(500);
        
        // Segnala che l'undocking è completato
        mower.handleEvent(Event::UNDOCKING_COMPLETE);
        return;
    }
    
    // Controlla i sensori durante l'undocking
    if (mower.isLifted()) {
        mower.handleEvent(Event::LIFT_DETECTED);
    } else if (mower.isCollisionDetected()) {
        mower.handleEvent(Event::OBSTACLE_DETECTED);
    } else if (mower.isBorderDetected()) {
        mower.handleEvent(Event::BORDER_DETECTED);
    } else if (mower.isBatteryLow()) {
        mower.handleEvent(Event::BATTERY_LOW);
    }
}

void UndockingState::exit(Mower& mower) {
#ifdef DEBUG_MODE
    DEBUG_PRINTLN(F("UNDOCKING: Exiting state"));
#endif
    
    // Ferma i motori
    mower.stopMotors();
    
    // Pulisci il display
    mower.clearLcdDisplay();
    
    // Resetta le variabili di stato
    isReversing_ = true;
    isRotating_ = false;
    undockingStartTime_ = 0;
    
    // Aggiorna il display
    mower.setLcdCursor(0, 0);
    mower.printToLcd("Undocking");
    mower.setLcdCursor(0, 1);
    mower.printToLcd("Complete!");
    
    // Breve segnale acustico di conferma
    mower.playBuzzerTone(2000, 200);
    delay(100);
    mower.playBuzzerTone(2500, 200);
    
    // Piccola pausa per mostrare il messaggio
    delay(1000);
}

void UndockingState::handleEvent(Mower& mower, Event event) {
#ifdef DEBUG_MODE
    DEBUG_PRINT(F("UNDOCKING: Handling event "));
    DEBUG_PRINTLN(mower.eventToString(event));
#endif

    switch (event) {
        case Event::UNDOCKING_COMPLETE:
            // L'undocking è stato completato con successo, passiamo allo stato di mowing
            mower.setState(mower.getMowingState());
            break;
            
        case Event::EMERGENCY_STOP:
            mower.setState(mower.getEmergencyStopState());
            break;
            
        case Event::ERROR_DETECTED:
            mower.setState(mower.getEmergencyStopState());
            break;
            
        case Event::LIFT_DETECTED:
            mower.setState(mower.getLiftedState());
            break;
            
        case Event::DOCK_DETECTED:
            // Se viene rilevato il dock durante l'undocking, potrebbe essere un falso positivo
            // Aspettiamo un po' per confermare che non sia un falso positivo
            if (millis() - undockingStartTime_ > 1000) {
                // Se è passato più di 1 secondo dall'inizio dell'undocking, probabilmente è un falso positivo
                // Possiamo ignorarlo o fermarci per sicurezza
                mower.stopMotors();
                // Piccola pausa per evitare falsi positivi
                delay(500);
                if (mower.isDocked()) {
                    // Se siamo ancora agganciati, torniamo allo stato di charging
                    mower.setState(mower.getChargingState());
                } else {
                    // Altrimenti riprendiamo l'undocking al 50% della velocità predefinita
                    mower.setLeftMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.5f);
                    mower.setRightMotorSpeed(-DEFAULT_MOTOR_SPEED * 0.5f);
                }
            }
            break;
            
        case Event::BATTERY_LOW:
            // Se la batteria è bassa durante l'undocking, torniamo al dock
            mower.setState(mower.getDockingState());
            break;
            
        case Event::OBSTACLE_DETECTED:
            // Se rileviamo un ostacolo durante l'undocking, proviamo a cambiare direzione
            mower.handleObstacle();
            break;
            
        // Ignora altri eventi durante l'undocking
        default:
#ifdef DEBUG_MODE
            DEBUG_PRINT(F("UNDOCKING: Ignoring event "));
            DEBUG_PRINTLN(mower.eventToString(event));
#endif
            break;
    }
}
