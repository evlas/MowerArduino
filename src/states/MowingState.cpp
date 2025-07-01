#include "MowingState.h"
#include "../functions/Mower.h"
#include "../functions/MowerTypes.h"
#include "IdleState.h"
#include "DockingState.h"
#include "EmergencyStopState.h"
#include "UndockingState.h"
#include <Arduino.h>

void MowingState::enter(Mower& mower) {
    DEBUG_PRINTLN(F("MOWING: Entering state"));
    DEBUG_PRINT(F("MOWING: Current battery level: "));
    DEBUG_PRINTLN(mower.getBatteryPercentage());
    
    // Verifica che i puntatori ai motori siano validi
    DEBUG_PRINTLN(F("MOWING: Starting motors..."));
    
    // LCD updates are now handled by the LCDMenu class
    // to prevent display conflicts
    
    // Avvia il taglio
    mower.setBladeSpeed(95.0f);
    mower.startBlades();
    mower.startDriveMotors();
    mower.setNavigationMode(Mower::NavigationMode::RANDOM);
    
    // Segnale acustico di inizio taglio
    mower.playBuzzerTone(2000, 100);
    delay(100);
    mower.playBuzzerTone(2500, 100);
    
    DEBUG_PRINTLN(F("MOWING: Initialization completed"));
    
    // Resetta il timer dell'inattività
    lastActivityTime_ = millis();
}

void MowingState::update(Mower& mower) {
    unsigned long currentTime = millis();
    
    // Verifica lo stato della batteria
    if (mower.isBatteryCritical()) {
        mower.handleEvent(Event::BATTERY_LOW);
        return;
    }
    
    if (mower.isBatteryFull()) {
        mower.handleEvent(Event::BATTERY_FULL);
        return;
    }
    
    // Verifica se è stato rilevato un sollevamento
    if (mower.isLifted()) {
        mower.handleEvent(Event::LIFT_DETECTED);
        return;
    }
    
    // Verifica condizioni meteorologiche
    if (mower.isRaining()) {
        mower.handleEvent(Event::RAIN_DETECTED);
        return;
    }
    
    // Controlla se è stato rilevato un ostacolo
    if (mower.isCollisionDetected()) {
        mower.handleEvent(Event::OBSTACLE_DETECTED);
        return;
    }
    
    // Verifica se è stato rilevato un bordo o un ostacolo
    if (mower.isBorderDetected()) {
        mower.handleEvent(Event::BORDER_DETECTED);
        return;
    }
    
    // Verifica presenza ostacoli
    if (mower.isObstacleDetected()) {
        mower.handleEvent(Event::OBSTACLE_DETECTED);
        return;
    }
    
    // Se è passato troppo tempo senza attività, torna alla base per sicurezza
    if (currentTime - lastActivityTime_ > MAX_INACTIVITY_TIME) {
        mower.handleEvent(Event::BATTERY_LOW);  // Usa l'evento di batteria scarica per tornare alla base
        return;
    }
    
    // Aggiorna il tempo dell'ultima attività
    lastActivityTime_ = currentTime;
    
    // Display updates are now handled by the LCDMenu class
    // to prevent conflicts
    
    // Update motors based on mowing mode
    mower.updateMotors();
    
    // Operating time tracking is now handled by the Mower class
}

void MowingState::exit(Mower& mower) {
#ifdef DEBUG_MODE
    DEBUG_PRINTLN(F("MOWING: Exiting state"));
#endif
    
    // Ferma i motori e le lame
    mower.stopMotors();
    mower.stopBlades();
    
    // Display cleaning is now handled by the LCDMenu class
    
    // Breve segnale acustico di uscita
    mower.playBuzzerTone(2500, 100);
    delay(100);
    mower.playBuzzerTone(2000, 100);
}

// Metodo privato per gestire l'evitamento dei bordi
void MowingState::handleBorderAvoidance(Mower& mower) {
    // Ferma i motori
    mower.stopMotors();
    
    // Breve pausa
    delay(500);
    
    // Esegui una manovra all'indietro e gira
    mower.setLeftMotorSpeed(-0.5f);  // Indietro al 50% di potenza
    mower.setRightMotorSpeed(-0.5f);
    delay(1000);  // Va indietro per 1 secondo
    
    // Gira di 90 gradi a destra
    mower.setLeftMotorSpeed(0.5f);
    mower.setRightMotorSpeed(-0.5f);
    delay(500);  // Regola questo valore in base alla velocità di rotazione
    
    // Riprendi il movimento in avanti
    mower.startDriveMotors();
}

void MowingState::handleEvent(Mower& mower, Event event) {
#ifdef DEBUG_MODE
    DEBUG_PRINT(F("MOWING: Handling event "));
    DEBUG_PRINTLN(mower.eventToString(event));
#endif

    switch (event) {
        case Event::STOP_MOWING:
            mower.changeState(mower.getIdleState());
            break;
            
        case Event::BATTERY_LOW:
            // Avvia il ritorno alla base
            mower.changeState(mower.getDockingState());
            break;
            
        case Event::BATTERY_CRITICAL:
            // Ferma tutto e vai in emergenza
            mower.stopMotors();
            mower.stopBlades();
            mower.changeState(mower.getEmergencyStopState());
            break;
            
        case Event::LIFT_DETECTED:
            // Passa allo stato di sollevamento
            mower.changeState(mower.getLiftedState());
            break;
            
        case Event::BORDER_DETECTED:
            // Esegui la manovra di evitamento bordo
            handleBorderAvoidance(mower);
            lastActivityTime_ = millis();  // Resetta il timer di inattività
            // Breve segnale acustico per il bordo
            mower.playBuzzerTone(1500, 100);
            break;
            
        case Event::OBSTACLE_DETECTED: {
            // Ostacolo rilevato, esegui manovra di evitamento
            // Ferma i motori
            mower.stopMotors();
            
            // Breve pausa
            delay(300);
            
            // Esegui una manovra all'indietro
            mower.setLeftMotorSpeed(-0.4f);
            mower.setRightMotorSpeed(-0.4f);
            delay(800);
            
            // Gira a destra di circa 45 gradi
            mower.setLeftMotorSpeed(0.5f);
            mower.setRightMotorSpeed(0.2f);
            delay(600);
            
            // Riprendi il movimento in avanti
            mower.startDriveMotors();
            
            lastActivityTime_ = millis();  // Resetta il timer di inattività
            // Segnale acustico per l'ostacolo
            mower.playBuzzerTone(1000, 200);
            break;
        }
            
        case Event::EMERGENCY_STOP:
            // Fermata di emergenza
            mower.changeState(mower.getEmergencyStopState());
            break;
            
        case Event::PAUSE:
            // Metti in pausa il taglio
            mower.changeState(mower.getIdleState());
            break;
            
        case Event::RAIN_DETECTED:
            // Vai alla stazione di ricarica se piove
            mower.changeState(mower.getDockingState());
            break;
            
        case Event::DOCK_DETECTED:
            // Se viene rilevato il dock durante il taglio, probabilmente è un falso positivo
            // Possiamo ignorarlo o fermarci per sicurezza
            mower.stopMotors();
            delay(500);
            if (mower.isDocked()) {
                mower.changeState(mower.getChargingState());
            } else {
                // Riprendi il taglio
                mower.startDriveMotors();
            }
            break;
            
        // Ignora altri eventi durante il taglio
        default:
#ifdef DEBUG_MODE
            DEBUG_PRINT(F("MOWING: Ignoring event "));
            DEBUG_PRINTLN(mower.eventToString(event));
#endif
            break;
    }
}
