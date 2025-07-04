#include "DockingState.h"
#include "../functions/Mower.h"
#include "../functions/PositionManager.h"
#include "../config.h"
#include <math.h>
#include "ChargingState.h"
#include "EmergencyStopState.h"
#include "../functions/MowerTypes.h"
#include <math.h>

// Soglie per la rilevazione della base di ricarica
const float DOCK_DETECTION_DISTANCE = 50.0f;  // cm
const unsigned long DOCKING_TIMEOUT = 5 * 60 * 1000UL;  // 5 minuti
const unsigned long ALIGNMENT_TIMEOUT = 30000UL;  // 30 secondi
const unsigned long DOCKING_CONFIRMATION_TIME = 3000UL;  // 3 secondi
const unsigned long PHASE_TIMEOUT = 30000UL;  // 30 secondi per fase

// Utility function to convert degrees to radians
static inline double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Utility function to convert radians to degrees
static inline double toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

void DockingState::enter(Mower& mower) {
#ifdef DEBUG_MODE
    DEBUG_PRINTLN(F("DOCKING: Entering state"));
#endif
    
    // Inizializza la fase di docking
    dockingPhase_ = DockingPhase::INITIAL_APPROACH;
    phaseStartTime_ = millis();
    lastPhaseChangeTime_ = millis();
    
    // Ferma le lame se erano in funzione
    mower.stopBlades();
    
    // Attiva i motori a bassa velocità (50% della velocità massima)
    const float DOCKING_MOTOR_SPEED = 50.0f;  // 50% della velocità massima (-100.0..100.0)
    mower.setLeftMotorSpeed(DOCKING_MOTOR_SPEED);
    mower.setRightMotorSpeed(DOCKING_MOTOR_SPEED);
    
    // Display updates are now handled by the LCDMenu class
    
    // Play docking start sound
    mower.playBuzzerTone(1500, 200);
    delay(100);
    mower.playBuzzerTone(1000, 200);
    
    // Registra il tempo di inizio
    dockingStartTime_ = millis();
}

void DockingState::update(Mower& mower) {
    unsigned long currentTime = millis();
    
    // Verifica il timeout generale di docking
    if (currentTime - dockingStartTime_ > DOCKING_TIMEOUT) {
#ifdef DEBUG_MODE
        DEBUG_PRINTLN(F("DOCKING: Timeout during docking"));
#endif
        mower.handleEvent(Event::ERROR_DETECTED);
        return;
    }
    
    // Controlla se la batteria è critica durante l'aggancio
    if (mower.isBatteryCritical()) {
        mower.handleEvent(Event::BATTERY_CRITICAL);
        return;
    }
    
    // Controlla se è stato rilevato un sollevamento
    if (mower.isLifted()) {
        mower.handleEvent(Event::LIFT_DETECTED);
        return;
    }
    
    // Gestisci la fase di aggancio corrente
    switch (dockingPhase_) {
        case DockingPhase::INITIAL_APPROACH:
            handleInitialApproach(mower);
            break;
            
        case DockingPhase::ALIGNMENT:
            handleAlignment(mower);
            break;
            
        case DockingPhase::FINAL_APPROACH:
            handleFinalApproach(mower);
            break;
            
        case DockingPhase::DOCKED:
            // Rimani agganciato alla base
            if (millis() - lastPhaseChangeTime_ > DOCKING_CONFIRMATION_TIME) {
                mower.handleEvent(Event::DOCK_DETECTED);
            }
            break;
            
        default:
            // In caso di fase sconosciuta, torna alla fase iniziale
            changePhase(DockingPhase::INITIAL_APPROACH);
    }
    
    // Se il tempo massimo per questa fase è scaduto, gestisci il timeout
    if (millis() - lastPhaseChangeTime_ > PHASE_TIMEOUT) {
        switch (dockingPhase_) {
            case DockingPhase::INITIAL_APPROACH:
                // Prova a cambiare direzione o angolo
                mower.setLeftMotorSpeed(30.0f);
                mower.setRightMotorSpeed(-30.0f);
                break;
                
            case DockingPhase::ALIGNMENT:
                // Riprova l'allineamento
                changePhase(DockingPhase::INITIAL_APPROACH);
                break;
                
            case DockingPhase::FINAL_APPROACH:
                // Riprova l'avvicinamento finale
                changePhase(DockingPhase::ALIGNMENT);
                break;
                
            default:
                // Se la fase non è gestita, riparti dall'inizio
                changePhase(DockingPhase::INITIAL_APPROACH);
                break;
        }
    }
}

void DockingState::exit(Mower& mower) {
    // Reset delle variabili di stato
    dockingPhase_ = DockingPhase::INITIAL_APPROACH;
    lastPhaseChangeTime_ = 0;
    dockingStartTime_ = 0;
    phaseStartTime_ = 0;
    
    // Ferma i motori
    mower.stopMotors();
    
    // Segnale acustico di fine aggancio
    mower.playBuzzerTone(1000, 200);
    
#ifdef DEBUG_MODE
    DEBUG_PRINTLN(F("DOCKING: Exiting state"));
#endif
}

void DockingState::handleEvent(Mower& mower, Event event) {
    switch (event) {
        case Event::DOCK_DETECTED:
            // Conferma l'aggancio
            changePhase(DockingPhase::DOCKED);
            break;
            
        case Event::EMERGENCY_STOP:
            mower.changeState(mower.getEmergencyStopState());
            break;
            
        case Event::ERROR_DETECTED:
            mower.changeState(mower.getErrorState());
            break;
            
        case Event::LIFT_DETECTED:
            mower.changeState(mower.getLiftedState());
            break;
            
        case Event::BATTERY_CRITICAL:
            // Se la batteria è critica durante l'aggancio, vai in emergenza
            mower.changeState(mower.getEmergencyStopState());
            break;
            
        default:
            // Ignora altri eventi durante l'aggancio
            break;
    }
}

void DockingState::handleInitialApproach(Mower& mower) {
    PositionManager* pm = mower.getPositionManager();
    if (!pm) {
        DEBUG_PRINTLN(F("PositionManager not available, using sensor-based approach"));
        changePhase(DockingPhase::FINAL_APPROACH);
        return;
    }
    
    // Verifica se abbiamo una posizione home valida e un fix GPS
    if (pm->hasValidHomePosition() && pm->isGPSEnabled() && pm->hasGPSFix()) {
        // Ottieni la distanza e il bearing verso casa
        float distance = pm->getDistanceToHome();
        float bearing = pm->getBearingToHome();
        
        // Debug info
        DEBUG_PRINT(F("Docking - Distance: "));
        DEBUG_PRINT(distance);
        DEBUG_PRINT(F("m, Bearing: "));
        DEBUG_PRINT(bearing);
        DEBUG_PRINTLN(F("°"));
        
        // Se siamo abbastanza vicini, passiamo all'approccio finale con i sensori
        if (distance >= 0 && distance < 2.0f) { // 2 metri per l'approccio finale
            DEBUG_PRINTLN(F("Close to home, switching to final approach"));
            changePhase(DockingPhase::FINAL_APPROACH);
            return;
        }
        
        // Se la distanza non è valida, passa all'approccio con sensori
        if (distance < 0) {
            DEBUG_PRINTLN(F("Invalid distance, switching to sensor-based approach"));
            changePhase(DockingPhase::FINAL_APPROACH);
            return;
        }
        
        // Ottieni l'orientamento attuale dal PositionManager
        RobotPosition pos = pm->getPosition();
        float heading = degrees(pos.theta); // Converti da radianti a gradi
        
        // Calcola la differenza angolare e normalizza tra -180 e 180 gradi
        float angleDiff = bearing - heading;
        while (angleDiff > 180.0f) angleDiff -= 360.0f;
        while (angleDiff < -180.0f) angleDiff += 360.0f;
        
        // Controllo dell'angolo prima di muoversi
        const float ANGLE_THRESHOLD = 15.0f; // Gradi di tolleranza
        
        if (abs(angleDiff) > ANGLE_THRESHOLD) {
            // Ruota verso la direzione corretta
            float rotationSpeed = 0.4f;
            if (angleDiff > 0) {
                // Ruota a sinistra
                mower.setLeftMotorSpeed(-rotationSpeed);
                mower.setRightMotorSpeed(rotationSpeed);
            } else {
                // Ruota a destra
                mower.setLeftMotorSpeed(rotationSpeed);
                mower.setRightMotorSpeed(-rotationSpeed);
            }
        } else {
            // Allineato, vai avanti
            const float baseSpeed = 0.5f;
            float speed = constrain(0.2f + (distance * 0.1f), 0.2f, 0.7f); // Velocità proporzionale alla distanza
            mower.setLeftMotorSpeed(speed);
            mower.setRightMotorSpeed(speed);
        }
    } else {
        // Fallback all'approccio basato su sensori se GPS non disponibile
        DEBUG_PRINTLN(F("GPS not available, using sensor-based approach"));
        handleSensorBasedApproach(mower);
    }
}

void DockingState::handleSensorBasedApproach(Mower& mower) {
    // Leggi le distanze dai sensori
    float leftDist = mower.getLeftSensorDistance();
    float rightDist = mower.getRightSensorDistance();
    
    // Soglie di distanza (in cm)
    const float ALIGN_THRESHOLD = 5.0f;  // Differenza massima per considerarsi allineati
    const float MIN_DISTANCE = 10.0f;    // Distanza minima di sicurezza
    const float MAX_DISTANCE = 200.0f;   // Distanza massima di rilevamento
    
    // Debug delle letture
    DEBUG_PRINT(F("Sensori - SX: "));
    DEBUG_PRINT(leftDist);
    DEBUG_PRINT(F(" cm, DX: "));
    DEBUG_PRINT(rightDist);
    DEBUG_PRINTLN(F(" cm"));
    
    // Verifica la validità delle letture
    bool leftValid = (leftDist >= MIN_DISTANCE && leftDist <= MAX_DISTANCE);
    bool rightValid = (rightDist >= MIN_DISTANCE && rightDist <= MAX_DISTANCE);
    
    if (!leftValid || !rightValid) {
        // Se uno dei due sensori non ha una lettura valida, fai una ricerca
        DEBUG_PRINTLN(F("Lettura sensori non valida, avvio ricerca..."));
        performSearchPattern(mower);
        return;
    }
    
    // Calcola la differenza tra i sensori
    float diff = leftDist - rightDist;
    
    if (fabs(diff) < ALIGN_THRESHOLD) {
        // Allineato, vai dritto
        DEBUG_PRINTLN(F("Allineato, vado dritto"));
        mower.setLeftMotorSpeed(30.0f);
        mower.setRightMotorSpeed(30.0f);
    } 
    else if (diff > 0) {
        // Più vicino a destra, gira a sinistra
        DEBUG_PRINTLN(F("Giro a sinistra"));
        mower.setLeftMotorSpeed(10.0f);
        mower.setRightMotorSpeed(40.0f);
    } 
    else {
        // Più vicino a sinistra, gira a destra
        DEBUG_PRINTLN(F("Giro a destra"));
        mower.setLeftMotorSpeed(40.0f);
        mower.setRightMotorSpeed(10.0f);
    }
}

void DockingState::performSearchPattern(Mower& mower) {
    static bool turningRight = true;
    static unsigned long lastTurnTime = 0;
    
    if (millis() - lastTurnTime > 2000) { // Cambia direzione ogni 2 secondi
        turningRight = !turningRight;
        lastTurnTime = millis();
        DEBUG_PRINTLN(turningRight ? F("Cambio direzione: destra") : F("Cambio direzione: sinistra"));
    }
    
    if (turningRight) {
        mower.setLeftMotorSpeed(40.0f);
        mower.setRightMotorSpeed(10.0f);
    } else {
        mower.setLeftMotorSpeed(10.0f);
        mower.setRightMotorSpeed(40.0f);
    }
}

void DockingState::handleAlignment(Mower& mower) {
    // Implementa la logica per l'allineamento con la base
    float correction = mower.calculateDockAlignmentCorrection();
    
    // Se la correzione è piccola, siamo allineati
    if (abs(correction) < 5.0f) {  // 5 cm di tolleranza
        changePhase(DockingPhase::FINAL_APPROACH);
    } else {
        // Altrimenti, ruota per allinearsi
        if (correction > 0) {
            // La destra è più lontana, ruota a destra
            mower.setLeftMotorSpeed(0.3f);
            mower.setRightMotorSpeed(-0.3f);
        } else {
            // Ruota a sinistra
            mower.setLeftMotorSpeed(-0.3f);
            mower.setRightMotorSpeed(0.3f);
        }
    }
}

void DockingState::handleFinalApproach(Mower& mower) {
    // Avvicinamento finale alla base
    // Usa i metodi pubblici per controllare lo stato
    if (mower.isDocked()) {
        // Se siamo agganciati, passa alla fase di aggancio
        changePhase(DockingPhase::DOCKED);
    } else {
        // Altrimenti, continua ad avanzare lentamente
        mower.setLeftMotorSpeed(20.0f);
        mower.setRightMotorSpeed(20.0f);
        
        // Controlla l'allineamento durante l'avvicinamento
        float correction = mower.calculateDockAlignmentCorrection();
        if (abs(correction) > 5.0f) {
            // Se ci stiamo disallineando, torna alla fase di allineamento
            changePhase(DockingPhase::ALIGNMENT);
        }
    }
}

void DockingState::changePhase(DockingPhase newPhase) {
    if (dockingPhase_ != newPhase) {
        dockingPhase_ = newPhase;
        lastPhaseChangeTime_ = millis();
        
#ifdef DEBUG_MODE
        const char* phaseName = "";
        switch (newPhase) {
            case DockingPhase::INITIAL_APPROACH: phaseName = "INITIAL_APPROACH"; break;
            case DockingPhase::ALIGNMENT: phaseName = "ALIGNMENT"; break;
            case DockingPhase::FINAL_APPROACH: phaseName = "FINAL_APPROACH"; break;
            case DockingPhase::DOCKED: phaseName = "DOCKED"; break;
        }
        DEBUG_PRINT(F("DOCKING: Changing phase to "));
        // Non possiamo usare F() qui perché phaseName è una variabile
#endif
    }
}
