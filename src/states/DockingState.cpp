#include "DockingState.h"
#include "../functions/Mower.h"
#include "ChargingState.h"
#include "EmergencyStopState.h"
#include "../functions/MowerTypes.h"

// Soglie per la rilevazione della base di ricarica
const float DOCK_DETECTION_DISTANCE = 50.0f;  // cm
const unsigned long DOCKING_TIMEOUT = 5 * 60 * 1000UL;  // 5 minuti
const unsigned long ALIGNMENT_TIMEOUT = 30000UL;  // 30 secondi
const unsigned long DOCKING_CONFIRMATION_TIME = 3000UL;  // 3 secondi
const unsigned long PHASE_TIMEOUT = 30000UL;  // 30 secondi per fase

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
    
    // Attiva i motori a bassa velocità
    const float DOCKING_MOTOR_SPEED = 0.5f;  // 50% della velocità massima
    mower.setLeftMotorSpeed(DOCKING_MOTOR_SPEED);
    mower.setRightMotorSpeed(DOCKING_MOTOR_SPEED);
    
    // Mostra il messaggio iniziale
    mower.clearLcdDisplay();
    mower.setLcdCursor(0, 0);
    mower.printToLcd("DOCKING");
    mower.setLcdCursor(0, 1);
    mower.printToLcd("Searching dock...");
    
    // Segnale acustico di inizio aggancio
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
                mower.setLeftMotorSpeed(0.3f);
                mower.setRightMotorSpeed(-0.3f);
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
    // Implementa la logica per l'avvicinamento iniziale alla base
    // Usa i sensori per allinearsi alla base
    
    // Usa i metodi pubblici per ottenere le distanze
    float leftDist = mower.isAlignedWithDock() ? 0 : 100; // Valori di esempio, sostituire con la logica corretta
    float rightDist = mower.isAlignedWithDock() ? 100 : 0; // Valori di esempio, sostituire con la logica corretta
    
    // Se entrambi i sensori vedono qualcosa a distanza ravvicinata, passiamo all'allineamento
    if (leftDist < DOCK_DETECTION_DISTANCE && rightDist < DOCK_DETECTION_DISTANCE) {
        changePhase(DockingPhase::ALIGNMENT);
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
        mower.setLeftMotorSpeed(0.2f);
        mower.setRightMotorSpeed(0.2f);
        
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
        DEBUG_PRINTLN(phaseName);
#endif
    }
}
