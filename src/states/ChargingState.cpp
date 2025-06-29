#include "ChargingState.h"
#include "../functions/Mower.h"
#include "IdleState.h"
#include "EmergencyStopState.h"
#include "MowingState.h"
#include "DockingState.h"
#include "UndockingState.h"
#include "LiftedState.h"
#include "../functions/MowerTypes.h"

// Tempo massimo di ricarica prima di considerarlo un errore (2 ore)
const unsigned long MAX_CHARGING_TIME_MS = 2 * 60 * 60 * 1000UL;
// Usa BATTERY_CHECK_INTERVAL da config.h

void ChargingState::enter(Mower& mower) {
#ifdef DEBUG
    SERIAL_DEBUG.println(F("CHARGING: Entering state"));
    SERIAL_DEBUG.print(F("Battery level: "));
    SERIAL_DEBUG.print(mower.batteryLevel_);
    SERIAL_DEBUG.println(F("%"));
#endif
    
    // Inizializza le variabili di stato
    lastBatteryCheck_ = millis();
    isFullyCharged_ = mower.isBatteryFull();
    
    // Attiva la ricarica
    if (!mower.enableCharging(true)) {
#ifdef DEBUG
        SERIAL_DEBUG.println(F("ERROR: Failed to enable charging"));
#endif
        mower.handleEvent(Event::ERROR_DETECTED);
        return;
    }
    
    // Ferma tutti i motori
    mower.stopMotors();
    
    // Mostra il messaggio sul display
    mower.clearLcdDisplay();
    mower.setLcdCursor(0, 0);
    mower.printToLcd("Charging...");
    mower.setLcdCursor(0, 1);
    mower.printToLcd("Battery: ");
    int batteryLevel = static_cast<int>(mower.getBatteryLevel());
    if (batteryLevel < 10) mower.printToLcd(" ");
    mower.printToLcd(batteryLevel);
    mower.printToLcd("%");
    
    // Segnale acustico di inizio ricarica
    mower.playBuzzerTone(1000, 200);  // Frequenza 1000Hz per 200ms
}

void ChargingState::update(Mower& mower) {
    unsigned long currentTime = millis();
    
    // Verifica il tempo di ricarica massimo
    if (currentTime - lastBatteryCheck_ > MAX_CHARGING_TIME_MS) {
#ifdef DEBUG
        SERIAL_DEBUG.println(F("CHARGING: Maximum charging time exceeded"));
#endif
        mower.handleEvent(Event::ERROR_DETECTED);
        return;
    }
    
    // Controlla periodicamente il livello della batteria
    if (currentTime - lastBatteryCheck_ > BATTERY_CHECK_INTERVAL) {
        lastBatteryCheck_ = currentTime;
        
        // Aggiorna il display con il livello della batteria
        int batteryLevel = static_cast<int>(mower.getBatteryLevel());
        
#ifdef DEBUG
        SERIAL_DEBUG.print(F("CHARGING: Battery level "));
        SERIAL_DEBUG.print(batteryLevel);
        SERIAL_DEBUG.println(F("%"));
#endif
        
        mower.setLcdCursor(9, 1);
        if (batteryLevel < 10) mower.printToLcd(" ");
        mower.printToLcd(batteryLevel);
        mower.printToLcd("%   ");
        
        // Controlla se la batteria è completamente carica
        if (mower.isBatteryFull()) {
            if (!isFullyCharged_) {
                isFullyCharged_ = true;
                // Emetti un segnale acustico per indicare la carica completata
                mower.playBuzzerTone(1500, 300);  // Frequenza 1500Hz per 300ms
                
                // Aggiorna il display
                mower.updateLcdDisplay("Charging Complete!", "Battery: 100%");
                
#ifdef DEBUG
                SERIAL_DEBUG.println(F("CHARGING: Battery fully charged"));
#endif
            }
        } else {
            // Se la batteria si è scaricata dopo essere stata completamente carica
            if (isFullyCharged_) {
                isFullyCharged_ = false;
                mower.updateLcdDisplay("Resuming charge");
            }
        }
        
        // Controlla se la batteria è sufficientemente carica per tornare a tagliare
        if (mower.isBatteryCharged() && !mower.isDocked()) {
            // Se non siamo agganciati alla stazione di ricarica, torniamo a tagliare
            mower.handleEvent(Event::BATTERY_CHARGED);
        }
    }
}

void ChargingState::exit(Mower& mower) {
#ifdef DEBUG
    SERIAL_DEBUG.println(F("CHARGING: Exiting state"));
    SERIAL_DEBUG.print(F("Final battery level: "));
    SERIAL_DEBUG.print(mower.batteryLevel_);
    SERIAL_DEBUG.println(F("%"));
#endif
    
    // Disattiva la ricarica quando si esce dallo stato
    if (!mower.enableCharging(false)) {
#ifdef DEBUG
        SERIAL_DEBUG.println(F("WARNING: Failed to disable charging"));
#endif
    }
    
    // Clear the display
    mower.updateLcdDisplay("");
}

void ChargingState::handleEvent(Mower& mower, Event event) {
#ifdef DEBUG
    SERIAL_DEBUG.print(F("CHARGING: Handling event "));
    SERIAL_DEBUG.println(mower.eventToString(event));
#endif

    switch (event) {
        case Event::BATTERY_FULL:
            // La batteria è completamente carica
            isFullyCharged_ = true;
            // Aggiorna il display
            mower.setLcdCursor(0, 0);
            mower.printToLcd("Battery Full!    ");
            // Emetti un segnale acustico
            mower.playBuzzerTone(2000, 500);
            break;
            
        case Event::BATTERY_CHARGED:
            // Se la batteria è sufficientemente carica, possiamo riprendere il taglio
            if (mower.isBatteryCharged()) {
                if (mower.isDocked()) {
                    // Se siamo agganciati, usciamo dalla stazione di ricarica
                    mower.setState(mower.getUndockingState());
                } else {
                    // Altrimenti torniamo direttamente a tagliare
                    mower.setState(mower.getMowingState());
                }
            }
            break;
            
        case Event::CHARGING_STOPPED:
            // Se la ricarica si interrompe inaspettatamente
            if (!isFullyCharged_ && !mower.isBatteryCharged()) {
                // Solo se non è ancora carico e non abbiamo finito la ricarica
                mower.handleEvent(Event::ERROR_DETECTED);
            }
            break;
            
        case Event::UNDOCK_DETECTED:
            // Se viene rimosso dalla stazione di ricarica
            if (mower.isBatteryCharged()) {
                mower.setState(mower.getMowingState());
            } else {
                mower.setState(mower.getIdleState());
            }
            break;
            
        case Event::START_MOWING:
            // Se viene richiesto di tagliare, verifichiamo lo stato della batteria
            if (mower.isBatteryCharged()) {
                mower.setState(mower.getMowingState());
            } else {
#ifdef DEBUG
                SERIAL_DEBUG.println(F("CHARGING: Battery not sufficiently charged to start mowing"));
#endif
                // Segnale acustico per indicare che non può partire
                mower.playBuzzerTone(1000, 200);
                delay(200);
                mower.playBuzzerTone(1000, 200);
            }
            break;
            
        case Event::EMERGENCY_STOP:
            mower.setState(mower.getEmergencyStopState());
            break;
            
        case Event::ERROR_DETECTED:
            mower.setState(mower.getEmergencyStopState());
            break;
            
        case Event::LIFT_DETECTED:
            // Se il tosaerba viene sollevato durante la ricarica
            mower.setState(mower.getLiftedState());
            break;
            
        // Ignora altri eventi durante la ricarica
        default:
#ifdef DEBUG
            SERIAL_DEBUG.print(F("CHARGING: Ignoring event "));
            SERIAL_DEBUG.println(mower.eventToString(event));
#endif
            break;
    }
}
