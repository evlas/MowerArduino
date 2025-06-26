#include <Arduino.h>
#include "src/Mower.h"
#include "src/MowerStateMachine.h"

Mower mower;
MowerStateMachine stateMachine(mower);

void printHelp() {
    Serial.println("\n=== Mower State Machine Tester ===");
    Serial.println("Comandi disponibili:");
    Serial.println("0 - Mostra questo aiuto");
    Serial.println("1 - START_MOWING");
    Serial.println("2 - STOP_MOWING");
    Serial.println("3 - EMERGENCY_STOP");
    Serial.println("4 - ENTER_MANUAL_MODE");
    Serial.println("5 - EXIT_MANUAL_MODE");
    Serial.println("6 - DOCKING_DETECTED");
    Serial.println("7 - CHARGING_STARTED");
    Serial.println("8 - CHARGING_COMPLETED");
    Serial.println("9 - Mostra stato attuale");
    Serial.println("p - PAUSA/RIPRENDI");
    Serial.println("s - SIMULA OSTACOLO");
    Serial.println("b - SIMULA BORDO");
    Serial.println("r - Reset EMERGENCY STOP");
    Serial.println("m - Attiva/Disattiva manutenzione");
    Serial.println("z - Attiva/Disattiva modalità sonno");
    Serial.println("x - Attiva/Disattiva pausa pioggia");
    Serial.println("c - Attiva/Disattiva controllo ROS");
    Serial.println("e - Simula errore");
    Serial.println("===============================\n");
}

void printCurrentState() {
    mower.printStatus();
}

void processCommand(char cmd) {
    switch(cmd) {
        case '0': 
            printHelp(); 
            return;
            
        case '1': // Avvia taglio
            Serial.println("Avvio modalità taglio...");
            stateMachine.transitionToState(MowerStateMachine::MowerState::MOWING);
            break;
            
        case '2': // Ferma taglio
            Serial.println("Arresto modalità taglio...");
            stateMachine.transitionToState(MowerStateMachine::MowerState::IDLE);
            break;
            
        case '3': // Emergenza
            Serial.println("ATTIVAZIONE EMERGENZA!");
            stateMachine.transitionToState(MowerStateMachine::MowerState::EMERGENCY_STOP);
            break;
            
        case '4': // Modalità manuale
            Serial.println("Attivazione modalità manuale...");
            stateMachine.transitionToState(MowerStateMachine::MowerState::MANUAL_CONTROL);
            break;
            
        case '5': // Esci da manuale
            Serial.println("Uscita modalità manuale...");
            stateMachine.transitionToState(MowerStateMachine::MowerState::IDLE);
            break;
            
        case '6': // Docking
            Serial.println("Rilevata base di ricarica...");
            stateMachine.transitionToState(MowerStateMachine::MowerState::DOCKING);
            break;
            
        case '7': // Inizio carica
            Serial.println("Inizio ricarica");
            mower.enableCharging(true);
            stateMachine.transitionToState(MowerStateMachine::MowerState::CHARGING);
            break;
            
        case '8': // Fine carica
            Serial.println("Ricarica completata!");
            stateMachine.transitionToState(MowerStateMachine::MowerState::IDLE);
            break;
            
        case '9': // Stato
            printCurrentState();
            return;
            
        case 'p': // Pausa/Riprendi
            if (mower.getState() == Mower::State::PAUSED) {
                Serial.println("Ripresa dal pausa");
                stateMachine.transitionToState(MowerStateMachine::MowerState::IDLE);
            } else {
                stateMachine.transitionToState(MowerStateMachine::MowerState::PAUSED);
            }
            break;
            
        case 's': // Simula ostacolo
            Serial.println("Simulazione rilevamento ostacolo");
            mower.handleObstacle();
            break;
            
        case 'b': // Simula bordo
            Serial.println("Simulazione rilevamento bordo");
            mower.handleBorder();
            break;
            
        case 'r': // Reset emergenza
            Serial.println("Reset EMERGENCY STOP");
            mower.resetEmergencyStop();
            stateMachine.transitionToState(MowerStateMachine::MowerState::IDLE);
            break;
            
        case 'm': // Manutenzione
            if (mower.getState() == Mower::State::MAINTENANCE_NEEDED) {
                Serial.println("Uscita dalla modalità manutenzione");
                stateMachine.transitionToState(MowerStateMachine::MowerState::IDLE);
            } else {
                stateMachine.transitionToState(MowerStateMachine::MowerState::MAINTENANCE_NEEDED);
            }
            break;
            
        case 'z': // Sonno
            if (mower.getState() == Mower::State::SLEEP) {
                Serial.println("Uscita dalla modalità sonno");
                stateMachine.transitionToState(MowerStateMachine::MowerState::IDLE);
            } else {
                Serial.println("Attivazione modalità sonno");
                stateMachine.transitionToState(MowerStateMachine::MowerState::SLEEP);
            }
            break;
            
        case 'x': // Pausa pioggia
            if (mower.getState() == Mower::State::RAIN_DELAY) {
                Serial.println("Fine pausa pioggia");
                stateMachine.transitionToState(MowerStateMachine::MowerState::IDLE);
            } else {
                Serial.println("Attivazione pausa pioggia (1h)");
                stateMachine.transitionToState(MowerStateMachine::MowerState::RAIN_DELAY);
            }
            break;
            
        case 'c': // Controllo ROS
            if (mower.getState() == Mower::State::ROS_CONTROL) {
                Serial.println("Disattivazione controllo ROS");
                stateMachine.transitionToState(MowerStateMachine::MowerState::IDLE);
            } else {
                Serial.println("Attivazione controllo ROS");
                stateMachine.transitionToState(MowerStateMachine::MowerState::ROS_CONTROL);
            }
            break;
            
        case 'e': // Simula errore
            Serial.println("Simulazione errore");
            stateMachine.transitionToState(MowerStateMachine::MowerState::ERROR);
            break;
            
        default:
            Serial.println("Comando non riconosciuto. Digita '0' per l'aiuto.");
            return;
    }
    
    printCurrentState();
}

void setup() {
    // Inizializza la comunicazione seriale
    Serial.begin(115200);
    while (!Serial) {
        ; // Attendi che la porta seriale si connetta
    }
    
    // Inizializza il tosaerba
    mower.begin();
    
    // Collega la macchina a stati al tosaerba
    mower.setStateMachine(&stateMachine);
    
    // Inizializza la macchina a stati
    stateMachine.begin();
    stateMachine.transitionToState(MowerStateMachine::MowerState::IDLE);
    
    // Stampa il messaggio di benvenuto
    Serial.println("\n=== TEST MACCHINA A STATI TOSAERBA ===");
    printHelp();
    Serial.println("\n=== COMANDI SENSORI (invio richiesto) ===");
    Serial.println("LIFT 1/0     - Attiva/disattiva sensore di sollevamento");
    Serial.println("BORDER 1/0   - Attiva/disattiva sensore di bordo");
    Serial.println("COLLISION 1/0- Attiva/disattiva sensore di collisione");
    Serial.println("DOCK 1/0     - Imposta lo stato di aggancio");
    Serial.println("CHARGE 1/0   - Attiva/disattiva la ricarica");
    Serial.println("STATUS       - Mostra lo stato attuale");
    Serial.println("HELP         - Mostra l'aiuto");
    Serial.println("===============================\n");
    
    printCurrentState();
}

String inputString = "";         // Stringa per memorizzare l'input
bool stringComplete = false;      // Flag per indicare quando abbiamo un comando completo

void loop() {
    static unsigned long lastUpdate = 0;
    
    // Leggi i caratteri in arrivo dalla porta seriale
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        
        // Se riceviamo un newline, il comando è completo
        if (inChar == '\n') {
            stringComplete = true;
        } else if (inChar != '\r') {  // Ignora i ritorni a capo
            inputString += inChar;
        }
        
        // Se il buffer è troppo lungo, lo tronchiamo per evitare overflow
        if (inputString.length() > 32) {
            inputString = "";
        }
    }
    
    // Se abbiamo un comando completo, processalo
    if (stringComplete) {
        // Se il comando inizia con un carattere di comando speciale, usa il vecchio sistema
        if (inputString.length() == 1 && 
            (inputString[0] >= '0' && inputString[0] <= '9' || 
             inputString[0] == 'p' || inputString[0] == 's' || 
             inputString[0] == 'b' || inputString[0] == 'r' || 
             inputString[0] == 'm' || inputString[0] == 'z' || 
             inputString[0] == 'x' || inputString[0] == 'c' || 
             inputString[0] == 'e')) {
            processCommand(inputString[0]);
        } else {
            // Altrimenti, passa il comando al gestore dei comandi del mower
            mower.processSerialCommand(inputString);
        }
        
        // Pulisci la stringa per il prossimo comando
        inputString = "";
        stringComplete = false;
    }
    
    // Aggiornamento periodico (ogni 100ms)
    unsigned long now = millis();
    if (now - lastUpdate >= 100) {
        lastUpdate = now;
        
        // Aggiorna i sensori e la logica del tosaerba
        mower.updateSensors();
        
        // Aggiorna la macchina a stati
        stateMachine.update();
    }
}
