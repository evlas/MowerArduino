#include <Arduino.h>  // Deve essere incluso per primo per Serial
#include <Wire.h>     // Necessario per I2C
#include <EEPROM.h>
#include "LCDMenu.h"
#include "../config.h" // Ensure DEBUG_MODE and debug macros are available

LCDMenu::LCDMenu() 
    : lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE),
      currentState(STATUS_DISPLAY),
      currentItem(0),
      currentPidParam(PID_KP),
      lastDisplayUpdate(0),  // Inizializza a 0 per forzare il primo aggiornamento
      mowingRequested(false),
      maintenanceMode(false),
      kp(1.0), ki(0.1), kd(0.05),
      lastButtonPress(0),
      lastButtonCheck(0),
      lastUserActivity(millis()),
      backlightOn(true),
      mowerPtr(nullptr) {  // Inizializza esplicitamente a nullptr
}

void LCDMenu::begin() {
    // Inizializza LCD
    DEBUG_PRINTLN("\n=== Inizializzazione LCD ===");
    
    lcd.begin(16, 2);    
    delay(50);
    lcd.backlight();
    delay(50);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mower Control");
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");
    
    // Initialize buttons with debug
    DEBUG_PRINTLN(F("Initializing buttons..."));
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(PLUS_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MINUS_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    DEBUG_PRINTLN(F("Buttons initialized"));
    
    // Load saved PID values
    DEBUG_PRINTLN("Loading PID values from EEPROM...");
    loadPidFromEeprom();
    
    DEBUG_PRINTLN("LCD initialization complete");
 
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mower Control");
    lcd.setCursor(0, 1);
    lcd.print("Avvio...");

    delay(1000); // Show initialization message
    updateStatusDisplay(); // Mostra subito lo stato invece del menu
}

void LCDMenu::update() {
    unsigned long currentTime = millis();
    
    // Gestione retroilluminazione
    updateBacklight();
    
    // Controlla i pulsanti ogni 50ms
    if (currentTime - lastButtonCheck >= 50) {
        lastButtonCheck = currentTime;
        
        // Ignora i nuovi input durante il periodo di debounce
        if (currentTime - lastButtonPress < BUTTON_DEBOUNCE) {
            return;
        }
        
        bool buttonPressed = false;
        
        // Se siamo in STATUS_DISPLAY, solo il tasto START porta al menu principale
        if (currentState == STATUS_DISPLAY) {
            if (digitalRead(START_BUTTON_PIN) == LOW) {
                DEBUG_PRINTLN("START button pressed in STATUS_DISPLAY, switching to MAIN_MENU");
                currentState = MAIN_MENU;
                currentItem = 0;
                lastButtonPress = currentTime;
                updateDisplay();
                return;
            }
            // Ignora gli altri pulsanti in STATUS_DISPLAY
            else if (digitalRead(PLUS_BUTTON_PIN) == LOW || 
                    digitalRead(MINUS_BUTTON_PIN) == LOW || 
                    digitalRead(STOP_BUTTON_PIN) == LOW) {
                // Registra il premuto ma non fare nulla
                lastButtonPress = currentTime;
                return;
            }
        }
        // Se siamo in un menu, gestisci i pulsanti normalmente
        else {
            if (digitalRead(START_BUTTON_PIN) == LOW) {
                DEBUG_PRINTLN("START button pressed");
                lastButtonPress = currentTime;
                handleButtonPress(START_BUTTON_PIN);
                buttonPressed = true;
            } 
            else if (digitalRead(PLUS_BUTTON_PIN) == LOW) {
                DEBUG_PRINTLN("PLUS button pressed");
                lastButtonPress = currentTime;
                handleButtonPress(PLUS_BUTTON_PIN);
                buttonPressed = true;
            } 
            else if (digitalRead(MINUS_BUTTON_PIN) == LOW) {
                DEBUG_PRINTLN("MINUS button pressed");
                lastButtonPress = currentTime;  
                handleButtonPress(MINUS_BUTTON_PIN);
                buttonPressed = true;
            } 
            else if (digitalRead(STOP_BUTTON_PIN) == LOW) {
                DEBUG_PRINTLN("STOP button pressed");
                lastButtonPress = currentTime;
                // Se siamo nel menu principale, il tasto STOP torna allo stato
                if (currentState == MAIN_MENU) {
                    currentState = STATUS_DISPLAY;
                    DEBUG_PRINTLN("Returning to STATUS_DISPLAY from MAIN_MENU");
                } else {
                    handleButtonPress(STOP_BUTTON_PIN);
                }
                buttonPressed = true;
            }
            
            // Aggiorna il display se un pulsante è stato premuto
            if (buttonPressed) {
                updateDisplay();
                lastDisplayUpdate = currentTime;
            }
        }
    }
    
    // Aggiornamento periodico del display (ogni 500ms)
    if (backlightOn && (currentTime - lastDisplayUpdate >= 500)) {
        // Aggiorna sempre lo stato se siamo in STATUS_DISPLAY
        // oppure se il mower è in EMERGENCY_STOP (così il display si aggiorna anche in menu)
        if (currentState == STATUS_DISPLAY || (mowerPtr && mowerPtr->getState() == State::EMERGENCY_STOP)) {
            updateStatusDisplay();
        } else {
            // Se siamo in un menu, aggiorna solo se c'è stato un cambiamento di stato
            // o se è passato più tempo dall'ultimo aggiornamento
            static unsigned long lastMenuUpdate = 0;
            if (currentTime - lastMenuUpdate >= 1000) {  // Aggiorna il menu ogni secondo invece che ogni 500ms
                updateDisplay();
                lastMenuUpdate = currentTime;
            }
        }
        lastDisplayUpdate = currentTime;
    }
}

void LCDMenu::handleButtonPress(uint8_t button) {
    // Riattiva la retroilluminazione se spenta
    if (!backlightOn) {
        lcd.backlight();
        backlightOn = true;
        DEBUG_PRINTLN("LCD backlight turned on by button press");
    }
    
    // Aggiorna il timestamp dell'ultima attività
    lastUserActivity = millis();
    
    // Se il mower è in stato di emergenza, permetti di resettare con START o STOP
    if (mowerPtr && mowerPtr->getState() == State::EMERGENCY_STOP) {
        if (button == START_BUTTON_PIN || button == STOP_BUTTON_PIN) {
            mowerPtr->handleEvent(Event::RESET);
            return; // Non processare oltre
        }
    }
    
    switch (currentState) {
        case MAIN_MENU:
            handleMainMenu(button);
            break;
        case MOWING_MENU:
            handleMowingMenu(button);
            break;
        case MAINTENANCE_MENU:
            handleMaintenanceMenu(button);
            break;
        case PID_CONFIG_MENU:
            handlePidConfigMenu(button);
            break;
        case PID_EDIT:
            handlePidEdit(button);
            break;
    }
    
    updateDisplay();
}

void LCDMenu::updateBacklight() {
    unsigned long currentTime = millis();
    
    // Se è passato più tempo del timeout e la retroilluminazione è accesa
    if (currentTime - lastUserActivity > LCD_BACKLIGHT_TIMEOUT && backlightOn) {
        lcd.noBacklight();
        backlightOn = false;
        DEBUG_PRINTLN("LCD backlight turned off due to inactivity");
    }
}

const char* LCDMenu::getStateName(MenuState state) {
    switch(state) {
        case MAIN_MENU: return "MAIN_MENU";
        case MOWING_MENU: return "MOWING_MENU";
        case MAINTENANCE_MENU: return "MAINTENANCE_MENU";
        case PID_CONFIG_MENU: return "PID_CONFIG_MENU";
        case PID_EDIT: return "PID_EDIT";
        default: return "UNKNOWN";
    }
}

void LCDMenu::updateDisplay() {
    lastUserActivity = millis();  // Aggiorna l'ultima attività anche quando il display viene aggiornato
    unsigned long currentTime = millis();
    
    // Limita l'aggiornamento per evitare sfarfallio
    if (currentTime - lastDisplayUpdate < DISPLAY_REFRESH_INTERVAL) {
        return;
    }
    
    lcd.clear();
    
    switch (currentState) {
        case MAIN_MENU: {
            const char* items[] = {"Start Mowing", "Maintenance", "PID Config"};
            lcd.print(">");
            lcd.print(items[currentItem]);
            lcd.setCursor(0, 1);
            lcd.print(" ");
            if (currentItem < MAIN_MENU_ITEMS_COUNT - 1) {
                lcd.print(items[currentItem + 1]);
            }
            break;
        }
        case MOWING_MENU:
            lcd.print("Start Mowing?");
            lcd.setCursor(0, 1);
            lcd.print("START to confirm");
            break;
        case MAINTENANCE_MENU:
            lcd.print("Maintenance Mode");
            lcd.setCursor(0, 1);
            lcd.print("Not implemented");
            break;
        case PID_CONFIG_MENU: {
            const char* params[] = {"Kp:", "Ki:", "Kd:"};
            lcd.print("Set PID:");
            lcd.setCursor(0, 1);
            lcd.print(params[currentPidParam]);
            lcd.print(" ");
            lcd.print(currentPidParam == PID_KP ? kp : 
                     (currentPidParam == PID_KI ? ki : kd));
            break;
        }
        case PID_EDIT: {
            lcd.print("Editing ");
            const char* params[] = {"Kp", "Ki", "Kd"};
            lcd.print(params[currentPidParam]);
            lcd.setCursor(0, 1);
            lcd.print("+/- to adjust");
            break;
        }
    }
}

void LCDMenu::handleMainMenu(uint8_t button) {
    switch (button) {
        case PLUS_BUTTON_PIN:
            if (currentItem > 0) currentItem--;
            break;
        case MINUS_BUTTON_PIN:
            if (currentItem < MAIN_MENU_ITEMS_COUNT - 1) currentItem++;
            break;
        case START_BUTTON_PIN:
            if (currentItem == ITEM_START_MOWING) {
                currentState = MOWING_MENU;
            } else if (currentItem == ITEM_MAINTENANCE) {
                currentState = MAINTENANCE_MENU;
                maintenanceMode = true;
            } else if (currentItem == ITEM_PID_CONFIG) {
                currentState = PID_CONFIG_MENU;
                currentPidParam = PID_KP;
            }
            break;
    }
}

void LCDMenu::handleMowingMenu(uint8_t button) {
    if (button == START_BUTTON_PIN) {
        mowingRequested = true;
        // Genera l'evento di inizio taglio
        if (mowerPtr) {
            mowerPtr->handleEvent(Event::START_MOWING);
        }
        // Switch to status display after starting mowing
        currentState = STATUS_DISPLAY;
        DEBUG_PRINTLN("Mowing started, switching to STATUS_DISPLAY");
    } else if (button == STOP_BUTTON_PIN) {
        currentState = MAIN_MENU;
    }
}

void LCDMenu::handleMaintenanceMenu(uint8_t button) {
    if (button == STOP_BUTTON_PIN) {
        maintenanceMode = false;
        currentState = MAIN_MENU;
    }
}

void LCDMenu::handlePidConfigMenu(uint8_t button) {
    if (button == PLUS_BUTTON_PIN) {
        if (currentPidParam > 0) currentPidParam = (PidParam)(currentPidParam - 1);
    } else if (button == MINUS_BUTTON_PIN) {
        if (currentPidParam < PID_PARAM_COUNT - 1) currentPidParam = (PidParam)(currentPidParam + 1);
    } else if (button == START_BUTTON_PIN) {
        currentState = PID_EDIT;
    } else if (button == STOP_BUTTON_PIN) {
        currentState = MAIN_MENU;
    }
}

void LCDMenu::handlePidEdit(uint8_t button) {
    float* param = currentPidParam == PID_KP ? &kp : 
                  (currentPidParam == PID_KI ? &ki : &kd);
    float step = 0.1f;
    
    if (button == PLUS_BUTTON_PIN) {
        *param += step;
    } else if (button == MINUS_BUTTON_PIN) {
        *param = max(0.0f, *param - step);
    } else if (button == START_BUTTON_PIN) {
        savePidToEeprom();
        currentState = PID_CONFIG_MENU;
    } else if (button == STOP_BUTTON_PIN) {
        loadPidFromEeprom(); // Revert changes
        currentState = PID_CONFIG_MENU;
    }
}

void LCDMenu::savePidToEeprom() {
    EEPROM.put(EEPROM_KP_ADDR, kp);
    EEPROM.put(EEPROM_KI_ADDR, ki);
    EEPROM.put(EEPROM_KD_ADDR, kd);
}

void LCDMenu::loadPidFromEeprom() {
    // Default values if EEPROM is not initialized
    if (EEPROM.read(EEPROM_KP_ADDR) == 0xFF) {
        kp = 1.0f;
        ki = 0.1f;
        kd = 0.05f;
        savePidToEeprom();
    } else {
        EEPROM.get(EEPROM_KP_ADDR, kp);
        EEPROM.get(EEPROM_KI_ADDR, ki);
        EEPROM.get(EEPROM_KD_ADDR, kd);
    }
}

void LCDMenu::updateStatusDisplay() {
    lcd.clear();
    
    // Prima riga: stato del robot
    lcd.setCursor(0, 0);
    lcd.print("Stato: ");
    
    if (mowerPtr) {
        // Ottieni lo stato attuale dal mower
        State currentState = mowerPtr->getState();
        
        // Verifica se il puntatore al mower è valido
        if (mowerPtr == nullptr) {
            DEBUG_PRINTLN("ERRORE: mowerPtr è nullo!");
            return;
        }
        
        // Converti lo stato in stringa
        const char* stateStr = "SCONOSCIUTO";
        switch (currentState) {
            case State::IDLE: stateStr = "IN ATTESA"; break;
            case State::MOWING: stateStr = "TAGLIO"; break;
            case State::DOCKING: stateStr = "AGGANCIO"; break;
            case State::UNDOCKING: stateStr = "USCITA BASE"; break;
            case State::CHARGING: stateStr = "RICARICA"; break;
            case State::MANUAL_CONTROL: stateStr = "MANUALE"; break;
            case State::EMERGENCY_STOP: stateStr = "EMERGENZA"; break;
            case State::BORDER_DETECTED: stateStr = "BORDO"; break;
            case State::LIFTED: stateStr = "SOLLEVATO"; break;
            case State::TESTING: stateStr = "TEST"; break;
            case State::PAUSED: stateStr = "PAUSA"; break;
            case State::SLEEP: stateStr = "SOSPESO"; break;
            case State::RAIN_DELAY: stateStr = "PIOGGIA"; break;
            case State::MAINTENANCE_NEEDED: stateStr = "MANUTENZIONE"; break;
            case State::ROS_CONTROL: stateStr = "CONTROLLO ROS"; break;
            default: stateStr = "ALTRO";
        }
        
        lcd.print(stateStr);
        // Pulisci eventuali caratteri residui sulla riga
        int len = strlen(stateStr);
        for (int i = len; i < 16; i++) lcd.print(' ');
        
        // Seconda riga: livello batteria
        lcd.setCursor(0, 1);
        lcd.print("Batteria: ");
        lcd.print(static_cast<int>(mowerPtr->getBatteryPercentage()));
        lcd.print("%");
    } else {
        lcd.print("Nessun collegamento");
    }
}