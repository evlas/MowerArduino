#include <Arduino.h>  // Deve essere incluso per primo per Serial
#include <Wire.h>     // Necessario per I2C
#include <EEPROM.h>
#include "LCDMenu.h"
#include "../config.h" // Ensure DEBUG_MODE and debug macros are available

LCDMenu::LCDMenu() 
    : lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE),
      currentState(MAIN_MENU),
      currentItem(0),
      currentPidParam(PID_KP),
      mowingRequested(false),
      maintenanceMode(false),
      kp(1.0), ki(0.1), kd(0.05),
      lastButtonPress(0) {
}

void LCDMenu::begin() {
   // Inizializza LCD
    DEBUG_PRINTLN("Inizializzazione LCD...");
    lcd.begin(16, 2);
    delay(50);   // Piccola pausa dopo l'inizializzazione LCD
    
    lcd.backlight();
    delay(50);   // Piccola pausa dopo l'accensione retroilluminazione
    lcd.clear();
    delay(50);   // Piccola pausa dopo il clear
    lcd.print("Mower Control");
    #ifdef DEBUG_MODE
        DEBUG_PRINTLN("Initializing buttons...");
    #endif
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");
    
    // Initialize buttons
    #ifdef DEBUG_MODE
        DEBUG_PRINTLN("Loading PID values from EEPROM...");
    #endif

    #ifdef DEBUG_MODE
        DEBUG_PRINTLN("LCD initialization complete");
    #endif
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(PLUS_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MINUS_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    
    // Load saved PID values
    loadPidFromEeprom();
    
    delay(1000); // Show initialization message
    updateDisplay();
}

void LCDMenu::update() {
    // Check button presses with debounce
    unsigned long currentTime = millis();
    if (currentTime - lastButtonPress < DEBOUNCE_DELAY) {
        return;
    }
    
    if (digitalRead(START_BUTTON_PIN) == LOW) {
        lastButtonPress = currentTime;
    #ifdef DEBUG_MODE
        DEBUG_PRINTLN("Display updated after button press");
    #endif
        handleButtonPress(START_BUTTON_PIN);
    } 
    else if (digitalRead(PLUS_BUTTON_PIN) == LOW) {
        lastButtonPress = currentTime;
        handleButtonPress(PLUS_BUTTON_PIN);
    } 
    else if (digitalRead(MINUS_BUTTON_PIN) == LOW) {
        lastButtonPress = currentTime;
        handleButtonPress(MINUS_BUTTON_PIN);
    } 
    else if (digitalRead(STOP_BUTTON_PIN) == LOW) {
        lastButtonPress = currentTime;
        handleButtonPress(STOP_BUTTON_PIN);
    }
}

void LCDMenu::handleButtonPress(uint8_t button) {
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

void LCDMenu::updateDisplay() {
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
        currentState = MAIN_MENU;
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
