#ifndef LCD_MENU_H
#define LCD_MENU_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "../pin_config.h"
#include "../config.h"

// Forward declarations
class Mower;  // Forward declaration di Mower

// Menu states

// Menu states
enum MenuState {
    STATUS_DISPLAY,  // Visualizza stato e batteria
    MAIN_MENU,
    MOWING_MENU,
    MAINTENANCE_MENU,
    PID_CONFIG_MENU,
    PID_EDIT
};

// Menu items for main menu
enum MainMenuItems {
    ITEM_START_MOWING,
    ITEM_MAINTENANCE,
    ITEM_PID_CONFIG,
    MAIN_MENU_ITEMS_COUNT
};

// PID parameters that can be configured
enum PidParam {
    PID_KP,
    PID_KI,
    PID_KD,
    PID_PARAM_COUNT
};

class LCDMenu {
public:
    LCDMenu();
    void begin();
    void update();
    void handleButtonPress(uint8_t button);
    
    // Getters for menu state
    bool isMowingRequested() const { return mowingRequested; }
    bool isMaintenanceMode() const { return maintenanceMode; }
    
    // Debug function to get state name
    const char* getStateName(MenuState state);
    void updateBacklight();
    
    void setState(MenuState newState) {
        if (currentState != newState) {
            DEBUG_PRINT("Stato cambiato da ");
            DEBUG_PRINT(getStateName(currentState));
            DEBUG_PRINT(" a ");
            DEBUG_PRINTLN(getStateName(newState));
            currentState = newState;
            lastDisplayUpdate = 0; // Forza l'aggiornamento del display
        }
    }
    
    // PID getters
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
    
    // LCD Operations
    void clear() { lcd.clear(); }
    void setCursor(uint8_t col, uint8_t row) { lcd.setCursor(col, row); }
    void print(const String &text) { lcd.print(text); }
    void print(int number) { lcd.print(number); }
    void print(float number) { lcd.print(number); }
    void print(char c) { lcd.print(c); }
    void backlight() { lcd.backlight(); }
    
    /**
     * @brief Force backlight on and set internal flag
     */
    void refreshStatus() { updateStatusDisplay(); }
    
    void forceBacklightOn() {
        lcd.backlight();
        backlightOn = true;
    }
    void noBacklight() { lcd.noBacklight(); }
    void display() { lcd.display(); }
    void noDisplay() { lcd.noDisplay(); }

private:
    void updateDisplay();
    void updateStatusDisplay();
    void handleMainMenu(uint8_t button);
    void handleMowingMenu(uint8_t button);
    void handleMaintenanceMenu(uint8_t button);
    void handlePidConfigMenu(uint8_t button);
    void handlePidEdit(uint8_t button);
    void savePidToEeprom();
    void loadPidFromEeprom();
    
    LiquidCrystal_I2C lcd;
    MenuState currentState;
    uint8_t currentItem;
    PidParam currentPidParam;
    unsigned long lastDisplayUpdate;  // Timestamp dell'ultimo aggiornamento del display
    bool mowingRequested;
    bool maintenanceMode;
    
    // PID parameters
    float kp, ki, kd;
    
    // Backlight control
    unsigned long lastUserActivity;    // Timestamp dell'ultima attività dell'utente
    bool backlightOn;                  // Stato della retroilluminazione
    
    // Button state tracking
    unsigned long lastButtonPress;
    unsigned long lastButtonCheck;
    
    // EEPROM addresses for PID values
    static const int EEPROM_KP_ADDR = 0;
    static const int EEPROM_KI_ADDR = sizeof(float);
    static const int EEPROM_KD_ADDR = 2 * sizeof(float);
    
    // Mower instance pointer
    Mower* mowerPtr;
    
public:
    // Setter for mower pointer
    void setMower(Mower* mower) {
        mowerPtr = mower;
    }
};

// Includi Mower.h dopo la definizione di LCDMenu
#include "../functions/Mower.h"

#endif // LCD_MENU_H
