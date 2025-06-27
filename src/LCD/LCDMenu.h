#ifndef LCD_MENU_H
#define LCD_MENU_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "../pin_config.h"

// Menu states
enum MenuState {
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
    
    // PID getters
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }

private:
    void updateDisplay();
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
    bool mowingRequested;
    bool maintenanceMode;
    
    // PID parameters
    float kp, ki, kd;
    
    // Button state tracking
    unsigned long lastButtonPress;
    static const unsigned long DEBOUNCE_DELAY = 200;
    
    // EEPROM addresses for PID values
    static const int EEPROM_KP_ADDR = 0;
    static const int EEPROM_KI_ADDR = sizeof(float);
    static const int EEPROM_KD_ADDR = 2 * sizeof(float);
};

extern LCDMenu lcdMenu;

#endif // LCD_MENU_H
