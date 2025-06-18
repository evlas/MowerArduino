#ifndef LCD_MANAGER_H
#define LCD_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "../../pin_config.h"

// Definizione dello stato del menu
class MenuState {
    public:
        enum State {
            MAIN_MENU,
            CONFIG_MENU,
            START_MENU,
            STOP_MENU,
            SPEED_MENU,
            BATTERY_MENU
        } currentState;
        
        MenuState() : currentState(MAIN_MENU) {}
        void setState(State newState) { currentState = newState; }
        State getState() const { return currentState; }
};

// Definizione dello stato del robot
class RobotState {
    public:
        enum State {
            IDLE,
            STARTING,
            RUNNING,
            STOPPED,
            ERROR,
            CHARGING,
            EMERGENCY
        } currentState;
        
        RobotState() : currentState(IDLE) {}
        void setState(State newState) { currentState = newState; }
        State getState() const { return currentState; }
};

class LCDManager {
    public:
        LCDManager();
        ~LCDManager();
        
        // Inizializzazione
        void begin();
        
        // Aggiornamento dello stato
        void update();
        
        // Gestione del boot
        void showBootScreen();
        void updateBootProgress(float progress);
        
        // Gestione dello stato del robot
        void setRobotState(RobotState::State state);
        void showRobotState();
        
        // Gestione del menu
        void showMainMenu();
        void handleMenuNavigation();
        void showConfigMenu();
        void showSpeedMenu();
        void showBatteryMenu();
        
        // Gestione dei pulsanti
        bool isStartPressed() const;
        bool isStopPressed() const;
        bool isPlusPressed() const;
        bool isMinusPressed() const;
        
    private:
        // Display
        LiquidCrystal_I2C _lcd = LiquidCrystal_I2C(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
        
        // Stati
        MenuState _menuState;
        RobotState _robotState;
        
        // Variabili per il debounce dei pulsanti
        unsigned long _lastDebounceTime;
        bool _lastStartState, _startState;
        bool _lastStopState, _stopState;
        bool _lastPlusState, _plusState;
        bool _lastMinusState, _minusState;
        
        // Progresso del boot
        float _bootProgress;
        
        // Costanti
        static const unsigned long DEBOUNCE_DELAY = 50;
        
        // Funzioni private
        void updateButtonStates();
        void clearDisplay();
        void showMenuTitle(const char* title);
        void showProgressBar(float progress);
};

extern LCDManager lcdManager;

#endif
