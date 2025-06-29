#ifndef LCD_MANAGER_H
#define LCD_MANAGER_H

#include <Arduino.h>
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "../../pin_config.h"
#include "../eeprom/EEPROMConfig.h"
#include "../eeprom/EEPROMManager.h"

/**
 * @brief Enumerates the possible menu states for the LCD display.
 */
class MenuState {
    public:
        enum State {
            MAIN_MENU,
            CONFIG_MENU,
            CONFIG_MOWING,
            CONFIG_NAVIGATION,
            CONFIG_SENSORS,
            CONFIG_MOTORS,
            CONFIG_BATTERY,
            CONFIG_SYSTEM,
            CONFIG_MAINTENANCE,
            CONFIG_HOME_POSITION,
            CONFIG_RESET,
            CONFIG_PID,
            START_MENU,
            STOP_MENU,
            SPEED_MENU,
            BATTERY_MENU
        } currentState;
        
        /**
         * @brief Construct a new MenuState object with MAIN_MENU as default state.
         */
        MenuState() : currentState(MAIN_MENU) {}
        
        /**
         * @brief Set the current menu state.
         * @param newState The new state to set.
         */
        void setState(State newState) { currentState = newState; }
        
        /**
         * @brief Get the current menu state.
         * @return The current menu state.
         */
        State getState() const { return currentState; }
};

/**
 * @brief Enumerates the possible states of the robot.
 */
class RobotState {
    public:
        enum State {
            IDLE,
            STARTING,
            RUNNING,
            MANUAL,
            RETURNING,
            STOPPED,
            ERROR,
            CHARGING,
            EMERGENCY
        } currentState;
        
        /**
         * @brief Construct a new RobotState object with IDLE as default state.
         */
        RobotState() : currentState(IDLE) {}
        
        /**
         * @brief Set the current robot state.
         * @param newState The new state to set.
         */
        void setState(State newState) { currentState = newState; }
        
        /**
         * @brief Get the current robot state.
         * @return The current robot state.
         */
        State getState() const { return currentState; }
};

// Forward declarations
struct PIDParams;
struct PositionPID;

/**
 * @brief Manages the LCD display and user interface for the Mower Arduino.
 * 
 * This class handles all display operations, menu navigation, and user input
 * through the LCD display and associated buttons.
 */
class LCDManager {
    public:
        LCDManager();
        ~LCDManager();
        
        /**
         * @brief Initialize the LCD display and set initial states.
         */
        void begin();
        
        /**
         * @brief Update the display and handle user input.
         * 
         * This method should be called in the main loop to keep the display
         * and user interface responsive.
         */
        void update();
        
        /**
         * @brief Display the boot screen on the LCD.
         */
        void showBootScreen();
        
        /**
         * @brief Update the boot progress indicator.
         * @param progress Progress value between 0.0 and 1.0
         */
        void updateBootProgress(float progress);
        
        /**
         * @brief Show message when setup is finished (second row)
         */
        void showSetupComplete();
        
        /**
         * @brief Set the robot's current state and update the display.
         * @param state The new robot state
         */
        void setRobotState(RobotState::State state);
        
        /**
         * @brief Display the current robot state on the LCD.
         */
        void showRobotState();
        
        /**
         * @brief Display the main menu on the LCD.
         */
        void showMainMenu();
        
        /**
         * @brief Handle menu navigation based on button presses.
         */
        void handleMenuNavigation();
        
        /**
         * @brief Display the configuration menu.
         */
        void showConfigMenu();
        
        /**
         * @brief Display the speed control menu.
         */
        void showSpeedMenu();
        
        /**
         * @brief Display the battery information menu.
         */
        void showBatteryMenu();
        
        /**
         * @brief Display the home position configuration menu.
         * 
         * Allows the user to set or clear the home position.
         */
        void showHomePositionConfig();
        
        // Funzioni per i menu di configurazione
        void showMowingConfig();
        void showNavigationConfig();
        void showSystemConfig();
        void showNetworkConfig();
        void showResetConfirm();
        
        // Funzioni di utilità
        void showEditValue(const char* label, const char* value, uint8_t line, bool selected, bool editing = false);
        void showEditValue(const char* label, int value, uint8_t line, bool selected, bool editing = false, const char* suffix = "");
        void showEditValue(const char* label, float value, uint8_t line, bool selected, bool editing = false, uint8_t decimals = 1, const char* suffix = "");
        void showEditBool(const char* label, bool value, uint8_t line, bool selected, bool editing = false);
        void showEditList(const char* label, const char* const* items, uint8_t count, uint8_t selectedIndex, uint8_t line, bool selected, bool editing = false);
        
        // Gestione del salvataggio
        void showSavingMessage();
        void showSavedMessage();
        
        // Gestione PID
        void showPIDMenu();
        void handlePIDEdit();
        
        /**
         * @brief Check if the start button is pressed.
         * @return true if pressed, false otherwise
         */
        bool isStartPressed() const;
        
        /**
         * @brief Check if the stop button is pressed.
         * @return true if pressed, false otherwise
         */
        bool isStopPressed() const;
        
        /**
         * @brief Check if the plus button is pressed.
         * @return true if pressed, false otherwise
         */
        bool isPlusPressed() const;
        
        /**
         * @brief Check if the minus button is pressed.
         * @return true if pressed, false otherwise
         */
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
        static const int PID_TYPES_COUNT = 4;
        static const int PID_PARAMS_COUNT = 6;
        
        // Limiti parametri PID
        static const float PID_MIN_K = 0.0f;
        static const float PID_MAX_K = 10.0f;
        static const float PID_MIN_OUT = -1.0f;
        static const float PID_MAX_OUT = 1.0f;
        static const float PID_MIN_ILIMIT = 0.0f;
        static const float PID_MAX_ILIMIT = 5.0f;
        static const float PID_STEP = 0.1f;
        
        // Funzioni private
        void updateButtonStates();
        void clearDisplay();
        void showMenuTitle(const char* title);
        void showProgressBar(float progress);
        
        // Funzioni helper PID
        float getPIDValue(const PIDParams& params, uint8_t param) const;
        float* getPIDParamPtr(PIDParams& params, uint8_t param);
        float getMinLimit(uint8_t param) const;
        float getMaxLimit(uint8_t param) const;
};

extern LCDManager lcdManager;

#endif
