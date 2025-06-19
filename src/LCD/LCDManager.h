#ifndef LCD_MANAGER_H
#define LCD_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "../../pin_config.h"

/**
 * @brief Enumerates the possible menu states for the LCD display.
 */
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
        
        // Funzioni private
        void updateButtonStates();
        void clearDisplay();
        void showMenuTitle(const char* title);
        void showProgressBar(float progress);
};

extern LCDManager lcdManager;

#endif
