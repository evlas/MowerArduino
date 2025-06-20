#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "LCDManager.h"

// Variabili globali per il menu
const char* menuItems[] = {
    "Start",
    "Stop",
    "Speed",
    "Battery",
    "Config"
};
const int MENU_ITEMS_COUNT = sizeof(menuItems) / sizeof(menuItems[0]);
int selectedMenuItem = 0;

/**
 * @brief Construct a new LCDManager object.
 * 
 * Initializes the LCD display and sets up button pins with pull-up resistors.
 */
LCDManager::LCDManager() {
    _bootProgress = 0.0f;
    _lastDebounceTime = 0;
    
    // Inizializza i pin dei pulsanti
    pinMode(Start_Key, INPUT_PULLUP);
    pinMode(Plus_Key, INPUT_PULLUP);
    pinMode(Minus_Key, INPUT_PULLUP);
    pinMode(Stop_Key, INPUT_PULLUP);
}

/**
 * @brief Destroy the LCDManager object.
 */
LCDManager::~LCDManager() {
    // Cleanup if needed
}

/**
 * @brief Initialize the LCD display and set initial states.
 * 
 * Sets up the LCD display with backlight and initializes the menu
 * and robot states to their default values.
 */
void LCDManager::begin() {
    // Inizializza il display
    _lcd.begin(16, 2);
    _lcd.backlight();
    
    // Inizializza lo stato
    _menuState.setState(MenuState::MAIN_MENU);
    _robotState.setState(RobotState::IDLE);
    
    // Mostra lo schermo di boot
    showBootScreen();
}

/**
 * @brief Update the display and handle user input.
 * 
 * This method should be called in the main loop to keep the display
 * and user interface responsive. It updates button states and handles
 * the current menu navigation.
 */
void LCDManager::update() {
    // Aggiorna lo stato dei pulsanti
    updateButtonStates();
    
    // Gestisce la navigazione nel menu
    handleMenuNavigation();
    
    // Mostra lo stato corrente
    switch (_menuState.getState()) {
        case MenuState::MAIN_MENU:
            showMainMenu();
            break;
        case MenuState::CONFIG_MENU:
            showConfigMenu();
            break;
        case MenuState::SPEED_MENU:
            showSpeedMenu();
            break;
        case MenuState::BATTERY_MENU:
            showBatteryMenu();
            break;
        default:
            showMainMenu();
            break;
    }
}

/**
 * @brief Display the boot screen on the LCD.
 * 
 * Shows the initial boot message with a progress bar at 0%.
 */
void LCDManager::showBootScreen() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Mower Arduino");
    // Mostra la barra di avanzamento sulla seconda riga
    showProgressBar(0.0f);
}

/**
 * @brief Update the boot progress indicator.
 * 
 * @param progress Progress value between 0.0 and 1.0
 */
void LCDManager::updateBootProgress(float progress) {
    _bootProgress = progress;
    if (_bootProgress > 1.0f) _bootProgress = 1.0f;
    showProgressBar(_bootProgress);
}

/**
 * @brief Set the robot's current state and update the display.
 * 
 * @param state The new robot state to set
 */
void LCDManager::setRobotState(RobotState::State state) {
    _robotState.setState(state);
    showRobotState();
}

/**
 * @brief Display the current robot state on the LCD.
 * 
 * Shows a text representation of the current robot state
 * (e.g., "Idle", "Running", "Error").
 */
void LCDManager::showRobotState() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Status:");
    
    // Posiziona il testo dello stato sulla seconda riga
    _lcd.setCursor(0, 1);
    
    switch (_robotState.getState()) {
        case RobotState::IDLE:
            _lcd.print("Idle");
            break;
        case RobotState::STARTING:
            _lcd.print("Starting...");
            break;
        case RobotState::RUNNING:
            _lcd.print("Running");
            break;
        case RobotState::STOPPED:
            _lcd.print("Stopped");
            break;
        case RobotState::ERROR:
            _lcd.print("Error!");
            break;
        case RobotState::CHARGING:
            _lcd.print("Charging");
            break;
        case RobotState::EMERGENCY:
            _lcd.print("Emergency!");
            break;
        case RobotState::MANUAL:
            _lcd.print("Manual");
            break;
        case RobotState::RETURNING:
            _lcd.print("Returning");
            break;
    }
}

/**
 * @brief Display the main menu on the LCD.
 * 
 * Shows the currently selected menu item and instructions
 * for navigation.
 */
void LCDManager::showMainMenu() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print(menuItems[selectedMenuItem]);
    _lcd.setCursor(0, 1);
    _lcd.print("Use + - to select");
}

/**
 * @brief Handle menu navigation based on button presses.
 * 
 * Processes button presses to navigate through menu items
 * and change menu states.
 */
void LCDManager::handleMenuNavigation() {
    if (isPlusPressed()) {
        selectedMenuItem = (selectedMenuItem + 1) % MENU_ITEMS_COUNT;
        showMainMenu();
    }
    
    if (isMinusPressed()) {
        selectedMenuItem = (selectedMenuItem - 1 + MENU_ITEMS_COUNT) % MENU_ITEMS_COUNT;
        showMainMenu();
    }
    
    if (isStartPressed()) {
        switch (selectedMenuItem) {
            case 0: // Start
                _menuState.setState(MenuState::START_MENU);
                break;
            case 1: // Stop
                _menuState.setState(MenuState::STOP_MENU);
                break;
            case 2: // Speed
                _menuState.setState(MenuState::SPEED_MENU);
                break;
            case 3: // Battery
                _menuState.setState(MenuState::BATTERY_MENU);
                break;
            case 4: // Config
                _menuState.setState(MenuState::CONFIG_MENU);
                break;
        }
    }
}

/**
 * @brief Display the configuration menu.
 * 
 * Shows the configuration options (currently under development).
 */
void LCDManager::showConfigMenu() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Config Menu");
    _lcd.setCursor(0, 1);
    _lcd.print("Under development");
}

/**
 * @brief Display the speed control menu.
 * 
 * Shows speed control options (currently under development).
 */
void LCDManager::showSpeedMenu() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Speed Menu");
    _lcd.setCursor(0, 1);
    _lcd.print("Under development");
}

/**
 * @brief Display the battery information menu.
 * 
 * Shows battery status and information (currently under development).
 */
void LCDManager::showBatteryMenu() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Battery Menu");
    _lcd.setCursor(0, 1);
    _lcd.print("Under development");
}

/**
 * @brief Check if the start button is currently pressed.
 * 
 * @return true if the start button is pressed (LOW due to pull-up)
 * @return false if the start button is not pressed
 */
bool LCDManager::isStartPressed() const {
    return _startState == LOW;
}

/**
 * @brief Check if the stop button is currently pressed.
 * 
 * @return true if the stop button is pressed (LOW due to pull-up)
 * @return false if the stop button is not pressed
 */
bool LCDManager::isStopPressed() const {
    return _stopState == LOW;
}

/**
 * @brief Check if the plus button is currently pressed.
 * 
 * @return true if the plus button is pressed (LOW due to pull-up)
 * @return false if the plus button is not pressed
 */
bool LCDManager::isPlusPressed() const {
    return _plusState == LOW;
}

/**
 * @brief Check if the minus button is currently pressed.
 * 
 * @return true if the minus button is pressed (LOW due to pull-up)
 * @return false if the minus button is not pressed
 */
bool LCDManager::isMinusPressed() const {
    return _minusState == LOW;
}

void LCDManager::updateButtonStates() {
    unsigned long currentTime = millis();
    
    // Start button
    int startReading = digitalRead(Start_Key);
    if (currentTime - _lastDebounceTime > DEBOUNCE_DELAY) {
        if (startReading != _lastStartState) {
            _lastDebounceTime = currentTime;
            _startState = startReading;
        }
    }
    _lastStartState = startReading;
    
    // Stop button
    int stopReading = digitalRead(Stop_Key);
    if (currentTime - _lastDebounceTime > DEBOUNCE_DELAY) {
        if (stopReading != _lastStopState) {
            _lastDebounceTime = currentTime;
            _stopState = stopReading;
        }
    }
    _lastStopState = stopReading;
    
    // Plus button
    int plusReading = digitalRead(Plus_Key);
    if (currentTime - _lastDebounceTime > DEBOUNCE_DELAY) {
        if (plusReading != _lastPlusState) {
            _lastDebounceTime = currentTime;
            _plusState = plusReading;
        }
    }
    _lastPlusState = plusReading;
    
    // Minus button
    int minusReading = digitalRead(Minus_Key);
    if (currentTime - _lastDebounceTime > DEBOUNCE_DELAY) {
        if (minusReading != _lastMinusState) {
            _lastDebounceTime = currentTime;
            _minusState = minusReading;
        }
    }
    _lastMinusState = minusReading;
}

/**
 * @brief Clear the LCD display and reset cursor position.
 */
void LCDManager::clearDisplay() {
    _lcd.clear();
    _lcd.setCursor(0, 0);
}

/**
 * @brief Display a title on the LCD screen.
 * 
 * Clears the display and shows the given title on the first line.
 * 
 * @param title The title text to display
 */
void LCDManager::showMenuTitle(const char* title) {
    clearDisplay();
    _lcd.print(title);
    _lcd.setCursor(0, 1);
}

/**
 * @brief Display a progress bar on the LCD.
 * 
 * Shows a horizontal progress bar using block characters.
 * 
 * @param progress Progress value between 0.0 and 1.0
 */
void LCDManager::showProgressBar(float progress) {
    // Ensure progress is between 0 and 1
    progress = constrain(progress, 0.0f, 1.0f);
    
    // Calculate how many full block characters to show
    int width = 16;  // Display width in characters
    int blocks = progress * width;
    
    // Go to second row, clear then draw
    _lcd.setCursor(0, 1);
    for (int i = 0; i < width; i++) {
        _lcd.print(" ");
    }
    _lcd.setCursor(0, 1);
    for (int i = 0; i < blocks; i++) {
        _lcd.write((uint8_t)255);  // Full block character
    }
}

void LCDManager::showSetupComplete() {
    _lcd.setCursor(0, 1);
    _lcd.print("Setup completo  "); // padding to overwrite bar
}
