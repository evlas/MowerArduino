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

LCDManager::LCDManager() {
    _bootProgress = 0.0f;
    _lastDebounceTime = 0;
    
    // Inizializza i pin dei pulsanti
    pinMode(Start_Key, INPUT_PULLUP);
    pinMode(Plus_Key, INPUT_PULLUP);
    pinMode(Minus_Key, INPUT_PULLUP);
    pinMode(Stop_Key, INPUT_PULLUP);
}

LCDManager::~LCDManager() {
    // Null
}

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

void LCDManager::showBootScreen() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Mower Arduino");
    _lcd.setCursor(0, 1);
    _lcd.print("Starting...");
    showProgressBar(0.0f);
}

void LCDManager::updateBootProgress(float progress) {
    _bootProgress = progress;
    if (_bootProgress > 1.0f) _bootProgress = 1.0f;
    showProgressBar(_bootProgress);
}

void LCDManager::setRobotState(RobotState::State state) {
    _robotState.setState(state);
    showRobotState();
}

void LCDManager::showRobotState() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Status:");
    
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
    }
}

void LCDManager::showMainMenu() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print(menuItems[selectedMenuItem]);
    _lcd.setCursor(0, 1);
    _lcd.print("Use + - to select");
}

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

void LCDManager::showConfigMenu() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Config Menu");
    _lcd.setCursor(0, 1);
    _lcd.print("Under development");
}

void LCDManager::showSpeedMenu() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Speed Menu");
    _lcd.setCursor(0, 1);
    _lcd.print("Under development");
}

void LCDManager::showBatteryMenu() {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Battery Menu");
    _lcd.setCursor(0, 1);
    _lcd.print("Under development");
}

bool LCDManager::isStartPressed() const {
    return _startState == LOW;
}

bool LCDManager::isStopPressed() const {
    return _stopState == LOW;
}

bool LCDManager::isPlusPressed() const {
    return _plusState == LOW;
}

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

void LCDManager::clearDisplay() {
    _lcd.clear();
    _lcd.setCursor(0, 0);
}

void LCDManager::showMenuTitle(const char* title) {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print(title);
}

void LCDManager::showProgressBar(float progress) {
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Progress:");
    
    // Calcola il numero di caratteri per la barra
    int barLength = static_cast<int>(progress * 16);
    _lcd.setCursor(0, 1);
    for (int i = 0; i < 16; i++) {
        if (i < barLength) {
            _lcd.print("=");
        } else {
            _lcd.print(" ");
        }
    }
}
