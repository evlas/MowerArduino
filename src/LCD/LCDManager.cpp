#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "LCDManager.h"
#include "../eeprom/EEPROMConfig.h"
#include "../eeprom/EEPROMManager.h"

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

// Costanti per i menu
const char* const MOWING_PATTERNS[] = {"Random", "Parallelo", "Spirale", "Zig-Zag"};
const int MOWING_PATTERN_COUNT = 4;

const char* const NAV_MODES[] = {"Wire", "Virtuale", "GPS"};
const int NAV_MODES_COUNT = 3;

const char* const LANGUAGES[] = {"Italiano", "English", "Deutsch", "Français"};
const int LANGUAGES_COUNT = 4;

const char* const TIME_FORMATS[] = {"24h", "12h"};
const int TIME_FORMATS_COUNT = 2;

const char* const UNITS[] = {"Metrico", "Imperiale"};
const int UNITS_COUNT = 2;

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
        case MenuState::CONFIG_MOWING:
            showMowingConfig();
            break;
        case MenuState::CONFIG_NAVIGATION:
            // TODO: Implementa il menu di navigazione
            _menuState.setState(MenuState::CONFIG_MENU);
            break;
        case MenuState::CONFIG_SENSORS:
            // TODO: Implementa il menu sensori
            _menuState.setState(MenuState::CONFIG_MENU);
            break;
        case MenuState::CONFIG_MOTORS:
            // TODO: Implementa il menu motori
            _menuState.setState(MenuState::CONFIG_MENU);
            break;
        case MenuState::CONFIG_BATTERY:
            // TODO: Implementa il menu batteria
            _menuState.setState(MenuState::CONFIG_MENU);
            break;
        case MenuState::CONFIG_SYSTEM:
            // TODO: Implementa il menu di sistema
            _menuState.setState(MenuState::CONFIG_MENU);
            break;
        case MenuState::CONFIG_MAINTENANCE:
            // TODO: Implementa il menu di manutenzione
            _menuState.setState(MenuState::CONFIG_MENU);
            break;
        case MenuState::CONFIG_HOME_POSITION:
            showHomePositionConfig();
            break;
        case MenuState::CONFIG_RESET:
            // TODO: Implementa il ripristino delle impostazioni
            _menuState.setState(MenuState::CONFIG_MENU);
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
// Voci del menu di configurazione
const char* configMenuItems[] = {
    "1. Taglio",
    "2. Navigazione",
    "3. Sensori",
    "4. Motori",
    "5. Batteria",
    "6. Sistema",
    "7. Manutenzione",
    "8. Imposta Home",
    "9. Ripristino",
    "0. Torna al Menu"
};
const int CONFIG_MENU_ITEMS = sizeof(configMenuItems) / sizeof(configMenuItems[0]);

// Variabile per tenere traccia delle voci di menu effettive
int actualConfigMenuItems = CONFIG_MENU_ITEMS;

// Struttura per la navigazione nei menu
struct MenuNavigation {
    uint8_t selectedItem;
    bool editing;
    bool needsSave;
    
    void reset() {
        selectedItem = 0;
        editing = false;
        needsSave = false;
    }
};

MenuNavigation _nav;

void LCDManager::showConfigMenu() {
    static bool firstRun = true;
    
    // Inizializzazione
    if (firstRun) {
        _nav.reset();
        firstRun = false;
        clearDisplay();
        
        // Verifica se il GPS è abilitato
        EEPROMSettings settings;
        EEPROMManager::loadSettings(settings);
        actualConfigMenuItems = CONFIG_MENU_ITEMS;
        
        // Se il GPS non è abilitato, riduci il numero di voci di menu
        if (!settings.navigation.gpsEnabled) {
            actualConfigMenuItems--; // Rimuovi "Imposta Home"
        }
    }
    
    // Gestione navigazione
    if (isPlusPressed()) {
        _nav.selectedItem = (_nav.selectedItem + 1) % actualConfigMenuItems;
        delay(200);
    } else if (isMinusPressed()) {
        _nav.selectedItem = (_nav.selectedItem - 1 + actualConfigMenuItems) % actualConfigMenuItems;
        delay(200);
    }
    
    // Gestione selezione
    if (isStartPressed()) {
        // Se il GPS non è abilitato, aggiusta l'indice per saltare la voce "Imposta Home"
        int adjustedIndex = _nav.selectedItem;
        EEPROMSettings settings;
        EEPROMManager::loadSettings(settings);
        
        if (!settings.navigation.gpsEnabled && adjustedIndex >= 7) {
            adjustedIndex++; // Salta la voce "Imposta Home" (indice 7)
        }
        
        switch (adjustedIndex) {
            case 0: _menuState.setState(MenuState::CONFIG_MOWING); break;
            case 1: _menuState.setState(MenuState::CONFIG_NAVIGATION); break;
            case 2: _menuState.setState(MenuState::CONFIG_SENSORS); break;
            case 3: _menuState.setState(MenuState::CONFIG_MOTORS); break;
            case 4: _menuState.setState(MenuState::CONFIG_BATTERY); break;
            case 5: _menuState.setState(MenuState::CONFIG_SYSTEM); break;
            case 6: _menuState.setState(MenuState::CONFIG_MAINTENANCE); break;
            case 7: _menuState.setState(MenuState::CONFIG_HOME_POSITION); break;
            case 8: _menuState.setState(MenuState::CONFIG_RESET); break;
            case 9: _menuState.setState(MenuState::MAIN_MENU); break;
        }
        firstRun = true;
        delay(200);
        return;
    }
    
    // Aggiornamento display
    _lcd.setCursor(0, 0);
    _lcd.print("CONFIGURAZIONE    ");
    
    // Mostra le voci di menu con scroll, saltando "Imposta Home" se il GPS non è abilitato
    EEPROMSettings settings;
    EEPROMManager::loadSettings(settings);
    
    int itemsToShow = min(3, actualConfigMenuItems - _nav.selectedItem);
    int displayedItems = 0;
    
    for (int i = 0; i < 3 && _nav.selectedItem + i < CONFIG_MENU_ITEMS; i++) {
        // Se il GPS non è abilitato, salta la voce "Imposta Home" (indice 7)
        if (!settings.navigation.gpsEnabled && (_nav.selectedItem + i) == 7) {
            continue;
        }
        
        _lcd.setCursor(0, displayedItems + 1);
        _lcd.print(displayedItems == 0 ? ">" : " ");
        _lcd.print(configMenuItems[_nav.selectedItem + i]);
        
        displayedItems++;
        if (displayedItems >= itemsToShow) break;
    }
    
    // Mostra frecce di scroll
    _lcd.setCursor(15, 0);
    _lcd.print(_nav.selectedItem > 0 ? "^" : " ");
    _lcd.setCursor(15, 3);
    _lcd.print(_nav.selectedItem < actualConfigMenuItems - 3 ? "v" : " ");
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
 * @brief Display the mowing configuration menu.
 */
void LCDManager::showMowingConfig() {
    static bool firstRun = true;
    static uint8_t selectedParam = 0;
    static EEPROMSettings eepromSettings;
    
    // Carica le impostazioni correnti
    if (firstRun) {
        EEPROMManager::loadSettings(eepromSettings);
        firstRun = false;
    }
    
    // Riferimento alle impostazioni di taglio
    MowerSettings& settings = eepromSettings.mower;
    
    // Navigazione tra i parametri
    if (isPlusPressed()) {
        selectedParam = (selectedParam + 1) % 3;
        delay(200);
    } else if (isMinusPressed()) {
        selectedParam = (selectedParam - 1 + 3) % 3;
        delay(200);
    }
    
    // Modifica valori
    if (isStartPressed()) {
        switch (selectedParam) {
            case 0: // Altezza di taglio
                settings.cuttingHeight = (settings.cuttingHeight % 10) + 1;
                break;
            case 1: // Velocità lama
                settings.bladeSpeed = (settings.bladeSpeed + 10) % 110;
                if (settings.bladeSpeed < 30) settings.bladeSpeed = 30; // Min 30%
                break;
            case 2: // Pattern di taglio
                settings.mowingPattern = (settings.mowingPattern + 1) % 3;
                break;
        }
        // Salva le modifiche in EEPROM
        EEPROMManager::saveSettings(eepromSettings);
        delay(200);
    }
    
    // Torna al menu precedente
    if (isStopPressed()) {
        _menuState.setState(MenuState::CONFIG_MENU);
        firstRun = true;
        delay(200);
        return;
    }
    
    // Aggiorna display
    clearDisplay();
    _lcd.setCursor(0, 0);
    _lcd.print("Impost. Taglio");
    
    // Altezza di taglio
    _lcd.setCursor(0, 1);
    _lcd.print(selectedParam == 0 ? ">" : " ");
    _lcd.print("Altezza: ");
    _lcd.print(settings.cuttingHeight);
    
    // Velocità lama
    _lcd.setCursor(0, 2);
    _lcd.print(selectedParam == 1 ? ">" : " ");
    _lcd.print("Vel. lama: ");
    _lcd.print(settings.bladeSpeed);
    _lcd.print("%");
    
    // Pattern di taglio
    _lcd.setCursor(0, 3);
    _lcd.print(selectedParam == 2 ? ">" : " ");
    _lcd.print("Pattern: ");
    const char* patterns[] = {"Random", "Parallelo", "Spirale"};
    _lcd.print(patterns[settings.mowingPattern]);
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
    delay(50);  // wait 1/20 second to make progress visible
}

void LCDManager::showSetupComplete() {
    _lcd.setCursor(0, 1);
    _lcd.print("Setup completo  "); // padding to overwrite bar
    delay(5000);  // wait 1/20 second to make progress visible
}
