[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)

# LCDMenu Class

## Methods

### Quick Reference

- [void backlight()](#backlight)
- [void begin()](#begin)
- [void clear()](#clear)
- [void display()](#display)
- [void forceBacklightOn()](#forcebacklighton)
- [float getKd()](#getkd)
- [float getKi()](#getki)
- [float getKp()](#getkp)
- [const char* getStateName(MenuState state)](#getstatename)
- [void handleButtonPress(uint8_t button)](#handlebuttonpress)
- [void handleMainMenu(uint8_t button)](#handlemainmenu)
- [void handleMaintenanceMenu(uint8_t button)](#handlemaintenancemenu)
- [void handleMowingMenu(uint8_t button)](#handlemowingmenu)
- [void handlePidConfigMenu(uint8_t button)](#handlepidconfigmenu)
- [void handlePidEdit(uint8_t button)](#handlepidedit)
- [bool isMaintenanceMode()](#ismaintenancemode)
- [bool isMowingRequested()](#ismowingrequested)
- [void loadPidFromEeprom()](#loadpidfromeeprom)
- [void noBacklight()](#nobacklight)
- [void noDisplay()](#nodisplay)
- [void print(const String )](#print)
- [void print(int number)](#print)
- [void print(float number)](#print)
- [void print(char c)](#print)
- [void refreshStatus()](#refreshstatus)
- [void savePidToEeprom()](#savepidtoeeprom)
- [void setCursor(uint8_t col, uint8_t row)](#setcursor)
- [public:
    
    void setMower(Mower* mower)](#setmower)
- [void setState(MenuState newState)](#setstate)
- [2 * sizeof()](#sizeof)
- [void update()](#update)
- [void updateBacklight()](#updatebacklight)
- [private:
    void updateDisplay()](#updatedisplay)
- [void updateStatusDisplay()](#updatestatusdisplay)

### Method Details

#### backlight

```cpp
void backlight()
```

**Returns:**

void - 

---

#### begin

```cpp
void begin()
```

**Returns:**

void - 

---

#### clear

```cpp
void clear()
```

**Returns:**

void - 

---

#### display

```cpp
void display()
```

**Returns:**

void - 

---

#### forceBacklightOn

```cpp
void forceBacklightOn()
```

**Returns:**

void - 

---

#### getKd

```cpp
float getKd()
```

**Returns:**

float - 

---

#### getKi

```cpp
float getKi()
```

**Returns:**

float - 

---

#### getKp

```cpp
float getKp()
```

**Returns:**

float - 

---

#### getStateName

```cpp
const char* getStateName(MenuState state)
```

**Parameters:**

- `state` (MenuState): No description

**Returns:**

const char* - 

---

#### handleButtonPress

```cpp
void handleButtonPress(uint8_t button)
```

**Parameters:**

- `button` (uint8_t): No description

**Returns:**

void - 

---

#### handleMainMenu

```cpp
void handleMainMenu(uint8_t button)
```

**Parameters:**

- `button` (uint8_t): No description

**Returns:**

void - 

---

#### handleMaintenanceMenu

```cpp
void handleMaintenanceMenu(uint8_t button)
```

**Parameters:**

- `button` (uint8_t): No description

**Returns:**

void - 

---

#### handleMowingMenu

```cpp
void handleMowingMenu(uint8_t button)
```

**Parameters:**

- `button` (uint8_t): No description

**Returns:**

void - 

---

#### handlePidConfigMenu

```cpp
void handlePidConfigMenu(uint8_t button)
```

**Parameters:**

- `button` (uint8_t): No description

**Returns:**

void - 

---

#### handlePidEdit

```cpp
void handlePidEdit(uint8_t button)
```

**Parameters:**

- `button` (uint8_t): No description

**Returns:**

void - 

---

#### isMaintenanceMode

```cpp
bool isMaintenanceMode()
```

**Returns:**

bool - 

---

#### isMowingRequested

```cpp
bool isMowingRequested()
```

**Returns:**

bool - 

---

#### loadPidFromEeprom

```cpp
void loadPidFromEeprom()
```

**Returns:**

void - 

---

#### noBacklight

```cpp
void noBacklight()
```

**Returns:**

void - 

---

#### noDisplay

```cpp
void noDisplay()
```

**Returns:**

void - 

---

#### print

```cpp
void print(const String )
```

**Parameters:**

- `` (const String): No description

**Returns:**

void - 

---

#### print

```cpp
void print(int number)
```

**Parameters:**

- `number` (int): No description

**Returns:**

void - 

---

#### print

```cpp
void print(float number)
```

**Parameters:**

- `number` (float): No description

**Returns:**

void - 

---

#### print

```cpp
void print(char c)
```

**Parameters:**

- `c` (char): No description

**Returns:**

void - 

---

#### refreshStatus

```cpp
void refreshStatus()
```

**Returns:**

void - 

---

#### savePidToEeprom

```cpp
void savePidToEeprom()
```

**Returns:**

void - 

---

#### setCursor

```cpp
void setCursor(uint8_t col, uint8_t row)
```

**Parameters:**

- `col` (uint8_t): No description
- `row` (uint8_t): No description

**Returns:**

void - 

---

#### setMower

```cpp
public:
    
    void setMower(Mower* mower)
```

**Parameters:**

- `mower` (Mower*): No description

**Returns:**

public:
    
    void - 

---

#### setState

```cpp
void setState(MenuState newState)
```

**Parameters:**

- `newState` (MenuState): No description

**Returns:**

void - 

---

#### sizeof

```cpp
2 * sizeof()
```

**Returns:**

2 * - 

---

#### update

```cpp
void update()
```

**Returns:**

void - 

---

#### updateBacklight

```cpp
void updateBacklight()
```

**Returns:**

void - 

---

#### updateDisplay

```cpp
private:
    void updateDisplay()
```

**Returns:**

private:
    void - 

---

#### updateStatusDisplay

```cpp
void updateStatusDisplay()
```

**Returns:**

void - 

---

---

[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)
