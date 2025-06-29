[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)

# LCDMenu Class

## Methods

### Quick Reference

- [void begin()](#begin)
- [float getKd()](#getkd)
- [float getKi()](#getki)
- [float getKp()](#getkp)
- [void handleButtonPress(uint8_t button)](#handlebuttonpress)
- [void handleMainMenu(uint8_t button)](#handlemainmenu)
- [void handleMaintenanceMenu(uint8_t button)](#handlemaintenancemenu)
- [void handleMowingMenu(uint8_t button)](#handlemowingmenu)
- [void handlePidConfigMenu(uint8_t button)](#handlepidconfigmenu)
- [void handlePidEdit(uint8_t button)](#handlepidedit)
- [bool isMaintenanceMode()](#ismaintenancemode)
- [bool isMowingRequested()](#ismowingrequested)
- [void loadPidFromEeprom()](#loadpidfromeeprom)
- [void savePidToEeprom()](#savepidtoeeprom)
- [2 * sizeof()](#sizeof)
- [void update()](#update)
- [private:
    void updateDisplay()](#updatedisplay)

### Method Details

#### begin

```cpp
void begin()
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

#### savePidToEeprom

```cpp
void savePidToEeprom()
```

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

#### updateDisplay

```cpp
private:
    void updateDisplay()
```

**Returns:**

private:
    void - 

---

---

[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)
