[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)

# Mower Class

## Methods

### Quick Reference

- [void begin()](#begin)
- [float calculateDockAlignmentCorrection()](#calculatedockalignmentcorrection)
- [void changeState(MowerState& newState)](#changestate)
- [bool checkObstacles()](#checkobstacles)
- [void checkSafety()](#checksafety)
- [void clearLcdDisplay()](#clearlcddisplay)
- [void emergencyStop()](#emergencystop)
- [bool enableCharging(bool enable)](#enablecharging)
- [const char* eventToString(Event event)](#eventtostring)
- [void followPerimeter()](#followperimeter)
- [float getBatteryLevel()](#getbatterylevel)
- [float getBatteryPercentage()](#getbatterypercentage)
- [MowerState& getChargingState()](#getchargingstate)
- [MowerState* getCurrentState()](#getcurrentstate)
- [MowerState& getDockingState()](#getdockingstate)
- [MowerState& getEmergencyStopState()](#getemergencystopstate)
- [MowerState& getErrorState()](#geterrorstate)
- [MowerState& getIdleState()](#getidlestate)
- [String getLastError()](#getlasterror)
- [MowerState& getLiftedState()](#getliftedstate)
- [MowerState& getMowingState()](#getmowingstate)
- [NavigationMode getNavigationMode()](#getnavigationmode)
- [State getState()](#getstate)
- [MowerState& getUndockingState()](#getundockingstate)
- [unsigned long getUptime()](#getuptime)
- [void handleBorder()](#handleborder)
- [void handleEvent(Event event)](#handleevent)
- [void handleObstacle()](#handleobstacle)
- [void init()](#init)
- [void initializeComponents()](#initializecomponents)
- [bool isAlignedWithDock()](#isalignedwithdock)
- [bool isBatteryCharged()](#isbatterycharged)
- [bool isBatteryCritical()](#isbatterycritical)
- [bool isBatteryFull()](#isbatteryfull)
- [bool isBatteryLow()](#isbatterylow)
- [bool isBorderDetected()](#isborderdetected)
- [bool isCharging()](#ischarging)
- [bool isCollisionDetected()](#iscollisiondetected)
- [bool isDocked()](#isdocked)
- [bool isEmergencyStopActive()](#isemergencystopactive)
- [bool isErrorResolved()](#iserrorresolved)
- [bool isLifted()](#islifted)
- [bool isObstacleDetected()](#isobstacledetected)
- [bool isRaining()](#israining)
- [void logError(const String& error)](#logerror)
- [void logStatus()](#logstatus)
- [const char* navigationModeToString(NavigationMode mode)](#navigationmodetostring)
- [void playBuzzerTone(unsigned int frequency, unsigned long duration)](#playbuzzertone)
- [void printDebugInfo()](#printdebuginfo)
- [void printToLcd(const char* text)](#printtolcd)
- [void printToLcd(int number)](#printtolcd)
- [void printToLcd(float number, int decimals = 2)](#printtolcd)
- [void resetEmergencyStop()](#resetemergencystop)
- [void setBladeSpeed(float speed)](#setbladespeed)
- [void setBorderDetected(bool detected)](#setborderdetected)
- [void setCharging(bool charging)](#setcharging)
- [void setCollisionDetected(bool detected)](#setcollisiondetected)
- [void setDocked(bool docked)](#setdocked)
- [void setLcdCursor(uint8_t col, uint8_t row)](#setlcdcursor)
- [void setLeftMotorSpeed(float speed)](#setleftmotorspeed)
- [void setLifted(bool lifted)](#setlifted)
- [void setNavigationMode(NavigationMode mode)](#setnavigationmode)
- [void setRightMotorSpeed(float speed)](#setrightmotorspeed)
- [void setState(State newState)](#setstate)
- [void setState(MowerState& newState)](#setstate)
- [void startBlades()](#startblades)
- [void startDocking()](#startdocking)
- [void startDriveMotors()](#startdrivemotors)
- [void startMowing()](#startmowing)
- [void startRandomMovement()](#startrandommovement)
- [const char* stateToString(State state)](#statetostring)
- [void stopBlades()](#stopblades)
- [void stopBuzzer()](#stopbuzzer)
- [void stopDriveMotors()](#stopdrivemotors)
- [void stopMotors()](#stopmotors)
- [void stopRandomMovement()](#stoprandommovement)
- [void update()](#update)
- [void updateLcdDisplay(const char* line1, const char* line2 = "")](#updatelcddisplay)
- [void updateMotors()](#updatemotors)
- [void updateSensors()](#updatesensors)

### Method Details

#### begin

```cpp
void begin()
```

**Returns:**

void - 

---

#### calculateDockAlignmentCorrection

```cpp
float calculateDockAlignmentCorrection()
```

**Returns:**

float - 

---

#### changeState

```cpp
void changeState(MowerState& newState)
```

**Parameters:**

- `newState` (MowerState&): No description

**Returns:**

void - 

---

#### checkObstacles

```cpp
bool checkObstacles()
```

**Returns:**

bool - 

---

#### checkSafety

```cpp
void checkSafety()
```

**Returns:**

void - 

---

#### clearLcdDisplay

```cpp
void clearLcdDisplay()
```

**Returns:**

void - 

---

#### emergencyStop

```cpp
void emergencyStop()
```

**Returns:**

void - 

---

#### enableCharging

```cpp
bool enableCharging(bool enable)
```

**Parameters:**

- `enable` (bool): No description

**Returns:**

bool - 

---

#### eventToString

```cpp
const char* eventToString(Event event)
```

**Parameters:**

- `event` (Event): No description

**Returns:**

const char* - 

---

#### followPerimeter

```cpp
void followPerimeter()
```

**Returns:**

void - 

---

#### getBatteryLevel

```cpp
float getBatteryLevel()
```

**Returns:**

float - 

---

#### getBatteryPercentage

```cpp
float getBatteryPercentage()
```

**Returns:**

float - 

---

#### getChargingState

```cpp
MowerState& getChargingState()
```

**Returns:**

MowerState& - 

---

#### getCurrentState

```cpp
MowerState* getCurrentState()
```

**Returns:**

MowerState* - Reference to the ErrorState singleton

---

#### getDockingState

```cpp
MowerState& getDockingState()
```

**Returns:**

MowerState& - 

---

#### getEmergencyStopState

```cpp
MowerState& getEmergencyStopState()
```

**Returns:**

MowerState& - 

---

#### getErrorState

```cpp
MowerState& getErrorState()
```

**Returns:**

MowerState& - 

---

#### getIdleState

```cpp
MowerState& getIdleState()
```

**Returns:**

MowerState& - 

---

#### getLastError

```cpp
String getLastError()
```

**Returns:**

String - 

---

#### getLiftedState

```cpp
MowerState& getLiftedState()
```

**Returns:**

MowerState& - 

---

#### getMowingState

```cpp
MowerState& getMowingState()
```

**Returns:**

MowerState& - 

---

#### getNavigationMode

```cpp
NavigationMode getNavigationMode()
```

**Returns:**

NavigationMode - 

---

#### getState

```cpp
State getState()
```

**Returns:**

State - 

---

#### getUndockingState

```cpp
MowerState& getUndockingState()
```

**Returns:**

MowerState& - 

---

#### getUptime

```cpp
unsigned long getUptime()
```

**Returns:**

unsigned long - 

---

#### handleBorder

```cpp
void handleBorder()
```

**Returns:**

void - 

---

#### handleEvent

```cpp
void handleEvent(Event event)
```

**Parameters:**

- `event` (Event): No description

**Returns:**

void - 

---

#### handleObstacle

```cpp
void handleObstacle()
```

**Returns:**

void - 

---

#### init

```cpp
void init()
```

**Returns:**

void - 

---

#### initializeComponents

```cpp
void initializeComponents()
```

**Returns:**

void - 

---

#### isAlignedWithDock

```cpp
bool isAlignedWithDock()
```

**Returns:**

bool - 

---

#### isBatteryCharged

```cpp
bool isBatteryCharged()
```

**Returns:**

bool - 

---

#### isBatteryCritical

```cpp
bool isBatteryCritical()
```

**Returns:**

bool - 

---

#### isBatteryFull

```cpp
bool isBatteryFull()
```

**Returns:**

bool - 

---

#### isBatteryLow

```cpp
bool isBatteryLow()
```

**Returns:**

bool - 

---

#### isBorderDetected

```cpp
bool isBorderDetected()
```

**Returns:**

bool - 

---

#### isCharging

```cpp
bool isCharging()
```

**Returns:**

bool - 

---

#### isCollisionDetected

```cpp
bool isCollisionDetected()
```

**Returns:**

bool - 

---

#### isDocked

```cpp
bool isDocked()
```

**Returns:**

bool - 

---

#### isEmergencyStopActive

```cpp
bool isEmergencyStopActive()
```

**Returns:**

bool - 

---

#### isErrorResolved

```cpp
bool isErrorResolved()
```

**Returns:**

bool - 

---

#### isLifted

```cpp
bool isLifted()
```

**Returns:**

bool - 

---

#### isObstacleDetected

```cpp
bool isObstacleDetected()
```

**Returns:**

bool - 

---

#### isRaining

```cpp
bool isRaining()
```

**Returns:**

bool - 

---

#### logError

```cpp
void logError(const String& error)
```

**Parameters:**

- `error` (const String&): No description

**Returns:**

void - 

---

#### logStatus

```cpp
void logStatus()
```

**Returns:**

void - 

---

#### navigationModeToString

```cpp
const char* navigationModeToString(NavigationMode mode)
```

**Parameters:**

- `mode` (NavigationMode): No description

**Returns:**

const char* - 

---

#### playBuzzerTone

```cpp
void playBuzzerTone(unsigned int frequency, unsigned long duration)
```

**Parameters:**

- `frequency` (unsigned int): No description
- `duration` (unsigned long): No description

**Returns:**

void - 

---

#### printDebugInfo

```cpp
void printDebugInfo()
```

**Returns:**

void - 

---

#### printToLcd

```cpp
void printToLcd(const char* text)
```

**Parameters:**

- `text` (const char*): No description

**Returns:**

void - 

---

#### printToLcd

```cpp
void printToLcd(int number)
```

**Parameters:**

- `number` (int): No description

**Returns:**

void - 

---

#### printToLcd

```cpp
void printToLcd(float number, int decimals = 2)
```

**Parameters:**

- `number` (float): No description
- `2` (int decimals =): No description

**Returns:**

void - 

---

#### resetEmergencyStop

```cpp
void resetEmergencyStop()
```

**Returns:**

void - 

---

#### setBladeSpeed

```cpp
void setBladeSpeed(float speed)
```

**Parameters:**

- `speed` (float): No description

**Returns:**

void - 

---

#### setBorderDetected

```cpp
void setBorderDetected(bool detected)
```

**Parameters:**

- `detected` (bool): No description

**Returns:**

void - 

---

#### setCharging

```cpp
void setCharging(bool charging)
```

**Parameters:**

- `charging` (bool): No description

**Returns:**

void - 

---

#### setCollisionDetected

```cpp
void setCollisionDetected(bool detected)
```

**Parameters:**

- `detected` (bool): No description

**Returns:**

void - 

---

#### setDocked

```cpp
void setDocked(bool docked)
```

**Parameters:**

- `docked` (bool): No description

**Returns:**

void - 

---

#### setLcdCursor

```cpp
void setLcdCursor(uint8_t col, uint8_t row)
```

**Parameters:**

- `col` (uint8_t): No description
- `row` (uint8_t): No description

**Returns:**

void - 

---

#### setLeftMotorSpeed

```cpp
void setLeftMotorSpeed(float speed)
```

**Parameters:**

- `speed` (float): No description

**Returns:**

void - 

---

#### setLifted

```cpp
void setLifted(bool lifted)
```

**Parameters:**

- `lifted` (bool): No description

**Returns:**

void - 

---

#### setNavigationMode

```cpp
void setNavigationMode(NavigationMode mode)
```

**Parameters:**

- `mode` (NavigationMode): No description

**Returns:**

void - 

---

#### setRightMotorSpeed

```cpp
void setRightMotorSpeed(float speed)
```

**Parameters:**

- `speed` (float): No description

**Returns:**

void - 

---

#### setState

```cpp
void setState(State newState)
```

**Parameters:**

- `newState` (State): No description

**Returns:**

void - 

---

#### setState

```cpp
void setState(MowerState& newState)
```

**Parameters:**

- `newState` (MowerState&): No description

**Returns:**

void - 

---

#### startBlades

```cpp
void startBlades()
```

**Returns:**

void - 

---

#### startDocking

```cpp
void startDocking()
```

**Returns:**

void - 

---

#### startDriveMotors

```cpp
void startDriveMotors()
```

**Returns:**

void - 

---

#### startMowing

```cpp
void startMowing()
```

**Returns:**

void - 

---

#### startRandomMovement

```cpp
void startRandomMovement()
```

**Returns:**

void - 

---

#### stateToString

```cpp
const char* stateToString(State state)
```

**Parameters:**

- `state` (State): No description

**Returns:**

const char* - 

---

#### stopBlades

```cpp
void stopBlades()
```

**Returns:**

void - 

---

#### stopBuzzer

```cpp
void stopBuzzer()
```

**Returns:**

void - 

---

#### stopDriveMotors

```cpp
void stopDriveMotors()
```

**Returns:**

void - 

---

#### stopMotors

```cpp
void stopMotors()
```

**Returns:**

void - 

---

#### stopRandomMovement

```cpp
void stopRandomMovement()
```

**Returns:**

void - 

---

#### update

```cpp
void update()
```

**Returns:**

void - 

---

#### updateLcdDisplay

```cpp
void updateLcdDisplay(const char* line1, const char* line2 = "")
```

**Parameters:**

- `line1` (const char*): No description
- `""` (const char* line2 =): No description

**Returns:**

void - 

---

#### updateMotors

```cpp
void updateMotors()
```

**Returns:**

void - 

---

#### updateSensors

```cpp
void updateSensors()
```

**Returns:**

void - 

---

---

[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)
