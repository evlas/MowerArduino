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
- [float getBatteryCurrent()](#getbatterycurrent)
- [float getBatteryLevel()](#getbatterylevel)
- [float getBatteryPercentage()](#getbatterypercentage)
- [float getBatteryVoltage()](#getbatteryvoltage)
- [float getBearingToHome()](#getbearingtohome)
- [MowerState& getChargingState()](#getchargingstate)
- [float getCompassHeading()](#getcompassheading)
- [MowerState* getCurrentState()](#getcurrentstate)
- [float getDistanceToHome()](#getdistancetohome)
- [MowerState& getDockingState()](#getdockingstate)
- [MowerState& getEmergencyStopState()](#getemergencystopstate)
- [MowerState& getErrorState()](#geterrorstate)
- [float getGPSHDOP()](#getgpshdop)
- [double getGPSLatitude()](#getgpslatitude)
- [double getGPSLongitude()](#getgpslongitude)
- [uint8_t getGPSSatellites()](#getgpssatellites)
- [IMUData getIMUData()](#getimudata)
- [MowerState& getIdleState()](#getidlestate)
- [String getLastError()](#getlasterror)
- [float getLatitude()](#getlatitude)
- [MowerState& getLiftedState()](#getliftedstate)
- [float getLongitude()](#getlongitude)
- [MowerState& getMowingState()](#getmowingstate)
- [NavigationMode getNavigationMode()](#getnavigationmode)
- [PositionManager* getPositionManager()](#getpositionmanager)
- [const PositionManager* getPositionManager()](#getpositionmanager)
- [uint8_t getSatellites()](#getsatellites)
- [State getState()](#getstate)
- [uint8_t getUltrasonicDistances(float* distances, uint8_t maxDistances)](#getultrasonicdistances)
- [MowerState& getUndockingState()](#getundockingstate)
- [unsigned long getUptime()](#getuptime)
- [void handleBorder()](#handleborder)
- [void handleEvent(Event event)](#handleevent)
- [void handleObstacle()](#handleobstacle)
- [bool hasGPSFix()](#hasgpsfix)
- [bool hasValidHomePosition()](#hasvalidhomeposition)
- [void init()](#init)
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
- [bool isGPSValid()](#isgpsvalid)
- [bool isLifted()](#islifted)
- [bool isObstacleDetected()](#isobstacledetected)
- [bool isRaining()](#israining)
- [void logError(const String& error)](#logerror)
- [void logStatus()](#logstatus)
- [const char* navigationModeToString(NavigationMode mode)](#navigationmodetostring)
- [void playBuzzerTone(unsigned int frequency, unsigned long duration)](#playbuzzertone)
- [void printDebugInfo()](#printdebuginfo)
- [void printToLcd(const String )](#printtolcd)
- [void printToLcd(int number)](#printtolcd)
- [void resetEmergencyStop()](#resetemergencystop)
- [public:
    
    bool saveHomePosition()](#savehomeposition)
- [void setBladeSpeed(float speed)](#setbladespeed)
- [void setBorderDetected(bool detected)](#setborderdetected)
- [void setCharging(bool charging)](#setcharging)
- [void setCollisionDetected(bool detected)](#setcollisiondetected)
- [void setDocked(bool docked)](#setdocked)
- [void setLCDMenu(LCDMenu& menu)](#setlcdmenu)
- [void setLcdCursor(uint8_t col, uint8_t row)](#setlcdcursor)
- [void setLeftMotorSpeed(float speed)](#setleftmotorspeed)
- [void setLifted(bool lifted)](#setlifted)
- [void setManualControl(float leftSpeed, float rightSpeed)](#setmanualcontrol)
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
- [void updateLcdDisplay(const String , const String &line2 = "")](#updatelcddisplay)
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

#### getBatteryCurrent

```cpp
float getBatteryCurrent()
```

**Returns:**

float - 

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

#### getBatteryVoltage

```cpp
float getBatteryVoltage()
```

**Returns:**

float - 

---

#### getBearingToHome

```cpp
float getBearingToHome()
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

#### getCompassHeading

```cpp
float getCompassHeading()
```

**Returns:**

float - 

---

#### getCurrentState

```cpp
MowerState* getCurrentState()
```

**Returns:**

MowerState* - 

---

#### getDistanceToHome

```cpp
float getDistanceToHome()
```

**Returns:**

float - 

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

#### getGPSHDOP

```cpp
float getGPSHDOP()
```

**Returns:**

float - 

---

#### getGPSLatitude

```cpp
double getGPSLatitude()
```

**Returns:**

double - 

---

#### getGPSLongitude

```cpp
double getGPSLongitude()
```

**Returns:**

double - 

---

#### getGPSSatellites

```cpp
uint8_t getGPSSatellites()
```

**Returns:**

uint8_t - 

---

#### getIMUData

```cpp
IMUData getIMUData()
```

**Returns:**

IMUData - 

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

#### getLatitude

```cpp
float getLatitude()
```

**Returns:**

float - 

---

#### getLiftedState

```cpp
MowerState& getLiftedState()
```

**Returns:**

MowerState& - 

---

#### getLongitude

```cpp
float getLongitude()
```

**Returns:**

float - 

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

#### getPositionManager

```cpp
PositionManager* getPositionManager()
```

**Returns:**

PositionManager* - 

---

#### getPositionManager

```cpp
const PositionManager* getPositionManager()
```

**Returns:**

const PositionManager* - 

---

#### getSatellites

```cpp
uint8_t getSatellites()
```

**Returns:**

uint8_t - 

---

#### getState

```cpp
State getState()
```

**Returns:**

State - 

---

#### getUltrasonicDistances

```cpp
uint8_t getUltrasonicDistances(float* distances, uint8_t maxDistances)
```

**Parameters:**

- `distances` (float*): No description
- `maxDistances` (uint8_t): No description

**Returns:**

uint8_t - 

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

#### hasGPSFix

```cpp
bool hasGPSFix()
```

**Returns:**

bool - 

---

#### hasValidHomePosition

```cpp
bool hasValidHomePosition()
```

**Returns:**

bool - 

---

#### init

```cpp
void init()
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

#### isGPSValid

```cpp
bool isGPSValid()
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
void printToLcd(const String )
```

**Parameters:**

- `` (const String): No description

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

#### resetEmergencyStop

```cpp
void resetEmergencyStop()
```

**Returns:**

void - 

---

#### saveHomePosition

```cpp
public:
    
    bool saveHomePosition()
```

**Returns:**

public:
    
    bool - 

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

#### setLCDMenu

```cpp
void setLCDMenu(LCDMenu& menu)
```

**Parameters:**

- `menu` (LCDMenu&): No description

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

#### setManualControl

```cpp
void setManualControl(float leftSpeed, float rightSpeed)
```

**Parameters:**

- `leftSpeed` (float): No description
- `rightSpeed` (float): No description

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
void updateLcdDisplay(const String , const String &line2 = "")
```

**Parameters:**

- `` (const String): No description
- `""` (const String &line2 =): No description

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
