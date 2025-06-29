[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)

# PositionManager Class

## Methods

### Quick Reference

- [bool begin()](#begin)
- [void calculateCovariance()](#calculatecovariance)
- [void enableGPS(bool enable)](#enablegps)
- [void enableIMU(bool enable)](#enableimu)
- [void enableOdometry(bool enable)](#enableodometry)
- [void fuseMeasurements()](#fusemeasurements)
- [RobotPosition getPosition()](#getposition)
- [float getWheelBase()](#getwheelbase)
- [void initializeKalman()](#initializekalman)
- [bool isGPSEnabled()](#isgpsenabled)
- [bool isIMUEnabled()](#isimuenabled)
- [bool isOdometryEnabled()](#isodometryenabled)
- [void moveArc(float radius, float angle, float speed)](#movearc)
- [void moveStraight(float distance, float speed)](#movestraight)
- [void predictState()](#predictstate)
- [void setGPS(float latitude, float longitude, float speed)](#setgps)
- [void setIMU(float theta, float omega)](#setimu)
- [void setOdometry(float x, float y, float theta, float speed, float omega)](#setodometry)
- [void turn(float angle, float speed)](#turn)
- [void update()](#update)
- [void updateState()](#updatestate)

### Method Details

#### begin

```cpp
bool begin()
```

**Returns:**

bool - 

---

#### calculateCovariance

```cpp
void calculateCovariance()
```

**Returns:**

void - 

---

#### enableGPS

```cpp
void enableGPS(bool enable)
```

Parametri del filtro di Kalman

**Parameters:**

- `enable` (bool): No description

**Returns:**

void - 

---

#### enableIMU

```cpp
void enableIMU(bool enable)
```

**Parameters:**

- `enable` (bool): No description

**Returns:**

void - 

---

#### enableOdometry

```cpp
void enableOdometry(bool enable)
```

**Parameters:**

- `enable` (bool): No description

**Returns:**

void - 

---

#### fuseMeasurements

```cpp
void fuseMeasurements()
```

**Returns:**

void - 

---

#### getPosition

```cpp
RobotPosition getPosition()
```

**Returns:**

RobotPosition - 

---

#### getWheelBase

```cpp
float getWheelBase()
```

**Returns:**

float - 

---

#### initializeKalman

```cpp
void initializeKalman()
```

**Returns:**

void - 

---

#### isGPSEnabled

```cpp
bool isGPSEnabled()
```

**Returns:**

bool - 

---

#### isIMUEnabled

```cpp
bool isIMUEnabled()
```

**Returns:**

bool - 

---

#### isOdometryEnabled

```cpp
bool isOdometryEnabled()
```

**Returns:**

bool - 

---

#### moveArc

```cpp
void moveArc(float radius, float angle, float speed)
```

**Parameters:**

- `radius` (float): No description
- `angle` (float): No description
- `speed` (float): No description

**Returns:**

void - 

---

#### moveStraight

```cpp
void moveStraight(float distance, float speed)
```

**Parameters:**

- `distance` (float): No description
- `speed` (float): No description

**Returns:**

void - 

---

#### predictState

```cpp
void predictState()
```

**Returns:**

void - 

---

#### setGPS

```cpp
void setGPS(float latitude, float longitude, float speed)
```

**Parameters:**

- `latitude` (float): No description
- `longitude` (float): No description
- `speed` (float): No description

**Returns:**

void - 

---

#### setIMU

```cpp
void setIMU(float theta, float omega)
```

**Parameters:**

- `theta` (float): No description
- `omega` (float): No description

**Returns:**

void - 

---

#### setOdometry

```cpp
void setOdometry(float x, float y, float theta, float speed, float omega)
```

**Parameters:**

- `x` (float): No description
- `y` (float): No description
- `theta` (float): No description
- `speed` (float): No description
- `omega` (float): No description

**Returns:**

void - 

---

#### turn

```cpp
void turn(float angle, float speed)
```

**Parameters:**

- `angle` (float): No description
- `speed` (float): No description

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

#### updateState

```cpp
void updateState()
```

**Returns:**

void - 

---

---

[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)
