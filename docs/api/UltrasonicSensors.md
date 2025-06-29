[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)

# UltrasonicSensors Class

## Methods

### Quick Reference

- [void begin()](#begin)
- [void getAllDistances(float& left, float& center, float& right)](#getalldistances)
- [int getDirectionWithMostSpace()](#getdirectionwithmostspace)
- [float getFrontCenterDistance()](#getfrontcenterdistance)
- [float getFrontLeftDistance()](#getfrontleftdistance)
- [float getFrontRightDistance()](#getfrontrightdistance)
- [bool isObstacleDetected(float minDistance = OBSTACLE_CLEARANCE)](#isobstacledetected)
- [private:
    
    float measureDistance(int trigPin, int echoPin)](#measuredistance)

### Method Details

#### begin

```cpp
void begin()
```

**Returns:**

void - 

---

#### getAllDistances

```cpp
void getAllDistances(float& left, float& center, float& right)
```

**Parameters:**

- `left` (float&): No description
- `center` (float&): No description
- `right` (float&): No description

**Returns:**

void - 

---

#### getDirectionWithMostSpace

```cpp
int getDirectionWithMostSpace()
```

**Returns:**

int - 

---

#### getFrontCenterDistance

```cpp
float getFrontCenterDistance()
```

**Returns:**

float - 

---

#### getFrontLeftDistance

```cpp
float getFrontLeftDistance()
```

**Returns:**

float - 

---

#### getFrontRightDistance

```cpp
float getFrontRightDistance()
```

**Returns:**

float - 

---

#### isObstacleDetected

```cpp
bool isObstacleDetected(float minDistance = OBSTACLE_CLEARANCE)
```

**Parameters:**

- `OBSTACLE_CLEARANCE` (float minDistance =): No description

**Returns:**

bool - 

---

#### measureDistance

```cpp
private:
    
    float measureDistance(int trigPin, int echoPin)
```

**Parameters:**

- `trigPin` (int): No description
- `echoPin` (int): No description

**Returns:**

private:
    
    float - 

---

---

[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)
