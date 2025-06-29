[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)

# Maneuver Class

## Methods

### Quick Reference

- [void backward(int duration, float speed)](#backward)
- [void begin()](#begin)
- [void forward(int duration, float speed)](#forward)
- [float getAngularVelocity()](#getangularvelocity)
- [int getLeftSpeed()](#getleftspeed)
- [float getLinearVelocity()](#getlinearvelocity)
- [int getRightSpeed()](#getrightspeed)
- [int getSpeed()](#getspeed)
- [bool isAvoidingObstacle()](#isavoidingobstacle)
- [bool isMoving()](#ismoving)
- [bool isTurning()](#isturning)
- [void moveBy(float deltaX, float deltaY, float speed)](#moveby)
- [void moveStraight(int distance, float speed = 50.0f)](#movestraight)
- [void moveTo(float targetX, float targetY, float speed)](#moveto)
- [float normalizeAngle(float angle)](#normalizeangle)
- [void resumeAfterObstacle()](#resumeafterobstacle)
- [void rotate(int degrees, float speed = 50.0f)](#rotate)
- [void rotateLeft(float speed)](#rotateleft)
- [void rotateRight(float speed)](#rotateright)
- [void setDirection(bool leftForward, bool rightForward)](#setdirection)
- [void setObstacleDetected(bool detected)](#setobstacledetected)
- [void setSpeed(float speed, bool isTrajectoryCorrection = false)](#setspeed)
- [void setSpeed(float leftSpeed, float rightSpeed, bool isTrajectoryCorrection = true)](#setspeed)
- [void spiral(float maxRadius = 5.0f, float speed = 0.5f)](#spiral)
- [void startMowing(float bladeWidth, float speed = 50.0f)](#startmowing)
- [void startSpiral(float speed = 50.0f)](#startspiral)
- [void stop()](#stop)
- [void stopZigzag()](#stopzigzag)
- [void turnLeft(int angle, float speed)](#turnleft)
- [void turnRight(int angle, float speed)](#turnright)
- [void updateAcceleration()](#updateacceleration)
- [private:
    
    bool updateMotorSpeed(float , float target, float step)](#updatemotorspeed)
- [void updateMovement()](#updatemovement)
- [void updateObstacleAvoidance()](#updateobstacleavoidance)
- [void updateZigzag()](#updatezigzag)
- [void zigzag(int distance, int width, float speed = 50.0f)](#zigzag)

### Method Details

#### backward

```cpp
void backward(int duration, float speed)
```

**Parameters:**

- `duration` (int): No description
- `speed` (float): No description

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

#### forward

```cpp
void forward(int duration, float speed)
```

**Parameters:**

- `duration` (int): No description
- `speed` (float): No description

**Returns:**

void - 

---

#### getAngularVelocity

```cpp
float getAngularVelocity()
```

**Returns:**

float - 

---

#### getLeftSpeed

```cpp
int getLeftSpeed()
```

**Returns:**

int - 

---

#### getLinearVelocity

```cpp
float getLinearVelocity()
```

**Returns:**

float - 

---

#### getRightSpeed

```cpp
int getRightSpeed()
```

**Returns:**

int - 

---

#### getSpeed

```cpp
int getSpeed()
```

**Returns:**

int - 

---

#### isAvoidingObstacle

```cpp
bool isAvoidingObstacle()
```

**Returns:**

bool - 

---

#### isMoving

```cpp
bool isMoving()
```

**Returns:**

bool - 

---

#### isTurning

```cpp
bool isTurning()
```

**Returns:**

bool - 

---

#### moveBy

```cpp
void moveBy(float deltaX, float deltaY, float speed)
```

**Parameters:**

- `deltaX` (float): No description
- `deltaY` (float): No description
- `speed` (float): No description

**Returns:**

void - 

---

#### moveStraight

```cpp
void moveStraight(int distance, float speed = 50.0f)
```

**Parameters:**

- `distance` (int): No description
- `50.0f` (float speed =): No description

**Returns:**

void - 

---

#### moveTo

```cpp
void moveTo(float targetX, float targetY, float speed)
```

**Parameters:**

- `targetX` (float): No description
- `targetY` (float): No description
- `speed` (float): No description

**Returns:**

void - 

---

#### normalizeAngle

```cpp
float normalizeAngle(float angle)
```

**Parameters:**

- `angle` (float): No description

**Returns:**

float - 

---

#### resumeAfterObstacle

```cpp
void resumeAfterObstacle()
```

**Returns:**

void - 

---

#### rotate

```cpp
void rotate(int degrees, float speed = 50.0f)
```

**Parameters:**

- `degrees` (int): No description
- `50.0f` (float speed =): No description

**Returns:**

void - 

---

#### rotateLeft

```cpp
void rotateLeft(float speed)
```

**Parameters:**

- `speed` (float): No description

**Returns:**

void - 

---

#### rotateRight

```cpp
void rotateRight(float speed)
```

**Parameters:**

- `speed` (float): No description

**Returns:**

void - 

---

#### setDirection

```cpp
void setDirection(bool leftForward, bool rightForward)
```

**Parameters:**

- `leftForward` (bool): No description
- `rightForward` (bool): No description

**Returns:**

void - 

---

#### setObstacleDetected

```cpp
void setObstacleDetected(bool detected)
```

**Parameters:**

- `detected` (bool): No description

**Returns:**

void - 

---

#### setSpeed

```cpp
void setSpeed(float speed, bool isTrajectoryCorrection = false)
```

**Parameters:**

- `speed` (float): No description
- `false` (bool isTrajectoryCorrection =): No description

**Returns:**

void - 

---

#### setSpeed

```cpp
void setSpeed(float leftSpeed, float rightSpeed, bool isTrajectoryCorrection = true)
```

**Parameters:**

- `leftSpeed` (float): No description
- `rightSpeed` (float): No description
- `true` (bool isTrajectoryCorrection =): No description

**Returns:**

void - 

---

#### spiral

```cpp
void spiral(float maxRadius = 5.0f, float speed = 0.5f)
```

**Parameters:**

- `5.0f` (float maxRadius =): No description
- `0.5f` (float speed =): No description

**Returns:**

void - 

---

#### startMowing

```cpp
void startMowing(float bladeWidth, float speed = 50.0f)
```

**Parameters:**

- `bladeWidth` (float): No description
- `50.0f` (float speed =): No description

**Returns:**

void - 

---

#### startSpiral

```cpp
void startSpiral(float speed = 50.0f)
```

**Parameters:**

- `50.0f` (float speed =): No description

**Returns:**

void - 

---

#### stop

```cpp
void stop()
```

**Returns:**

void - 

---

#### stopZigzag

```cpp
void stopZigzag()
```

**Returns:**

void - 

---

#### turnLeft

```cpp
void turnLeft(int angle, float speed)
```

**Parameters:**

- `angle` (int): No description
- `speed` (float): No description

**Returns:**

void - 

---

#### turnRight

```cpp
void turnRight(int angle, float speed)
```

**Parameters:**

- `angle` (int): No description
- `speed` (float): No description

**Returns:**

void - 

---

#### updateAcceleration

```cpp
void updateAcceleration()
```

**Returns:**

void - 

---

#### updateMotorSpeed

```cpp
private:
    
    bool updateMotorSpeed(float , float target, float step)
```

**Parameters:**

- `` (float): No description
- `target` (float): No description
- `step` (float): No description

**Returns:**

private:
    
    bool - 

---

#### updateMovement

```cpp
void updateMovement()
```

**Returns:**

void - 

---

#### updateObstacleAvoidance

```cpp
void updateObstacleAvoidance()
```

**Returns:**

void - 

---

#### updateZigzag

```cpp
void updateZigzag()
```

**Returns:**

void - 

---

#### zigzag

```cpp
void zigzag(int distance, int width, float speed = 50.0f)
```

**Parameters:**

- `distance` (int): No description
- `width` (int): No description
- `50.0f` (float speed =): No description

**Returns:**

void - 

---

---

[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)
