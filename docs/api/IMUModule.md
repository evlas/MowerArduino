[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)

# IMUModule Class

## Methods

### Quick Reference

- [return atan2()](#atan2)
- [void begin()](#begin)
- [void calibrate(uint16_t samples = 1000)](#calibrate)
- [float getAngularVelocity()](#getangularvelocity)
- [IMUData getData()](#getdata)
- [void getROSAngularVelocity(ROS_Vector3 )](#getrosangularvelocity)
- [void getROSEuler(ROS_Vector3 )](#getroseuler)
- [ROS_ImuMsg getROSImuMsg()](#getrosimumsg)
- [void getROSLinearAcceleration(ROS_Vector3 )](#getroslinearacceleration)
- [void getROSQuaternion(float q[4])](#getrosquaternion)
- [float getTheta()](#gettheta)
- [bool isCalibrated()](#iscalibrated)
- [bool isValid()](#isvalid)
- [void printCalibration()](#printcalibration)
- [void update()](#update)
- [private:
        
        void updateOrientation()](#updateorientation)

### Method Details

#### atan2

```cpp
return atan2()
```

**Returns:**

return - 

---

#### begin

```cpp
void begin()
```

**Returns:**

void - 

---

#### calibrate

```cpp
void calibrate(uint16_t samples = 1000)
```

**Parameters:**

- `1000` (uint16_t samples =): No description

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

#### getData

```cpp
IMUData getData()
```

**Returns:**

IMUData - 

---

#### getROSAngularVelocity

```cpp
void getROSAngularVelocity(ROS_Vector3 )
```

**Parameters:**

- `` (ROS_Vector3): No description

**Returns:**

void - 

---

#### getROSEuler

```cpp
void getROSEuler(ROS_Vector3 )
```

**Parameters:**

- `` (ROS_Vector3): No description

**Returns:**

void - 

---

#### getROSImuMsg

```cpp
ROS_ImuMsg getROSImuMsg()
```

**Returns:**

ROS_ImuMsg - 

---

#### getROSLinearAcceleration

```cpp
void getROSLinearAcceleration(ROS_Vector3 )
```

**Parameters:**

- `` (ROS_Vector3): No description

**Returns:**

void - 

---

#### getROSQuaternion

```cpp
void getROSQuaternion(float q[4])
```

**Parameters:**

- `q[4]` (float): No description

**Returns:**

void - 

---

#### getTheta

```cpp
float getTheta()
```

**Returns:**

float - 

---

#### isCalibrated

```cpp
bool isCalibrated()
```

**Returns:**

bool - 

---

#### isValid

```cpp
bool isValid()
```

**Returns:**

bool - 

---

#### printCalibration

```cpp
void printCalibration()
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

#### updateOrientation

```cpp
private:
        
        void updateOrientation()
```

**Returns:**

private:
        
        void - 

---

---

[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)
