[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)

# PositionManager Class

## Methods

### Quick Reference

- [float angleDifference(float angle1, float angle2)](#angledifference)
- [void begin()](#begin)
- [static float calculateBearing(double lat1, double lon1, double lat2, double lon2)](#calculatebearing)
- [void calculateCovariance()](#calculatecovariance)
- [static void computeInnovationCovariance(const float H[6][9], const float P[9][9], const float R[6][6], float S[6][6])](#computeinnovationcovariance)
- [static bool computeKalmanGain(const float P[9][9], const float H[6][9], const float S[6][6], float K[9][6])](#computekalmangain)
- [void convertGPSToXY(double lat, double lon, float& x, float& y)](#convertgpstoxy)
- [static void convertLLAToXY(double lat1, double lon1, double lat2, double lon2, float& x, float& y)](#convertllatoxy)
- [void convertXYToGPS(float x, float y, double& lat, double& lon)](#convertxytogps)
- [void enableGPS(bool enable)](#enablegps)
- [void enableIMU(bool enable)](#enableimu)
- [void enableOdometry(bool enable)](#enableodometry)
- [void fuseMeasurements()](#fusemeasurements)
- [float getBearingToHome()](#getbearingtohome)
- [float getDistanceToHome()](#getdistancetohome)
- [float getGPSHDOP()](#getgpshdop)
- [double getGPSLatitude()](#getgpslatitude)
- [double getGPSLongitude()](#getgpslongitude)
- [uint8_t getGPSSatellites()](#getgpssatellites)
- [const KalmanParams& getKalmanParams()](#getkalmanparams)
- [const KalmanState& getKalmanState()](#getkalmanstate)
- [RobotPosition getPosition()](#getposition)
- [float getWheelBase()](#getwheelbase)
- [bool hasGPSFix()](#hasgpsfix)
- [bool hasValidHomePosition()](#hasvalidhomeposition)
- [void initializeKalman()](#initializekalman)
- [bool isGPSEnabled()](#isgpsenabled)
- [bool isIMUEnabled()](#isimuenabled)
- [bool isOdometryEnabled()](#isodometryenabled)
- [static bool matrixInverse2x2(const float A[2][2], float B[2][2])](#matrixinverse2x2)
- [static bool matrixInverse3x3(const float A[3][3], float B[3][3])](#matrixinverse3x3)
- [static void matrixMultiply(const float A[9][9], const float B[9][9], float C[9][9], int rowsA, int colsA, int colsB)](#matrixmultiply)
- [static void matrixTranspose(const float A[9][9], float B[9][9], int rows, int cols)](#matrixtranspose)
- [void moveArc(float radius, float angle, float speed)](#movearc)
- [void moveMotors(float leftSpeed, float rightSpeed)](#movemotors)
- [void moveStraight(float speedPercent, float distanceMeters = 0)](#movestraight)
- [float normalizeAngle(float angle)](#normalizeangle)
- [void predictState()](#predictstate)
- [void resetPosition()](#resetposition)
- [bool saveHomePosition()](#savehomeposition)
- [void setGPS(float latitude, float longitude, float speed)](#setgps)
- [void setIMU(float theta, float omega)](#setimu)
- [void setKalmanParams(const KalmanParams& params)](#setkalmanparams)
- [void setOdometry(float x, float y, float theta, float speed, float omega)](#setodometry)
- [void setPosition(const RobotPosition& position)](#setposition)
- [void stopMotors()](#stopmotors)
- [void turn(float angle, float speed)](#turn)
- [void update()](#update)
- [void updateGPS()](#updategps)
- [bool updateHomePosition(double lat, double lon, float hdop = 0, uint8_t satellites = 0)](#updatehomeposition)
- [void updateIMU()](#updateimu)
- [void updateOdometry()](#updateodometry)
- [void updatePosition()](#updateposition)
- [void updateProcessNoise(float dt)](#updateprocessnoise)
- [void updateState()](#updatestate)

### Method Details

#### angleDifference

```cpp
float angleDifference(float angle1, float angle2)
```

**Parameters:**

- `angle1` (float): No description
- `angle2` (float): No description

**Returns:**

float - 

---

#### begin

```cpp
void begin()
```

Kalman filter dimensions

**Returns:**

void - 

---

#### calculateBearing

```cpp
static float calculateBearing(double lat1, double lon1, double lat2, double lon2)
```

**Parameters:**

- `lat1` (double): No description
- `lon1` (double): No description
- `lat2` (double): No description
- `lon2` (double): No description

**Returns:**

static float - 

---

#### calculateCovariance

```cpp
void calculateCovariance()
```

**Returns:**

void - 

---

#### computeInnovationCovariance

```cpp
static void computeInnovationCovariance(const float H[6][9], const float P[9][9], const float R[6][6], float S[6][6])
```

**Parameters:**

- `H[6][9]` (const float): No description
- `P[9][9]` (const float): No description
- `R[6][6]` (const float): No description
- `S[6][6]` (float): No description

**Returns:**

static void - 

---

#### computeKalmanGain

```cpp
static bool computeKalmanGain(const float P[9][9], const float H[6][9], const float S[6][6], float K[9][6])
```

**Parameters:**

- `P[9][9]` (const float): No description
- `H[6][9]` (const float): No description
- `S[6][6]` (const float): No description
- `K[9][6]` (float): No description

**Returns:**

static bool - 

---

#### convertGPSToXY

```cpp
void convertGPSToXY(double lat, double lon, float& x, float& y)
```

**Parameters:**

- `lat` (double): No description
- `lon` (double): No description
- `x` (float&): No description
- `y` (float&): No description

**Returns:**

void - 

---

#### convertLLAToXY

```cpp
static void convertLLAToXY(double lat1, double lon1, double lat2, double lon2, float& x, float& y)
```

**Parameters:**

- `lat1` (double): No description
- `lon1` (double): No description
- `lat2` (double): No description
- `lon2` (double): No description
- `x` (float&): No description
- `y` (float&): No description

**Returns:**

static void - 

---

#### convertXYToGPS

```cpp
void convertXYToGPS(float x, float y, double& lat, double& lon)
```

**Parameters:**

- `x` (float): No description
- `y` (float): No description
- `lat` (double&): No description
- `lon` (double&): No description

**Returns:**

void - 

---

#### enableGPS

```cpp
void enableGPS(bool enable)
```

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

#### getBearingToHome

```cpp
float getBearingToHome()
```

**Returns:**

float - 

---

#### getDistanceToHome

```cpp
float getDistanceToHome()
```

**Returns:**

float - 

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

#### getKalmanParams

```cpp
const KalmanParams& getKalmanParams()
```

**Returns:**

const KalmanParams& - 

---

#### getKalmanState

```cpp
const KalmanState& getKalmanState()
```

**Returns:**

const KalmanState& - 

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

#### matrixInverse2x2

```cpp
static bool matrixInverse2x2(const float A[2][2], float B[2][2])
```

**Parameters:**

- `A[2][2]` (const float): No description
- `B[2][2]` (float): No description

**Returns:**

static bool - 

---

#### matrixInverse3x3

```cpp
static bool matrixInverse3x3(const float A[3][3], float B[3][3])
```

**Parameters:**

- `A[3][3]` (const float): No description
- `B[3][3]` (float): No description

**Returns:**

static bool - 

---

#### matrixMultiply

```cpp
static void matrixMultiply(const float A[9][9], const float B[9][9], float C[9][9], int rowsA, int colsA, int colsB)
```

**Parameters:**

- `A[9][9]` (const float): No description
- `B[9][9]` (const float): No description
- `C[9][9]` (float): No description
- `rowsA` (int): No description
- `colsA` (int): No description
- `colsB` (int): No description

**Returns:**

static void - 

---

#### matrixTranspose

```cpp
static void matrixTranspose(const float A[9][9], float B[9][9], int rows, int cols)
```

**Parameters:**

- `A[9][9]` (const float): No description
- `B[9][9]` (float): No description
- `rows` (int): No description
- `cols` (int): No description

**Returns:**

static void - 

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

#### moveMotors

```cpp
void moveMotors(float leftSpeed, float rightSpeed)
```

**Parameters:**

- `leftSpeed` (float): No description
- `rightSpeed` (float): No description

**Returns:**

void - 

---

#### moveStraight

```cpp
void moveStraight(float speedPercent, float distanceMeters = 0)
```

**Parameters:**

- `speedPercent` (float): No description
- `0` (float distanceMeters =): No description

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

#### predictState

```cpp
void predictState()
```

**Returns:**

void - 

---

#### resetPosition

```cpp
void resetPosition()
```

**Returns:**

void - 

---

#### saveHomePosition

```cpp
bool saveHomePosition()
```

**Returns:**

bool - 

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

#### setKalmanParams

```cpp
void setKalmanParams(const KalmanParams& params)
```

**Parameters:**

- `params` (const KalmanParams&): No description

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

#### setPosition

```cpp
void setPosition(const RobotPosition& position)
```

**Parameters:**

- `position` (const RobotPosition&): No description

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

#### updateGPS

```cpp
void updateGPS()
```

**Returns:**

void - 

---

#### updateHomePosition

```cpp
bool updateHomePosition(double lat, double lon, float hdop = 0, uint8_t satellites = 0)
```

**Parameters:**

- `lat` (double): No description
- `lon` (double): No description
- `0` (float hdop =): No description
- `0` (uint8_t satellites =): No description

**Returns:**

bool - 

---

#### updateIMU

```cpp
void updateIMU()
```

**Returns:**

void - 

---

#### updateOdometry

```cpp
void updateOdometry()
```

**Returns:**

void - 

---

#### updatePosition

```cpp
void updatePosition()
```

**Returns:**

void - 

---

#### updateProcessNoise

```cpp
void updateProcessNoise(float dt)
```

**Parameters:**

- `dt` (float): No description

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
