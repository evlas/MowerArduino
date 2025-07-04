#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include <Arduino.h>
#include "../sensors/IMUModule/IMUModule.h"
#include "../sensors/GPSModule/GPSModule.h"
#include "../motors/DriveMotor/DriveMotor.h"

// Kalman filter dimensions
#define KALMAN_STATE_DIM 9  // State vector dimension (x, y, theta, vx, vy, omega, bias_gyro, bias_accel_x, bias_accel_y)
#define KALMAN_MEAS_DIM 9   // Measurement vector dimension (x, y, theta, vx, vy, omega)

// Struttura per rappresentare la posizione del robot
struct RobotPosition {
    float x;        // Posizione X in metri
    float y;        // Posizione Y in metri
    float theta;    // Orientamento in radianti
    float speed;    // Velocità lineare in m/s
    float omega;    // Velocità angolare in rad/s
    bool isValid;   // Indica se la posizione è valida
};

// Kalman filter state structure
struct KalmanState {
    // State variables
    float x;              // X position (m)
    float y;              // Y position (m)
    float theta;          // Orientation (rad)
    float vx;             // Linear velocity X (m/s)
    float vy;             // Linear velocity Y (m/s)
    float omega;          // Angular velocity (rad/s)
    float bias_gyro;      // Gyroscope bias (rad/s)
    float bias_accel_x;   // Accelerometer X bias (m/s²)
    float bias_accel_y;   // Accelerometer Y bias (m/s²)
    
    // Covariance matrices
    float P[KALMAN_STATE_DIM][KALMAN_STATE_DIM];  // State covariance matrix
    float Q[KALMAN_STATE_DIM][KALMAN_STATE_DIM];  // Process noise matrix
    
    // Timestamp and initialization flag
    unsigned long last_update;
    bool initialized;
    
    // Constructor
    KalmanState() : x(0), y(0), theta(0), vx(0), vy(0), omega(0), 
                   bias_gyro(0), bias_accel_x(0), bias_accel_y(0),
                   last_update(0), initialized(false) {
        // Initialize covariance matrices to zero
        for(int i = 0; i < KALMAN_STATE_DIM; i++) {
            for(int j = 0; j < KALMAN_STATE_DIM; j++) {
                P[i][j] = 0.0f;
                Q[i][j] = 0.0f;
            }
        }
    }
};

// Kalman filter parameters
struct KalmanParams {
    // Process noise parameters
    float process_noise_position;          // Position process noise (m^2/s)
    float process_noise_velocity;          // Velocity process noise (m^2/s^3)
    float process_noise_orientation;       // Orientation process noise (rad^2/s)
    float process_noise_angular_velocity;  // Angular velocity process noise (rad^2/s^3)
    float process_noise_gyro_bias;         // Gyro bias process noise (rad^2/s^5)
    float process_noise_accel_bias;        // Accelerometer bias process noise (m^2/s^5)
    
    // Measurement noise parameters
    float measurement_noise_gps;           // GPS position noise (m^2)
    float measurement_noise_imu_yaw;       // IMU yaw noise (rad^2)
    float measurement_noise_odom_vel;      // Odometry velocity noise (m^2/s^2)
    float measurement_noise_imu_gyro;      // IMU gyro noise (rad^2/s^2)
    
    // Constructor with default values
    KalmanParams() : 
        process_noise_position(0.1f),
        process_noise_velocity(0.1f),
        process_noise_orientation(0.01f),
        process_noise_angular_velocity(0.01f),
        process_noise_gyro_bias(0.001f),
        process_noise_accel_bias(0.001f),
        measurement_noise_gps(0.1f),
        measurement_noise_imu_yaw(0.01f),
        measurement_noise_odom_vel(0.1f),
        measurement_noise_imu_gyro(0.01f) {}
};

class PositionManager {
public:
    // Constructor/Destructor
    PositionManager(IMUModule* imu = nullptr, GPSModule* gps = nullptr, 
                  DriveMotor* leftMotor = nullptr, DriveMotor* rightMotor = nullptr,
                  float wheelBase = 0.3f);
    ~PositionManager();
    
    // Initialization
    void begin();
    
    // Main update function
    void update();
    
    // Position management
    RobotPosition getPosition() const;
    void setPosition(const RobotPosition& position);
    void resetPosition();
    
    // Sensor control
    void enableIMU(bool enable) { _isIMUEnabled = enable; }
    void enableGPS(bool enable) { _isGPSEnabled = enable; }
    void enableOdometry(bool enable) { _isOdometryEnabled = enable; }
    
    // Sensor data setters
    void setOdometry(float x, float y, float theta, float speed, float omega);
    void setIMU(float theta, float omega);
    void setGPS(float latitude, float longitude, float speed);
    
    // Kalman filter access
    const KalmanState& getKalmanState() const { return _kalmanState; }
    const KalmanParams& getKalmanParams() const { return _kalmanParams; }
    void setKalmanParams(const KalmanParams& params) { _kalmanParams = params; }
    
    // Home position management
    bool saveHomePosition() {
        if (!_homePosition.isValid) return false;
        
        // Here you would typically save to EEPROM or other persistent storage
        // For example:
        // EEPROM.put(EEPROM_HOME_LAT_ADDR, _homePosition.latitude);
        // EEPROM.put(EEPROM_HOME_LON_ADDR, _homePosition.longitude);
        // EEPROM.put(EEPROM_HOME_VALID_ADDR, true);
        
        return true;
    }
    
    bool hasValidHomePosition() const { return _homePosition.isValid; }
    float getDistanceToHome() const;
    float getBearingToHome() const;
    
    // Coordinate conversion
    void convertGPSToXY(double lat, double lon, float& x, float& y);
    void convertXYToGPS(float x, float y, double& lat, double& lon);
    static void convertLLAToXY(double lat1, double lon1, double lat2, double lon2, float& x, float& y);
    static float calculateBearing(double lat1, double lon1, double lat2, double lon2);
    
    // Robot control
    float getWheelBase() const { return _wheelBase; }
    void moveStraight(float speedPercent, float distanceMeters = 0);
    void turn(float angle, float speed);
    void moveArc(float radius, float angle, float speed);
    
    // Sensor status
    bool isOdometryEnabled() const { return _isOdometryEnabled; }
    bool isIMUEnabled() const { return _isIMUEnabled; }
    bool isGPSEnabled() const { return _isGPSEnabled; }
    
    // GPS interface
    double getGPSLatitude() const { return _gps ? _gps->getLatitude() : 0.0; }
    double getGPSLongitude() const { return _gps ? _gps->getLongitude() : 0.0; }
    float getGPSHDOP() const { return _gps ? _gps->getHDOP() : 99.9f; }
    uint8_t getGPSSatellites() const { return _gps ? _gps->getSatellites() : 0; }
    bool hasGPSFix() const { 
        return _gps && _gps->isValid() && 
               _gps->getHDOP() < 5.0f && 
               _gps->getSatellites() >= 4; 
    }
    
private:
    // Sensor and actuator pointers
    DriveMotor* _leftMotor;
    DriveMotor* _rightMotor;
    IMUModule* _imu;
    GPSModule* _gps;
    
    // Kalman filter state and parameters
    KalmanState _kalmanState;
    KalmanParams _kalmanParams;
    
    // Sensor enable flags
    bool _isIMUEnabled;
    bool _isGPSEnabled;
    bool _isOdometryEnabled;
    
    // Sensor data
    RobotPosition _odometryData;
    
    struct IMUReading {
        float yaw = 0.0f;     // Angolo di imbardata (yaw) in radianti
        float pitch = 0.0f;   // Angolo di beccheggio (pitch) in radianti
        float roll = 0.0f;    // Angolo di rollio (roll) in radianti
        float theta = 0.0f;   // Sinonimo di yaw per compatibilità
        float omega = 0.0f;   // Velocità angolare in rad/s
        float gyroX = 0.0f;
        float gyroY = 0.0f;
        float gyroZ = 0.0f;
        float accelX = 0.0f;
        float accelY = 0.0f;
        float accelZ = 0.0f;
    } _imuData;
    
    struct GPSReading {
        double latitude = 0.0;
        double longitude = 0.0;
        float speed = 0.0f;
        float heading = 0.0f;
        uint8_t satellites = 0;
        bool fix = false;
        bool isValid = false;  // Indicates if the GPS data is valid
    } _gpsData;
    
    // Robot parameters
    float _wheelBase;  // Distance between wheels in meters
    
        // Home position
    struct HomePosition {
        double latitude = 0.0;
        double longitude = 0.0;
        bool isValid = false;
    } _homePosition;
    
    bool _homePositionSet = false;
    
    // Kalman filter functions
    void initializeKalman();
    void predictState();
    void updateState();
    void fuseMeasurements();
    void calculateCovariance();
    void updateProcessNoise(float dt);
    
    // Sensor update functions
    void updateOdometry();
    void updateIMU();
    void updateGPS();
    
    // Motor control
    void moveMotors(float leftSpeed, float rightSpeed);
    void stopMotors();
    
    // Position update
    void updatePosition();
    
    // Home position management
    bool updateHomePosition(double lat, double lon, float hdop = 0, uint8_t satellites = 0) {
        // Basic validation
        if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
            return false;
        }
        
        // Quality checks
        if (satellites < 3) return false;  // Require at least 3 satellites for a valid fix
        if (hdop > 2.0f) return false;    // Require HDOP <= 2.0 for good accuracy
        
        // Update home position
        _homePosition.latitude = lat;
        _homePosition.longitude = lon;
        _homePosition.isValid = true;
        _homePositionSet = true;
        
        // Save to persistent storage
        bool saved = saveHomePosition();
        if (!saved) {
            _homePosition.isValid = false;
            _homePositionSet = false;
            return false;
        }
        
        return true;
    }
    
    // Math utilities
    float normalizeAngle(float angle);
    float angleDifference(float angle1, float angle2);
    
    // Matrix operations for Kalman filter
    static void matrixMultiply(const float A[9][9], const float B[9][9], float C[9][9], 
                             int rowsA, int colsA, int colsB);
    static void matrixTranspose(const float A[9][9], float B[9][9], int rows, int cols);
    static bool matrixInverse3x3(const float A[3][3], float B[3][3]);
    static bool matrixInverse2x2(const float A[2][2], float B[2][2]);
    static void computeInnovationCovariance(const float H[6][9], const float P[9][9], 
                                          const float R[6][6], float S[6][6]);
    static bool computeKalmanGain(const float P[9][9], const float H[6][9], 
                                const float S[6][6], float K[9][6]);
        // No duplicate declarations - all methods and members are already defined above
};

extern PositionManager positionManager;

#endif // POSITION_MANAGER_H
