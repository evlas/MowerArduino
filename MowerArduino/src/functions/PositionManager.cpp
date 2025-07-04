#include "PositionManager.h"
#include "../sensors/IMUModule/IMUModule.h"
#include "../sensors/GPSModule/GPSModule.h"
#include "../motors/DriveMotor/DriveMotor.h"
#include "../config.h"
#include <math.h>

#define STATE_DIM 6

// Helper functions for matrix operations
void matrixMultiply(const float A[][STATE_DIM], const float B[][STATE_DIM], float C[][STATE_DIM], int m, int n, int p) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            C[i][j] = 0;
            for (int k = 0; k < n; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void matrixTranspose(const float A[][STATE_DIM], float A_T[][STATE_DIM], int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            A_T[j][i] = A[i][j];
        }
    }
}

void matrixInverse3x3(const float A[][3], float A_inv[][3]) {
    // Calcola il determinante
    float det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    
    if (det == 0) {
        // Matrice singolare, restituisci matrice identità
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                A_inv[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        return;
    }
    
    // Calcola la matrice dei cofattori
    A_inv[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / det;
    A_inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det;
    A_inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;
    A_inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det;
    A_inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
    A_inv[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) / det;
    A_inv[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / det;
    A_inv[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) / det;
    A_inv[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / det;
}

// Indici per lo stato del filtro di Kalman
enum StateIndex {
    X = 0,      // Posizione x (m)
    Y = 1,      // Posizione y (m)
    THETA = 2,  // Orientamento (rad)
    V_X = 3,    // Velocità lineare x (m/s)
    V_Y = 4,    // Velocità lineare y (m/s)
    OMEGA = 5,  // Velocità angolare (rad/s)
    B_GYRO = 6, // Bias giroscopio (rad/s)
    B_ACC_X = 7,// Bias accelerometro x (m/s²)
    B_ACC_Y = 8 // Bias accelerometro y (m/s²)
};

// Indici per le misurazioni
enum MeasurementIndex {
    M_X = 0,
    M_Y = 1,
    M_THETA = 2,
    M_V_X = 3,
    M_V_Y = 4,
    M_OMEGA = 5
};

// Costanti per la conversione GPS
#define EARTH_RADIUS_CM 637100000.0f  // Raggio terrestre in cm
// DEG_TO_RAD è già definito in Arduino.h

// Funzione per normalizzare un angolo in [-pi, pi]
float normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// Funzione per calcolare la differenza angolare
float angleDifference(float angle1, float angle2) {
    float diff = normalizeAngle(angle1 - angle2);
    if (diff > M_PI) diff -= 2.0f * M_PI;
    if (diff < -M_PI) diff += 2.0f * M_PI;
    return diff;
}

// Funzione per convertire coordinate GPS in coordinate cartesiane
void convertGPSToXY(float latitude, float longitude, float& x, float& y) {
    static float firstLat = 0.0f;
    static float firstLon = 0.0f;
    static bool isFirstCall = true;

    if (isFirstCall) {
        firstLat = latitude;
        firstLon = longitude;
        isFirstCall = false;
    }

    // Calcolo la differenza in longitudine e latitudine
    float deltaLat = (latitude - firstLat) * DEG_TO_RAD;
    float deltaLon = (longitude - firstLon) * DEG_TO_RAD;

    // Conversione in coordinate cartesiane
    float lat1 = firstLat * DEG_TO_RAD;
    x = EARTH_RADIUS_CM * deltaLon * cos(lat1);
    y = EARTH_RADIUS_CM * deltaLat;
}

// Funzioni ausiliarie per la matematica
void matrixMultiply(float A[6][6], float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 6; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void matrixVectorMultiply(float A[6][6], float x[6], float y[6]) {
    for (int i = 0; i < 6; i++) {
        y[i] = 0.0f;
        for (int j = 0; j < 6; j++) {
            y[i] += A[i][j] * x[j];
        }
    }
}

void matrixAdd(float A[6][6], float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void matrixSubtract(float A[6][6], float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

void matrixTranspose(float A[6][6], float B[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            B[i][j] = A[j][i];
        }
    }
}

void matrixCopy(float A[6][6], float B[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            B[i][j] = A[i][j];
        }
    }
}

void vectorCopy(float A[6], float B[6]) {
    for (int i = 0; i < 6; i++) {
        B[i] = A[i];
    }
}

PositionManager::PositionManager(IMUModule* imu, GPSModule* gps, DriveMotor* leftMotor, DriveMotor* rightMotor, float wheelBase) :
    _leftMotor(leftMotor),
    _rightMotor(rightMotor),
    _imu(imu),
    _gps(gps),
    _wheelBase(wheelBase),
    _isIMUEnabled(imu != nullptr),
    _isGPSEnabled(gps != nullptr),
    _isOdometryEnabled(leftMotor != nullptr && rightMotor != nullptr) {
    
    // Initialize data structures
    memset(&_odometryData, 0, sizeof(_odometryData));
    memset(&_imuData, 0, sizeof(_imuData));
    memset(&_gpsData, 0, sizeof(_gpsData));
    
    // Initialize GPS data
    _gpsData.latitude = 0.0f;
    _gpsData.longitude = 0.0f;
    _gpsData.speed = 0.0f;
    _gpsData.isValid = false;
    
    // Kalman filter will be initialized on first sensor update
}

PositionManager::~PositionManager() {
    // Niente da fare, i sensori vengono gestiti altrove
}

void PositionManager::begin() {
    // Initialize enabled sensors
    if (_isIMUEnabled && _imu) {
        _imu->begin();
    }
    
    if (_isGPSEnabled && _gps) {
        _gps->begin();
    }
    
    // Initialize the Kalman filter
    initializeKalman();
}

void PositionManager::initializeKalman() {
    // Process noise parameters
    _kalmanParams.process_noise_position = 0.1f;          // (m²/s)
    _kalmanParams.process_noise_velocity = 0.1f;          // (m²/s³)
    _kalmanParams.process_noise_orientation = 0.01f;      // (rad²/s)
    _kalmanParams.process_noise_angular_velocity = 0.01f; // (rad²/s³)
    _kalmanParams.process_noise_gyro_bias = 0.001f;       // (rad²/s⁵)
    _kalmanParams.process_noise_accel_bias = 0.001f;      // (m²/s⁵)
    
    // Measurement noise parameters
    _kalmanParams.measurement_noise_gps = 1.0f;           // m²
    _kalmanParams.measurement_noise_imu_yaw = 0.01f;       // rad²
    _kalmanParams.measurement_noise_odom_vel = 0.04f;      // m²/s²
    _kalmanParams.measurement_noise_imu_gyro = 0.04f;      // rad²/s²
    
    // Inizializza lo stato
    memset(&_kalmanState, 0, sizeof(_kalmanState));
    _kalmanState.x = 0.0f;
    _kalmanState.y = 0.0f;
    _kalmanState.theta = 0.0f;
    _kalmanState.vx = 0.0f;
    _kalmanState.vy = 0.0f;
    _kalmanState.omega = 0.0f;
    _kalmanState.bias_gyro = 0.0f;
    _kalmanState.bias_accel_x = 0.0f;
    _kalmanState.bias_accel_y = 0.0f;
    
    // Inizializza la matrice di covarianza
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            _kalmanState.P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Imposta le incertezze iniziali (varianze)
    _kalmanState.P[0][0] = 0.1f;  // x (m²)
    _kalmanState.P[1][1] = 0.1f;  // y (m²)
    _kalmanState.P[2][2] = 0.1f;  // theta (rad²)
    _kalmanState.P[3][3] = 0.1f;  // vx (m/s)²
    _kalmanState.P[4][4] = 0.1f;  // vy (m/s)²
    _kalmanState.P[5][5] = 0.1f;  // omega (rad/s)²
    _kalmanState.P[6][6] = 0.01f; // bias_gyro (rad/s)²
    _kalmanState.P[7][7] = 0.01f; // bias_accel_x (m/s²)²
    _kalmanState.P[8][8] = 0.01f; // bias_accel_y (m/s²)²
    
    // Inizializza la matrice di rumore di processo
    updateProcessNoise(0.01f);  // dt=0.01s per inizializzazione
    
    _kalmanState.last_update = millis();
    _kalmanState.initialized = true;
}

void PositionManager::updateProcessNoise(float dt) {
    // Calculate process noise terms
    float pos_noise = _kalmanParams.process_noise_position * dt;  // m²
    float vel_noise = _kalmanParams.process_noise_velocity * dt;  // (m/s)²
    float theta_noise = _kalmanParams.process_noise_orientation * dt;  // rad²
    float omega_noise = _kalmanParams.process_noise_angular_velocity * dt;  // (rad/s)²
    float bias_gyro_noise = _kalmanParams.process_noise_gyro_bias * dt;  // (rad/s)²
    float bias_accel_noise = _kalmanParams.process_noise_accel_bias * dt;  // (m/s²)²
    
    // Zero out the Q matrix
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            _kalmanState.Q[i][j] = 0.0f;
        }
    }
    
    // Set diagonal elements (independent noise for each state)
    _kalmanState.Q[0][0] = pos_noise;  // x position
    _kalmanState.Q[1][1] = pos_noise;  // y position
    _kalmanState.Q[2][2] = theta_noise;  // theta (orientation)
    _kalmanState.Q[3][3] = vel_noise;  // vx (velocity x)
    _kalmanState.Q[4][4] = vel_noise;  // vy (velocity y)
    _kalmanState.Q[5][5] = omega_noise;  // omega (angular velocity)
    _kalmanState.Q[6][6] = bias_gyro_noise;  // gyro bias
    _kalmanState.Q[7][7] = bias_accel_noise;  // x-accel bias
    _kalmanState.Q[8][8] = bias_accel_noise;  // y-accel bias
    
    // Add cross-covariance terms between position and velocity
    // This models the correlation between position and velocity
    float pos_vel_cov = 0.5f * sqrtf(pos_noise * vel_noise);
    _kalmanState.Q[0][3] = pos_vel_cov;  // cov(x, vx)
    _kalmanState.Q[3][0] = pos_vel_cov;
    _kalmanState.Q[1][4] = pos_vel_cov;  // cov(y, vy)
    _kalmanState.Q[4][1] = pos_vel_cov;
    
    // Covariance between angle and angular velocity
    float theta_omega_cov = 0.5f * sqrtf(theta_noise * omega_noise);
    _kalmanState.Q[2][5] = theta_omega_cov;  // cov(theta, omega)
    _kalmanState.Q[5][2] = theta_omega_cov;
    
    // Covariance between velocity and acceleration bias
    float vel_bias_cov = 0.1f * sqrtf(vel_noise * bias_accel_noise);
    _kalmanState.Q[3][7] = vel_bias_cov;  // cov(vx, bias_accel_x)
    _kalmanState.Q[7][3] = vel_bias_cov;
    _kalmanState.Q[4][8] = vel_bias_cov;  // cov(vy, bias_accel_y)
    _kalmanState.Q[8][4] = vel_bias_cov;
    
    // Covariance between angular velocity and gyro bias
    float omega_bias_cov = 0.1f * sqrtf(omega_noise * bias_gyro_noise);
    _kalmanState.Q[5][6] = omega_bias_cov;  // cov(omega, bias_gyro)
    _kalmanState.Q[6][5] = omega_bias_cov;
}

void PositionManager::update() {
    // Ottieni i dati dai sensori abilitati
    float x, y;
    float speed, omega;
    bool hasGPS = false;
    float gpsX = 0.0f, gpsY = 0.0f;

    // Ottieni dati dall'odometria (sempre disponibile)
    float leftX = _leftMotor->getX();
    float leftY = _leftMotor->getY();
    float rightX = _rightMotor->getX();
    float rightY = _rightMotor->getY();
    
    // Calcola la posizione media
    x = (leftX + rightX) / 2.0f;
    y = (leftY + rightY) / 2.0f;
    
    // Calcola la velocità lineare e angolare
    float leftSpeed = _leftMotor->getLinearVelocity();
    float rightSpeed = _rightMotor->getLinearVelocity();
    speed = (leftSpeed + rightSpeed) / 2.0f;
    omega = (rightSpeed - leftSpeed) / _wheelBase;

    // Ottieni dati dall'IMU (sempre disponibile)
    float imuTheta = _imu->getTheta();
    omega = _imu->getAngularVelocity();  // Usa l'IMU come fonte principale per omega

    // Ottieni dati dal GPS
    if (_isGPSEnabled && _gps != nullptr) {
        GPSData gpsData = _gps->getData();
        if (gpsData.fix) {  // Usa fix invece di isValid
            // Converte coordinate GPS in coordinate cartesiane
            convertGPSToXY(gpsData.latitude, gpsData.longitude, gpsX, gpsY);
            hasGPS = true;
        }
    }

    // Usa l'angolo dall'IMU come riferimento principale
    float theta = imuTheta;

    // Se abbiamo GPS, calcola la correzione della posizione
    if (_isGPSEnabled && _gps != nullptr) {
        // Usa un peso per la correzione GPS
        float gpsWeight = 0.1f;  // Pesare il GPS meno dell'odometria
        x = (1.0f - gpsWeight) * x + gpsWeight * gpsX;
        y = (1.0f - gpsWeight) * y + gpsWeight * gpsY;
    }

    // Normalizza l'angolo
    theta = normalizeAngle(theta);

    // Update sensor data
    updateOdometry();
    updateIMU();
    updateGPS();
    
    // Update Kalman filter
    predictState();
    updateState();
    
    // The position is now obtained directly from the Kalman state when getPosition() is called
    // This ensures we always get the most up-to-date position from the Kalman filter
}

void PositionManager::predictState() {
    // Calculate time since last update
    unsigned long current_time = millis();
    float dt = (current_time - _kalmanState.last_update) / 1000.0f;  // in seconds
    _kalmanState.last_update = current_time;
    
    if (dt <= 0 || dt > 1.0f) {
        // If time has not advanced or too much time has passed, don't predict
        return;
    }
    
    // Update process noise matrix based on elapsed time
    updateProcessNoise(dt);
    
    // State prediction
    float cos_theta = cosf(_kalmanState.theta);
    float sin_theta = sinf(_kalmanState.theta);
    
    // Motion model: constant velocity model with heading
    float x_pred = _kalmanState.x + _kalmanState.vx * cos_theta * dt - _kalmanState.vy * sin_theta * dt;
    float y_pred = _kalmanState.y + _kalmanState.vx * sin_theta * dt + _kalmanState.vy * cos_theta * dt;
    float theta_pred = normalizeAngle(_kalmanState.theta + _kalmanState.omega * dt);
    
    // Store predicted state (velocities and biases remain the same in this model)
    _kalmanState.x = x_pred;
    _kalmanState.y = y_pred;
    _kalmanState.theta = theta_pred;
    
    // Calculate Jacobian matrix F (partial derivatives of motion model)
    float F[9][9] = {0};
    
    // Diagonal elements
    for (int i = 0; i < 9; i++) {
        F[i][i] = 1.0f;
    }
    
    // Non-zero elements
    F[0][2] = -(_kalmanState.vx * sin_theta + _kalmanState.vy * cos_theta) * dt;
    F[0][3] = cos_theta * dt;
    F[0][4] = -sin_theta * dt;
    
    F[1][2] = (_kalmanState.vx * cos_theta - _kalmanState.vy * sin_theta) * dt;
    F[1][3] = sin_theta * dt;
    F[1][4] = cos_theta * dt;
    
    // d(theta)/d(omega)
    F[2][5] = dt;
    
    // Predizione della covarianza: P = F * P * F' + Q
    float FP[9][9] = {0};
    float FPT[9][9] = {0};
    
    // FP = F * P
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            FP[i][j] = 0;
            for (int k = 0; k < 9; k++) {
                FP[i][j] += F[i][k] * _kalmanState.P[k][j];
            }
        }
    }
    
    // FPT = FP * F' (F' è la trasposta di F)
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            FPT[i][j] = 0;
            for (int k = 0; k < 9; k++) {
                FPT[i][j] += FP[i][k] * F[j][k];
            }
        }
    }
    
    // P = FPT + Q
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            _kalmanState.P[i][j] = FPT[i][j] + _kalmanState.Q[i][j];
        }
    }
    
    _kalmanState.last_update = millis();
}

void PositionManager::updateState() {
    if (!_kalmanState.initialized) {
        return;
    }
    
    // Measurement vector
    float z[6] = {0};
    
    // Observation matrix H (Jacobian of the measurement model)
    float H[6][9] = {0};
    
    // Vector to indicate which measurements are valid
    bool valid_measurements[6] = {false};
    
    // Measurement noise covariance matrix
    float R[6][6] = {0};
    
    // 1. IMU Measurements
    if (_isIMUEnabled && _imu != nullptr) {
        // Get IMU data - using the IMUModule interface
        IMUData imuData = _imu->getData();
        
        if (imuData.isValid) {
            // Convert gyroZ from raw data to rad/s (assuming 250 deg/s range and 16-bit resolution)
            const float GYRO_SCALE = (250.0f / 32768.0f) * (M_PI / 180.0f);
            float gyroZ_rads = imuData.gyroZ * GYRO_SCALE;
            
            // Get orientation from IMU
            float yaw = _imu->getTheta();  // Get orientation in radians
            
            // Update measurement vector
            z[2] = yaw;  // Theta (yaw angle)
            z[5] = gyroZ_rads - _kalmanState.bias_gyro;  // Angular rate minus bias
            
            // Update measurement matrix H (Jacobian)
            H[2][2] = 1.0f;  // d(theta_meas)/d(theta)
            H[5][2] = 0.0f;  // d(omega_meas)/d(theta)
            H[5][6] = -1.0f; // d(omega_meas)/d(bias_gyro)
            
            // Set measurement noise covariance
            float imu_yaw_std = sqrtf(_kalmanParams.measurement_noise_imu_yaw);
            float imu_gyro_std = sqrtf(_kalmanParams.measurement_noise_imu_gyro);
            R[2][2] = imu_yaw_std * imu_yaw_std;
            R[5][5] = imu_gyro_std * imu_gyro_std;
            
            // Set measurement as valid
            valid_measurements[2] = true;  // Yaw measurement
            valid_measurements[5] = true;  // Angular rate measurement
        } else {
            // Handle invalid IMU data
            // ...
        }
    } else {
        // Handle IMU not enabled or null
        // ...
    }
    
    // 2. Odometry measurements
    if (_isOdometryEnabled && _leftMotor != nullptr && _rightMotor != nullptr) {
        // Calculate linear and angular velocity from wheel speeds
        float v_left = _leftMotor->getLinearVelocity();
        float v_right = _rightMotor->getLinearVelocity();
        
        // Velocità lineare e angolare nel sistema di riferimento del robot
        float v = (v_right + v_left) / 2.0f;  // Velocità lineare media
        float omega = (v_right - v_left) / _wheelBase;  // Velocità angolare
        
        // Trasforma nel sistema di riferimento globale
        float theta = _kalmanState.theta;
        float cos_theta = cos(theta);
        float sin_theta = sin(theta);
        
        z[3] = v * cos_theta;  // vx
        z[4] = v * sin_theta;  // vy
        z[5] = omega;          // omega
        
        // Jacobiano per le misurazioni odometriche
        H[3][3] = cos_theta;  // d(vx_meas)/d(vx)
        H[3][4] = -v * sin_theta; // d(vx_meas)/d(theta)
        
        H[4][3] = sin_theta;  // d(vy_meas)/d(vx)
        H[4][4] = v * cos_theta;  // d(vy_meas)/d(theta)
        
        H[5][5] = 1.0f;  // d(omega_meas)/d(omega)
        
        valid_measurements[3] = true;
        valid_measurements[4] = true;
        valid_measurements[5] = true;
    }
    
    // 3. GPS measurements (if available)
    if (_isGPSEnabled && _gps != nullptr) {
        // Get GPS data using the GPSModule interface
        if (_gps->isValid() && _gps->getHDOP() < 2.0f) {  // Only if signal quality is good
            // Convert GPS coordinates to local coordinates
            float gps_x, gps_y;
            convertGPSToXY(_gps->getLatitude(), _gps->getLongitude(), gps_x, gps_y);
            
            z[0] = gps_x;  // x position
            z[1] = gps_y;  // y position
            
            // Jacobian for GPS measurements
            H[0][0] = 1.0f;  // d(x_gps)/d(x)
            H[1][1] = 1.0f;  // d(y_gps)/d(y)
            
            valid_measurements[0] = true;
            valid_measurements[1] = true;
        }
    }
    
    // Se non ci sono misurazioni valide, esci
    bool has_valid_measurements = false;
    for (int i = 0; i < 6; i++) {
        if (valid_measurements[i]) {
            has_valid_measurements = true;
            break;
        }
    }
    
    if (!has_valid_measurements) {
        return;
    }
    
    // Inizializza la matrice di rumore di misura R
    // Questa matrice rappresenta l'incertezza delle misurazioni dei sensori
    // R è già dichiarata all'inizio della funzione, qui la inizializziamo a zero
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            R[i][j] = 0.0f;
        }
    }
    
    // Imposta le varianze di misura in base ai sensori abilitati
    if (valid_measurements[0] && valid_measurements[1]) {
        // GPS position noise (m)
        float gps_std = _kalmanParams.measurement_noise_gps;
        R[0][0] = gps_std * gps_std;  // x position variance (m²)
        R[1][1] = gps_std * gps_std;  // y position variance (m²)
    }
    
    if (valid_measurements[2]) {
        // IMU yaw noise (rad)
        float yaw_std = _kalmanParams.measurement_noise_imu_yaw;
        R[2][2] = yaw_std * yaw_std;  // yaw variance (rad²)
    }
    
    if (valid_measurements[3] && valid_measurements[4]) {
        // Odometry velocity noise (m/s)
        float odom_vel_std = _kalmanParams.measurement_noise_odom_vel;
        R[3][3] = odom_vel_std * odom_vel_std;  // vx variance ((m/s)²)
        R[4][4] = odom_vel_std * odom_vel_std;  // vy variance ((m/s)²)
    }
    
    if (valid_measurements[5]) {
        // IMU gyro noise (rad/s)
        float gyro_std = _kalmanParams.measurement_noise_imu_gyro;
        R[5][5] = gyro_std * gyro_std;  // gyro variance ((rad/s)²)
    }
    
    // Fase di correzione del filtro di Kalman
    // Calculate innovation: y = z - Hx
    float y[6] = {0};
    for (int i = 0; i < 6; i++) {
        if (valid_measurements[i]) {
            y[i] = z[i];
            
            // Calculate H*x for this measurement
            float hx = 0.0f;
            for (int j = 0; j < 9; j++) {
                float state_j = 
                    (j == 0) ? _kalmanState.x :
                    (j == 1) ? _kalmanState.y :
                    (j == 2) ? _kalmanState.theta :
                    (j == 3) ? _kalmanState.vx :
                    (j == 4) ? _kalmanState.vy :
                    (j == 5) ? _kalmanState.omega :
                    (j == 6) ? _kalmanState.bias_gyro :
                    (j == 7) ? _kalmanState.bias_accel_x :
                    _kalmanState.bias_accel_y;
                
                hx += H[i][j] * state_j;
            }
            
            y[i] -= hx;
            
            // Normalize angle for orientation
            if (i == 2) {  // Theta
                y[i] = normalizeAngle(y[i]);
            }
        }
    }

    // Calculate the innovation covariance matrix S = H*P*H' + R
    float S[6][6] = {0};
    
    // Compute H*P*H' + R
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            S[i][j] = R[i][j];  // Start with measurement noise
            
            // Add H*P*H'
            for (int k = 0; k < 9; k++) {
                for (int l = 0; l < 9; l++) {
                    S[i][j] += H[i][k] * _kalmanState.P[k][l] * H[j][l];
                }
            }
        }
    }
    
    // Calculate the Kalman gain K = P*H'*S^-1
    float K[9][6] = {0};
    
    // For simplicity, we'll use a diagonal approximation of S^-1
    // since we don't have a full matrix inversion function
    for (int i = 0; i < 6; i++) {
        if (S[i][i] > 1e-6) {  // Avoid division by zero
            for (int j = 0; j < 9; j++) {
                // K = P * H' * S^-1 (diagonal approximation)
                float sum = 0.0f;
                for (int k = 0; k < 9; k++) {
                    sum += _kalmanState.P[j][k] * H[i][k];
                }
                K[j][i] = sum / S[i][i];
            }
        }
    }
    
    // Update the state using the Kalman gain
    float state_update[9] = {0};
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 6; j++) {
            if (valid_measurements[j]) {
                state_update[i] += K[i][j] * y[j];
            }
        }
    }
    
    // Apply state updates
    _kalmanState.x += state_update[0];
    _kalmanState.y += state_update[1];
    _kalmanState.theta = normalizeAngle(_kalmanState.theta + state_update[2]);
    _kalmanState.vx += state_update[3];
    _kalmanState.vy += state_update[4];
    _kalmanState.omega += state_update[5];
    _kalmanState.bias_gyro += state_update[6];
    _kalmanState.bias_accel_x += state_update[7];
    _kalmanState.bias_accel_y += state_update[8];
    
    // Update the covariance matrix using the Joseph form for numerical stability
    // P = (I - K*H)*P*(I - K*H)' + K*R*K'
    float I_KH[9][9] = {0};
    
    // Initialize identity matrix
    for (int i = 0; i < 9; i++) {
        I_KH[i][i] = 1.0f;
    }
    
    // Compute I - K*H
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 6; k++) {
                I_KH[i][j] -= K[i][k] * H[k][j];
            }
        }
    }
    
    // Temporary matrix for (I-KH)*P
    float temp1[9][9] = {0};
    
    // Compute (I-KH)*P
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                temp1[i][j] += I_KH[i][k] * _kalmanState.P[k][j];
            }
        }
    }
    
    // Compute (I-KH)*P*(I-KH)'
    float temp2[9][9] = {0};
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                temp2[i][j] += temp1[i][k] * I_KH[j][k];
            }
        }
    }
    
    // Compute K*R*K'
    float KRKt[9][9] = {0};
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 6; k++) {
                for (int l = 0; l < 6; l++) {
                    KRKt[i][j] += K[i][k] * R[k][l] * K[j][l];
                }
            }
        }
    }
    
    // Final covariance update
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            _kalmanState.P[i][j] = temp2[i][j] + KRKt[i][j];
        }
    }
    // In case of numerical errors, we'll just continue with the predicted state
    // This is a simplified error handling approach for embedded systems
}

void PositionManager::calculateCovariance() {
    // Aggiorna la matrice di covarianza
    float I[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    // Calcola il guadagno di Kalman per l'IMU
    float R_imu = _kalmanParams.measurement_noise_imu_yaw;
    float K[3] = {_kalmanState.P[0][0] / (_kalmanState.P[0][0] + R_imu),
                 _kalmanState.P[1][0] / (_kalmanState.P[0][0] + R_imu),
                 _kalmanState.P[2][0] / (_kalmanState.P[0][0] + R_imu)};
    
    // P = (I - KH)P
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            _kalmanState.P[i][j] = (I[i][j] - K[i]) * _kalmanState.P[i][j];
        }
    }
}


RobotPosition PositionManager::getPosition() const {
    RobotPosition pos;
    pos.x = _kalmanState.x;
    pos.y = _kalmanState.y;
    pos.theta = _kalmanState.theta;
    pos.speed = sqrtf(_kalmanState.vx * _kalmanState.vx + _kalmanState.vy * _kalmanState.vy);
    pos.omega = _kalmanState.omega;
    pos.isValid = true;  // Assuming the position is valid if we have a Kalman state
    return pos;
}

void PositionManager::moveStraight(float speedPercent, float distanceMeters) {
    if (_leftMotor && _rightMotor) {
        // Limita la velocità tra -100% e 100%
        speedPercent = constrain(speedPercent, -100.0f, 100.0f);
        
        // Imposta la velocità lineare in percentuale
        _leftMotor->setLinearSpeed(speedPercent);
        _rightMotor->setLinearSpeed(speedPercent);
        
        // Se è specificata una distanza, attendi che venga raggiunta
        if (distanceMeters != 0) {
            // Calcola la velocità in m/s per il calcolo del tempo
            float speedMps = (abs(speedPercent) / 100.0f) * MAX_LINEAR_SPEED;
            if (speedMps > 0) {
                // Calcola il tempo necessario in millisecondi
                unsigned long moveTime = static_cast<unsigned long>(
                    (distanceMeters / speedMps) * 1000.0f);
                
                // Attendi il completamento del movimento
                delay(moveTime);
                
                // Ferma i motori
                _leftMotor->setLinearSpeed(0);
                _rightMotor->setLinearSpeed(0);
            }
        }
    }
}

void PositionManager::turn(float angle, float speed) {
    if (_leftMotor && _rightMotor) {
        // Calcola la velocità angolare (rad/s)
        // angle è in radianti, speed è la velocità lineare massima (m/s)
        float omega = (2.0f * speed) / WHEEL_BASE;  // Velocità angolare massima
        
        // Calcola il tempo necessario per la rotazione (secondi)
        float time = abs(angle / omega);
        
        // Imposta la velocità dei motori in percentuale
        // Ruotare su se stessi: un motore avanti e uno indietro
        _leftMotor->setLinearSpeed(speed);
        _rightMotor->setLinearSpeed(-speed);
        
        // Attendi il tempo necessario
        delay(static_cast<unsigned long>(time * 1000));
        
        // Ferma i motori
        _leftMotor->setLinearSpeed(0);
        _rightMotor->setLinearSpeed(0);
    }
}

void PositionManager::moveArc(float radius, float angularSpeed, float angle) {
    if (_leftMotor && _rightMotor) {
        // Calcola le velocità lineari dei due motori per un movimento circolare
        float leftSpeed = angularSpeed * (radius - _wheelBase / 2.0f);
        float rightSpeed = angularSpeed * (radius + _wheelBase / 2.0f);
        
        // Imposta le velocità lineari
        _leftMotor->setLinearSpeed(leftSpeed);
        _rightMotor->setLinearSpeed(rightSpeed);
        
        // Se è specificato un angolo, attendi che venga raggiunto
        if (angle > 0) {
            // Calcola il tempo necessario per percorrere l'arco (in millisecondi)
            unsigned long moveTime = static_cast<unsigned long>((angle / angularSpeed) * 1000);
            delay(moveTime);
            
            // Ferma i motori
            _leftMotor->setLinearSpeed(0);
            _rightMotor->setLinearSpeed(0);
        }
    }
}

// I metodi isOdometryEnabled, isIMUEnabled e isGPSEnabled sono definiti inline nell'header

void PositionManager::setOdometry(float x, float y, float theta, float speed, float omega) {
    // Update odometry data
    _odometryData.x = x;
    _odometryData.y = y;
    _odometryData.theta = normalizeAngle(theta);
    _odometryData.speed = speed;
    _odometryData.omega = omega;
    
    // If this is the first odometry reading, initialize the Kalman filter
    if (!_kalmanState.initialized) {
        _kalmanState.x = x;
        _kalmanState.y = y;
        _kalmanState.theta = normalizeAngle(theta);
        _kalmanState.vx = speed * cosf(theta);
        _kalmanState.vy = speed * sinf(theta);
        _kalmanState.omega = omega;
        _kalmanState.initialized = true;
    }
}

void PositionManager::setIMU(float theta, float omega) {
    // Update IMU data
    _imuData.yaw = normalizeAngle(theta);
    _imuData.theta = _imuData.yaw;  // Mantenere per compatibilità
    _imuData.gyroZ = omega;         // omega è la velocità angolare attorno all'asse Z
    _imuData.omega = _imuData.gyroZ; // Mantenere per compatibilità
    
    // If this is the first IMU reading and Kalman filter is not initialized yet,
    // we can use it to initialize the orientation
    if (!_kalmanState.initialized) {
        _kalmanState.theta = normalizeAngle(theta);
        _kalmanState.omega = omega;
    }
}

void PositionManager::setGPS(float latitude, float longitude, float speed) {
    // Update GPS data
    _gpsData.latitude = latitude;
    _gpsData.longitude = longitude;
    _gpsData.speed = speed;
    _gpsData.isValid = true;
    
    // If this is the first GPS reading and we don't have a position yet,
    // use it to initialize the position
    if (!_kalmanState.initialized) {
        // Convert GPS coordinates to local coordinates (if needed)
        // For now, just store the raw values
        _kalmanState.x = 0.0f;  // Should be converted from GPS to local coordinates
        _kalmanState.y = 0.0f;  // Should be converted from GPS to local coordinates
        _kalmanState.vx = speed;  // Assuming speed is in the direction of travel
        _kalmanState.vy = 0.0f;
        
        // If we have IMU data, use it for orientation
        if (_isIMUEnabled) {
            _kalmanState.theta = _imuData.theta;
            _kalmanState.omega = _imuData.omega;
        }
        
        _kalmanState.initialized = true;
    }
}

void PositionManager::fuseMeasurements() {
    // Questo metodo è ora gestito all'interno di updateState()
}

// Metodi per la gestione della posizione home
// Le implementazioni di saveHomePosition e updateHomePosition sono definite inline nell'header

float PositionManager::getDistanceToHome() const {
    if (!_homePosition.isValid || !_gps || !_gps->isValid()) {
        return -1.0f;
    }
    
    double currentLat = _gps->getLatitude();
    double currentLon = _gps->getLongitude();
    
    // Formula di Haversine per calcolare la distanza tra due punti
    double dLat = radians(_homePosition.latitude - currentLat);
    double dLon = radians(_homePosition.longitude - currentLon);
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(radians(currentLat)) * cos(radians(_homePosition.latitude)) * 
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return static_cast<float>(EARTH_RADIUS_CM * c / 100.0); // Converti in metri
}

float PositionManager::getBearingToHome() const {
    if (!_homePosition.isValid || !_gps || !_gps->isValid()) {
        return -1.0f;
    }
    
    double currentLat = _gps->getLatitude();
    double currentLon = _gps->getLongitude();
    
    return calculateBearing(currentLat, currentLon, _homePosition.latitude, _homePosition.longitude);
}

void PositionManager::convertLLAToXY(double lat1, double lon1, double lat2, double lon2, float& x, float& y) {
    // Conversione da coordinate geografiche a coordinate cartesiane piane (approssimazione per brevi distanze)
    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    
    // Approssimazione per brevi distanze
    x = static_cast<float>(EARTH_RADIUS_CM * dLon * cos(radians(lat1)) / 100.0); // in metri
    y = static_cast<float>(EARTH_RADIUS_CM * dLat / 100.0); // in metri
}

float PositionManager::calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    // Converte le coordinate in radianti
    lat1 = radians(lat1);
    lon1 = radians(lon1);
    lat2 = radians(lat2);
    lon2 = radians(lon2);
    
    // Calcola la differenza di longitudine
    double dLon = lon2 - lon1;
    
    // Calcola il bearing
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    // Converte in gradi e normalizza tra 0 e 360
    float bearing = static_cast<float>(degrees(atan2(y, x)));
    bearing = fmod((bearing + 360.0), 360.0);
    
    return bearing;
}

// Matrix multiplication: C = A * B
void PositionManager::matrixMultiply(const float A[9][9], const float B[9][9], float C[9][9], int rowsA, int colsA, int colsB) {
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            C[i][j] = 0;
            for (int k = 0; k < colsA; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Matrix transpose: B = A^T
void PositionManager::matrixTranspose(const float A[9][9], float B[9][9], int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            B[j][i] = A[i][j];
        }
    }
}

// 3x3 matrix inversion
bool PositionManager::matrixInverse3x3(const float A[3][3], float B[3][3]) {
    // Calculate determinant
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    
    if (fabs(det) < 1e-6) {
        // Matrix is singular or nearly singular
        return false;
    }
    
    // Calculate cofactor matrix
    float invDet = 1.0f / det;
    B[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
    B[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
    B[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;
    B[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
    B[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
    B[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) * invDet;
    B[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
    B[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
    B[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) * invDet;
    
    return true;
}

// 2x2 matrix inversion
bool PositionManager::matrixInverse2x2(const float A[2][2], float B[2][2]) {
    float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    
    if (fabs(det) < 1e-6) {
        // Matrix is singular or nearly singular
        return false;
    }
    
    float invDet = 1.0f / det;
    B[0][0] =  A[1][1] * invDet;
    B[0][1] = -A[0][1] * invDet;
    B[1][0] = -A[1][0] * invDet;
    B[1][1] =  A[0][0] * invDet;
    
    return true;
}

// Compute innovation covariance: S = H*P*H' + R
void PositionManager::computeInnovationCovariance(const float H[6][9], const float P[9][9], const float R[6][6], float S[6][6]) {
    // HP = H * P
    float HP[6][9] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 9; j++) {
            HP[i][j] = 0;
            for (int k = 0; k < 9; k++) {
                HP[i][j] += H[i][k] * P[k][j];
            }
        }
    }
    
    // S = HP * H' + R
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            S[i][j] = R[i][j];
            for (int k = 0; k < 9; k++) {
                S[i][j] += HP[i][k] * H[j][k];  // H' is transpose of H
            }
        }
    }
}

// Compute Kalman gain: K = P * H' * S^-1
bool PositionManager::computeKalmanGain(const float P[9][9], const float H[6][9], const float S[6][6], float K[9][6]) {
    // Calculate S^-1 (inverse of S)
    // For simplicity, we'll only invert the 2x2 or 3x3 diagonal blocks
    // This is an approximation that works well if S is nearly block-diagonal
    
    // Invert the 2x2 GPS position block
    float S_pos[2][2] = {{S[0][0], S[0][1]}, {S[1][0], S[1][1]}};
    float S_pos_inv[2][2];
    if (!matrixInverse2x2(S_pos, S_pos_inv)) return false;
    
    // Invert the 1x1 angle block
    if (fabs(S[2][2]) < 1e-6) return false;
    float S_theta_inv = 1.0f / S[2][2];
    
    // Invert the 2x2 velocity block
    float S_vel[2][2] = {{S[3][3], S[3][4]}, {S[4][3], S[4][4]} };
    float S_vel_inv[2][2];
    if (!matrixInverse2x2(S_vel, S_vel_inv)) return false;
    
    // Invert the 1x1 gyro block
    if (fabs(S[5][5]) < 1e-6) return false;
    float S_gyro_inv = 1.0f / S[5][5];
    
    // Build the approximate S_inv (diagonal blocks only)
    float S_inv[6][6] = {0};
    S_inv[0][0] = S_pos_inv[0][0];
    S_inv[0][1] = S_pos_inv[0][1];
    S_inv[1][0] = S_pos_inv[1][0];
    S_inv[1][1] = S_pos_inv[1][1];
    
    S_inv[2][2] = S_theta_inv;
    
    S_inv[3][3] = S_vel_inv[0][0];
    S_inv[3][4] = S_vel_inv[0][1];
    S_inv[4][3] = S_vel_inv[1][0];
    S_inv[4][4] = S_vel_inv[1][1];
    
    S_inv[5][5] = S_gyro_inv;
    
    // Calculate K = P * H' * S_inv
    // First calculate PHt = P * H'
    float PHt[9][6] = {0};
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 6; j++) {
            PHt[i][j] = 0;
            for (int k = 0; k < 9; k++) {
                PHt[i][j] += P[i][k] * H[j][k];  // H' is transpose of H
            }
        }
    }
    
    // Then calculate K = PHt * S_inv
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 6; j++) {
            K[i][j] = 0;
            for (int k = 0; k < 6; k++) {
                K[i][j] += PHt[i][k] * S_inv[k][j];
            }
        }
    }
    
    return true;
}

// Define the global positionManager instance
PositionManager positionManager(nullptr, nullptr, nullptr, nullptr, 0.0f);
