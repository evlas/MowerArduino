#include "PositionManager.h"
#include "../sensors/IMU.h"
#include "../sensors/GPS.h"

// Costanti per il filtro di Kalman
#define KALMAN_Q 0.01f  // Fattore di rumore di processo
#define KALMAN_R 0.1f   // Fattore di rumore di misurazione

// Costanti per la conversione GPS
#define EARTH_RADIUS_CM 637100000.0f  // Raggio terrestre in cm
#define DEG_TO_RAD 0.0174532925f      // Conversione da gradi a radianti

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

PositionManager::PositionManager() :
    _motorController(nullptr),
    _imu(nullptr),
    _gps(nullptr),
    _isOdometryEnabled(false),
    _isIMUEnabled(false),
    _isGPSEnabled(false)
{
    // Inizializza i dati dei sensori
    _odometryData.x = 0.0f;
    _odometryData.y = 0.0f;
    _odometryData.theta = 0.0f;
    _odometryData.speed = 0.0f;
    _odometryData.omega = 0.0f;
    
    _imuData.theta = 0.0f;
    _imuData.omega = 0.0f;
    
    _gpsData.latitude = 0.0f;
    _gpsData.longitude = 0.0f;
    _gpsData.speed = 0.0f;
    
    // Inizializza la posizione corrente
    _currentPosition.x = 0.0f;
    _currentPosition.y = 0.0f;
    _currentPosition.theta = 0.0f;
    _currentPosition.speed = 0.0f;
    _currentPosition.omega = 0.0f;
    _currentPosition.isValid = false;
}

PositionManager::~PositionManager() {
    // Niente da fare, i sensori vengono gestiti altrove
}

bool PositionManager::begin() {
    // Inizializza il filtro di Kalman
    initializeKalman();
    return true;
}

void PositionManager::initializeKalman() {
    // Inizializza la covarianza dello stato
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            _kalman.P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Inizializza la matrice di transizione
    for (int i = 0; i < 6; i++) {
        _kalman.F[i][i] = 1.0f;
    }
    
    // Inizializza la matrice di controllo
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 2; j++) {
            _kalman.B[i][j] = 0.0f;
        }
    }
    
    // Inizializza la matrice di osservazione
    for (int i = 0; i < 6; i++) {
        _kalman.H[i][i] = 1.0f;
    }
    
    // Inizializza la covarianza del rumore di processo
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            _kalman.Q[i][j] = (i == j) ? KALMAN_Q : 0.0f;
        }
    }
    
    // Inizializza la covarianza del rumore di misurazione
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            _kalman.R[i][j] = (i == j) ? KALMAN_R : 0.0f;
        }
    }
    
    // Inizializza lo stato
    for (int i = 0; i < 6; i++) {
        _kalmanState.x[i] = 0.0f;
        _kalmanState.z[i] = 0.0f;
    }
}

void PositionManager::update() {
    // Ottieni i dati dai sensori abilitati
    float x = 0.0f, y = 0.0f;
    float speed = 0.0f, omega = 0.0f;
    float imuTheta = 0.0f;
    bool hasIMU = false;
    bool hasOdometry = false;
    bool hasGPS = false;
    float gpsX = 0.0f, gpsY = 0.0f;

    // Ottieni dati dall'odometria
    if (_isOdometryEnabled && _motorController != nullptr) {
        x = _motorController->getX();
        y = _motorController->getY();
        speed = _motorController->getLinearVelocity();
        omega = _motorController->getAngularVelocity();
        hasOdometry = true;
    }

    // Ottieni dati dall'IMU
    if (_isIMUEnabled && _imu != nullptr) {
        imuTheta = _imu->getTheta();
        omega = _imu->getAngularVelocity();  // Usa l'IMU come fonte principale per omega
        hasIMU = true;
    }

    // Ottieni dati dal GPS
    if (_isGPSEnabled && _gps != nullptr) {
        GPSData gpsData = _gps->getData();
        if (gpsData.isValid) {
            // Converte coordinate GPS in coordinate cartesiane
            convertGPSToXY(gpsData.latitude, gpsData.longitude, gpsX, gpsY);
            hasGPS = true;
        }
    }

    // Gestione dell'orientamento
    float theta;
    if (hasOdometry) {
        theta = _motorController->getTheta();
    } else if (hasIMU) {
        theta = imuTheta;
    } else {
        theta = 0.0f;  // Default se non ci sono sensori
    }

    // Se abbiamo GPS, calcola la correzione della posizione
    if (hasGPS) {
        // Usa un peso per la correzione GPS
        float gpsWeight = 0.1f;  // Pesare il GPS meno dell'odometria
        x = (1.0f - gpsWeight) * x + gpsWeight * gpsX;
        y = (1.0f - gpsWeight) * y + gpsWeight * gpsY;
    }

    // Normalizza l'angolo
    theta = normalizeAngle(theta);

    // Aggiorna i dati dei sensori
    setOdometry(x, y, theta, speed, omega);
    
    // Aggiorna il filtro di Kalman
    predictState();
    fuseMeasurements();
    updateState();
    calculateCovariance();
    
    // Aggiorna la posizione corrente
    _currentPosition.x = _kalmanState.x[0];
    _currentPosition.y = _kalmanState.x[1];
    _currentPosition.theta = normalizeAngle(_kalmanState.x[2]);
    _currentPosition.speed = _kalmanState.x[3];
    _currentPosition.omega = _kalmanState.x[4];

    // Verifica la validità dei dati
    if (hasOdometry || hasIMU || hasGPS) {
        _currentPosition.isValid = true;
    } else {
        _currentPosition.isValid = false;
    }
}

void PositionManager::enableOdometry(bool enable) {
    _isOdometryEnabled = enable;
    if (enable && _motorController == nullptr) {
        _motorController = &motorController;
    }
}

void PositionManager::enableIMU(bool enable) {
    _isIMUEnabled = enable;
    if (enable) {
#ifdef ENABLE_IMU
        _imu = &imu;
#else
        _imu = nullptr; // IMU non disponibile
#endif
    } else {
        _imu = nullptr;
    }
}

void PositionManager::enableGPS(bool enable) {
    _isGPSEnabled = enable;
    if (enable) {
#ifdef ENABLE_GPS
        _gps = &gps;
#else
        _gps = nullptr; // GPS non disponibile
#endif
    } else {
        _gps = nullptr;
    }
}

RobotPosition PositionManager::getPosition() {
    return _currentPosition;
}

void PositionManager::moveStraight(float distance, float speed) {
    if (_motorController != nullptr) {
        // Calcola il tempo necessario
        float time = abs(distance / speed);
        
        // Imposta la velocità dei motori
        _motorController->setLeftMotorSpeed(speed);
        _motorController->setRightMotorSpeed(speed);
        
        // Attendi il tempo necessario
        delay(time * 1000);
        
        // Ferma i motori
        _motorController->setLeftMotorSpeed(0);
        _motorController->setRightMotorSpeed(0);
    }
}

void PositionManager::turn(float angle, float speed) {
    if (_motorController != nullptr) {
        // Calcola la velocità angolare
        float omega = speed / _motorController->getWheelBase();
        
        // Calcola il tempo necessario
        float time = abs(angle / omega);
        
        // Imposta la direzione dei motori
        bool leftDir = (angle > 0);
        bool rightDir = !leftDir;
        
        // Imposta i motori
        _motorController->setLeftMotorDirection(leftDir);
        _motorController->setRightMotorDirection(rightDir);
        _motorController->setLeftMotorSpeed(speed);
        _motorController->setRightMotorSpeed(speed);
        
        // Attendi il tempo necessario
        delay(time * 1000);
        
        // Ferma i motori
        _motorController->setLeftMotorSpeed(0);
        _motorController->setRightMotorSpeed(0);
    }
}

void PositionManager::moveArc(float radius, float angle, float speed) {
    if (_motorController != nullptr) {
        // Calcola le velocità dei motori
        float innerSpeed = speed * (radius - _motorController->getWheelBase() / 2) / radius;
        float outerSpeed = speed * (radius + _motorController->getWheelBase() / 2) / radius;
        
        // Calcola il tempo necessario
        float time = abs(angle * radius / speed);
        
        // Imposta la direzione dei motori
        bool leftDir = (angle > 0);
        bool rightDir = !leftDir;
        
        // Imposta i motori
        _motorController->setLeftMotorDirection(leftDir);
        _motorController->setRightMotorDirection(rightDir);
        _motorController->setLeftMotorSpeed(innerSpeed);
        _motorController->setRightMotorSpeed(outerSpeed);
        
        // Attendi il tempo necessario
        delay(time * 1000);
        
        // Ferma i motori
        _motorController->setLeftMotorSpeed(0);
        _motorController->setRightMotorSpeed(0);
    }
}

bool PositionManager::isOdometryEnabled() const {
    return _isOdometryEnabled;
}

bool PositionManager::isIMUEnabled() const {
    return _isIMUEnabled;
}

bool PositionManager::isGPSEnabled() const {
    return _isGPSEnabled;
}

// Setter per i sensori
void PositionManager::setOdometry(float x, float y, float theta, float speed, float omega) {
    if (_isOdometryEnabled) {
        _odometryData.x = x;
        _odometryData.y = y;
        _odometryData.theta = theta;
        _odometryData.speed = speed;
        _odometryData.omega = omega;
    }
}

void PositionManager::setIMU(float theta, float omega) {
    if (_isIMUEnabled) {
        _imuData.theta = theta;
        _imuData.omega = omega;
    }
}

void PositionManager::setGPS(float latitude, float longitude, float speed) {
    if (_isGPSEnabled) {
        _gpsData.latitude = latitude;
        _gpsData.longitude = longitude;
        _gpsData.speed = speed;
    }
}

// Metodi privati del filtro di Kalman
void PositionManager::predictState() {
    // Implementazione della predizione dello stato
    // Questo è un placeholder - la reale implementazione dipenderà dal modello del robot
}

void PositionManager::updateState() {
    // Implementazione dell'aggiornamento dello stato
    // Questo è un placeholder - la reale implementazione dipenderà dal modello del robot
}

void PositionManager::fuseMeasurements() {
    // Implementazione della fusione delle misurazioni
    // Questo è un placeholder - la reale implementazione dipenderà dal modello del robot
}

void PositionManager::calculateCovariance() {
    // Implementazione del calcolo della covarianza
    // Questo è un placeholder - la reale implementazione dipenderà dal modello del robot
}
