#include "PositionManager.h"
#include "IMUModule.h"
#include "GPSModule.h"
#include "config.h"

// Costanti per il filtro di Kalman
#define KALMAN_Q 0.01f  // Fattore di rumore di processo
#define KALMAN_R 0.1f   // Fattore di rumore di misurazione

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

PositionManager::PositionManager(DriveMotor* leftMotor, DriveMotor* rightMotor, IMUModule* imu, GPSModule* gps, float wheelBase) :
    _leftMotor(leftMotor),
    _rightMotor(rightMotor),
    _imu(imu),
    _gps(gps),
    _isOdometryEnabled(leftMotor != nullptr && rightMotor != nullptr),
    _isIMUEnabled(true),
    _isGPSEnabled(gps != nullptr),
    _wheelBase(wheelBase)
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
    // Inizializza lo stato
    _kalmanState.x[0] = 0.0f;  // Theta iniziale
    _kalmanState.x[1] = 0.0f;  // Theta_dot iniziale (velocità angolare)
    _kalmanState.x[2] = 0.0f;  // Bias iniziale del giroscopio
    
    // Inizializza la matrice di covarianza dello stato
    // Valori iniziali alti per permettere una convergenza rapida
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            _kalmanState.P[i][j] = 0.0f;
        }
    }
    _kalmanState.P[0][0] = 1.0f;    // Varianza dell'angolo
    _kalmanState.P[1][1] = 0.1f;    // Varianza della velocità angolare
    _kalmanState.P[2][2] = 0.01f;   // Varianza del bias
    
    // Inizializza la matrice di rumore di processo
    // Q rappresenta l'incertezza nel modello del sistema
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            _kalmanState.Q[i][j] = 0.0f;
        }
    }
    _kalmanState.Q[0][0] = 0.001f;  // Rumore dell'angolo
    _kalmanState.Q[1][1] = 0.003f;  // Rumore della velocità angolare
    _kalmanState.Q[2][2] = 0.0001f; // Rumore del bias (cambia lentamente)
    
    // Inizializza i rumori di misura
    _kalmanState.R_imu = 0.01f;     // Rumore della misura IMU (più affidabile)
    _kalmanState.R_odo = 0.05f;     // Rumore della misura odometrica (meno affidabile)
    
    _kalmanState.last_theta = 0.0f;
    _kalmanState.last_update = millis();
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

    // Aggiorna i dati dei sensori
    setOdometry(x, y, theta, speed, omega);
    
    // Aggiorna il filtro di Kalman
    predictState();
    updateState();
    calculateCovariance();
    
    // Aggiorna la posizione corrente
    _currentPosition.x = _kalmanState.x[0];
    _currentPosition.y = _kalmanState.x[1];
    _currentPosition.theta = normalizeAngle(_kalmanState.x[2]);
    _currentPosition.speed = _kalmanState.x[3];
    _currentPosition.omega = _kalmanState.x[4];

    // I dati sono sempre validi poiché odometria e IMU sono sempre abilitati
    _currentPosition.isValid = true;
}

void PositionManager::predictState() {
    unsigned long now = millis();
    float dt = (now - _kalmanState.last_update) / 1000.0f;  // Converti in secondi
    
    // Predizione dello stato
    // x = [theta, theta_dot, bias]
    // Predici il nuovo stato usando il modello del sistema
    float theta_pred = _kalmanState.x[0] + (_kalmanState.x[1] - _kalmanState.x[2]) * dt;
    float theta_dot_pred = _kalmanState.x[1];  // Assumiamo velocità angolare costante
    float bias_pred = _kalmanState.x[2];       // Assumiamo bias costante
    
    // Aggiorna lo stato predetto
    _kalmanState.x[0] = normalizeAngle(theta_pred);
    _kalmanState.x[1] = theta_dot_pred;
    _kalmanState.x[2] = bias_pred;
    
    // Matrice Jacobiana della transizione di stato
    float F[3][3] = {
        {1.0f, dt, -dt},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    
    // Aggiorna la matrice di covarianza
    // P = F*P*F' + Q
    float FP[3][3] = {{0}};
    float FPFT[3][3] = {{0}};
    
    // FP = F*P
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            for(int k = 0; k < 3; k++) {
                FP[i][j] += F[i][k] * _kalmanState.P[k][j];
            }
        }
    }
    
    // FPFT = FP*F'
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            for(int k = 0; k < 3; k++) {
                FPFT[i][j] += FP[i][k] * F[j][k];  // Nota: F[j][k] invece di F[k][j] per la trasposizione
            }
        }
    }
    
    // P = FPFT + Q
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            _kalmanState.P[i][j] = FPFT[i][j] + _kalmanState.Q[i][j];
        }
    }
    
    _kalmanState.last_update = now;
}

void PositionManager::updateState() {
    // Misurazioni
    float z_imu = _imuData.theta;
    float z_odo = _odometryData.theta;
    float z_gps = 0.0f;
    bool has_gps = false;
    
    // Se il GPS è abilitato e valido, calcola l'heading
    if (_isGPSEnabled && _gps != nullptr) {
        GPSData gpsData = _gps->getData();
        if (gpsData.fix) {
            // Calcola l'heading dal GPS usando la differenza di posizione
            static float last_gps_x = 0.0f, last_gps_y = 0.0f;
            float gps_x, gps_y;
            convertGPSToXY(gpsData.latitude, gpsData.longitude, gps_x, gps_y);
            
            float dx = gps_x - last_gps_x;
            float dy = gps_y - last_gps_y;
            if (dx*dx + dy*dy > 1.0f) {  // Solo se ci siamo mossi abbastanza
                z_gps = atan2(dy, dx);
                has_gps = true;
                last_gps_x = gps_x;
                last_gps_y = gps_y;
            }
        }
    }
    
    // Matrice di osservazione H = [1, 0, 0]
    // Calcolo del guadagno di Kalman per IMU
    float K_imu[3];
    float S_imu = _kalmanState.P[0][0] + _kalmanState.R_imu;
    K_imu[0] = _kalmanState.P[0][0] / S_imu;
    K_imu[1] = _kalmanState.P[1][0] / S_imu;
    K_imu[2] = _kalmanState.P[2][0] / S_imu;
    
    // Calcolo del guadagno di Kalman per odometria
    float K_odo[3];
    float S_odo = _kalmanState.P[0][0] + _kalmanState.R_odo;
    K_odo[0] = _kalmanState.P[0][0] / S_odo;
    K_odo[1] = _kalmanState.P[1][0] / S_odo;
    K_odo[2] = _kalmanState.P[2][0] / S_odo;
    
    // Aggiorna lo stato con le misure dell'IMU (peso maggiore)
    float innovation_imu = angleDifference(z_imu, _kalmanState.x[0]);
    _kalmanState.x[0] += K_imu[0] * innovation_imu;
    _kalmanState.x[1] += K_imu[1] * innovation_imu;
    _kalmanState.x[2] += K_imu[2] * innovation_imu;
    
    // Aggiorna lo stato con le misure dell'odometria (peso minore)
    float innovation_odo = angleDifference(z_odo, _kalmanState.x[0]);
    _kalmanState.x[0] += 0.3f * K_odo[0] * innovation_odo;
    _kalmanState.x[1] += 0.3f * K_odo[1] * innovation_odo;
    _kalmanState.x[2] += 0.3f * K_odo[2] * innovation_odo;
    
    // Se disponibile, aggiorna con il GPS (peso molto basso)
    if (has_gps) {
        float K_gps[3];
        float R_gps = 0.5f;  // Alta incertezza per il GPS
        float S_gps = _kalmanState.P[0][0] + R_gps;
        K_gps[0] = _kalmanState.P[0][0] / S_gps;
        K_gps[1] = _kalmanState.P[1][0] / S_gps;
        K_gps[2] = _kalmanState.P[2][0] / S_gps;
        
        float innovation_gps = angleDifference(z_gps, _kalmanState.x[0]);
        _kalmanState.x[0] += 0.1f * K_gps[0] * innovation_gps;
        _kalmanState.x[1] += 0.1f * K_gps[1] * innovation_gps;
        _kalmanState.x[2] += 0.1f * K_gps[2] * innovation_gps;
    }
    
    // Normalizza l'angolo
    _kalmanState.x[0] = normalizeAngle(_kalmanState.x[0]);
}

void PositionManager::calculateCovariance() {
    // Aggiorna la matrice di covarianza
    float I[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    float K[3] = {_kalmanState.P[0][0] / (_kalmanState.P[0][0] + _kalmanState.R_imu),
                  _kalmanState.P[1][0] / (_kalmanState.P[0][0] + _kalmanState.R_imu),
                  _kalmanState.P[2][0] / (_kalmanState.P[0][0] + _kalmanState.R_imu)};
    
    // P = (I - KH)P
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            _kalmanState.P[i][j] = (I[i][j] - K[i]) * _kalmanState.P[i][j];
        }
    }
}

void PositionManager::enableOdometry(bool enable) {
    // L'odometria è sempre abilitata
    (void)enable;  // Evita warning per parametro non utilizzato
}

void PositionManager::enableIMU(bool enable) {
    // L'IMU è sempre abilitata
    (void)enable;  // Evita warning per parametro non utilizzato
}

void PositionManager::enableGPS(bool enable) {
    _isGPSEnabled = enable;
    // _gps should be set through constructor or setter
    // We don't use a global GPS instance here
}

RobotPosition PositionManager::getPosition() {
    return _currentPosition;
}

void PositionManager::moveStraight(float speed, float distance) {
    if (_leftMotor && _rightMotor) {
        // Imposta la velocità lineare per entrambi i motori (in m/s)
        _leftMotor->setLinearSpeed(speed);
        _rightMotor->setLinearSpeed(speed);
        
        // Se è specificata una distanza, attendi che venga raggiunta
        if (distance > 0) {
            // Implementa la logica per attendere il completamento del movimento
            // (da implementare con un loop di controllo)
            delay(static_cast<unsigned long>(distance / speed * 1000));
            
            // Ferma i motori
            _leftMotor->setLinearSpeed(0);
            _rightMotor->setLinearSpeed(0);
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

bool PositionManager::isOdometryEnabled() const {
    return true;  // L'odometria è sempre abilitata
}

bool PositionManager::isIMUEnabled() const {
    return true;  // L'IMU è sempre abilitata
}

bool PositionManager::isGPSEnabled() const {
    #ifdef ENABLE_GPS
    return (_gps != nullptr);  // Il GPS è abilitato se è configurato in config.h
    #else
    return false;  // GPS non disponibile
    #endif
}

void PositionManager::setOdometry(float x, float y, float theta, float speed, float omega) {
    _odometryData.x = x;
    _odometryData.y = y;
    _odometryData.theta = theta;
    _odometryData.speed = speed;
    _odometryData.omega = omega;
}

void PositionManager::setIMU(float theta, float omega) {
    _imuData.theta = theta;
    _imuData.omega = omega;
}

void PositionManager::setGPS(float latitude, float longitude, float speed) {
    _gpsData.latitude = latitude;
    _gpsData.longitude = longitude;
    _gpsData.speed = speed;
}

void PositionManager::fuseMeasurements() {
    // Questo metodo è ora gestito all'interno di updateState()
}

// Define the global positionManager instance
PositionManager positionManager(nullptr, nullptr, nullptr, nullptr, 0.0f);
