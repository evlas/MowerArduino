#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include <Arduino.h>
#include "../sensors/IMUModule/IMUModule.h"
#include "../sensors/GPSModule/GPSModule.h"
#include "../motors/DriveMotor/DriveMotor.h"

// Struttura per rappresentare la posizione del robot
struct RobotPosition {
    float x;      // Posizione X in cm
    float y;      // Posizione Y in cm
    float theta;  // Orientamento in radianti
    float speed;  // Velocità lineare in cm/s
    float omega;  // Velocità angolare in rad/s
    bool isValid; // Flag di validità dei dati
};

// Parametri del filtro di Kalman
struct KalmanParams {
    float P[6][6];  // Covarianza dello stato
    float F[6][6];  // Transizione
    float B[6][2];  // Controllo
    float H[6][6];  // Osservazione
    float Q[6][6];  // Rumore di processo
    float R[6][6];  // Rumore di misurazione
};

// Stato del filtro di Kalman
struct KalmanState {
    float x[3];         // Stato: [theta, theta_dot, bias]
    float P[3][3];     // Matrice di covarianza dello stato
    float Q[3][3];     // Rumore di processo
    float R_imu;       // Rumore di misura dell'IMU
    float R_odo;       // Rumore di misura dell'odometria
    float last_theta;  // Ultimo angolo stimato
    unsigned long last_update;  // Timestamp ultimo aggiornamento
};

class PositionManager {
    private:
        // Puntatori ai sensori e attuatori
        DriveMotor* _leftMotor = nullptr;
        DriveMotor* _rightMotor = nullptr;
        IMUModule* _imu = nullptr;
        GPSModule* _gps = nullptr;
        
        // Altri membri...
        
    public:
        PositionManager(DriveMotor* leftMotor = nullptr, DriveMotor* rightMotor = nullptr, IMUModule* imu = nullptr, GPSModule* gps = nullptr, float wheelBase = 0.0f);
        ~PositionManager();
        
        // Inizializzazione
        bool begin();
        
        // Aggiornamento posizione
        void update();
        
        // Gestione dei sensori
        void enableOdometry(bool enable);
        void enableIMU(bool enable);
        void enableGPS(bool enable);
        
        // Ottiene la posizione corrente
        RobotPosition getPosition();
        
        /**
         * @brief Muove il robot in linea retta
         * @param speedPercent Velocità in percentuale (-100% a 100%)
         * @param distanceMeters Distanza in metri (0 per movimento continuo)
         */
        void moveStraight(float speedPercent, float distanceMeters = 0);
        void turn(float angle, float speed);
        void moveArc(float radius, float angle, float speed);
        
        // Ottiene lo stato dei sensori
        bool isOdometryEnabled() const;
        bool isIMUEnabled() const;
        bool isGPSEnabled() const;
        
        // Getter for wheel base
        float getWheelBase() const { return _wheelBase; }

        // Setter per i sensori
        void setOdometry(float x, float y, float theta, float speed, float omega);
        void setIMU(float theta, float omega);
        void setGPS(float latitude, float longitude, float speed);
        
    private:
        // Lettura sensori
        RobotPosition _odometryData;
        struct IMUReading {
            float theta;
            float omega;
        } _imuData;
        
        struct GPSReading {
            float latitude;
            float longitude;
            float speed;
        } _gpsData;
        
        // I riferimenti ai motori e ai sensori sono già dichiarati all'inizio della classe
        
        // Parametri di Kalman
        KalmanParams _kalman;
        KalmanState _kalmanState;
        
        // Stato
        RobotPosition _currentPosition;
        bool _isOdometryEnabled;
        bool _isIMUEnabled;
        bool _isGPSEnabled;
        float _wheelBase;  // Distanza tra le ruote in metri
        
        // Metodi privati
        void initializeKalman();
        void predictState();
        void updateState();
        void fuseMeasurements();
        void calculateCovariance();
};

extern PositionManager positionManager;

#endif // POSITION_MANAGER_H
