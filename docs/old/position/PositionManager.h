#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include <Arduino.h>
#include "../motors/MotorController.h"
#include "../sensors/IMU.h"
#include "../sensors/GPS.h"

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
    float x[6];     // Stato attuale
    float z[6];     // Misurazione
    float K[6][6];  // Guadagno di Kalman
};

class PositionManager {
    public:
        PositionManager();
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
        
        // Gestione movimenti
        void moveStraight(float distance, float speed);
        void turn(float angle, float speed);
        void moveArc(float radius, float angle, float speed);
        
        // Ottiene lo stato dei sensori
        bool isOdometryEnabled() const;
        bool isIMUEnabled() const;
        bool isGPSEnabled() const;

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
        
        // Sensori
        MotorController* _motorController;
        IMUModule* _imu;
        GPSModule* _gps;
        
        // Parametri di Kalman
        KalmanParams _kalman;
        KalmanState _kalmanState;
        
        // Stato
        RobotPosition _currentPosition;
        bool _isOdometryEnabled;
        bool _isIMUEnabled;
        bool _isGPSEnabled;
        
        // Metodi privati
        void initializeKalman();
        void predictState();
        void updateState();
        void fuseMeasurements();
        void calculateCovariance();
};

extern PositionManager positionManager;

#endif // POSITION_MANAGER_H
