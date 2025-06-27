#ifndef IMU_H
#define IMU_H

#include <MPU6050.h>

// Dati IMU
struct IMUData {
    bool isValid;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t temp;
};

class IMUModule {
    private:
        MPU6050 imul;
        IMUData currentData;
        
    public:
        IMUModule() {
            currentData.isValid = false;
            currentData.gyroX = 0;
            currentData.gyroY = 0;
            currentData.gyroZ = 0;
            currentData.accelX = 0;
            currentData.accelY = 0;
            currentData.accelZ = 0;
            currentData.temp = 0;
        }
        
        void begin() {
            imul.initialize();
        }
        
        void update() {
            imul.getAcceleration(&currentData.accelX, &currentData.accelY, &currentData.accelZ);
            imul.getRotation(&currentData.gyroX, &currentData.gyroY, &currentData.gyroZ);
            currentData.temp = imul.getTemperature();
            currentData.isValid = true;
        }
        
        IMUData getData() {
            return currentData;
        }
        
        float getTheta() const {
            // Calcola l'orientamento dall'accelerometro
            // Questo è un metodo semplice che usa solo l'accelerometro
            // Per un risultato più preciso si dovrebbe usare un filtro complementare
            return atan2(currentData.accelY, currentData.accelX);
        }
        
        float getAngularVelocity() const {
            // Restituisce la velocità angolare dal giroscopio
            return currentData.gyroZ;  // Usiamo il giroscopio Z per la rotazione verticale
        }
        
        bool isValid() {
            return currentData.isValid;
        }
};

extern IMUModule imu;

#endif // IMU_H