#ifndef IMUMODULE_H
#define IMUMODULE_H

#include <MPU6050.h>
#include <math.h>

// Strutture dati compatibili con ROS
typedef struct {
    double x, y, z, w;
} ROS_Quaternion;

typedef struct {
    double x, y, z;
} ROS_Vector3;

typedef struct {
    double x, y, z;
} ROS_Point;

typedef struct {
    ROS_Quaternion orientation;
    ROS_Vector3 angular_velocity;
    ROS_Vector3 linear_acceleration;
} ROS_ImuMsg;

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
    // Struttura per i dati del filtro complementare
    struct {
        float pitch = 0.0;
        float roll = 0.0;
        float yaw = 0.0;
        unsigned long lastUpdate = 0;
    } orientation;
    private:
        MPU6050 imu;
        IMUData currentData;
        
        // Calibration data
        struct {
            int16_t accelX_offset;
            int16_t accelY_offset;
            int16_t accelZ_offset;
            int16_t gyroX_offset;
            int16_t gyroY_offset;
            int16_t gyroZ_offset;
            bool calibrated;
        } calibration;
        
    public:
        // Coefficiente del filtro complementare (0-1)
        // Più alto = più influenza del giroscopio
        float complementaryFilterAlpha = 0.98f;
        
        IMUModule() {
            currentData.isValid = false;
            currentData.gyroX = 0;
            currentData.gyroY = 0;
            currentData.gyroZ = 0;
            currentData.accelX = 0;
            currentData.accelY = 0;
            currentData.accelZ = 0;
            currentData.temp = 0;
            
            // Initialize calibration data
            calibration.accelX_offset = 0;
            calibration.accelY_offset = 0;
            calibration.accelZ_offset = 0;
            calibration.gyroX_offset = 0;
            calibration.gyroY_offset = 0;
            calibration.gyroZ_offset = 0;
            calibration.calibrated = false;
        }
        
        void begin() {
            imu.initialize();
        }
        
        void update() {
            // Read raw data
            imu.getAcceleration(&currentData.accelX, &currentData.accelY, &currentData.accelZ);
            imu.getRotation(&currentData.gyroX, &currentData.gyroY, &currentData.gyroZ);
            
            // Apply calibration if available
            if (calibration.calibrated) {
                currentData.accelX += calibration.accelX_offset;
                currentData.accelY += calibration.accelY_offset;
                currentData.accelZ += calibration.accelZ_offset;
                
                currentData.gyroX += calibration.gyroX_offset;
                currentData.gyroY += calibration.gyroY_offset;
                currentData.gyroZ += calibration.gyroZ_offset;
            }
            
            // Aggiorna l'orientamento con filtro complementare
            updateOrientation();
            
            currentData.temp = imu.getTemperature();
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
        
        /**
         * Calibrates the IMU by calculating offsets
         * Should be called when the IMU is in a stable, flat position
         * @param samples Number of samples to take for calibration (default: 1000)
         */
        void calibrate(uint16_t samples = 1000) {
            int32_t ax = 0, ay = 0, az = 0;
            int32_t gx = 0, gy = 0, gz = 0;
            
            // Target values for accelerometer (assuming Z is pointing up)
            const int16_t targetZ = 16384;  // 1g in raw values (assuming 2g range)
            
            Serial.println("Calibrating IMU. Keep the device stable and flat...");
            delay(1000);
            
            // Collect samples
            for (uint16_t i = 0; i < samples; i++) {
                imu.getAcceleration(&currentData.accelX, &currentData.accelY, &currentData.accelZ);
                imu.getRotation(&currentData.gyroX, &currentData.gyroY, &currentData.gyroZ);
                
                ax += currentData.accelX;
                ay += currentData.accelY;
                az += currentData.accelZ - targetZ;  // Expecting 1g in Z axis
                
                gx += currentData.gyroX;
                gy += currentData.gyroY;
                gz += currentData.gyroZ;
                
                if (i % 100 == 0) {
                    Serial.print(".");
                }
                delay(5);
            }
            
            // Calculate average offsets
            calibration.accelX_offset = -ax / samples;
            calibration.accelY_offset = -ay / samples;
            calibration.accelZ_offset = -az / samples;
            
            calibration.gyroX_offset = -gx / samples;
            calibration.gyroY_offset = -gy / samples;
            calibration.gyroZ_offset = -gz / samples;
            
            calibration.calibrated = true;
            
            Serial.println("\nCalibration complete!");
            printCalibration();
        }
        
        /**
         * Prints the current calibration offsets
         */
        void printCalibration() {
            Serial.println("\n--- IMU Calibration Data ---");
            Serial.print("Accel Offsets - X: "); Serial.print(calibration.accelX_offset);
            Serial.print("  Y: "); Serial.print(calibration.accelY_offset);
            Serial.print("  Z: "); Serial.println(calibration.accelZ_offset);
            
            Serial.print("Gyro Offsets  - X: "); Serial.print(calibration.gyroX_offset);
            Serial.print("  Y: "); Serial.print(calibration.gyroY_offset);
            Serial.print("  Z: "); Serial.println(calibration.gyroZ_offset);
            Serial.println("----------------------------");
        }
        
        /**
         * Checks if the IMU has been calibrated
         * @return true if calibrated, false otherwise
         */
        bool isCalibrated() {
            return calibration.calibrated;
        }
        
        // ===== METODI ROS-COMPATIBILI =====
        
        /**
         * Restituisce un messaggio IMU compatibile con ROS
         */
        ROS_ImuMsg getROSImuMsg() {
            ROS_ImuMsg msg;
            
            // Calcola l'orientamento
            float q[4];
            getROSQuaternion(q);
            
            // Imposta l'orientamento
            msg.orientation.x = q[0];
            msg.orientation.y = q[1];
            msg.orientation.z = q[2];
            msg.orientation.w = q[3];
            
            // Velocità angolare (rad/s)
            msg.angular_velocity.x = currentData.gyroX * (M_PI / 180.0f) * 0.0175f; // Converti in rad/s
            msg.angular_velocity.y = currentData.gyroY * (M_PI / 180.0f) * 0.0175f;
            msg.angular_velocity.z = currentData.gyroZ * (M_PI / 180.0f) * 0.0175f;
            
            // Accelerazione lineare (m/s²)
            const float accel_scale = 9.81f / 16384.0f; // Scala per MPU6050 a ±2g
            msg.linear_acceleration.x = currentData.accelX * accel_scale;
            msg.linear_acceleration.y = currentData.accelY * accel_scale;
            msg.linear_acceleration.z = currentData.accelZ * accel_scale;
            
            return msg;
        }
        
        /**
         * Restituisce l'orientamento come quaternione ROS
         * @param q Array di 4 float per memorizzare il quaternione [x, y, z, w]
         */
        void getROSQuaternion(float q[4]) {
            // Converti angoli di Eulero in quaternione
            float cy = cos(orientation.yaw * 0.5f);
            float sy = sin(orientation.yaw * 0.5f);
            float cp = cos(orientation.pitch * 0.5f);
            float sp = sin(orientation.pitch * 0.5f);
            float cr = cos(orientation.roll * 0.5f);
            float sr = sin(orientation.roll * 0.5f);
            
            q[0] = sr * cp * cy - cr * sp * sy; // x
            q[1] = cr * sp * cy + sr * cp * sy; // y
            q[2] = cr * cp * sy - sr * sp * cy; // z
            q[3] = cr * cp * cy + sr * sp * sy; // w
        }
        
        /**
         * Restituisce la velocità angolare in un formato compatibile con ROS
         * @param angular_velocity Struttura per memorizzare la velocità angolare (rad/s)
         */
        void getROSAngularVelocity(ROS_Vector3 &angular_velocity) {
            const float degToRad = M_PI / 180.0f * 0.0175f; // Conversione in rad/s
            angular_velocity.x = currentData.gyroX * degToRad;
            angular_velocity.y = currentData.gyroY * degToRad;
            angular_velocity.z = currentData.gyroZ * degToRad;
        }
        
        /**
         * Restituisce l'accelerazione lineare in un formato compatibile con ROS
         * @param linear_acceleration Struttura per memorizzare l'accelerazione lineare (m/s²)
         */
        void getROSLinearAcceleration(ROS_Vector3 &linear_acceleration) {
            const float accel_scale = 9.81f / 16384.0f; // Scala per MPU6050 a ±2g
            linear_acceleration.x = currentData.accelX * accel_scale;
            linear_acceleration.y = currentData.accelY * accel_scale;
            linear_acceleration.z = currentData.accelZ * accel_scale;
        }
        
        /**
         * Restituisce l'orientamento in angoli di Eulero (radianti)
         * @param euler Struttura per memorizzare gli angoli di Eulero [roll, pitch, yaw]
         */
        void getROSEuler(ROS_Vector3 &euler) {
            euler.x = orientation.roll * (M_PI / 180.0f);  // roll in radianti
            euler.y = orientation.pitch * (M_PI / 180.0f); // pitch in radianti
            euler.z = orientation.yaw * (M_PI / 180.0f);   // yaw in radianti
        }
        
    private:
        /**
         * Aggiorna l'orientamento usando un filtro complementare
         */
        void updateOrientation() {
            // Calcola l'intervallo di tempo dall'ultimo aggiornamento
            unsigned long now = millis();
            float dt = (now - orientation.lastUpdate) / 1000.0f; // in secondi
            orientation.lastUpdate = now;
            
            if (dt > 0.1f) return; // Evita valori anomali
            
            // Leggi i dati grezzi (già calibrati)
            float ax = currentData.accelX;
            float ay = currentData.accelY;
            float az = currentData.accelZ;
            
            // Normalizza il vettore di accelerazione
            float norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm > 0) {
                ax /= norm;
                ay /= norm;
                az /= norm;
            }
            
            // Calcola gli angoli dall'accelerometro
            float accPitch = atan2(ay, az) * 180.0f / M_PI;
            float accRoll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
            
            // Applica il filtro complementare
            orientation.pitch = complementaryFilterAlpha * (orientation.pitch + currentData.gyroY * dt) + 
                              (1.0f - complementaryFilterAlpha) * accPitch;
                              
            orientation.roll = complementaryFilterAlpha * (orientation.roll + currentData.gyroX * dt) + 
                             (1.0f - complementaryFilterAlpha) * accRoll;
            
            // Per lo yaw usiamo solo il giroscopio (senza riferimento magnetico)
            orientation.yaw += currentData.gyroZ * dt;
            
            // Normalizza gli angoli tra -180° e 180°
            if (orientation.yaw > 180.0f) orientation.yaw -= 360.0f;
            if (orientation.yaw < -180.0f) orientation.yaw += 360.0f;
        }
};

extern IMUModule imu;

#endif // IMU_H