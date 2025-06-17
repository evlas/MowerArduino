#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <MPU6050.h>

#define MPU6050_ADDRESS 0x68

class IMU {
public:
    IMU();
    ~IMU();
    
    bool begin();
    void update();
    
    float getRoll();
    float getPitch();
    float getYaw();
    
    float getAccX();
    float getAccY();
    float getAccZ();
    
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    
private:
    MPU6050 mpu;
    
    float roll, pitch, yaw;
    int16_t rawAccX, rawAccY, rawAccZ;
    int16_t rawGyroX, rawGyroY, rawGyroZ;
    
    // Conversion constants
    const float accScale = 16384.0;  // 8g range
    const float gyroScale = 16.4;    // 2000dps range
};

#endif
