#ifdef ENABLE_IMU
#include "IMU.h"

IMU::IMU() {
    // Initialize MPU6050
}

IMU::~IMU() {
    // Destructor
}

bool IMU::begin() {
    mpu.initialize();
    
    // Check connection
    if (!mpu.testConnection()) {
        return false;
    }
    
    // Configure MPU6050
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    mpu.setDLPFMode(MPU6050_DLPF_BW_5);
    
    return true;
}

void IMU::update() {
    // Read raw data from MPU6050
    mpu.getMotion6(&rawAccX, &rawAccY, &rawAccZ, &rawGyroX, &rawGyroY, &rawGyroZ);
    
    // Convert raw data to meaningful values
    accX = rawAccX / accScale;
    accY = rawAccY / accScale;
    accZ = rawAccZ / accScale;
    
    gyroX = rawGyroX / gyroScale;
    gyroY = rawGyroY / gyroScale;
    gyroZ = rawGyroZ / gyroScale;
    
    // Calculate roll and pitch using accelerometer data
    roll = atan2(accY, accZ) * 180.0 / M_PI;
    pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / M_PI;
    yaw = 0;  // Yaw is not available from accelerometer
}

float IMU::getRoll() {
    return roll;
}

float IMU::getPitch() {
    return pitch;
}

float IMU::getYaw() {
    return yaw;
}

float IMU::getAccX() {
    return accX;
}

float IMU::getAccY() {
    return accY;
}

float IMU::getAccZ() {
    return accZ;
}

float IMU::getGyroX() {
    return gyroX;
}

float IMU::getGyroY() {
    return gyroY;
}

float IMU::getGyroZ() {
    return gyroZ;
}
#endif  // ENABLE_IMU