#include <Arduino.h>
#include "IMUModule.h"
#include "config.h"
#include "pin_config.h"

IMUModule imu;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for serial port or 2 seconds
  
  Serial.println("\nInitializing IMU...");
  imu.begin();
  
  // Small delay to let IMU stabilize
  delay(100);
  
  // Update once to check if IMU is responding
  imu.update();
  
  if (!imu.isValid()) {
    Serial.println("ERROR: IMU not detected! Check wiring.");
    while(1); // Halt
  }
  
  Serial.println("IMU initialized successfully!");
  printHelp();
  
  // Print initial calibration status
  if (imu.isCalibrated()) {
    Serial.println("\nUsing stored calibration data.");
  } else {
    Serial.println("\nIMU is not calibrated. For best results, run 'c' to calibrate.");
  }
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 'a': // Read accelerometer
        imu.update();
        printAccelerometer();
        break;
        
      case 'g': // Read gyroscope
        imu.update();
        printGyroscope();
        break;
        
      case 'm': // Read magnetometer
        imu.update();
        printMagnetometer();
        break;
        
      case 'y': // Read yaw, pitch, roll
        imu.update();
        printYPR();
        break;
        
      case 'c': // Calibrate
        calibrateIMU();
        break;
        
      case 'h':
        printHelp();
        break;
        
      case 's': // Toggle streaming
        streamIMUData();
        break;
        
      case 'p': // Print calibration data
        if (imu.isCalibrated()) {
          imu.printCalibration();
        } else {
          Serial.println("IMU is not calibrated. Run 'c' to calibrate first.");
        }
        break;
        
      default:
        Serial.println("Unknown command. Type 'h' for help.");
        break;
    }
  }
}

void printAccelerometer() {
  IMUData data = imu.getData();
  if (!imu.isValid()) {
    Serial.println("No IMU data available. Call update() first.");
    return;
  }
  
  // Convert from raw values to m/s² (assuming typical MPU6050 sensitivity of 16384 LSB/g)
  const float accel_scale = 9.81f / 16384.0f;
  
  Serial.println("\n--- Accelerometer ---");
  Serial.print("X: "); Serial.print(data.accelX * accel_scale, 4); Serial.println(" m/s²");
  Serial.print("Y: "); Serial.print(data.accelY * accel_scale, 4); Serial.println(" m/s²");
  Serial.print("Z: "); Serial.print(data.accelZ * accel_scale, 4); Serial.println(" m/s²");
  Serial.println();
}

void printGyroscope() {
  IMUData data = imu.getData();
  if (!imu.isValid()) {
    Serial.println("No IMU data available. Call update() first.");
    return;
  }
  
  // Convert from raw values to °/s (assuming typical MPU6050 sensitivity of 131 LSB/°/s)
  const float gyro_scale = 1.0f / 131.0f;
  
  Serial.println("\n--- Gyroscope ---");
  Serial.print("X: "); Serial.print(data.gyroX * gyro_scale, 2); Serial.println(" °/s");
  Serial.print("Y: "); Serial.print(data.gyroY * gyro_scale, 2); Serial.println(" °/s");
  Serial.print("Z: "); Serial.print(data.gyroZ * gyro_scale, 2); Serial.println(" °/s");
  Serial.println();
}

void printMagnetometer() {
  Serial.println("\n--- Magnetometer ---");
  Serial.println("Magnetometer not available with MPU6050");
  Serial.println("Use an external magnetometer (like HMC5883L) for compass functionality");
  Serial.println();
}

void printYPR() {
  IMUData data = imu.getData();
  if (!imu.isValid()) {
    Serial.println("No IMU data available. Call update() first.");
    return;
  }
  
  // Simple orientation from accelerometer (not as accurate as a full IMU fusion algorithm)
  float accX = data.accelX;
  float accY = data.accelY;
  float accZ = data.accelZ;
  
  // Calculate pitch and roll from accelerometer
  float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
  float roll = atan2(accY, accZ) * 180.0 / PI;
  
  // Yaw requires a magnetometer or gyro integration over time
  float yaw = 0.0; // Not available without magnetometer or gyro integration
  
  Serial.println("\n--- Orientation ---");
  Serial.print("Pitch: "); Serial.print(pitch, 1); Serial.println("°");
  Serial.print("Roll:  "); Serial.print(roll, 1); Serial.println("°");
  Serial.println("Note: Yaw not available without magnetometer or gyro integration");
  Serial.println();
}

void calibrateIMU() {
  Serial.println("\n--- IMU Calibration ---");
  Serial.println("1. Place the IMU on a flat, level surface");
  Serial.println("2. Keep it completely still during calibration");
  Serial.println("3. This will take about 5 seconds...");
  Serial.println("\nStarting calibration in 3 seconds...");
  
  delay(3000);
  
  // Perform calibration
  imu.calibrate(500);  // Use 500 samples for calibration (about 5 seconds)
  
  // Print calibration status
  if (imu.isCalibrated()) {
    Serial.println("\nIMU calibration successful!");
  } else {
    Serial.println("\nIMU calibration failed!");
  }
}

void streamIMUData() {
  Serial.println("\nStarting IMU data streaming (press any key to stop)...");
  
  unsigned long lastPrint = 0;
  const unsigned long printInterval = 100; // 10 Hz
  
  // Conversion factors
  const float accel_scale = 9.81f / 16384.0f;  // Convert raw accel to m/s²
  const float gyro_scale = 1.0f / 131.0f;      // Convert raw gyro to °/s
  
  while (!Serial.available()) {
    if (millis() - lastPrint >= printInterval) {
      imu.update();
      IMUData data = imu.getData();
      
      if (!imu.isValid()) {
        Serial.println("No IMU data available");
        continue;
      }
      
      // Clear previous lines
      for (int i = 0; i < 10; i++) {
        Serial.println();
      }
      
      // Calculate orientation from accelerometer
      float accX = data.accelX;
      float accY = data.accelY;
      float accZ = data.accelZ;
      float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
      float roll = atan2(accY, accZ) * 180.0 / PI;
      
      // Print current values
      Serial.println("--- IMU Live Data ---");
      Serial.print("Pitch: "); Serial.print(pitch, 1);
      Serial.print("°  Roll: "); Serial.print(roll, 1);
      Serial.println("°  (Yaw not available without magnetometer)");
      
      // Print accelerometer data
      Serial.print("Accel (m/s²): ");
      Serial.print("X="); Serial.print(data.accelX * accel_scale, 2);
      Serial.print(" Y="); Serial.print(data.accelY * accel_scale, 2);
      Serial.print(" Z="); Serial.println(data.accelZ * accel_scale, 2);
      
      // Print gyroscope data
      Serial.print("Gyro  (°/s):  ");
      Serial.print("X="); Serial.print(data.gyroX * gyro_scale, 2);
      Serial.print(" Y="); Serial.print(data.gyroY * gyro_scale, 2);
      Serial.print(" Z="); Serial.println(data.gyroZ * gyro_scale, 2);
      
      lastPrint = millis();
    }
  }
  
  // Clear the input buffer
  while (Serial.available()) {
    Serial.read();
  }
  
  Serial.println("\nStopped streaming.");
}

void printHelp() {
  Serial.println("\n=== IMU Test ===");
  Serial.println("a - Read accelerometer (m/s²)");
  Serial.println("g - Read gyroscope (°/s)");
  Serial.println("m - Magnetometer status");
  Serial.println("y - Show orientation (pitch/roll)");
  Serial.println("c - Calibrate IMU (flat surface required)");
  Serial.println("s - Start/stop data streaming");
  Serial.println("p - Print calibration data");
  Serial.println("h - Show this help");
  Serial.println("================");
  
  // Show calibration status
  Serial.print("Calibration: ");
  if (imu.isCalibrated()) {
    Serial.println("CALIBRATED");
  } else {
    Serial.println("NOT CALIBRATED (recommend running 'c' first)");
  }
  Serial.println("================");
}
