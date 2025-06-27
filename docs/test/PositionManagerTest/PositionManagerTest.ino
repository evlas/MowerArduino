#include <Arduino.h>
#include <assert.h>
#include "PositionManager.h"
#include "DriveMotor.h"
#include "IMUModule.h"
#include "GPSModule.h"

// Define the IMU instance
IMUModule imu;

// Simple test framework
#define TEST_CASE(name) void name##_test()

// Forward declarations
void TestInitialization_test();
void TestEnableSensors_test();
void TestOdometryUpdate_test();
void TestRotation_test();
void TestGPSUpdate_test();
void TestErrorConditions_test();
#define TEST_ASSERT(condition) \
    if (!(condition)) { \
        Serial.print("Test failed at " __FILE__ ":"); \
        Serial.print(__LINE__); \
        Serial.print(" - " #condition); \
        while(1); \
    } else { \
        Serial.print("Test passed: " #condition); \
    } \
    Serial.println()

#define TEST_ASSERT_EQUAL(expected, actual) \
    if ((expected) != (actual)) { \
        Serial.print("Test failed at " __FILE__ ":"); \
        Serial.print(__LINE__); \
        Serial.print(" - Expected: "); \
        Serial.print(expected); \
        Serial.print(" Actual: "); \
        Serial.print(actual); \
        while(1); \
    } else { \
        Serial.print("Test passed: " #actual " == " #expected); \
    } \
    Serial.println()

#define TEST_ASSERT_FLOAT_EQUAL(expected, actual, epsilon) \
    if (abs((expected) - (actual)) > (epsilon)) { \
        Serial.print("Test failed at " __FILE__ ":"); \
        Serial.print(__LINE__); \
        Serial.print(" - Expected: "); \
        Serial.print(expected, 6); \
        Serial.print(" Actual: "); \
        Serial.print(actual, 6); \
        while(1); \
    } else { \
        Serial.print("Test passed: " #actual " ~= " #expected); \
    } \
    Serial.println()

// Forward declarations
class DriveMotor;
class GPSModule;

// Global instances for testing
DriveMotor* leftMotor = nullptr;
DriveMotor* rightMotor = nullptr;
GPSModule* gps = nullptr;

void run_tests() {
    Serial.println("\n=== Starting Tests ===");
    
    // Initialize hardware components
    // Note: You'll need to provide the correct pin numbers for your hardware
    leftMotor = new DriveMotor(/* pwmPin */ 3, /* dirPin */ 4, /* encoderPin */ 2);
    rightMotor = new DriveMotor(/* pwmPin */ 5, /* dirPin */ 6, /* encoderPin */ 3);
    
    // Initialize GPS (using Serial1 for hardware UART)
    gps = new GPSModule();
    
    // Initialize IMU
    imu.begin();
    
    // Initialize the global positionManager instance
    positionManager = PositionManager(leftMotor, rightMotor, &imu, gps, 30.0f);
    
    // Initialize all components
    leftMotor->begin();
    rightMotor->begin();
    gps->begin();
    positionManager.begin();
    
    // Wait for components to initialize
    delay(1000);
    
    // Esegui i test
    TestInitialization_test();
    TestEnableSensors_test();
    TestOdometryUpdate_test();
    TestRotation_test();
    TestGPSUpdate_test();
    TestErrorConditions_test();
    
    Serial.println("=== All tests completed ===");
}

// Using real implementations instead of mocks
// IMU: Using global 'imu' instance from IMUModule.h
// GPS: Using real GPSModule
// DriveMotors: Using real DriveMotor implementation

TEST_CASE(TestInitialization) {
        TEST_ASSERT_EQUAL(30.0f, positionManager.getWheelBase());
}

TEST_CASE(TestEnableSensors) {
    positionManager.enableOdometry(true);
    TEST_ASSERT(positionManager.isOdometryEnabled());
    
    positionManager.enableIMU(true);
    TEST_ASSERT(positionManager.isIMUEnabled());
    
    positionManager.enableGPS(true);
    TEST_ASSERT(positionManager.isGPSEnabled());
}

TEST_CASE(TestOdometryUpdate) {
    positionManager.enableOdometry(true);
    positionManager.enableIMU(false);
    positionManager.enableGPS(false);
    
    // With real motors, we can't directly set the position
    // Instead, we need to move the motors and let the encoders update
    leftMotor->setLinearSpeed(0.1f);  // Move forward at 0.1 m/s
    rightMotor->setLinearSpeed(0.1f);
    delay(1000);  // Move for 1 second (should move ~0.1m)
    leftMotor->setLinearSpeed(0);
    rightMotor->setLinearSpeed(0);
    
    positionManager.update();
    
    RobotPosition pos = positionManager.getPosition();
    TEST_ASSERT(pos.x != 0.0f);
    TEST_ASSERT(pos.isValid);
}

TEST_CASE(TestRotation) {
    positionManager.enableIMU(true);
    
    // For testing with real IMU, we can't directly set the yaw and gyro values
    // Instead, we'll need to physically rotate the IMU or use a test fixture
    // For now, we'll just test that the IMU is working
    imu.update();  // Update IMU readings
    
    positionManager.update();
    
    RobotPosition pos = positionManager.getPosition();
    TEST_ASSERT_FLOAT_EQUAL(PI/2, pos.theta, 0.01);
    TEST_ASSERT(pos.isValid);
}

TEST_CASE(TestGPSUpdate) {
    // Verifica che la posizione sia stata aggiornata
    RobotPosition pos = positionManager.getPosition();
    float x = pos.x;
    float y = pos.y;
    bool valid = pos.isValid;
    TEST_ASSERT(x != 0.0f);
    TEST_ASSERT(y != 0.0f);
    TEST_ASSERT(valid);
}

TEST_CASE(TestErrorConditions) {
    // Test con sensori disabilitati
    positionManager.enableOdometry(false);
    positionManager.enableIMU(false);
    positionManager.enableGPS(false);
    
    // Dovrebbe gestire l'assenza di sensori
    positionManager.update();
    
    // La posizione dovrebbe rimanere invariata (0,0)
    RobotPosition pos = positionManager.getPosition();
    TEST_ASSERT_EQUAL(0.0f, pos.x);
    TEST_ASSERT_EQUAL(0.0f, pos.y);
    
    // Test with GPS enabled
    positionManager.enableGPS(true);
    
    // Wait for GPS fix (if available)
    unsigned long startTime = millis();
    bool hasFix = false;
    while (millis() - startTime < 30000) {  // Wait up to 30 seconds for GPS fix
        gps->update();
        if (gps->hasFix()) {
            hasFix = true;
            break;
        }
        delay(100);
    }
    
    // Update position with GPS data
    positionManager.update();
    
    // Get and log the position
    RobotPosition pos2 = positionManager.getPosition();
    if (hasFix) {
        Serial.print("GPS Fix acquired. Position: ");
        Serial.print(pos2.x);
        Serial.print(", ");
        Serial.println(pos2.y);
    } else {
        Serial.println("No GPS fix available");
    }
}

void setup() {
    delay(1000); // Wait for serial connection
    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect
    
    // Run the tests
    run_tests();
}

void loop() {
    // I test vengono eseguiti una sola volta in setup()
    delay(1000);
}
