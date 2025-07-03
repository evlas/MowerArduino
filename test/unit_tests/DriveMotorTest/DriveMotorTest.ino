#include <AUnit.h>
#include <Arduino.h>
#include "../../MowerArduino/src/motors/DriveMotor/DriveMotor.h"

// Test fixture for DriveMotor
class DriveMotorTest : public aunit::TestOnce {
protected:
  // Use the same pin numbers as in your actual configuration
  static const uint8_t PIN_PWM = 5;
  static const uint8_t PIN_IN1 = 6;
  static const uint8_t PIN_IN2 = 7;
  
  DriveMotor motor;
  
  void setup() override {
    TestOnce::setup();
    motor.initialize(PIN_PWM, PIN_IN1, PIN_IN2);
  }
  
  void teardown() override {
    motor.stop();
    TestOnce::teardown();
  }
};

testF(DriveMotorTest, ShouldInitializeCorrectly) {
  assertTrue(motor.isInitialized());
}

testF(DriveMotorTest, ShouldMoveForward) {
  motor.setSpeed(100); // 100% speed forward
  assertEqual(motor.getCurrentSpeed(), 100);
  // Note: In a real test, you might want to verify pin states here
}

testF(DriveMotorTest, ShouldMoveBackward) {
  motor.setSpeed(-50); // 50% speed backward
  assertEqual(motor.getCurrentSpeed(), -50);
}

testF(DriveMotorTest, ShouldStop) {
  motor.setSpeed(75);
  motor.stop();
  assertEqual(motor.getCurrentSpeed(), 0);
}

// Test runner setup
void setup() {
  delay(1000);
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("Starting DriveMotor Tests...");
}

void loop() {
  aunit::TestRunner::run();
  
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'r') {
      aunit::TestRunner::setTimeout(0);
    }
  }
}
