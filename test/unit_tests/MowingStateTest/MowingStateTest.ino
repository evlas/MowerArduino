#include <AUnit.h>
#include <Arduino.h>
#include "../../MowerArduino/src/states/MowingState.h"
#include "../../MowerArduino/src/functions/Mower.h"

// Mock Mower class for testing
class MockMower : public Mower {
public:
  using Mower::Mower; // Inherit constructors
  
  // Override methods as needed for testing
  void setLeftMotorSpeed(int16_t speed) override {
    lastLeftSpeed = speed;
  }
  
  void setRightMotorSpeed(int16_t speed) override {
    lastRightSpeed = speed;
  }
  
  void startBlades() override {
    bladesStarted = true;
  }
  
  void stopBlades() override {
    bladesStopped = true;
  }
  
  // Test helpers
  int16_t lastLeftSpeed = 0;
  int16_t lastRightSpeed = 0;
  bool bladesStarted = false;
  bool bladesStopped = false;
};

// Test fixture
class MowingStateTest : public aunit::TestOnce {
protected:
  MockLCDMenu lcd;
  MockMower mower{&lcd};
  MowingState mowingState{mower};
  
  void setup() override {
    TestOnce::setup();
    mower.setState(&mowingState);
  }
};

testF(MowingStateTest, ShouldStartBladesOnEnter) {
  mowingState.enter();
  assertTrue(mower.bladesStarted);
}

testF(MowingStateTest, ShouldMoveForwardOnUpdate) {
  mowingState.update();
  assertNotEqual(mower.lastLeftSpeed, 0);
  assertNotEqual(mower.lastRightSpeed, 0);
}

testF(MowingStateTest, ShouldStopBladesOnExit) {
  mowingState.exit();
  assertTrue(mower.bladesStopped);
}

// Test runner setup
void setup() {
  delay(1000);
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("Starting MowingState Tests...");
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
