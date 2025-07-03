#include <AUnit.h>
#include <Arduino.h>

// Include the Mower class and its dependencies
#include "../../MowerArduino/src/functions/Mower.h"
#include "../../MowerArduino/src/LCD/LCDMenu.h"

// Mock LCDMenu for testing
class MockLCDMenu : public LCDMenu {
public:
  void begin() override {}
  void update() override {}
  void clear() override {}
  void setCursor(uint8_t col, uint8_t row) override {}
  void print(const String& text) override {}
  void print(int number) override {}
  void setMower(Mower* mower) override {}
};

// Test fixture
class MowerTest : public aunit::TestOnce {
protected:
  MockLCDMenu lcd;
  Mower mower{&lcd};
  
  void setup() override {
    TestOnce::setup();
    // Initialize mower for testing
    mower.begin();
  }
};

testF(MowerTest, InitialStateShouldBeIdle) {
  assertEqual(static_cast<int>(mower.getState()), static_cast<int>(State::IDLE));
}

testF(MowerTest, ShouldTransitionToMowingState) {
  // When
  mower.handleEvent(Event::START_MOWING);
  
  // Then
  assertEqual(static_cast<int>(mower.getState()), static_cast<int>(State::MOWING));
}

testF(MowerTest, ShouldTransitionToDockingWhenBatteryLow) {
  // When
  mower.handleEvent(Event::BATTERY_LOW);
  
  // Then
  assertEqual(static_cast<int>(mower.getState()), static_cast<int>(State::DOCKING));
}

// Test runner setup
void setup() {
  delay(1000); // Wait for stability on some boards
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  
  Serial.println("Starting Mower Tests...");
}

void loop() {
  aunit::TestRunner::run();
  
  // Exit after tests complete
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'r') {
      // Reset tests if 'r' is received
      aunit::TestRunner::setTimeout(0);
    }
  }
}
