#include <AUnit.h>
#include <Arduino.h>
#include <Wire.h>
#include "src/sensors/BatterySensor/BatterySensor.h"

// Mock I2C for testing
class MockWire {
public:
  void begin() {}
  void beginTransmission(uint8_t) {}
  uint8_t endTransmission() { return 0; }
  size_t write(uint8_t) { return 1; }
  size_t write(const uint8_t*, size_t) { return 1; }
  uint8_t requestFrom(uint8_t, uint8_t) { return 1; }
  int available() { return 1; }
  int read() { return 0; }
};

// Create a mock I2C instance for testing
MockWire WireMock;

// Override the Wire object used by INA226
#undef Wire
#define Wire WireMock

// Test fixture for BatterySensor
class BatterySensorTest : public aunit::TestOnce {
protected:
  BatterySensor batterySensor;
  
  void setup() override {
    TestOnce::setup();
    // Initialize with mock I2C
    batterySensor = BatterySensor();
    batterySensor.begin();
  }
};

testF(BatterySensorTest, ShouldInitializeCorrectly) {
  // The begin() method returns a boolean indicating success
  bool initialized = batterySensor.begin();
  assertTrue(initialized);
}

testF(BatterySensorTest, ShouldReturnValidVoltage) {
  // Test with a mock voltage reading
  float voltage = batterySensor.readVoltage();
  // Voltage should be within a reasonable range (0-20V)
  assertMoreOrEqual(voltage, 0.0);
  assertLessOrEqual(voltage, 20.0);
}

testF(BatterySensorTest, ShouldReturnValidCurrent) {
  // Test with a mock current reading
  float current = batterySensor.readCurrent();
  // Current could be positive (discharging) or negative (charging)
  assertMoreOrEqual(current, -10.0);
  assertLessOrEqual(current, 10.0);
}

// Test runner setup
void setup() {
  delay(1000);
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("Starting BatterySensor Tests...");
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
