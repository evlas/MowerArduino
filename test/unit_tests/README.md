# Unit Tests for MowerArduino

This directory contains unit tests for the MowerArduino project using the AUnit testing framework.

## Test Structure

- `MowerTest/` - Tests for the main Mower class
- `BatterySensorTest/` - Tests for battery monitoring
- `DriveMotorTest/` - Tests for motor control
- `MowingStateTest/` - Tests for the mowing state machine

## Running Tests

### Using Arduino IDE

1. Open one of the test files (e.g., `MowerTest.ino`) in the Arduino IDE
2. Select the correct board: `Tools > Board > Arduino Mega or Mega 2560`
3. Select the correct port
4. Click the "Upload" button
5. Open the Serial Monitor at 115200 baud to see test results

### Using arduino-cli

```bash
# Navigate to the test directory
cd /path/to/MowerArduino/test/unit_tests/MowerTest

# Compile and upload
arduino-cli compile --fqbn arduino:avr:mega .
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega .

# Monitor serial output
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

## Adding New Tests

1. Create a new directory for your test
2. Create an `.ino` file with the test code
3. Follow the pattern of existing tests
4. Include necessary mocks for dependencies
5. Add test cases using the `test()` or `testF()` macros

## Test Guidelines

- Keep tests focused on a single unit of functionality
- Use descriptive test names
- Mock external dependencies
- Test both success and error cases
- Keep tests independent of each other
- Document any special setup required for the tests
