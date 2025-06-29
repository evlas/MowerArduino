#include <Arduino.h>
#include "DriveMotor.h"
#include "config.h"
#include "pin_config.h"

// Create motor instances using the pin definitions from pin_config.h
// Parameters: pwmPin, dirPin, encoderPin, wheelDiameter, pulsesPerRevolution, reversed
DriveMotor leftMotor(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_ENCODER_PIN,
                    WHEEL_DIAMETER, ENCODER_PULSES_PER_REV, MOTOR_LEFT_REVERSED);

DriveMotor rightMotor(MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_ENCODER_PIN,
                     WHEEL_DIAMETER, ENCODER_PULSES_PER_REV, MOTOR_RIGHT_REVERSED);

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  
  leftMotor.begin();
  rightMotor.begin();
  
  Serial.println("Drive Motors Test");
  Serial.println("-----------------");
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 's': // Stop
        leftMotor.stop();
        rightMotor.stop();
        Serial.println("Motors stopped");
        break;
        
      case 'f': // Forward
        leftMotor.setSpeed(100);
        rightMotor.setSpeed(100);
        Serial.println("Moving forward");
        break;
        
      case 'b': // Backward
        leftMotor.setSpeed(-100);
        rightMotor.setSpeed(-100);
        Serial.println("Moving backward");
        break;
        
      case 'l': // Left
        leftMotor.setSpeed(-50);
        rightMotor.setSpeed(50);
        Serial.println("Turning left");
        break;
        
      case 'r': // Right
        leftMotor.setSpeed(50);
        rightMotor.setSpeed(-50);
        Serial.println("Turning right");
        break;
        
      case '1': // Speed 25%
        leftMotor.setSpeed(25);
        rightMotor.setSpeed(25);
        Serial.println("Speed set to 25%");
        break;
        
      case '2': // Speed 50%
        leftMotor.setSpeed(50);
        rightMotor.setSpeed(50);
        Serial.println("Speed set to 50%");
        break;
        
      case '3': // Speed 75%
        leftMotor.setSpeed(75);
        rightMotor.setSpeed(75);
        Serial.println("Speed set to 75%");
        break;
        
      case '4': // Speed 100%
        leftMotor.setSpeed(100);
        rightMotor.setSpeed(100);
        Serial.println("Speed set to 100%");
        break;
        
      case 'e': // Encoder values
        Serial.print("Left encoder: ");
        Serial.print(leftMotor.getEncoderCount());
        Serial.print(", Right encoder: ");
        Serial.println(rightMotor.getEncoderCount());
        break;
        
      case 'h':
        printHelp();
        break;
        
      default:
        Serial.println("Unknown command. Type 'h' for help.");
        break;
    }
  }
  
  // Update motor controllers
  leftMotor.update();
  rightMotor.update();
  
  delay(100);
}

void printHelp() {
  Serial.println("\nDrive Motors Test - Commands:");
  Serial.println("s - Stop");
  Serial.println("f - Move forward");
  Serial.println("b - Move backward");
  Serial.println("l - Turn left (in place)");
  Serial.println("r - Turn right (in place)");
  Serial.println("1 - Set speed to 25%");
  Serial.println("2 - Set speed to 50%");
  Serial.println("3 - Set speed to 75%");
  Serial.println("4 - Set speed to 100%");
  Serial.println("e - Show encoder values");
  Serial.println("h - Show this help");
}
