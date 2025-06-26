#include <Arduino.h>
#include "BladeMotor.h"
#include "pin_config.h"

BladeMotor bladeMotor(BLADE_MOTOR_PWM_PIN, BLADE_MOTOR_DIR_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  
  bladeMotor.begin();
  
  Serial.println("Blade Motor Test");
  Serial.println("----------------");
  Serial.println("Commands:");
  Serial.println("s - Stop motor");
  Serial.println("1 - Set speed to 25%");
  Serial.println("2 - Set speed to 50%");
  Serial.println("3 - Set speed to 75%");
  Serial.println("4 - Set speed to 100%");
  Serial.println("f - Toggle direction");
  Serial.println("h - Show this help");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 's':
        bladeMotor.stop();
        Serial.println("Motor stopped");
        break;
        
      case '1':
        bladeMotor.setSpeed(64); // 25% of 255
        Serial.println("Speed set to 25%");
        break;
        
      case '2':
        bladeMotor.setSpeed(128); // 50% of 255
        Serial.println("Speed set to 50%");
        break;
        
      case '3':
        bladeMotor.setSpeed(192); // 75% of 255
        Serial.println("Speed set to 75%");
        break;
        
      case '4':
        bladeMotor.setSpeed(255); // 100% of 255
        Serial.println("Speed set to 100%");
        break;
        
      case 'f':
        bladeMotor.setDirection(!bladeMotor.getDirection());
        Serial.print("Direction changed to: ");
        Serial.println(bladeMotor.getDirection() ? "FORWARD" : "REVERSE");
        break;
        
      case 'h':
        printHelp();
        break;
        
      default:
        Serial.println("Unknown command. Type 'h' for help.");
        break;
    }
  }
  
  bladeMotor.update();
  delay(100);
}

void printHelp() {
  Serial.println("\nBlade Motor Test - Commands:");
  Serial.println("s - Stop motor");
  Serial.println("1 - Set speed to 25%");
  Serial.println("2 - Set speed to 50%");
  Serial.println("3 - Set speed to 75%");
  Serial.println("4 - Set speed to 100%");
  Serial.println("f - Toggle direction");
  Serial.println("h - Show this help");
}
