#include <Arduino.h>
#include "UltrasonicSensors.h"
#include "config.h"
#include "pin_config.h"

// Create an instance of the UltrasonicSensors class
UltrasonicSensors ultrasonicSensors;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  
  // Initialize the ultrasonic sensors
  ultrasonicSensors.begin();
  
  Serial.println("Ultrasonic Sensors Test");
  Serial.println("----------------------");
  Serial.println("Commands:");
  Serial.println("r - Read all ultrasonic sensors");
  Serial.println("o - Check for obstacles");
  Serial.println("d - Get direction with most space");
  Serial.println("h - Show this help");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 'r': {
        // Read all three sensors
        float left, center, right;
        ultrasonicSensors.getAllDistances(left, center, right);
        
        Serial.println("\n--- Sensor Readings ---");
        Serial.print("Left:   ");
        if (left < MAX_ULTRASONIC_DISTANCE) {
          Serial.print(left);
          Serial.println(" cm");
        } else {
          Serial.println("Out of range");
        }
        
        Serial.print("Center: ");
        if (center < MAX_ULTRASONIC_DISTANCE) {
          Serial.print(center);
          Serial.println(" cm");
        } else {
          Serial.println("Out of range");
        }
        
        Serial.print("Right:  ");
        if (right < MAX_ULTRASONIC_DISTANCE) {
          Serial.print(right);
          Serial.println(" cm");
        } else {
          Serial.println("Out of range");
        }
        break;
      }
      
      case 'o': {
        // Check for obstacles
        bool obstacle = ultrasonicSensors.isObstacleDetected(OBSTACLE_CLEARANCE);
        Serial.print("\nObstacle detected: ");
        Serial.println(obstacle ? "YES" : "NO");
        break;
      }
      
      case 'd': {
        // Get direction with most space
        int direction = ultrasonicSensors.getDirectionWithMostSpace();
        Serial.print("\nDirection with most space: ");
        switch (direction) {
          case 0: Serial.println("LEFT"); break;
          case 1: Serial.println("CENTER"); break;
          case 2: Serial.println("RIGHT"); break;
        }
        break;
      }
      
      case 'h':
        Serial.println("\nUltrasonic Sensors Test - Commands:");
        Serial.println("r - Read all ultrasonic sensors");
        Serial.println("o - Check for obstacles");
        Serial.println("d - Get direction with most space");
        Serial.println("h - Show this help");
        break;
        
      default:
        Serial.println("\nUnknown command. Type 'h' for help.");
        break;
    }
  }
  
  delay(100);
}
