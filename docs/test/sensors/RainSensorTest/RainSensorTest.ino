#include <Arduino.h>
#include "RainSensor.h"
#include "config.h"
#include "pin_config.h"

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  
  // Initialize the rain sensor
  rainSensor.begin();
  
  Serial.println("Rain Sensor Test");
  Serial.println("----------------");
  Serial.println("Commands:");
  Serial.println("r - Read rain sensor");
  Serial.println("h - Show this help");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 'r':
        Serial.print("Is Raining: ");
        Serial.println(rainSensor.isRaining() ? "YES" : "NO");
        Serial.print("Raw Value: ");
        Serial.println(analogRead(RAIN_SENSOR_PIN));
        break;
        
      case 'h':
        Serial.println("\nRain Sensor Test - Commands:");
        Serial.println("r - Read rain sensor");
        Serial.println("h - Show this help");
        break;
        
      default:
        Serial.println("Unknown command. Type 'h' for help.");
        break;
    }
  }
  
  delay(100);
}
