#include <Arduino.h>
#include "Relay.h"
#include "config.h"
#include "pin_config.h"

// Create relay instances
Relay motorRelay;
Relay bladeRelay;
Relay accessoryRelay;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  
  // Initialize the relays with their respective pins
  motorRelay.begin(RELAY_MOTORS_PIN);
  accessoryRelay.begin(CHARGING_RELAY_PIN);
  
  Serial.println("Relay Test");
  Serial.println("-----------");
  printStatus();
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      // Controllo relay motori
      case 'm':
        motorRelay.toggle();
        Serial.print("Motors relay: ");
        Serial.println(motorRelay.isOn() ? "ON" : "OFF");
        break;
        
      // Controllo relay accessori
      case 'a':
        accessoryRelay.toggle();
        Serial.print("Accessory relay: ");
        Serial.println(accessoryRelay.isOn() ? "ON" : "OFF");
        break;
        
      // Accendi tutto
      case '1':
        motorRelay.on();
        bladeRelay.on();
        accessoryRelay.on();
        Serial.println("All relays ON");
        break;
        
      // Spegni tutto
      case '0':
        motorRelay.off();
        bladeRelay.off();
        accessoryRelay.off();
        Serial.println("All relays OFF");
        break;
        
      // Stato attuale
      case 's':
        printStatus();
        break;
        
      // Aiuto
      case 'h':
        printHelp();
        break;
        
      default:
        Serial.println("Unknown command. Type 'h' for help.");
        break;
    }
  }
  
  delay(100);
}

void printStatus() {
  Serial.println("\nCurrent Relay Status:");
  Serial.println("-------------------");
  Serial.print("1. Motors:  ");
  Serial.println(motorRelay.isOn() ? "ON" : "OFF");
  
  Serial.print("3. Accessory:");
  Serial.println(accessoryRelay.isOn() ? "ON" : "OFF");
  Serial.println();
}

void printHelp() {
  Serial.println("\nRelay Test - Commands:");
  Serial.println("m - Toggle Motors relay");
  Serial.println("a - Toggle Accessory relay");
  Serial.println("1 - Turn ALL relays ON");
  Serial.println("0 - Turn ALL relays OFF");
  Serial.println("s - Show current status");
  Serial.println("h - Show this help");
  Serial.println("\nNote: Make sure the relay pins are correctly defined in pin_config.h");
}
