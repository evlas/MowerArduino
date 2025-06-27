#include <Arduino.h>
#include "BatterySensor.h"
#include "config.h"
#include "pin_config.h"

extern BatterySensor batterySensor;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  
  if (!batterySensor.begin()) {
    Serial.println("Failed to initialize Battery Sensor!");
    while (1);
  }
  
  Serial.println("Battery Sensor Test");
  Serial.println("-------------------");
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 'v': // Read voltage
        printVoltage();
        break;
        
      case 'c': // Read current
        printCurrent();
        break;
        
      case 'p': // Read power
        printPower();
        break;
        
      case 's': // Read all
        printAll();
        break;
        
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

void printVoltage() {
  float voltage = batterySensor.readVoltage();
  Serial.print("Battery Voltage: ");
  Serial.print(voltage, 2);
  Serial.println("V");
  
  float cellVoltage = voltage / BATTERY_CELLS;
  Serial.print("Cell Voltage: ");
  Serial.print(cellVoltage, 2);
  Serial.println("V/cell");
  
  estimateCharge(voltage);
}

void printCurrent() {
  float current = batterySensor.readCurrent();
  Serial.print("Current: ");
  Serial.print(current, 2);
  Serial.println("A");
}

void printPower() {
  float voltage = batterySensor.readVoltage();
  float current = batterySensor.readCurrent();
  Serial.print("Power: ");
  Serial.print(voltage * current, 2);
  Serial.println("W");
}

void printAll() {
  float voltage = batterySensor.readVoltage();
  float current = batterySensor.readCurrent();
  
  Serial.println("\n--- Battery Status ---");
  Serial.print("Voltage: "); Serial.print(voltage, 2); Serial.println("V");
  Serial.print("Current: "); Serial.print(current, 2); Serial.println("A");
  Serial.print("Power: "); Serial.print(voltage * current, 2); Serial.println("W");
  estimateCharge(voltage);
  Serial.println("-------------------\n");
}

void estimateCharge(float voltage) {
  float soc = (voltage - BATTERY_EMPTY_VOLTAGE) / 
             (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE) * 100.0;
  soc = constrain(soc, 0, 100);
  
  Serial.print("State of Charge: ");
  Serial.print(soc, 1);
  Serial.println("%");
  
  Serial.print("Battery Status: ");
  if (voltage < BATTERY_CRITICAL_THRESHOLD) {
    Serial.println("CRITICAL - Charge immediately!");
  } else if (voltage < BATTERY_LOW_THRESHOLD) {
    Serial.println("LOW - Charge soon");
  } else if (voltage >= BATTERY_FULL_VOLTAGE) {
    Serial.println("FULLY CHARGED");
  } else {
    Serial.println("NORMAL");
  }
}

void printHelp() {
  Serial.println("\nBattery Sensor Test - Commands:");
  Serial.println("v - Read voltage and state of charge");
  Serial.println("c - Read current");
  Serial.println("p - Read power consumption");
  Serial.println("s - Read all battery parameters");
  Serial.println("h - Show this help");
}
