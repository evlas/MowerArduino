#include <Arduino.h>
#include "config.h"
#include "pin_config.h"
#include "Buzzer.h"

// Variable to track buzzer state
bool isBuzzerOn = false;

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  
  Serial.println("Buzzer Test");
  Serial.println("------------");
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case '1': // Beep singolo
        beep(100);
        Serial.println("Single beep");
        break;
        
      case '2': // Doppio beep
        beep(50);
        delay(100);
        beep(50);
        Serial.println("Double beep");
        break;
        
      case '3': // Triplo beep
        beep(30);
        delay(70);
        beep(30);
        delay(70);
        beep(30);
        Serial.println("Triple beep");
        break;
        
      case '4': // Sirena
        siren(3, 1000);
        Serial.println("Siren");
        break;
        
      case 't': // Toggle continuo
        isBuzzerOn = !isBuzzerOn;
        digitalWrite(BUZZER_PIN, isBuzzerOn ? HIGH : LOW);
        Serial.print("Buzzer ");
        Serial.println(isBuzzerOn ? "ON" : "OFF");
        break;
        
      case 's': // Stop
        digitalWrite(BUZZER_PIN, LOW);
        isBuzzerOn = false;
        Serial.println("Buzzer stopped");
        break;
        
      case 'h':
        printHelp();
        break;
        
      default:
        Serial.println("Unknown command. Type 'h' for help.");
        break;
    }
  }
}

void beep(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

void siren(int cycles, int duration) {
  for (int i = 0; i < cycles; i++) {
    // Frequenza crescente
    for (int freq = 500; freq <= 1500; freq += 50) {
      tone(BUZZER_PIN, freq);
      delay(10);
    }
    // Frequenza decrescente
    for (int freq = 1500; freq >= 500; freq -= 50) {
      tone(BUZZER_PIN, freq);
      delay(10);
    }
  }
  noTone(BUZZER_PIN);
}

void printHelp() {
  Serial.println("\nBuzzer Test - Commands:");
  Serial.println("1 - Single beep");
  Serial.println("2 - Double beep");
  Serial.println("3 - Triple beep");
  Serial.println("4 - Siren (3 cycles)");
  Serial.println("t - Toggle continuous tone");
  Serial.println("s - Stop buzzer");
  Serial.println("h - Show this help");
}
