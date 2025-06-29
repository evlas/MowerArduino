/**
 * @file LCDMenu_Test.ino
 * @brief Test per la classe LCDMenu
 * 
 * Questo sketch testa tutte le funzionalità del menu LCD
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "LCDMenu.h"

// Crea un'istanza del menu LCD
LCDMenu lcdMenu;

// Variabili per il test
long lastUpdate = 0;
const long interval = 5000;  // Cambia schermata ogni 5 secondi
int testPhase = 0;

void setup() {
  // Inizializza la seriale per il debug
  Serial.begin(115200);
  while (!Serial);  // Attendi che la seriale sia priva
  
  Serial.println("Test LCDMenu - Inizializzazione...");
  
  // Inizializza il menu LCD
  lcdMenu.begin();
  
  Serial.println("Test LCDMenu - Pronto!");
  Serial.println("Premi i pulsanti per testare il menu");
}

void loop() {
  // Gestisci gli input dei pulsanti
  if (digitalRead(START_BUTTON_PIN) == LOW) {
    lcdMenu.handleButtonPress(START_BUTTON_PIN);
    Serial.println("Pulsante START premuto");
    delay(200);  // Debounce
  }
  
  if (digitalRead(PLUS_BUTTON_PIN) == LOW) {
    lcdMenu.handleButtonPress(PLUS_BUTTON_PIN);
    Serial.println("Pulsante + premuto");
    delay(200);  // Debounce
  }
  
  if (digitalRead(MINUS_BUTTON_PIN) == LOW) {
    lcdMenu.handleButtonPress(MINUS_BUTTON_PIN);
    Serial.println("Pulsante - premuto");
    delay(200);  // Debounce
  }
  
  if (digitalRead(STOP_BUTTON_PIN) == LOW) {
    lcdMenu.handleButtonPress(STOP_BUTTON_PIN);
    Serial.println("Pulsante STOP premuto");
    delay(200);  // Debounce
  }
  
  // Aggiorna il menu
  lcdMenu.update();
  
  // Test automatico che cicla tra le varie schermate
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdate >= interval) {
    lastUpdate = currentMillis;
    
    switch(testPhase) {
      case 0:
        Serial.println("Test: Menu Principale");
        // Simula pressione di tasti per navigare
        lcdMenu.handleButtonPress(PLUS_BUTTON_PIN);
        break;
        
      case 1:
        Serial.println("Test: Menu Taglio");
        lcdMenu.handleButtonPress(START_BUTTON_PIN);
        break;
        
      case 2:
        Serial.println("Test: Menu Manutenzione");
        lcdMenu.handleButtonPress(MINUS_BUTTON_PIN);
        lcdMenu.handleButtonPress(START_BUTTON_PIN);
        break;
        
      case 3:
        Serial.println("Test: Menu PID");
        lcdMenu.handleButtonPress(MINUS_BUTTON_PIN);
        lcdMenu.handleButtonPress(START_BUTTON_PIN);
        break;
        
      case 4:
        Serial.println("Test: Modifica PID");
        lcdMenu.handleButtonPress(START_BUTTON_PIN);
        break;
        
      default:
        testPhase = -1;  // Ricomincia il ciclo
        break;
    }
    
    testPhase++;
  }
}
