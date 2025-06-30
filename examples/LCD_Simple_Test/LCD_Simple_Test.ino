/**
 * @file LCD_Simple_Test.ino
 * @brief Test semplificato per LCD I2C
 * 
 * Questo sketch testa direttamente la libreria LiquidCrystal_I2C
 * per verificare se il problema è hardware o software
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Inizializza l'LCD con indirizzo I2C 0x27, 16 caratteri per 2 righe
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

void setup() {
  // Inizializza la seriale
  Serial.begin(115200);
  Serial.println("suca Test LCD avviato");
  // Inizializza I2C
  Wire.begin();
  
  // Inizializza LCD
  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();
  
  // Stampa un messaggio di test
  lcd.setCursor(0, 0);
  lcd.print("LCD Test");
  lcd.setCursor(0, 1);
  lcd.print("Hello World!");
  
  Serial.println("Test LCD avviato");
}

void loop() {
  // Non fare nulla nel loop
  delay(1000);
}
