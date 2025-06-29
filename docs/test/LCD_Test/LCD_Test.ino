/**
 * @file LCD_Test.ino
 * @brief Test program for the LCD display used in the Mower project
 * 
 * This test program demonstrates the basic functionality of the LCD display
 * connected via I2C. It shows how to initialize the display and print text.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Create LCD object
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("LCD Test Starting...");
  
  // Initialize the LCD
  lcd.begin(2,16);
  lcd.backlight();
  
  // Print a message to the LCD
  lcd.setCursor(0, 0);
  lcd.print("Mower LCD Test");
  lcd.setCursor(0, 1);
  lcd.print("Hello, World!");
  
  Serial.println("LCD Initialized and message displayed");
}

void loop() {
  // Blink the backlight to show the program is running
  static unsigned long lastBlink = 0;
  static bool backlightState = true;
  
  if (millis() - lastBlink > 1000) { // Toggle every second
    backlightState = !backlightState;
    if (backlightState) {
      lcd.backlight();
    } else {
      lcd.noBacklight();
    }
    lastBlink = millis();
    
    // Update the second line with the current time
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(millis() / 1000);
    lcd.print("s   ");
  }
  
  // Small delay to prevent watchdog reset
  delay(10);
}
