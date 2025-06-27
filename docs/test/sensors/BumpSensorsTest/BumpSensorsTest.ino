#include <Arduino.h>
#include "BumpSensors.h"
#include "config.h"
#include "pin_config.h"

BumpSensors bumpSensors;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;  // Wait for serial port to connect

  bumpSensors.begin();

  Serial.println("Bump Sensors Test");
  Serial.println("-----------------");
}

void loop() {
  int direzione = -1;
  if (Serial.available() > 0) {

    //  getAllBumpStatus();
    direzione = bumpSensors.getBumpDirection();
    if (direzione == 0) {
      Serial.println("Sinistra");

    } else if (direzione == 1) {
      Serial.println("Destra");

    } else if (direzione == 2) {
      Serial.println("Centro");
    } else {
      Serial.println("No BUMP");
    };  // Returns: 0=left, 1=right, 2=center, -1=no bump
    delay(100);
  }
}
