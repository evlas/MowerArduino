// Simple Arduino sketch to exercise WiFiRemote without real mower HW
#include <Arduino.h>
#include "src/config.h"
#include "src/functions/Mower.h"
#include "src/communications/WiFiRemote.h"

Mower mower;
WiFiRemote wifi(mower);

void setup() {
  Serial.begin(115200);
  wifi.setRemoteControlEnabled(true);

  // Send a few test commands
  CommandData data; data.speed = 0.5f;
  wifi.processRemoteCommand(RemoteCommandType::MOVE_FORWARD, data);
  delay(1000);
  wifi.processRemoteCommand(RemoteCommandType::TURN_LEFT, data);
  delay(1000);
  wifi.processRemoteCommand(RemoteCommandType::STOP_MOWING);

  // Stampa stato base
  RemoteStatus st; wifi.getRemoteStatus(st);
  Serial.print(F("Battery: ")); Serial.println(st.batteryLevel);
}

void loop() {
  // nothing
}
