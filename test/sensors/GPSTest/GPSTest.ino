#include <Arduino.h>
#include "GPSModule.h"
#include "config.h"
#include "pin_config.h"


// Crea l'istanza del GPS
GPSModule gps;

void setup() {
  // Inizializza la porta seriale di debug
  Serial.begin(115200);
  while (!Serial && millis() < 2000)
    ;  // Attendi la seriale o massimo 2 secondi

  // Inizializza la porta seriale del GPS

  SERIAL_GPS.begin(SERIAL_GPS_BAUD);

  // Inizializza il modulo GPS
  gps.begin();

  Serial.println("GPS Test");
  Serial.println("--------");
  Serial.print("Baudrate: ");
  Serial.println(SERIAL_GPS_BAUD);
  Serial.println("----------------");

  printHelp();
}

void loop() {
  gps.update();

  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case 'p':  // Print position
        printPosition();
        break;

      case 's':  // Print satellite info
        printSatelliteInfo();
        break;

      case 'r':  // Print raw NMEA
        printRawNMEA();
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

void printPosition() {
  GPSData data = gps.getData();
  if (data.fix) {
    Serial.println("\n--- GPS Position ---");
    Serial.print("Latitude:  ");
    Serial.println(data.latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(data.longitude, 6);
    Serial.print("Speed:     ");
    Serial.print(data.speed_kmph, 1);
    Serial.println(" km/h");
    Serial.print("Course:    ");
    Serial.print(data.course_deg, 1);
    Serial.println("°");
    Serial.print("Satellites:");
    Serial.println(data.satellites);
  } else {
    Serial.println("\nNo GPS fix available");
  }
}

void printSatelliteInfo() {
  GPSData data = gps.getData();
  Serial.println("\n--- Satellite Info ---");
  Serial.print("Satellites in view: ");
  Serial.println(data.satellites);

  // TinyGPS++ non fornisce informazioni dettagliate sui singoli satelliti
  // in modo diretto attraverso la nostra interfaccia
  Serial.println("\nDetailed satellite info not available with current configuration.");
}

void printRawNMEA() {
  Serial.println("\n--- Raw NMEA Data (10s) ---");
  Serial.println("(Press any key to stop)");

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    if (Serial1.available()) {
      char c = Serial1.read();
      Serial.write(c);
    }
    if (Serial.available()) {
      while (Serial.available()) Serial.read();  // Clear buffer
      break;
    }
  }
  Serial.println("\n--- End of Raw Data ---");
}

void printHelp() {
  Serial.println("\nGPS Test - Commands:");
  Serial.println("p - Print current position");
  Serial.println("s - Print satellite information");
  Serial.println("r - Print raw NMEA data (10s)");
  Serial.println("h - Show this help");
}
