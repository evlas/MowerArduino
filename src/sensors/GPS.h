#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "../../config.h"

// Dati GPS
struct GPSData {
    bool isValid;
    float latitude;
    float longitude;
    float speed;    // in km/h
    float heading;  // in gradi
    int satellites;
};

class GPSModule {
    private:
        TinyGPSPlus gps;
        GPSData currentData;
        
    public:
        GPSModule() {
            currentData.isValid = false;
            currentData.latitude = 0;
            currentData.longitude = 0;
            currentData.speed = 0;
            currentData.heading = 0;
            currentData.satellites = 0;
        }
        
        void begin() {
            SERIAL_GPS.begin(SERIAL_GPS_BAUD);
        }
        
        void update() {
            while (SERIAL_GPS.available() > 0) {
                gps.encode(SERIAL_GPS.read());
            }
            
            if (gps.location.isValid()) {
                currentData.isValid = true;
                currentData.latitude = gps.location.lat();
                currentData.longitude = gps.location.lng();
                currentData.speed = gps.speed.kmph();
                currentData.heading = gps.course.deg();
                currentData.satellites = gps.satellites.value();
            } else {
                currentData.isValid = false;
            }
        }
        
        GPSData getData() const {
            return currentData;
        }
        
        // Debug
        void printDebug() const {
            if (currentData.isValid) {
                SERIAL_DEBUG.print("GPS: Lat:");
                SERIAL_DEBUG.print(currentData.latitude, 6);
                SERIAL_DEBUG.print(", Lon:");
                SERIAL_DEBUG.print(currentData.longitude, 6);
                SERIAL_DEBUG.print(", Speed:");
                SERIAL_DEBUG.print(currentData.speed, 1);
                SERIAL_DEBUG.print(" km/h, Satellites:");
                SERIAL_DEBUG.println(currentData.satellites);
            } else {
                SERIAL_DEBUG.println("GPS: No valid data");
            }
        }
};

extern GPSModule gps;

#endif // GPS_H
