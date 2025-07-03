#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Struttura per i dati GPS
struct GPSData {
    bool fix;               // true se il fix è valido
    double latitude;        // gradi
    double longitude;       // gradi
    double speed_mps;       // metri al secondo
    double speed_kmph;     // chilometri orari
    double course_deg;     // rotta in gradi (0-360)
    double altitude_m;     // metri
    double altitude_km;    // chilometri
    uint8_t satellites;    // numero di satelliti
    float hdop;            // precisione orizzontale
    uint16_t year;         // anno
    uint8_t month;         // mese (1-12)
    uint8_t day;           // giorno (1-31)
    uint8_t hour;          // ora (0-23)
    uint8_t minute;        // minuti (0-59)
    uint8_t second;        // secondi (0-59)
};

class GPSModule {
private:
    TinyGPSPlus gps;
    HardwareSerial* gpsSerial;
    mutable GPSData currentData;
    bool hasNewData;
    
    void updateData() {
        currentData.fix = gps.location.isValid();
        
        if (currentData.fix) {
            // Posizione
            currentData.latitude = gps.location.lat();
            currentData.longitude = gps.location.lng();
            
            // Velocità
            currentData.speed_mps = gps.speed.mps();
            currentData.speed_kmph = gps.speed.kmph();
            
            // Rotta
            currentData.course_deg = gps.course.deg();
            
            // Altitudine
            currentData.altitude_m = gps.altitude.meters();
            currentData.altitude_km = gps.altitude.kilometers();
            
            // Satelliti e precisione
            currentData.satellites = gps.satellites.value();
            currentData.hdop = gps.hdop.hdop();
            
            // Data e ora
            if (gps.date.isValid()) {
                currentData.year = gps.date.year();
                currentData.month = gps.date.month();
                currentData.day = gps.date.day();
            }
            
            if (gps.time.isValid()) {
                currentData.hour = gps.time.hour();
                currentData.minute = gps.time.minute();
                currentData.second = gps.time.second();
            }
        }
    }

public:
    GPSModule() : gpsSerial(nullptr), hasNewData(false) {
        // Inizializza la struttura dei dati
        currentData = {false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    }
    
    // Inizializza il modulo GPS
    void begin(HardwareSerial &serial = Serial1) {
        gpsSerial = &serial;
        gpsSerial->begin(9600);  // Imposta il baud rate corretto
    }
    
    // Deve essere chiamato nel loop() per elaborare i dati in arrivo
    void update() {
        if (!gpsSerial) return;
        
        while (gpsSerial->available() > 0) {
            char c = gpsSerial->read();
            if (gps.encode(c)) {
                hasNewData = true;
                updateData();
            }
        }
        
        if (gps.charsProcessed() < 10 && millis() > 5000) {
            currentData.fix = false;
        }
    }
    
    // Restituisce true se c'è un fix valido
    bool hasFix() const { return currentData.fix; }
    
    // --- Metodi di posizione ---
    double getLatitude() const { return currentData.latitude; }     // gradi decimali
    double getLongitude() const { return currentData.longitude; }   // gradi decimali
    
    // --- Metodi di velocità ---
    double getSpeedMPS() const { return currentData.speed_mps; }    // metri al secondo
    double getSpeedKMPH() const { return currentData.speed_kmph; }  // chilometri orari
    
    // --- Metodi di rotta ---
    double getCourse() const { return currentData.course_deg; }     // gradi (0-360)
    
    // --- Metodi di altitudine ---
    double getAltitudeMeters() const { return currentData.altitude_m; }     // metri
    double getAltitudeKilometers() const { return currentData.altitude_km; } // chilometri
    
    // --- Metodi di stato ---
    uint8_t getSatellites() const { return currentData.satellites; } // numero di satelliti
    float getHDOP() const { return currentData.hdop; }               // precisione orizzontale
    
    // --- Metodi di data/ora ---
    void getDateTime(uint16_t &year, uint8_t &month, uint8_t &day, 
                    uint8_t &hour, uint8_t &minute, uint8_t &second) const {
        year = currentData.year;
        month = currentData.month;
        day = currentData.day;
        hour = currentData.hour;
        minute = currentData.minute;
        second = currentData.second;
    }
    
    // Restituisce tutti i dati GPS in una struttura
    GPSData getData() const { return currentData; }
    
    // --- Metodi di utilità statici ---
    static double distanceBetween(double lat1, double long1, double lat2, double long2) {
        return TinyGPSPlus::distanceBetween(lat1, long1, lat2, long2);
    }
    
    static double courseTo(double lat1, double long1, double lat2, double long2) {
        return TinyGPSPlus::courseTo(lat1, long1, lat2, long2);
    }
    
    static const char* cardinal(double course) {
        return TinyGPSPlus::cardinal(course);
    }
};

extern GPSModule gps;

#endif // GPS_MODULE_H