#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "../../config.h"  // Deve essere incluso per primi per definire ENABLE_PERIMETER

// Forward declarations
class MotorController;
class BladeController;
class Maneuver;
class PositionManager;

// Forward declarations for sensors
class UltrasonicSensors;
class BumpSensors;
class PerimeterSensors;


// Definizione del modo di navigazione
class NavigationMode {
    public:
        enum Mode {
            RANDOM,      // Movimento casuale
            PARALLEL,    // Linee parallele
            SPIRAL,      // Spirale
            STOPPED,     // Ferma
            MANUAL       // Controllo manuale
        } mode;
        
        NavigationMode() : mode(STOPPED) {}
        void setMode(Mode newMode) { mode = newMode; }
        Mode getMode() const { return mode; }
};

class Navigation {
    public:
        Navigation(Maneuver* maneuver, 
#ifdef ENABLE_ULTRASONIC
                 UltrasonicSensors* ultrasonic,
#else
                 void* ultrasonic,
#endif
#ifdef ENABLE_BUMP_SENSORS
                 BumpSensors* bumper,
#else
                 void* bumper,
#endif
#ifdef ENABLE_PERIMETER
                 PerimeterSensors* perimeter,
#else
                 void* perimeter,
#endif
                 PositionManager* positionManager);
        ~Navigation();
        
        // Inizializzazione
        void begin();
        
        // Aggiornamento dello stato
        void update();
        
        // Gestione del modo di navigazione
        void setMode(NavigationMode::Mode mode);
        NavigationMode::Mode getMode() const;
        
        // Stato della navigazione
        bool isNavigating() const;
        
    private:
        // Componenti
        PositionManager* _positionManager;
        Maneuver* _maneuver;
        UltrasonicSensors* _ultrasonic;
        BumpSensors* _bumper;
        PerimeterSensors* _perimeter;
        
        // Stato
        NavigationMode _mode;
        bool _isNavigating;
        
        // Parametri di navigazione
        float _currentDistance;
        float _targetDistance;
        float _currentAngle;
        float _targetAngle;
        
        // Funzioni di navigazione
        void navigateRandom();
        void navigateParallel();
        void navigateSpiral();
        
        // Gestione degli ostacoli
        bool checkObstacles();
        void handleObstacle();
        
        // Gestione del perimetro (sempre disponibile, ma controlla _perimeter != nullptr)
        bool checkPerimeter();
        void handlePerimeter();
};

extern NavigationMode navigationMode;
extern Navigation navigation;

#endif // NAVIGATION_H
