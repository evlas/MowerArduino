#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "../../config.h"  // Deve essere incluso per primi per definire ENABLE_PERIMETER

// Forward declaration per evitare dipendenze circolari
#ifdef ENABLE_PERIMETER
class PerimeterSensors;
#endif

#include "../motors/MotorController.h"
#include "../motors/BladeController.h"
#include "../sensors/UltrasonicSensors.h"
#include "../sensors/BumpSensors.h"
#include "Maneuver.h"
#include "../position/PositionManager.h"

// Definizione del modo di navigazione
class NavigationMode {
    public:
        enum Mode {
            RANDOM,      // Movimento casuale
            PARALLEL,    // Linee parallele
            SPIRAL,      // Spirale
            STOPPED      // Ferma
        } mode;
        
        NavigationMode() : mode(STOPPED) {}
        void setMode(Mode newMode) { mode = newMode; }
        Mode getMode() const { return mode; }
};

class Navigation {
    public:
#ifdef ENABLE_PERIMETER
        Navigation(Maneuver* maneuver, 
                 UltrasonicSensors* ultrasonic, 
                 BumpSensors* bumper, 
                 PerimeterSensors* perimeter,
                 PositionManager* positionManager);
#else
        Navigation(Maneuver* maneuver, 
                 UltrasonicSensors* ultrasonic, 
                 BumpSensors* bumper, 
                 void* perimeter,  // Dummy parameter when perimeter is disabled
                 PositionManager* positionManager);
#endif
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
        PositionManager _positionManager;
        Maneuver* _maneuver;
        UltrasonicSensors* _ultrasonic;
        BumpSensors* _bumper;
#ifdef ENABLE_PERIMETER
        PerimeterSensors* _perimeter;
#else
        void* _perimeter;  // Dummy pointer when perimeter is disabled
#endif
        
        // Stato
        NavigationMode _mode;
        bool _isNavigating;
        
        // Parametri di navigazione
        float _bladeWidth;
        float _currentDistance;
        float _targetDistance;
        float _currentAngle;
        float _targetAngle;
        int _currentRing;
        float _currentRadius;
        
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
