#include "Navigation.h"
#include <Arduino.h>

// Struttura per la lettura IMU
#ifdef ENABLE_IMU
struct IMUReading {
    float yaw;
    float pitch;
    float roll;
    bool isValid;
};
#endif

#include "../motors/MotorController.h"
#include "../position/PositionManager.h"
#include "Maneuver.h"

// Include condizionali dei sensori
#ifdef ENABLE_ULTRASONIC
#include "../sensors/UltrasonicSensors.h"
#endif

#ifdef ENABLE_BUMP_SENSORS
#include "../sensors/BumpSensors.h"
#endif

// Includi PerimeterSensors.h all'inizio per garantire che la classe sia completamente definita
// quando viene usata nei metodi
#ifdef ENABLE_PERIMETER
#include "../sensors/PerimeterSensors.h"
#endif

Navigation::Navigation(Maneuver* maneuver, 
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
                     PositionManager* positionManager) :
    _positionManager(positionManager),
    _maneuver(maneuver),
    _ultrasonic(reinterpret_cast<UltrasonicSensors*>(ultrasonic)),
    _bumper(reinterpret_cast<BumpSensors*>(bumper)),
    _perimeter(reinterpret_cast<PerimeterSensors*>(perimeter)),
    _mode(),
    _isNavigating(false),
    _currentDistance(0.0f),
    _targetDistance(0.0f),
    _currentAngle(0.0f),
    _targetAngle(0.0f) {
    // Inizializza PositionManager se disponibile
    if (_positionManager != nullptr) {
        _positionManager->begin();
        _positionManager->enableOdometry(true);
        _positionManager->enableIMU(true);
        _positionManager->enableGPS(true);
    }
}

void Navigation::begin() {
    _mode.setMode(NavigationMode::STOPPED);
    _isNavigating = false;
    _currentDistance = 0.0f;
    _currentAngle = 0.0f;

    // Inizializza i sensori se disponibili
#ifdef ENABLE_ULTRASONIC
    if (_ultrasonic != nullptr) {
        _ultrasonic->begin();
    }
#endif

#ifdef ENABLE_BUMP_SENSORS
    if (_bumper != nullptr) {
        _bumper->begin();
    }
#endif

#ifdef ENABLE_PERIMETER
    if (_perimeter != nullptr) {
        _perimeter->begin();
    }
#endif
}

Navigation::~Navigation() {
    // We don't delete _positionManager here as we don't own it
    // The owner of Navigation should handle the PositionManager's lifetime
}

void Navigation::update() {
    // Controlla ostacoli e perimetro
    bool obstacleDetected = checkObstacles();
    bool perimeterBreached = (_perimeter != nullptr) ? checkPerimeter() : false;
    
    if (obstacleDetected || perimeterBreached) {
        handleObstacle();
        return;
    }
    
    // Controlla l'errore di direzione se abbiamo un target angolare
    if (_positionManager != nullptr && _isNavigating) {
        // Ottieni la posizione corrente dal PositionManager
        RobotPosition currentPos = _positionManager->getPosition();
        float currentHeading = currentPos.theta;  // theta è già in radianti
        bool hasHeading = currentPos.isValid;
        
        // Se non abbiamo una posizione valida, usiamo una stima basata sull'odometria
        if (!hasHeading && _maneuver != nullptr) {
            // Stima approssimativa basata sull'odometria
            static float estimatedHeading = 0.0f;
            float angularVelocity = _maneuver->getAngularVelocity();
            static unsigned long lastUpdate = millis();
            unsigned long currentTime = millis();
            float deltaTime = (currentTime - lastUpdate) / 1000.0f; // in secondi
            lastUpdate = currentTime;
            
            estimatedHeading += angularVelocity * deltaTime;
            // Normalizza tra -PI e PI
            while (estimatedHeading > PI) estimatedHeading -= TWO_PI;
            while (estimatedHeading < -PI) estimatedHeading += TWO_PI;
            
            currentHeading = estimatedHeading;
            hasHeading = true;
        }
        
        if (hasHeading) {
            float headingError = atan2(sin(_targetAngle - currentHeading), 
                                    cos(_targetAngle - currentHeading));
                                      
            if (abs(headingError) > HEADING_TOLERANCE) {
                // Correggi la direzione con una rotazione proporzionale all'errore
                float rotationSpeed = constrain(headingError * 0.5f, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
                if (_maneuver != nullptr) {
                    // Converti la velocità angolare in un valore di velocità percentuale (0-100)
                    int speedPercent = static_cast<int>(map(abs(rotationSpeed), 0, MAX_ANGULAR_SPEED, 20, 80));
                    _maneuver->rotate(rotationSpeed > 0 ? 90 : -90, speedPercent);
                }
                return;  // Aspetta che la direzione sia corretta
            }
        }
    }
    
    // Se non stiamo navigando, esce
    if (!_isNavigating) {
        return;
    }
    
    // Esegue il modo di navigazione corrente
    switch (_mode.getMode()) {
        case NavigationMode::RANDOM:
            navigateRandom();
            break;
        case NavigationMode::PARALLEL:
            navigateParallel();
            break;
        case NavigationMode::SPIRAL:
            navigateSpiral();
            break;
        case NavigationMode::STOPPED:
            _maneuver->stop();
            break;
        case NavigationMode::MANUAL:
            // In modalità manuale, il movimento è gestito direttamente dal CommandHandler
            // Non facciamo nulla qui per non sovrascrivere i comandi manuali
            break;
    }
}

void Navigation::setMode(NavigationMode::Mode mode) {
    _mode.setMode(mode);
}

NavigationMode::Mode Navigation::getMode() const {
    return _mode.getMode();
}

bool Navigation::isNavigating() const {
    return _isNavigating;
}

// Navigazione casuale
void Navigation::navigateRandom() {
    if (!_isNavigating) {
        // Imposta una distanza casuale
        _targetDistance = random(RANDOM_MAX_DISTANCE) / 100.0f; // in metri
        _isNavigating = true;
    }
    
    // Muovi in linea retta con velocità default
    _maneuver->forward(DEFAULT_MOTOR_SPEED); // Velocità default dal config.h
    
    // Aggiorna la distanza percorsa usando la velocità lineare
    _currentDistance += _maneuver->getLinearVelocity() / 100.0f; // converti cm/s in m/s
    
    if (_currentDistance >= _targetDistance) {
        // Quando arrivi alla fine, ferma e prepara per la prossima virata
        _maneuver->stop();
        
        // Calcola un angolo casuale
        _targetAngle = random(RANDOM_TURN_MIN_ANGLE, RANDOM_TURN_MAX_ANGLE);
        
        // Gira di quell'angolo con velocità media (50%)
        _maneuver->rotate(static_cast<int>(_targetAngle), 50);
        
        // Resetta la distanza
        _currentDistance = 0.0f;
        _isNavigating = false;
    }
}

// Navigazione a linee parallele
void Navigation::navigateParallel() {
    // Naviga in linea retta a velocità ridotta quando ci si avvicina alla fine
    if (_positionManager != nullptr && _maneuver != nullptr) {
        // Stima la distanza percorsa in base al tempo e alla velocità
        static unsigned long lastUpdate = millis();
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastUpdate) / 1000.0f; // in secondi
        lastUpdate = currentTime;
        
        // Usa la velocità media dei due motori se disponibile
        float currentSpeed = 0.0f;
        if (_maneuver != nullptr) {
            float leftSpeed = _maneuver->getLeftSpeed();
            float rightSpeed = _maneuver->getRightSpeed();
            currentSpeed = (leftSpeed + rightSpeed) * 0.5f; // Velocità media
        }
        _currentDistance += currentSpeed * deltaTime;
        
        float distanceToTarget = abs(_currentDistance - _targetDistance);
        float speed = MAX_LINEAR_SPEED * 0.5f;  // 50% della velocità massima
        
        // Rallenta in prossimità del punto di svolta
        if (distanceToTarget < APPROACH_DISTANCE) {
            speed *= (distanceToTarget / APPROACH_DISTANCE);
        }
        
        _maneuver->forward(speed);
        
        // Usa POSITION_TOLERANCE per il controllo della posizione
        if (distanceToTarget < POSITION_TOLERANCE) {
            _targetDistance += PARALLEL_LINE_SPACING;
            _maneuver->rotate(90, 50);  // Gira di 90 gradi con velocità media (50%)
        }
    }
}

// Navigazione a spirale
void Navigation::navigateSpiral() {
    // Usa direttamente il metodo spiral di Maneuver
    // Impostazioni predefinite: raggio massimo di 5m e velocità di 0.5 m/s
    _maneuver->spiral(5.0f, 0.5f);
    _isNavigating = false;
}

// Gestione degli ostacoli
bool Navigation::checkObstacles() {
    bool obstacleDetected = false;
    
    // Controlla sensori ultrasonici se abilitati
#ifdef ENABLE_ULTRASONIC
    if (_ultrasonic != nullptr) {
        // Leggi le distanze dai 3 sensori
        float left, center, right;
        _ultrasonic->getAllDistances(left, center, right);
        
        float minDistance = OBSTACLE_CLEARANCE * 2; // Inizializza con un valore alto
        bool validReading = false;
        
        // Trova la distanza minima tra i sensori attivi
        if (left > 0) {
            validReading = true;
            if (left < minDistance) minDistance = left;
        }
        if (center > 0) {
            validReading = true;
            if (center < minDistance) minDistance = center;
        }
        if (right > 0) {
            validReading = true;
            if (right < minDistance) minDistance = right;
        }
        
        // Se abbiamo almeno una lettura valida e la distanza minima è inferiore alla soglia
        if (validReading && minDistance < OBSTACLE_CLEARANCE) {
            obstacleDetected = true;
        }
    }
#endif
    
    // Controlla sensori urto se abilitati
#ifdef ENABLE_BUMP_SENSORS
    if (_bumper != nullptr) {
        bool left, center, right;
        _bumper->getAllBumpStatus(left, center, right);
        if (left || center || right) {
            obstacleDetected = true;
        }
    }
#endif
    
    return obstacleDetected;
}

void Navigation::handleObstacle() {
    // Ferma il robot
    _maneuver->stop();
    delay(500); // Pausa per sicurezza
    
    // Torna indietro
    _maneuver->backward();
    delay(1000);
    
    // Gira di un angolo casuale (10-170 gradi) con velocità media (50%)
    int randomAngle = random(RANDOM_TURN_MIN_ANGLE, RANDOM_TURN_MAX_ANGLE);
    _maneuver->rotate(randomAngle, 50);
    delay(1000);
    
    // Riprendi la navigazione
    _maneuver->forward();
}

// Gestione del perimetro
bool Navigation::checkPerimeter() {
    // Se il perimetro è disabilitato, restituisci false
#ifndef ENABLE_PERIMETER
    return false;
#else
    if (_perimeter == nullptr) return false;
    
    // Considera il perimetro violato se il segnale è al di sotto della soglia
    bool perimeterDetected = _perimeter->isDetected();
    if (!perimeterDetected) {
        _isNavigating = false;
        return true; // Perimetro violato
    }
    return false; // Perimetro non violato
#endif
}

void Navigation::handlePerimeter() {
    // Se il perimetro è disabilitato a tempo di compilazione, esci subito
#ifndef ENABLE_PERIMETER
    return;
#else
    // Se il perimetro non è inizializzato, esci
    if (_perimeter == nullptr) {
        return;
    }
    
    // Il perimetro è abilitato e inizializzato, possiamo procedere con la gestione
    // Ferma il robot
    _maneuver->stop();
    delay(500); // Pausa per sicurezza
    
    // Torna indietro
    _maneuver->backward();
    delay(1000);
    
    // Gira di 180 gradi
    _maneuver->rotate(180);
    delay(1000);
    
    // Riprendi la navigazione
    _maneuver->forward();
#endif
}
