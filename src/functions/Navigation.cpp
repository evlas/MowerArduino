#include "Navigation.h"
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
    _ultrasonic(ultrasonic),
    _bumper(bumper),
    _perimeter(perimeter),
    _mode(),  // Initialize with default constructor
    _isNavigating(false),
    _bladeWidth(BLADE_WIDTH),
    _currentDistance(0.0f),
    _targetDistance(0.0f),
    _currentAngle(0.0f),
    _targetAngle(0.0f),
    _currentRing(0),
    _currentRadius(0.0f)
{
    // Initialize PositionManager if available
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
    _currentRing = 0;
    _currentRadius = 0.0f;

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
    if (checkObstacles() || (_perimeter != nullptr && checkPerimeter())) {
        handleObstacle();
        return;
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
        
        // Gira di quell'angolo
        _maneuver->rotate(_targetAngle);
        
        // Resetta la distanza
        _currentDistance = 0.0f;
        _isNavigating = false;
    }
}

// Navigazione a linee parallele
void Navigation::navigateParallel() {
    if (!_isNavigating) {
        // Imposta la distanza target per la linea successiva
        _targetDistance = _bladeWidth; // larghezza lame
        _isNavigating = true;
    }
    
    // Muovi in linea retta
    _maneuver->forward();
    
    // Aggiorna la distanza percorsa
    _currentDistance += _maneuver->getLinearVelocity() / 100.0f; // converti cm/s in m/s
    
    if (_currentDistance >= _targetDistance) {
        // Quando arrivi alla fine della linea
        _maneuver->stop();
        
        // Gira di 90 gradi per posizionarsi per la linea successiva
        _maneuver->rotate(PARALLEL_TURN_ANGLE);
        
        // Resetta la distanza
        _currentDistance = 0.0f;
        _isNavigating = false;
    }
}

// Navigazione a spirale
void Navigation::navigateSpiral() {
    if (!_isNavigating) {
        // Imposta il raggio iniziale
        _currentRadius = SPIRAL_START_RADIUS / 100.0f; // in metri
        _currentRing = 0;
        _isNavigating = true;
    }
    
    // Calcola il raggio target per questo anello
    float targetRadius = (_currentRing * SPIRAL_RADIUS_STEP + SPIRAL_START_RADIUS) / 100.0f;
    
    // Ottieni la posizione corrente dal PositionManager
    RobotPosition pos = _positionManager->getPosition();
    float x = pos.x;
    float y = pos.y;
    
    // Calcola la distanza dal centro
    float distance = sqrt(x * x + y * y);
    
    if (distance < targetRadius) {
        // Se siamo dentro il raggio target, muoviti in linea retta
        _maneuver->forward(DEFAULT_MOTOR_SPEED);
    } else {
        // Se siamo fuori dal raggio target, muoviti in spirale
        _maneuver->spiral(distance * 100, DEFAULT_MOTOR_SPEED);  // Converti m in cm
        
        // Se siamo completati l'anello, passa al successivo
        if (abs(atan2(y, x)) >= 2 * M_PI) {
            _currentRing++;
            _currentAngle = 0;
        }
    }
}

// Gestione degli ostacoli
bool Navigation::checkObstacles() {
    // Controlla i sensori ad ultrasuoni
#ifdef ENABLE_ULTRASONIC
    if (_ultrasonic != nullptr && _ultrasonic->isObstacleDetected(OBSTACLE_DISTANCE_THRESHOLD)) {
        return true;
    }
#endif
    
    // Controlla i sensori di urto
#ifdef ENABLE_BUMP_SENSORS
    if (_bumper != nullptr && (_bumper->isLeftBump() || _bumper->isRightBump() || _bumper->isCenterBump())) {
        return true;
    }
#endif
    
    return false;
}

void Navigation::handleObstacle() {
    // Ferma il robot
    _maneuver->stop();
    delay(500); // Pausa per sicurezza
    
    // Torna indietro
    _maneuver->backward();
    delay(1000);
    
    // Gira di un angolo casuale (10-170 gradi)
    int randomAngle = random(RANDOM_TURN_MIN_ANGLE, RANDOM_TURN_MAX_ANGLE);
    _maneuver->rotate(randomAngle);
    delay(1000);
    
    // Riprendi la navigazione
    _maneuver->forward();
}

// Gestione del perimetro
bool Navigation::checkPerimeter() {
    // Se il perimetro è disabilitato a tempo di compilazione, restituisci false
#ifndef ENABLE_PERIMETER
    return false;
#else
    // Se il perimetro non è inizializzato, restituisci false
    if (_perimeter == nullptr) {
        return false;
    }
    
    // Il perimetro è abilitato e inizializzato, possiamo usare il puntatore
    return _perimeter->isDetected();
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
