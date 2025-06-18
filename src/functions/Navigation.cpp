#include "Navigation.h"

// Includi l'header completo solo se necessario
#ifdef ENABLE_PERIMETER
#include "../../src/sensors/PerimeterSensors.h"
#endif

#ifdef ENABLE_PERIMETER
Navigation::Navigation(Maneuver* maneuver, 
                     UltrasonicSensors* ultrasonic, 
                     BumpSensors* bumper, 
                     PerimeterSensors* perimeter, 
                     PositionManager* positionManager) :
#else
Navigation::Navigation(Maneuver* maneuver, 
                     UltrasonicSensors* ultrasonic, 
                     BumpSensors* bumper, 
                     void* perimeter, 
                     PositionManager* positionManager) :
#endif
    _maneuver(maneuver),
    _ultrasonic(ultrasonic),
    _bumper(bumper),
    _perimeter(static_cast<decltype(_perimeter)>(perimeter)),
    _positionManager(*positionManager),  // Initialize with passed PositionManager
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
    // Initialize PositionManager
    _positionManager.begin();
    _positionManager.enableOdometry(true);
    _positionManager.enableIMU(true);
    _positionManager.enableGPS(true);
    
    // Initialize perimeter if available
#ifdef ENABLE_PERIMETER
    if (_perimeter != nullptr) {
        _perimeter->begin();
    }
#endif
}

Navigation::~Navigation() {
    // Null
}

void Navigation::begin() {
    _mode.setMode(NavigationMode::STOPPED);
    _isNavigating = false;
    _currentDistance = 0.0f;
    _currentAngle = 0.0f;
    _currentRing = 0;
    _currentRadius = 0.0f;
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
    _maneuver->forward(DEFAULT_SPEED); // Velocità default dal config.h
    
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
    RobotPosition pos = _positionManager.getPosition();
    float x = pos.x;
    float y = pos.y;
    
    // Calcola la distanza dal centro
    float distance = sqrt(x * x + y * y);
    
    if (distance < targetRadius) {
        // Se siamo dentro il raggio target, muoviti in linea retta
        _maneuver->forward(DEFAULT_SPEED);
    } else {
        // Se siamo fuori dal raggio target, muoviti in spirale
        _maneuver->spiral(distance * 100, DEFAULT_SPEED);  // Converti m in cm
        
        // Se siamo completati l'anello, passa al successivo
        if (abs(atan2(y, x)) >= 2 * M_PI) {
            _currentRing++;
            _currentAngle = 0;
        }
    }
}

// Gestione degli ostacoli
bool Navigation::checkObstacles() {
    if (_ultrasonic != nullptr && _ultrasonic->isObstacleDetected(OBSTACLE_DISTANCE_THRESHOLD)) {
        return true;
    }
    if (_bumper != nullptr && (_bumper->isLeftBump() || _bumper->isRightBump() || _bumper->isCenterBump())) {
        return true;
    }
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
    // Se il perimetro è disabilitato o non inizializzato, restituisci false
    if (_perimeter == nullptr) {
        return false;
    }
    
#ifdef ENABLE_PERIMETER
    // Il perimetro è abilitato, possiamo usare il puntatore
    return _perimeter->isDetected();
#else
    return false;  // Perimetro disabilitato a tempo di compilazione
#endif
}

void Navigation::handlePerimeter() {
    // Se il perimetro è disabilitato o non inizializzato, non fare nulla
    if (_perimeter == nullptr) {
        return;
    }
    
#ifdef ENABLE_PERIMETER
    // Il perimetro è abilitato, possiamo procedere con la gestione
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
