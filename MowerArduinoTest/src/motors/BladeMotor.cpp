#include "BladeMotor.h"
#include "../../config.h"
#include <Arduino.h>

/**
 * @brief Costruttore per singolo motore
 */
BladeMotor::BladeMotor(uint8_t pwmPin, uint8_t dirPin)
    : MotorBase(pwmPin, dirPin),  // Inizializza il motore base con i pin forniti
      motor2(nullptr),
      dirPin(dirPin),
      isRunning(false),
      hasSecondMotor(false),
      currentDirection(true),  // Direzione predefinita: avanti
      isStarting(false),
      totalRunTime(0) {
}

/**
 * @brief Costruttore per doppio motore
 */
BladeMotor::BladeMotor(uint8_t pwmPin1, uint8_t dirPin, uint8_t pwmPin2)
    : MotorBase(pwmPin1, dirPin),  // Inizializza il motore base con il primo motore
      motor2(new MotorBase(pwmPin2, dirPin)),  // Condivide lo stesso pin di direzione
      dirPin(dirPin),
      isRunning(false),
      hasSecondMotor(true),
      currentDirection(true),  // Direzione predefinita: avanti
      isStarting(false),
      totalRunTime(0) {
}

/**
 * @brief Distruttore - pulisce la memoria allocata
 */
BladeMotor::~BladeMotor() {
    if (hasSecondMotor) {
        delete motor2;
    }
}

/**
 * @brief Inizializza i motori
 * @param invertDirection Se true, inverte la direzione di entrambi i motori
 */
void BladeMotor::begin(bool invertDirection) {
    // Configura il pin di direzione
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, currentDirection ? HIGH : LOW);
    
    // Inizializza il motore base
    MotorBase::begin();
    MotorBase::setReversed(invertDirection);
    
    // Inizializza il secondo motore se presente
    if (hasSecondMotor) {
        motor2->begin();
        motor2->setReversed(invertDirection);
    }
    
    #ifdef DEBUG_MOTORS
    SERIAL_DEBUG.print(F("Blade motor(s) initialized on dirPin: "));
    SERIAL_DEBUG.println(dirPin);
    #endif
}

/**
 * @brief Aggiorna lo stato del motore
 */
void BladeMotor::update() {
    if (isStarting && (millis() - startTime >= STARTUP_DELAY)) {
        isStarting = false;
        if (hasSecondMotor) {
            motor2->setPowerPercent(MotorBase::getCurrentPowerPercent());
        }
        
        #ifdef DEBUG_MOTORS
        SERIAL_DEBUG.println(F("Blade motor(s) startup sequence complete"));
        #endif
    }
}

/**
 * @brief Avvia i motori in sequenza
 */
void BladeMotor::start() {
    if (!isRunning) {
        isRunning = true;
        isStarting = true;
        startTime = millis();
        
        // Imposta la direzione corrente
        digitalWrite(dirPin, currentDirection ? HIGH : LOW);
        
        // Avvia il motore base a velocità zero
        MotorBase::setPowerPercent(0);
        
        // Avvia il secondo motore se presente
        if (hasSecondMotor) {
            motor2->setPowerPercent(0);
        }
        
        #ifdef DEBUG_MOTORS
        SERIAL_DEBUG.print(F("Starting blade motor(s) in direction: "));
        SERIAL_DEBUG.println(currentDirection ? "FORWARD" : "REVERSE");
        #endif
    }
}

/**
 * @brief Ferma immediatamente i motori
 */
void BladeMotor::stop() {
    if (isRunning) {
        // Ferma il motore base
        MotorBase::setPowerPercent(0);
        
        // Ferma il secondo motore se presente
        if (hasSecondMotor) {
            motor2->setPowerPercent(0);
        }
        
        isRunning = false;
        isStarting = false;
        totalRunTime += (millis() - startTime);
        
        #ifdef DEBUG_MOTORS
        SERIAL_DEBUG.println(F("Blade motor(s) stopped"));
        SERIAL_DEBUG.print(F("Total run time: "));
        SERIAL_DEBUG.print(getTotalRunTime());
        SERIAL_DEBUG.println(F(" seconds"));
        #endif
    }
}

/**
 * @brief Imposta la velocità dei motori
 * @param percent Velocità in percentuale (0-100%)
 */
void BladeMotor::setSpeed(uint8_t percent) {
    if (!isRunning) return;
    
    // Limita tra 0-100%
    percent = constrain(percent, 0, 100);
    
    // Imposta la velocità sul motore base
    MotorBase::setPowerPercent(percent);
    
    // Imposta la stessa velocità sul secondo motore se presente e non in fase di avvio
    if (hasSecondMotor && !isStarting) {
        motor2->setPowerPercent(percent);
    }
    
    #ifdef DEBUG_MOTORS_VERBOSE
    static int lastSpeed = -1;
    if (percent != lastSpeed) {
        lastSpeed = percent;
        SERIAL_DEBUG.print(F("Blade motor speed set to: "));
        SERIAL_DEBUG.print(percent);
        SERIAL_DEBUG.println("%");
    }
    #endif
}

/**
 * @brief Imposta la direzione di rotazione
 * @param forward Se true, direzione in avanti
 */
void BladeMotor::setDirection(bool forward) {
    if (forward != currentDirection) {
        currentDirection = forward;
        digitalWrite(dirPin, currentDirection ? HIGH : LOW);
        
        #ifdef DEBUG_MOTORS
        SERIAL_DEBUG.print(F("Blade motor direction set to: "));
        SERIAL_DEBUG.println(currentDirection ? "FORWARD" : "REVERSE");
        #endif
    }
}

/**
 * @brief Inverte la direzione di rotazione
 */
void BladeMotor::toggleDirection() {
    setDirection(!currentDirection);
}

/**
 * @brief Ottiene il tempo totale di funzionamento
 * @return Tempo in secondi
 */
unsigned long BladeMotor::getTotalRunTime() const {
    unsigned long currentRunTime = isRunning ? (millis() - startTime) : 0;
    return (totalRunTime + currentRunTime) / 1000; // Converti in secondi
}

/**
 * @brief Azzera il contatore del tempo di funzionamento
 */
void BladeMotor::resetRunTime() {
    totalRunTime = 0;
    if (isRunning) {
        startTime = millis();
    }
    
    #ifdef DEBUG_MOTORS
    SERIAL_DEBUG.println(F("Blade motor runtime reset"));
    #endif
}
