#include "BladeMotor.h"
#include "../../config.h"
#include "../../pin_config.h"
#include <Arduino.h>

/**
 * @brief Costruttore per singolo motore
 */
BladeMotor::BladeMotor(uint8_t pwmPin, uint8_t dirPin) 
    : MotorBase(pwmPin, dirPin), hasSecondMotor(false), isStarting(false) {
    motor2 = nullptr;
}

/**
 * @brief Costruttore per doppio motore
 */
BladeMotor::BladeMotor(uint8_t pwmPin1, uint8_t dirPin, uint8_t pwmPin2) 
    : MotorBase(pwmPin1, dirPin), hasSecondMotor(true), isStarting(false) {
    motor2 = new MotorBase(pwmPin2, dirPin);
}

/**
 * @brief Distruttore - pulisce la memoria allocata
 */
BladeMotor::~BladeMotor() {
    if (motor2 != nullptr) {
        delete motor2;
    }
}

/**
 * @brief Inizializza i motori
 * @param invertDirection Se true, inverte la direzione di entrambi i motori
 */
void BladeMotor::begin(bool invertDirection) {
    // Inizializza il motore base
    MotorBase::begin();
    
    // Se richiesto, inverte la direzione
    if (invertDirection) {
        setReversed(!isReversed());
    }
    
    // Inizializza il secondo motore se presente
    if (hasSecondMotor && motor2 != nullptr) {
        motor2->begin();
        if (invertDirection) {
            motor2->setReversed(!motor2->isReversed());
        }
    }
}

/**
 * @brief Aggiorna lo stato del motore
 */
void BladeMotor::update() {
    // Gestione dell'avvio sequenziale per il secondo motore
    if (isStarting && hasSecondMotor && motor2 != nullptr) {
        if ((millis() - startTime) >= STARTUP_DELAY) {
            isStarting = false;
            // Avvia il secondo motore con la stessa velocità del primo
            motor2->setSpeed(getSpeed());
        }
    }
}

/**
 * @brief Avvia i motori alla velocità predefinita
 */
void BladeMotor::start() {
    // Imposta la velocità predefinita della lama
    setSpeed(DEFAULT_BLADE_SPEED);
    
    // Avvia la sequenza di avvio se c'è un secondo motore
    if (hasSecondMotor && motor2 != nullptr) {
        isStarting = true;
        startTime = millis();
    }
}

/**
 * @brief Ferma i motori
 */
void BladeMotor::stop() {
    // Ferma entrambi i motori
    setSpeed(0);
    isStarting = false;
    
    if (hasSecondMotor && motor2 != nullptr) {
        motor2->setSpeed(0);
    }
}

/**
 * @brief Imposta la velocità dei motori
 * @param percent Velocità in percentuale (0-100%)
 */
void BladeMotor::setSpeed(uint8_t percent) {
    // Limita tra 0-100%
    percent = constrain(percent, 0, 100);
    
    // Imposta la velocità sul motore base
    MotorBase::setSpeed(percent);
    
    // Se c'è un secondo motore e non è in fase di avvio, aggiorna anche quello
    if (hasSecondMotor && motor2 != nullptr && !isStarting) {
        motor2->setSpeed(percent);
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
// I metodi seguenti sono ora implementati inline nell'header:
// - setDirection
// - getDirection
// - toggleDirection

/**
 * @brief Ottiene il tempo totale di funzionamento
 * @return Tempo in secondi
 */
unsigned long BladeMotor::getTotalRunTime() const {
    return (millis() - startTime) / 1000; // Converti in secondi
}

/**
 * @brief Azzera il contatore del tempo di funzionamento
 */
void BladeMotor::resetRunTime() {
    startTime = millis();
}
