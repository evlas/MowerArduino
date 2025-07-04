#ifndef DRIVE_MOTOR_H
#define DRIVE_MOTOR_H

#include "../MotorBase.h"
#include "../../config.h"
#include <Arduino.h>

// Forward declaration for interrupt handler
class DriveMotor;

// Global instance pointers for interrupt handling
namespace DriveMotorISR {
    extern DriveMotor* leftMotorInstance;
    extern DriveMotor* rightMotorInstance;
    void handleLeftEncoderISR();
    void handleRightEncoderISR();
}

/**
 * @brief Classe per la gestione di un motore di trazione con encoder
 */
class DriveMotor : public MotorBase {
    friend void DriveMotorISR::handleLeftEncoderISR();
    friend void DriveMotorISR::handleRightEncoderISR();
    
private:
    // Configurazione fisica
    const float wheelDiameter_;          // Diametro ruota in metri
    const float wheelCircumference_;      // Circonferenza ruota in metri
    const int pulsesPerRevolution_;      // Impulsi per giro encoder
    
    // Stato encoder
    volatile long encoderCount_ = 0;      // Conteggio impulsi encoder
    volatile long lastEncoderCount_ = 0;  // Conteggio al precedente aggiornamento
    uint8_t encoderPin_;                 // Pin dell'encoder
    bool isLeftMotor_;                   // Se true, è il motore sinistro
    
    // Misure di movimento
    float currentLinearSpeed_ = 0.0f;     // Velocità lineare in m/s
    float distanceTraveled_ = 0.0f;       // Distanza percorsa in metri
    unsigned long lastSpeedUpdate_ = 0;   // Ultimo aggiornamento velocità
    
    // Gestione interrupt
    void handleEncoderISR();             // Metodo per gestire l'interrupt
    
    // Aggiorna la stima della velocità
    void updateSpeedEstimate();
    
    // Posizione del motore nel piano (per odometria)
    float x_ = 0.0f;  // Posizione X in metri
    float y_ = 0.0f;  // Posizione Y in metri
    float theta_ = 0.0f;  // Orientamento in radianti
    float lastUpdateTime_ = 0.0f;  // Ultimo aggiornamento posizione (secondi)
    
public:
    /**
     * @brief Costruttore del motore di trazione
     * @param pwmPin Pin PWM per il controllo di velocità
     * @param dirPin Pin per il controllo di direzione
     * @param encoderPin Pin dell'encoder
     * @param wheelDiameter Diametro della ruota in metri
     * @param pulsesPerRevolution Impulsi per giro dell'encoder
     * @param isLeft Se true, è il motore sinistro (influenza la direzione dell'encoder)
     */
    DriveMotor(uint8_t pwmPin, uint8_t dirPin, uint8_t encoderPin,
              float wheelDiameter = WHEEL_DIAMETER, 
              int pulsesPerRevolution = ENCODER_PULSES_PER_REV,
              bool isLeft = false);
    
    /**
     * @brief Inizializza il motore e l'encoder
     */
    void begin() override;
    
    /**
     * @brief Aggiorna lo stato del motore (da chiamare nel loop principale)
     */
    void update();
    
    /**
     * @brief Reimposta il conteggio dell'encoder e la distanza percorsa
     */
    void resetEncoder();
    
    /**
     * @brief Imposta la velocità lineare target in m/s
     * @param speed Velocità in m/s (positiva = avanti, negativa = indietro)
     */
    /**
     * @brief Imposta la velocità lineare del motore
     * @param speedPercent Velocità in percentuale (-100% a 100%)
     */
    void setLinearSpeed(float speedPercent);
    
    /**
     * @brief Aggiorna la posizione del motore in base alla velocità
     * @param dt Tempo trascorso dall'ultimo aggiornamento in secondi
     */
    void updatePosition(float dt);
    
    /**
     * @brief Imposta la posizione del robot
     * @param x Posizione X in metri
     * @param y Posizione Y in metri
     * @param theta Orientamento in radianti
     */
    void setPosition(float x, float y, float theta);
    
    /**
     * @brief Imposta la posizione del motore (solo x e y)
     * @param x Posizione X in metri
     * @param y Posizione Y in metri
     */
    void setPosition(float x, float y) { x_ = x; y_ = y; }
    
    /**
     * @brief Imposta la velocità angolare target in rad/s
     * @param omega Velocità angolare target in rad/s
     */
    void setAngularSpeed(float omega) {
        setLinearSpeed(omega * (wheelDiameter_ / 2.0f));
    }
    
    // Getters
    long getEncoderCount() const { return encoderCount_; }
    float getLinearSpeed() const { return currentLinearSpeed_; }
    float getDistanceTraveled() const { return distanceTraveled_; }
    float getX() const { return x_; }
    float getY() const { return y_; }
    float getTheta() const { return theta_; }
    float getDistance() const { return distanceTraveled_; }
    float getLinearVelocity() const { return getLinearSpeed(); }
    
    /**
     * @brief Restituisce la velocità angolare in rad/s
     */
    float getAngularSpeed() const { 
        return (currentLinearSpeed_ * 2.0f) / wheelDiameter_; 
    }
};

#endif // DRIVE_MOTOR_H
