#ifndef BLADE_MOTOR_H
#define BLADE_MOTOR_H

#include "MotorBase.h"

/**
 * @brief Classe per la gestione dei motori delle lame
 * 
 * Gestisce uno o due motori delle lame con avvio sequenziale e controllo di velocità.
 * Utilizza un unico pin di direzione per entrambi i motori.
 * Eredita da MotorBase per la gestione di base del motore.
 */
class BladeMotor : public MotorBase {
private:
    MotorBase* motor2;           // Secondo motore lamiera (opzionale)
    uint8_t dirPin;             // Pin di direzione condiviso
    bool isRunning;              // Stato di accensione
    bool hasSecondMotor;         // Se true, gestisce due motori
    bool currentDirection;       // Direzione corrente (true = avanti)
    unsigned long startTime;     // Timestamp di avvio
    bool isStarting;             // Se true, in fase di avvio
    unsigned long totalRunTime;  // Tempo totale di funzionamento in millisecondi
    
    static const unsigned long STARTUP_DELAY = 500; // 500ms tra l'avvio dei motori
    
public:
    /**
     * @brief Costruttore per singolo motore
     * @param pwmPin Pin PWM per il controllo di velocità
     * @param dirPin Pin condiviso per il controllo di direzione
     */
    BladeMotor(uint8_t pwmPin, uint8_t dirPin);
    
    /**
     * @brief Costruttore per doppio motore
     * @param pwmPin1 Pin PWM per il primo motore
     * @param dirPin Pin condiviso per il controllo di direzione
     * @param pwmPin2 Pin PWM per il secondo motore
     */
    BladeMotor(uint8_t pwmPin1, uint8_t dirPin, uint8_t pwmPin2);
    
    /**
     * @brief Distruttore - pulisce la memoria allocata
     */
    ~BladeMotor();
    
    /**
     * @brief Inizializza i motori
     * @param invertDirection Se true, inverte la direzione di entrambi i motori
     */
    void begin(bool invertDirection = false);
    
    /**
     * @brief Aggiorna lo stato del motore (da chiamare nel loop principale)
     */
    void update();
    
    /**
     * @brief Avvia i motori in sequenza
     */
    void start();
    
    /**
     * @brief Ferma immediatamente i motori
     */
    void stop();
    
    /**
     * @brief Imposta la velocità dei motori
     * @param percent Velocità in percentuale (0-100%)
     */
    void setSpeed(uint8_t percent);
    
    /**
     * @brief Imposta la direzione di rotazione
     * @param forward Se true, direzione in avanti
     */
    void setDirection(bool forward);
    
    /**
     * @brief Inverte la direzione di rotazione
     */
    void toggleDirection();
    
    /**
     * @brief Verifica se i motori sono accesi
     * @return true se i motori sono accesi
     */
    bool isEngaged() const { return isRunning; }
    
    /**
     * @brief Verifica se i motori sono in fase di avvio
     * @return true se in fase di avvio
     */
    bool isStartingUp() const { return isStarting; }
    
    /**
     * @brief Ottiene la direzione corrente
     * @return true se la direzione è in avanti
     */
    bool getDirection() const { return currentDirection; }
    
    /**
     * @brief Ottiene il tempo totale di funzionamento
     * @return Tempo in secondi
     */
    unsigned long getTotalRunTime() const;
    
    /**
     * @brief Azzera il contatore del tempo di funzionamento
     */
    void resetRunTime();
};

#endif // BLADE_MOTOR_H
