#ifndef MANEUVER_H
#define MANEUVER_H

#include "Arduino.h"
#include "DriveMotor.h"  // Include le costanti di configurazione
#include "config.h"

// Forward declarations
class DriveMotor;
class PositionManager;  // Usiamo direttamente PositionManager

// Struttura per il controllo PID
struct PIDController {
    float kp, ki, kd;      // Parametri del controllore
    float iTerm;           // Termine integrale
    float lastInput;       // Ultimo valore di ingresso
    float outMin, outMax;  // Limiti di uscita
    unsigned long lastTime; // Ultimo tempo di aggiornamento
    
    // Costruttore con valori di default
    PIDController(float Kp = 1.0f, float Ki = 0.1f, float Kd = 0.05f, float Min = -255.0f, float Max = 255.0f) {
        init(Kp, Ki, Kd, Min, Max);
    }
    
    // Inizializza il controllore
    void init(float Kp, float Ki, float Kd, float Min, float Max) {
        kp = Kp; ki = Ki; kd = Kd;
        iTerm = 0;
        lastInput = 0;
        outMin = Min; outMax = Max;
        lastTime = millis();
    }
    
    // Calcola l'output del controllore
    float compute(float input, float setpoint) {
        // Calcola il tempo trascorso
        unsigned long now = millis();
        float dT = (now - lastTime) / 1000.0f; // in secondi
        lastTime = now;
        
        if (dT <= 0) return 0; // Evita divisioni per zero
        
        // Calcola l'errore
        float error = setpoint - input;
        
        // Termine proporzionale
        float output = kp * error;
        
        // Termine integrale con anti-windup
        iTerm += (ki * error * dT);
        // Clamp del termine integrale
        iTerm = constrain(iTerm, outMin, outMax);
        
        // Termine derivativo (sul processo, non sull'errore per evitare derivate impulsive)
        float dInput = (input - lastInput) / dT;
        
        // Calcola l'output totale
        output = output + iTerm - (kd * dInput);
        
        // Clamp dell'output
        output = constrain(output, outMin, outMax);
        
        // Memorizza l'input corrente per il prossimo ciclo
        lastInput = input;
        
        return output;
    }
    
    // Resetta il controllore
    void reset() {
        iTerm = 0;
        lastInput = 0;
        lastTime = millis();
    }
};

class Maneuver {
  public:
    Maneuver(DriveMotor* left, DriveMotor* right, PositionManager* positionManager, float bladeWidth, float wheelBase, float maxLinearSpeed = 1.0f)
            : leftMotor_(left), 
              rightMotor_(right),
              positionManager_(positionManager), 
              moving_(false), 
              turning_(false),
              bladeWidth_(bladeWidth), 
              wheelBase_(wheelBase),
              maxLinearSpeed_(maxLinearSpeed),  // Valore in m/s
              currentLeftSpeed_(0),
              currentRightSpeed_(0),
              targetLeftSpeed_(0),
              targetRightSpeed_(0) {
        // Inizializzazione delle variabili di controllo della distanza
        startX_ = 0;
        startY_ = 0;
        targetDistance_ = 0;
        useTimerForDistance_ = false;
        movementEndTime_ = 0;
        
        // Inizializzazione PID
        leftPid_ = PIDController(1.0f, 0.1f, 0.05f);
        rightPid_ = PIDController(1.0f, 0.1f, 0.05f);
        
        // Altri flag
        isTrajectoryCorrection_ = false;
        isAccelerating_ = false;
        isDecelerating_ = false;
        lastAccelUpdate_ = 0;
        accelStep_ = 0;
    }
    ~Maneuver();
    
    // Inizializzazione
    void begin();
    
    // Basic movements
    void forward(int duration, float speed);
    void backward(int duration, float speed);
    void turnLeft(int angle, float speed);
    void turnRight(int angle, float speed);
    void rotateLeft(float speed);
    void rotateRight(float speed);
    void stop();
    
    // Path movement
    void moveTo(float targetX, float targetY, float speed);
    void moveBy(float deltaX, float deltaY, float speed);
    
    // Complex maneuvers
    void rotate(int degrees, float speed = 50.0f);
    void moveStraight(int distance, float speed = 50.0f);
    void zigzag(int distance, int width, float speed = 50.0f);
    void spiral(float maxRadius = 5.0f, float speed = 0.5f);  // maxRadius in metri, speed in m/s
    // Overload per compatibilità
    void startSpiral(float speed = 50.0f);  // Avvia una spirale con velocità predefinita
    
    // Metodi per il movimento a zigzag
    void updateZigzag();  // Da chiamare nel loop principale
    void stopZigzag();    // Ferma il movimento a zigzag
    
    // Metodo per avviare il taglio con la larghezza corretta della lama
    void startMowing(float bladeWidth, float speed = 50.0f);
    
    // Gestione ostacoli
    void setObstacleDetected(bool detected);
    bool isAvoidingObstacle() const { return isAvoidingObstacle_; }
    void updateObstacleAvoidance();
    void resumeAfterObstacle();
    
    // Status
    bool isMoving() const { return moving_; }
    bool isTurning() const { return turning_; }
    
    // Update methods
    void updateMovement();  // Da chiamare nel loop principale per aggiornare il movimento
    
    // Speed control
    void setSpeed(float speed, bool isTrajectoryCorrection = false);
    void setSpeed(float leftSpeed, float rightSpeed, bool isTrajectoryCorrection = true);
    void setDirection(bool leftForward, bool rightForward);
    void updateAcceleration();
    
    // Getters
    int getSpeed() const { 
        // Restituisce la velocità media dei due motori
        return static_cast<int>((abs(currentLeftSpeed_) + abs(currentRightSpeed_)) / 2.0f); 
    }
    
    int getLeftSpeed() const { return static_cast<int>(currentLeftSpeed_); }
    int getRightSpeed() const { return static_cast<int>(currentRightSpeed_); }
    
    float getLinearVelocity() const { 
        // Velocità lineare media in m/s
        return ((currentLeftSpeed_ + currentRightSpeed_) / 2.0f / 100.0f) * maxLinearSpeed_;
    }
    
    float getAngularVelocity() const {
        // Velocità angolare in rad/s
        return ((currentRightSpeed_ - currentLeftSpeed_) / wheelBase_) * (2.0f * PI);
    }
    
private:
    // Helper method per aggiornare la velocità con accelerazione
    bool updateMotorSpeed(float &current, float target, float step);
    
    // Normalizza un angolo in radianti tra -PI e PI
    float normalizeAngle(float angle);
    
    // Controllori PID (struttura interna)
    struct PIDController leftPid_;
    struct PIDController rightPid_;
  

    // Motori
    DriveMotor* leftMotor_;
    DriveMotor* rightMotor_;
    PositionManager* positionManager_;  // Riferimento al gestore di posizione per la gestione della posizione
    
    // Stato del movimento
    bool moving_;
    bool turning_;
    
    // Parametri fisici
    float bladeWidth_;    // Larghezza di taglio [m]
    float wheelBase_;     // Distanza tra le ruote [m]
    float maxLinearSpeed_; // Velocità massima lineare [m/s]
    
    // Controllo velocità
    float currentLeftSpeed_;
    float currentRightSpeed_;
    float targetLeftSpeed_;
    float targetRightSpeed_;
    float accelStep_;
    bool isAccelerating_;
    bool isDecelerating_;
    unsigned long lastAccelUpdate_;
    
  
    // Flag per la correzione di traiettoria
    bool isTrajectoryCorrection_;
    
    // Costanti per lo stato di zigzag
    enum ZigzagState {
        ZIGZAG_FORWARD,
        ZIGZAG_TURNING
    };
    
    // Variabili per il movimento a zigzag (parallelo)
    bool isZigzagging_;
    float zigzagDistance_;
    float zigzagWidth_;
    float zigzagSpeed_;
    ZigzagState zigzagState_;
    int zigzagDirection_;
    float zigzagStartAngle_;
    float zigzagTargetAngle_;
    unsigned long zigzagStartTime_;
    bool isTurning_;
    float targetAngle_;
    float startX_, startY_;
    float lastTurnX_, lastTurnY_;
    
    // Variabili per la gestione degli ostacoli
    bool isAvoidingObstacle_ = false;
    bool obstacleDetected_ = false;
    unsigned long obstacleDetectedTime_ = 0;
    float savedTargetLeftSpeed_ = 0;
    float savedTargetRightSpeed_ = 0;
    int avoidanceManeuver_ = 0;  // 0 = nessuno, 1 = indietro, 2 = gira a destra, 3 = avanti
    unsigned long avoidanceStartTime_ = 0;
    bool obstaclePhase1Done_ = false;
    bool obstaclePhase2Done_ = false;
    bool obstaclePhase3Done_ = false;
    
    // Variabili per il controllo della distanza
    float targetDistance_ = 0;       // Distanza target da percorrere (metri)
    bool useTimerForDistance_ = false; // Usa il timer invece della posizione
    unsigned long movementEndTime_ = 0; // Tempo di fine movimento (usato come fallback)
    
};

extern Maneuver mowerManeuver;

#endif // MANEUVER_H
