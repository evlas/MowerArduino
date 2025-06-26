#ifndef MANEUVER_H
#define MANEUVER_H

#include "Arduino.h"
#include "../motors/DriveMotor.h"  // Include le costanti di configurazione

// Forward declarations
class DriveMotor;

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
    Maneuver(DriveMotor* left, DriveMotor* right, float bladeWidth, float wheelBase, float maxLinearSpeed = 1.0f)
        : leftMotor_(left), 
          rightMotor_(right), 
          moving_(false), 
          turning_(false),
          bladeWidth_(bladeWidth), 
          wheelBase_(wheelBase),
          maxLinearSpeed_(maxLinearSpeed),  // Valore in m/s
          currentLeftSpeed_(0),
          currentRightSpeed_(0),
          targetLeftSpeed_(0),
          targetRightSpeed_(0),
          accelStep_(0),
          isAccelerating_(false),
          isDecelerating_(false),
          lastAccelUpdate_(0),
          leftPid_(1.0f, 0.1f, 0.05f),  // Valori di default, da tarare
          rightPid_(1.0f, 0.1f, 0.05f), // Limiti di uscita
          isTrajectoryCorrection_(false) {}
    ~Maneuver();
    
    // Inizializzazione
    void begin();
    
    // Basic movements

    void forward(int speed = 100);
    void backward(int speed = 100);
    void turnLeft(int speed = 100);
    void turnRight(int speed = 100);
    void stop();
    
    // Complex maneuvers
    void rotate(int degrees, int speed);
    void moveStraight(int distance, int speed);
    void zigzag(int distance, int width, int speed);
    void spiral(float maxRadius = 5.0f, float speed = 0.5f);  // maxRadius in metri, speed in m/s
    void spiral(int speed);  
    void spiral(int radius, int speed = 100);
    
    // Status
    bool isMoving() const { return moving_; }
    bool isTurning() const { return turning_; }
    
    // Speed control
    void setSpeed(int speed, bool isTrajectoryCorrection = false);
    void setSpeed(int leftSpeed, int rightSpeed, bool isTrajectoryCorrection = true);
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
    
    // Controllori PID (struttura interna)
    struct PIDController leftPid_;
    struct PIDController rightPid_;
  

    // Motori
    DriveMotor* leftMotor_;
    DriveMotor* rightMotor_;
    
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
};

extern Maneuver mowerManeuver;

#endif // MANEUVER_H
