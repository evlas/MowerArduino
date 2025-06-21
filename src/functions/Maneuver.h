#ifndef MANEUVER_H
#define MANEUVER_H

#include "Arduino.h"
#include "../../config.h"  // Include le costanti di configurazione

// Forward declarations
class MotorController;

class Maneuver {
  public:
    Maneuver(MotorController* motorController, 
             float bladeWidth = BLADE_WIDTH, 
             float wheelBase = WHEEL_BASE,
             float maxLinearSpeed = MAX_LINEAR_SPEED);
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
    bool isMoving();
    bool isTurning();
    
    // Speed control
    void setSpeed(int speed, bool isTrajectoryCorrection = false);
    void setSpeed(int leftSpeed, int rightSpeed, bool isTrajectoryCorrection = true);
    void setDirection(bool leftForward, bool rightForward);
    void updateAcceleration();
    int getSpeed() const;
    int getLeftSpeed() const;
    int getRightSpeed() const;
    float getLinearVelocity() const;
    float getAngularVelocity() const;
  
  private:
    // Struttura per il controllo PID
    struct PIDController {
        float kp, ki, kd;      // Parametri del controllore
        float iTerm;            // Termine integrale
        float lastInput;        // Ultimo valore di ingresso
        float outMin, outMax;   // Limiti di uscita
        unsigned long lastTime; // Ultimo tempo di aggiornamento
        
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
    
    MotorController* motorController;
    bool moving;
    bool turning;
    
    // Controllori PID per ogni lato
    PIDController _leftPid;
    PIDController _rightPid;
    
    // Parametri del robot
    const float _bladeWidth;      // Larghezza di taglio [m]
    const float _wheelBase;       // Distanza tra le ruote [m]
    const float _maxLinearSpeed;  // Velocità massima lineare [m/s]
    
    // Controllo di accelerazione
    unsigned long _lastAccelUpdate;
    float _currentLeftSpeed;
    float _currentRightSpeed;
    float _targetLeftSpeed;
    float _targetRightSpeed;
    float _accelStep;
    bool _isAccelerating;
    bool _isDecelerating;
    bool _isTrajectoryCorrection;  // Flag per indicare se stiamo correggendo la traiettoria
};

extern Maneuver mowerManeuver;

#endif // MANEUVER_H
