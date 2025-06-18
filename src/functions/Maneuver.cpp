#include "Maneuver.h"

Maneuver::Maneuver(MotorController* motorController) :
    motorController(motorController),
    moving(false),
    turning(false) {
    // Initialize maneuver system
}

Maneuver::~Maneuver() {
    stop();
}

// Basic movements
void Maneuver::forward(int speed) {
    moving = true;
    turning = false;
    setDirection(true, true);
    setSpeed(speed, speed);
}

void Maneuver::backward(int speed) {
    moving = true;
    turning = false;
    setDirection(false, false);
    setSpeed(speed, speed);
}

void Maneuver::turnLeft(int speed) {
    moving = false;
    turning = true;
    setDirection(true, false);
    setSpeed(speed, speed);
}

void Maneuver::turnRight(int speed) {
    moving = false;
    turning = true;
    setDirection(false, true);
    setSpeed(speed, speed);
}

void Maneuver::stop() {
    moving = false;
    turning = false;
    setSpeed(0, 0);
}

// Complex maneuvers
void Maneuver::rotate(int degrees, int speed) {
    // TODO: Implement rotation logic using encoders
    // This is a placeholder implementation
    if (degrees > 0) {
        turnRight(speed);
    } else {
        turnLeft(speed);
    }
    // Add delay or encoder-based rotation here
}

void Maneuver::moveStraight(int distance, int speed) {
    // TODO: Implement distance-based movement using encoders
    forward(speed);
    // Add encoder-based distance measurement here
}

void Maneuver::zigzag(int distance, int width, int speed) {
    // TODO: Implement zigzag pattern
    forward(speed);
    // Add zigzag logic using distance and width parameters
}

void Maneuver::spiral(int radius, int speed) {
    // Calcola i tempi per ogni cerchio basati sul raggio
    int totalCircles = 5;  // Numero di cerchi completi
    int baseRadius = radius;
    
    for (int i = 0; i < totalCircles; i++) {
        int currentRadius = baseRadius * (i + 1);
        int circumference = 2 * PI * currentRadius;
        
        // Calcola la differenza di velocità per creare il cerchio
        int speedDiff = speed / 4;  // Velocità differenziale per il cerchio
        
        // Calcola il tempo per completare il cerchio
        int time = (circumference * 1000) / speed;  // Tempo in millisecondi
        
        // Avvia il cerchio
        setSpeed(speed + speedDiff, speed - speedDiff);
        delay(time);
        
        // Piccolo intervallo tra i cerchi
        delay(100);
    }
    
    // Stop alla fine
    stop();
}

void Maneuver::begin() {
    // Resetta lo stato
    moving = false;
    turning = false;
    
    // Ferma i motori
    stop();
    
    // Resetta la posizione
    motorController->resetPositionLeft();
    motorController->resetPositionRight();
    
    // Resetta l'odometria
    motorController->resetOdometry();
}

// Status
bool Maneuver::isMoving() {
    return moving;
}

bool Maneuver::isTurning() {
    return turning;
}

// Private helper methods
void Maneuver::setSpeed(int speed) {
    setSpeed(speed, speed);
}

void Maneuver::setSpeed(int leftSpeed, int rightSpeed) {
    motorController->setLeftMotorSpeed(leftSpeed);
    motorController->setRightMotorSpeed(rightSpeed);
}

void Maneuver::setDirection(bool leftForward, bool rightForward) {
    motorController->setLeftMotorDirection(leftForward);
    motorController->setRightMotorDirection(rightForward);
}

int Maneuver::getSpeed() const {
    // Get average speed of both motors
    int leftSpeed = motorController->getLeftMotorSpeed();
    int rightSpeed = motorController->getRightMotorSpeed();
    return (leftSpeed + rightSpeed) / 2;
}

int Maneuver::getLeftSpeed() const {
    return motorController->getLeftMotorSpeed();
}

int Maneuver::getRightSpeed() const {
    return motorController->getRightMotorSpeed();
}

float Maneuver::getLinearVelocity() const {
    return motorController->getLinearVelocity();
}

float Maneuver::getAngularVelocity() const {
    return motorController->getAngularVelocity();
}
