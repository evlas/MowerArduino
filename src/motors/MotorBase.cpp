#include "MotorBase.h"
#include "../config.h"  // Include per le costanti di configurazione

// Helper per conversione percentuale -> PWM
static inline int16_t percentToPwm(float percent) {
    percent = constrain(percent, -100.0f, 100.0f);
    // Mappa da -100.0..100.0 a MOTOR_MIN_SPEED..MOTOR_MAX_SPEED
    float pwm = (percent / 100.0f) * (percent > 0 ? MAX_MOTOR_SPEED : -MIN_MOTOR_SPEED);
    return static_cast<int16_t>(constrain(pwm, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED));
}

MotorBase::MotorBase(uint8_t pwmPin, uint8_t dirPin) 
    : pwmPin(pwmPin), dirPin(dirPin),
      enabled_(false), reversed_(false),
      currentSpeed_(0), targetSpeed_(0), 
      lastUpdateTime_(0),
      maxAcceleration_(MAX_LINEAR_ACCEL),
      maxDeceleration_(MAX_LINEAR_DECEL) {}

void MotorBase::begin() {
    // Configura i pin di uscita
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    
    // Imposta i pin a livello basso
    digitalWrite(pwmPin, LOW);
    digitalWrite(dirPin, LOW);
    
    enabled_ = true;
    lastUpdateTime_ = micros();
    
    #if defined(__AVR_ATmega2560__)
    // Configura Timer3 (pin 5) per 20kHz se usato
    if (pwmPin == 5 || pwmPin == 2 || pwmPin == 3) {
        TCCR3A = 0;
        TCCR3B = 0;
        TCNT3 = 0;
        TCCR3A = (1 << WGM31) | (1 << WGM30) | (1 << COM3A1);
        TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS30);
        ICR3 = 799;  // 16MHz / (1 * (799 + 1)) = 20kHz
    }
    // Configura Timer4 (pin 7, 8) per 20kHz se usato
    else if (pwmPin == 7 || pwmPin == 8) {
        TCCR4A = 0;
        TCCR4B = 0;
        TCNT4 = 0;
        TCCR4A = (1 << WGM41) | (1 << WGM40) | (1 << COM4B1);
        TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS40);
        ICR4 = 799;  // 16MHz / (1 * (799 + 1)) = 20kHz
    }
    #endif
}

void MotorBase::setSpeed(float speed) {
    targetSpeed_ = constrain(speed, -100.0f, 100.0f);
}

void MotorBase::setSpeedImmediate(float speed) {
    setSpeed(speed);
    currentSpeed_ = targetSpeed_;
    applySpeed();
}

void MotorBase::updateSpeed() {
    if (!enabled_) return;
    
    unsigned long now = micros();
    float dt = (now - lastUpdateTime_) / 1000000.0f;  // in secondi
    lastUpdateTime_ = now;
    
    if (dt <= 0 || dt > 0.1f) return;  // Evita valori anomali
    
    if (abs(currentSpeed_ - targetSpeed_) < 0.5f) {
        currentSpeed_ = targetSpeed_;  // Soglia di arresto
    } else {
        float accel = (targetSpeed_ > currentSpeed_) ? maxAcceleration_ : -maxDeceleration_;
        currentSpeed_ += accel * dt;
        
        // Limita la velocità in base alla direzione
        if (accel > 0 && currentSpeed_ > targetSpeed_) currentSpeed_ = targetSpeed_;
        if (accel < 0 && currentSpeed_ < targetSpeed_) currentSpeed_ = targetSpeed_;
    }
    
    applySpeed();
}

void MotorBase::applySpeed() {
    if (!enabled_) {
        digitalWrite(pwmPin, LOW);
        digitalWrite(dirPin, LOW);
        return;
    }
    
    int absSpeed = abs(currentSpeed_);
    int pwmSpeed = map(absSpeed, 0, 100, 0, 255);
    
    // Applica la direzione in base a reversed_
    bool forward = (currentSpeed_ >= 0);
    if (reversed_) {
        forward = !forward;
    }
    
    digitalWrite(dirPin, forward ? HIGH : LOW);
    analogWrite(pwmPin, pwmSpeed);
}

void MotorBase::stop() {
    targetSpeed_ = 0;
}

void MotorBase::brake() {
    targetSpeed_ = 0;
    currentSpeed_ = 0;
    digitalWrite(pwmPin, LOW);
    digitalWrite(dirPin, LOW);
}

bool MotorBase::isMoving() const {
    return abs(currentSpeed_) > 0.1f;
}

// Le implementazioni di setEnabled e setReversed sono già nella dichiarazione della classe (inline)
