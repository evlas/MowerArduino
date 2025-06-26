#include "MotorBase.h"

// Helper per conversione percentuale -> PWM usando valori da config.h
static inline int16_t percentToPwm(int8_t percent) {
    percent = constrain(percent, -100, 100);
    return map(percent, -100, 100, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
}

MotorBase::MotorBase(uint8_t pin1, uint8_t pin2, bool reversed) 
    : pin1(pin1), pin2(pin2), currentSpeed_(0), targetSpeed_(0), isEnabled(false), isReversed(reversed) {
    // Determina automaticamente la modalità in base ai pin forniti
    if (pin2 == 255) {
        mode = MotorMode::SINGLE_PIN;
    } else {
        mode = MotorMode::H_BRIDGE;  // Modalità predefinita per 2 pin
    }
}

void MotorBase::begin() {
    // Configura i pin di uscita
    if (pin1 != 255) {
        pinMode(pin1, OUTPUT);
        digitalWrite(pin1, LOW);
    }
    
    if (mode != MotorMode::SINGLE_PIN && pin2 != 255) {
        pinMode(pin2, OUTPUT);
        digitalWrite(pin2, LOW);
    }
    
    isEnabled = true;
    
    #if defined(__AVR_ATmega2560__)
    // Configura Timer3 (pin 5) per 20kHz
    if (pin1 == 5 || pin2 == 5) {
        TCCR3A = 0;
        TCCR3B = 0;
        TCNT3 = 0;
        TCCR3A = (1 << WGM31) | (1 << WGM30) | (1 << COM3A1);
        TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS30);
        ICR3 = 799;  // 16MHz / (1 * (799 + 1)) = 20kHz
    }
    
    // Configura Timer4 (pin 7) per 20kHz
    if (pin1 == 7 || pin2 == 7) {
        TCCR4A = 0;
        TCCR4B = 0;
        TCNT4 = 0;
        TCCR4A = (1 << WGM41) | (1 << WGM40) | (1 << COM4B1);
        TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS40);
        ICR4 = 799;  // 16MHz / (1 * (799 + 1)) = 20kHz
    }
    #endif
}

void MotorBase::setSpeed(int speed) {
    // Constrain to -799 to 799 range for 20kHz PWM with ICR = 799
    targetSpeed_ = constrain(speed, -799, 799);
    if (isReversed) {
        targetSpeed_ = -targetSpeed_;
    }
}

void MotorBase::setSpeedImmediate(int speed) {
    setSpeed(speed);
    currentSpeed_ = targetSpeed_;
    applySpeed();
}

void MotorBase::updateSpeed() {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime_ < 10) return;  // Aggiorna a ~100Hz
    
    float dt = (currentTime - lastUpdateTime_) / 1000.0f;  // Secondi
    lastUpdateTime_ = currentTime;
    
    if (currentSpeed_ != targetSpeed_) {
        float direction = (targetSpeed_ > currentSpeed_) ? 1.0f : -1.0f;
        float maxChange = (direction > 0 ? maxAcceleration_ : maxDeceleration_) * dt;
        
        currentSpeed_ += direction * maxChange;
        
        // Evita l'overshoot
        if ((direction > 0 && currentSpeed_ > targetSpeed_) ||
            (direction < 0 && currentSpeed_ < targetSpeed_)) {
            currentSpeed_ = targetSpeed_;
        }
        
        applySpeed();
    }
}

void MotorBase::applySpeed() {
    if (!isEnabled) {
        analogWrite(pin1, 0);
        if (mode != MotorMode::SINGLE_PIN) {
            digitalWrite(pin2, LOW);
        }
        return;
    }
    
    int absSpeed = abs(currentSpeed_);
    bool isForward = currentSpeed_ >= 0;
    
    switch (mode) {
        case MotorMode::SINGLE_PIN:
            analogWrite(pin1, absSpeed);
            break;
            
        case MotorMode::DUAL_PIN:
            analogWrite(pin1, absSpeed);
            digitalWrite(pin2, isForward ? LOW : HIGH);
            break;
            
        case MotorMode::H_BRIDGE:
            if (isForward) {
                analogWrite(pin1, absSpeed);
                analogWrite(pin2, 0);
            } else {
                analogWrite(pin1, 0);
                analogWrite(pin2, absSpeed);
            }
            break;
    }
}

void MotorBase::stop() {
    targetSpeed_ = 0;
    currentSpeed_ = 0;
    applySpeed();
}

void MotorBase::brake() {
    if (mode == MotorMode::H_BRIDGE) {
        // Per un ponte H, cortocircuitare i terminali del motore
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, HIGH);
    } else {
        // Per altri tipi, semplicemente fermare il motore
        stop();
    }
}

bool MotorBase::isMoving() const {
    return abs(currentSpeed_) > 0;
}

void MotorBase::setEnabled(bool enabled) {
    isEnabled = enabled;
    if (!enabled) {
        stop();
    }
}

void MotorBase::setReversed(bool reversed) {
    if (isReversed != reversed) {
        isReversed = reversed;
        setSpeed(currentSpeed_);
    }
}
