#include <Arduino.h>
#include <math.h>
#include "Mower.h"
#include "../../config.h"
#include "../../pin_config.h"
#include "../statemachine/MowerStateMachine.h"
#include "../sensors/UltrasonicSensors.h"
#include "../sensors/BumpSensors.h"
#include "../sensors/PerimeterSensors.h"
#include "../sensors/RainSensor.h"
#include "../sensors/BatterySensor.h"
#include "../sensors/IMU.h"
#include "../sensors/GPS.h"
#include "../actuators/Relay.h"

// Debug macros
#ifdef DEBUG_MODE
#define DEBUG_PRINT(x)     if (SERIAL_DEBUG) SERIAL_DEBUG.print(x)
#define DEBUG_PRINTLN(x)  if (SERIAL_DEBUG) SERIAL_DEBUG.println(x)
#define DEBUG_PRINTF(...) if (SERIAL_DEBUG) SERIAL_DEBUG.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif

// Navigation constants from config.h
const float ROTATION_ANGLE = ROTATION_ANGLE_DEG * DEG_TO_RAD; // Convert degrees to radians
// ANGLE_TOLERANCE is now defined in Mower.h


Mower::Mower() :
    // Initialize sensors
    rainSensor_(RAIN_SENSOR_PIN),
    batterySensor_(),
    imu_(IMU_SDA_PIN, IMU_SCL_PIN),
    gps_(GPS_RX_PIN, GPS_TX_PIN),
    
    // Initialize motors
    leftMotor_(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_IN1_PIN, LEFT_MOTOR_IN2_PIN, 
             LEFT_MOTOR_ENCODER_A_PIN, LEFT_MOTOR_ENCODER_B_PIN, WHEEL_DIAMETER, ENCODER_CPR),
    rightMotor_(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_IN1_PIN, RIGHT_MOTOR_IN2_PIN, 
              RIGHT_MOTOR_ENCODER_A_PIN, RIGHT_MOTOR_ENCODER_B_PIN, WHEEL_DIAMETER, ENCODER_CPR),
    bladeMotor_(BLADE_MOTOR_PIN),
    
    // Initialize relays
    motorsRelay_(MOTOR_RELAY_PIN),
    
    // Initialize sensor arrays
    ultrasonicSensors_(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PINS, NUM_ULTRASONIC_SENSORS),
    bumpSensors_(BUMP_SENSOR_PINS, NUM_BUMP_SENSORS),
    perimeterSensors_(PERIMETER_LEFT_PIN, PERIMETER_RIGHT_PIN),
    
    // Initialize position manager (will be set up in begin())
    positionManager_(nullptr),
    
    // Initialize state machine and maneuver (will be created in begin())
    stateMachine_(nullptr),
    maneuver_(nullptr),
    
    // Initialize timing
    lastLoopTime_(0) {
    
    // Inizializzazione aggiuntiva se necessaria
    
    // Inizializza il relay nello stato spento
    motorsRelay_.off();
}

void Mower::begin() {
    // Initialize serial debug if enabled
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD);
    while (!SERIAL_DEBUG) {
        ; // Wait for serial connection
    }
    #endif
    
    // Initialize I2C for IMU and other I2C devices
    Wire.begin();
    
    // Initialize IMU if enabled
    #ifdef ENABLE_IMU
    imu_.begin();
    // Configure MPU6050 if needed
    Wire.beginTransmission(0x68);  // MPU6050 default address
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // Wake up MPU-6050
    Wire.endTransmission(true);
    #endif
    
    // Initialize components
    motorsRelay_.begin();
    leftMotor_.begin();
    rightMotor_.begin();
    bladeMotor_.begin();
    
    // Set motor accelerations
    leftMotor_.setAcceleration(MAX_LINEAR_ACCEL, MAX_LINEAR_DECEL);
    rightMotor_.setAcceleration(MAX_LINEAR_ACCEL, MAX_LINEAR_DECEL);
    
    // Initialize sensors
    ultrasonicSensors_.begin();
    bumpSensors_.begin();
    perimeterSensors_.begin();
    // rainSensor doesn't have a begin() method
    // batterySensor doesn't have a begin() method
    
    // Initialize position manager
    positionManager_.begin();
    
    // Initialize state machine
    if (stateMachine_) {
        stateMachine_->begin();
    }
    
    // Set initial state
    stopMotors();
    bladeMotor_.setSpeed(0);
    
    // Initialize timing variables
    lastSensorUpdate = millis();
    lastTelemetryUpdate = millis();
    lastNavigationUpdate = millis();
    lastBatteryUpdate = millis();
    lastMotorUpdate = millis();
    lastOdometryUpdate = millis();
    lastSafetyCheckUpdate = millis();
    lastLoopTime = millis();
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.println(F("Mower initialization complete"));
    #endif
}

void Mower::update(unsigned long currentTime) {
    // If no time provided, use current time
    if (currentTime == 0) {
        currentTime = millis();
    }
    
    // 1. Update odometry (10ms = 100Hz)
    if (currentTime - lastOdometryUpdate >= ODOMETRY_UPDATE_INTERVAL) {
        // Update position from odometry
        positionManager_.update();
        RobotPosition pos = positionManager_.getPosition();
        currentX = pos.x;
        currentY = pos.y;
        currentAngle = pos.theta;
    }
    
    // 2. Update sensors (50ms = 20Hz)
    if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
        lastSensorUpdate = currentTime;
        updateSensors();
    }
    
    // 3. Update safety checks (50ms = 20Hz)
    if (currentTime - lastSafetyCheckUpdate >= SAFETY_CHECK_INTERVAL) {
        lastSafetyCheckUpdate = currentTime;
        checkSafety();
    }
    
    // 4. Update motors (every loop)
    leftMotor_.update();
    rightMotor_.update();
    bladeMotor_.update();
    
    // 5. Update navigation if enabled (100ms = 10Hz)
    if (currentTime - lastNavigationUpdate >= NAVIGATION_UPDATE_INTERVAL) {
        lastNavigationUpdate = currentTime;
        updateNavigationTimed();
    }
    
    // 6. Update battery status (5000ms = 0.2Hz)
    if (currentTime - lastBatteryUpdate >= BATTERY_UPDATE_INTERVAL) {
        lastBatteryUpdate = currentTime;
        updateBattery();
    }
    
    // 7. Update telemetry if enabled (1000ms = 1Hz)
    #ifdef ENABLE_TELEMETRY
    if (currentTime - lastTelemetryUpdate >= TELEMETRY_UPDATE_INTERVAL) {
        lastTelemetryUpdate = currentTime;
        updateTelemetry();
    }
    #endif
    
    // 8. Update state machine (every loop)
    stateMachine_->update();
    
    // 9. Calculate loop time and delay if needed
    unsigned long loopTime = millis() - currentTime;
    if (loopTime < MAIN_LOOP_DELAY) {
        delay(MAIN_LOOP_DELAY - loopTime);
    } else {
        // Loop is taking too long, log a warning
        #ifdef DEBUG_MODE
            SERIAL_DEBUG.print(F("Warning: Loop time exceeded: "));
            SERIAL_DEBUG.print(loopTime);
            SERIAL_DEBUG.println(F("ms"));
        #endif
    }
}

void Mower::updateSensors() {
    // Update all sensors
    // rainSensor doesn't have an update() method
    // batterySensor doesn't have an update() method
    // ultrasonicSensors doesn't have an update() method
    // bumpSensors doesn't have an update() method
    // perimeterSensors doesn't have an update() method
    
    // Update position from odometry/IMU
    positionManager_.update();
    
    // Update timing
    lastSensorUpdate = millis();
}

void Mower::checkSafety() {
    // Check for emergency stop conditions
    // Check bump sensors
    bool leftBump = bumpSensors_.isLeftBump();
    bool rightBump = bumpSensors_.isRightBump();
    if (leftBump || rightBump) {
        emergencyStop();
        #ifdef DEBUG_MODE
            SERIAL_DEBUG.println(F("Emergency stop: Bumper triggered"));
        #endif
        return;
    }
    
    // Check for lift detection
    if (isLifted()) {
        emergencyStop();
        #ifdef DEBUG_MODE
            SERIAL_DEBUG.println(F("Emergency stop: Mower lifted"));
        #endif
        return;
    }
    
    // Check for perimeter wire
    if (isBorderDetected()) {
        handleBorder();
        #ifdef DEBUG_MODE
            SERIAL_DEBUG.println(F("Border detected"));
        #endif
    }
    
    // Check for obstacles
    if (checkObstacles()) {
        handleObstacle();
        #ifdef DEBUG_MODE
            SERIAL_DEBUG.println(F("Obstacle detected"));
        #endif
    }
}

void Mower::updateBattery() {
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();
    
    // Update battery status at defined interval
    if (now - lastUpdate < BATTERY_UPDATE_INTERVAL) {
        return;
    }
    lastUpdate = now;
    
    // Read battery status
    float voltage = batterySensor_.readVoltage();
    float current = batterySensor_.readCurrent();
    
    // Calculate per-cell voltage and state of charge
    float cellVoltage = voltage / BATTERY_CELLS;
    float soc = 100.0f * (cellVoltage - BATTERY_EMPTY_VOLTAGE) / 
               (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE);
    soc = constrain(soc, 0.0f, 100.0f);
    
    // Update charging state based on current direction
    bool wasCharging = isCharging_;
    isCharging_ = (current < -0.1f); // Negative current means charging
    
    // Handle charging state changes
    if (isCharging_ != wasCharging) {
        #ifdef DEBUG_MODE
        if (SERIAL_DEBUG) {
            SERIAL_DEBUG.print(F("Charging state: "));
            SERIAL_DEBUG.println(isCharging_ ? F("STARTED") : F("STOPPED"));
            SERIAL_DEBUG.print(F("Current: "));
            SERIAL_DEBUG.print(current, 3);
            SERIAL_DEBUG.println(F("A"));
        }
        #endif
    }
    
    // Handle charging logic when at docking station
    if (isAtDockingStation()) {
        // Calculate charge complete threshold (0.1C of battery capacity)
        float chargeCompleteCurrent = 0.1f * BATTERY_CAPACITY; // 0.1C
        
        // If battery is full or current drops below threshold, stop charging
        if (isCharging_ && (soc >= 95.0f || abs(current) < chargeCompleteCurrent)) {
            enableCharging(false);
            #ifdef DEBUG_MODE
            if (SERIAL_DEBUG) {
                SERIAL_DEBUG.println(F("Battery fully charged, stopping charge"));
            }
            #endif
        }
        // If battery not full and not charging (but should be), start charging
        else if (!isCharging_ && soc < 95.0f && !chargingEnabled_) {
            enableCharging(true);
        }
    }
    
    // Handle battery state changes and safety checks
    if (isBatteryCritical()) {
        if (stateMachine_ && stateMachine_->getCurrentState() != MowerState::DOCKING) {
            stateMachine_->requestStateChange(MowerState::DOCKING);
            #ifdef DEBUG_MODE
            if (SERIAL_DEBUG) {
                SERIAL_DEBUG.print(F("Battery CRITICAL ("));
                SERIAL_DEBUG.print(voltage, 2);
                SERIAL_DEBUG.print(F("V, "));
                SERIAL_DEBUG.print(soc, 1);
                SERIAL_DEBUG.println(F("%) - Returning to dock"));
            }
            #endif
        }
    } 
    else if (isBatteryLow()) {
        #ifdef DEBUG_MODE
        static unsigned long lastLowBatteryWarning = 0;
        if (now - lastLowBatteryWarning > 60000) { // Every minute
            if (SERIAL_DEBUG) {
                SERIAL_DEBUG.print(F("Battery LOW ("));
                SERIAL_DEBUG.print(voltage, 2);
                SERIAL_DEBUG.print(F("V, "));
                SERIAL_DEBUG.print(soc, 1);
                SERIAL_DEBUG.println(F("%)"));
            }
            lastLowBatteryWarning = now;
        }
        #endif
    }
    
    // Log battery status periodically
    #ifdef DEBUG_MODE
    static unsigned long lastLogTime = 0;
    if (now - lastLogTime >= 60000) { // Every minute
        if (SERIAL_DEBUG) {
            SERIAL_DEBUG.print(F("Battery: "));
            SERIAL_DEBUG.print(voltage, 2);
            SERIAL_DEBUG.print(F("V ("));
            SERIAL_DEBUG.print(cellVoltage, 2);
            SERIAL_DEBUG.print(F("V/cell), "));
            SERIAL_DEBUG.print(current, 3);
            SERIAL_DEBUG.print(F("A, "));
            SERIAL_DEBUG.print(soc, 1);
            SERIAL_DEBUG.println(F("%)"));
        }
        lastLogTime = now;
    }
    #endif
}

void Mower::updateNavigationTimed() {
    if (isEmergencyStop_) {
        stopMotors();
        return;
    }
    
    // Only update navigation if in automatic mode and not in manual control
    if (!isManualControl_) {
        updatePosition();
        updateNavigation();
    }
}

void Mower::updateTelemetry() {
    // Send telemetry data
    #ifdef DEBUG_MODE
        SERIAL_DEBUG.println(F("Sending telemetry"));
    #endif
    
    // TODO: Implement actual telemetry sending
    // Example:
    // telemetry.send("battery", batterySensor_.getVoltage());
    // telemetry.send("state", static_cast<int>(stateMachine_->getCurrentState()));
}

// ===== State Machine Interface Methods =====

void Mower::stopMotors() {
    // Stop all motors
    leftMotor_.setSpeed(0);
    rightMotor_.setSpeed(0);
    bladeMotor_.setSpeed(0);
    
    #ifdef DEBUG_MOTORS
    if (SERIAL_DEBUG) {
        SERIAL_DEBUG.println(F("All motors stopped"));
    }
    #endif
}

void Mower::begin() {
    // Initialize serial communication if in debug mode
    #ifdef DEBUG_MODE
    if (SERIAL_DEBUG) {
        SERIAL_DEBUG.begin(115200);
        while (!SERIAL_DEBUG) {
            ; // wait for serial port to connect
        }
        DEBUG_PRINTLN(F("Mower initialization started"));
    }
    #endif
    
    // Initialize motors
    DEBUG_PRINTLN(F("Initializing motors..."));
    leftMotor_.begin();
    rightMotor_.begin();
    bladeMotor_.begin();
    
    // Initialize sensors
    DEBUG_PRINTLN(F("Initializing sensors..."));
    
    if (!batterySensor_.begin()) {
        DEBUG_PRINTLN(F("Error initializing battery sensor!"));
    }
    
    if (!imu_.begin()) {
        DEBUG_PRINTLN(F("Error initializing IMU!"));
    }
    
    // Initialize position manager
    DEBUG_PRINTLN(F("Initializing position manager..."));
    positionManager_ = new PositionManager(&leftMotor_, &rightMotor_, &imu_, &gps_, WHEEL_BASE);
    if (positionManager_) {
        positionManager_->begin();
    } else {
        DEBUG_PRINTLN(F("Failed to create position manager!"));
    }
    
    // Create maneuver controller
    DEBUG_PRINTLN(F("Initializing maneuver controller..."));
    maneuver_ = new Maneuver(&leftMotor_, &rightMotor_, BLADE_WIDTH, WHEEL_BASE, MAX_LINEAR_VELOCITY);
    
    // Initialize state machine
    DEBUG_PRINTLN(F("Initializing state machine..."));
    stateMachine_ = new MowerStateMachine(*this);
    if (stateMachine_) {
        stateMachine_->begin();
    } else {
        DEBUG_PRINTLN(F("Failed to create state machine!"));
    }
    
    // Initialize relays
    DEBUG_PRINTLN(F("Initializing relays..."));
    motorsRelay_.begin();
    
    // Set initial state
    DEBUG_PRINTLN(F("Setting initial state..."));
    stopMotors();
    navigationMode_ = NavigationMode::STOPPED;
    
    // Initialize sensor timers
    lastSensorUpdate_ = millis();
    lastMotorUpdate_ = millis();
    lastBladeUpdate_ = millis();
    lastNavigationUpdate_ = millis();
    lastObstacleTime_ = millis();
    lastDebugPrint_ = millis();
    lastLoopTime_ = millis();
    
    DEBUG_PRINTLN(F("Mower initialization complete"));
    
    // Update state
    isMowing = false;
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.println(F("All motors and maneuvers stopped"));
    #endif
}

void Mower::enableCharging(bool enable) {
    if (enable == chargingEnabled_) {
        return;  // Già nello stato richiesto
    }
    
    chargingEnabled_ = enable;
    digitalWrite(CHARGING_RELAY_PIN, enable ? HIGH : LOW);
    
    if (enable) {
        chargingStartTime_ = millis();
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println(F("Charging enabled"));
        #endif
    } else {
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println(F("Charging disabled"));
        #endif
    }
}

void Mower::startDocking() {
    // Stop any current movement
    stopMotors();
    
    // Set navigation mode to stopped
    setNavigationMode(NavigationMode::STOPPED);
    
    // Set mowing flag to false
    isMowing = false;
    
    // Transition to DOCKING state
    stateMachine_->requestStateChange(MowerState::DOCKING);
    
    #ifdef DEBUG_MODE
    if (SERIAL_DEBUG) {
        SERIAL_DEBUG.println(F("Docking started"));
    }
    #endif
}

void Mower::startMowing() {
    // Check if it's safe to start mowing
    if (isBatteryLow() || isRainDetected()) {
        Serial.println("Cannot start mowing: ");
        if (isBatteryLow()) Serial.println("- Battery low");
        if (isRainDetected()) Serial.println("- Rain detected");
        return;
    }
    
    // Start the blade motor
    bladeMotor_.start();
    
    // Start with random navigation
    setNavigationMode(Mower::NavigationMode::RANDOM);
    
    // Transition to MOWING state
    stateMachine_->requestStateChange(MowerState::MOWING);
    
    Serial.println("Mowing started");
}

Mower::~Mower() {
    // Clean up dynamically allocated objects
    delete stateMachine_;
    delete maneuver_;
}

void Mower::setNavigationMode(NavigationMode mode) {
    currentNavigationMode = mode;
    if (mode == NavigationMode::STOPPED) {
        stopMotors();
    }
}

Mower::NavigationMode Mower::getNavigationMode() const {
    return currentNavigationMode;
}


bool Mower::checkPerimeter() {
    return perimeterSensors_.isDetected();
}

bool Mower::checkObstacles() {
    // Check ultrasonic sensors
    for (int i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
        float distance = ultrasonicSensors_.getDistance(i);
        if (distance > 0 && distance < OBSTACLE_DISTANCE_THRESHOLD) {
            return true;
        }
    }
    
    // Check bump sensors
    if (bumpSensors_.checkAny()) {
        return true;
    }
    
    return false;
}

void Mower::emergencyStop() {
    stopMotors();
    bladeMotor_.setSpeed(0);
    isEmergencyStop_ = true;
    
    // Spegni il relay in caso di emergenza
    motorsRelay_.off();
    
    #ifdef DEBUG_MODE
    SERIAL_DEBUG.println(F("EMERGENCY STOP ACTIVATED!"));
    SERIAL_DEBUG.println(F("Motors relay turned OFF for safety"));
    #endif
}

// Sensor checks
bool Mower::isAtDockingStation() const {
    // Check if current is negative (charging)
    return isCharging_;
}

bool Mower::isCharging() const {
    // Return the current charging state
    // Note: The actual state is updated in updateBattery()
    return isCharging_;
}

bool Mower::isBatteryFull() {
    // Check if battery voltage is above threshold
    float voltage = batterySensor_.readVoltage();
    return voltage >= BATTERY_FULL_VOLTAGE;
}

bool Mower::isBatteryLow() {
    float voltage = batterySensor_.readVoltage();
    return (voltage < BATTERY_LOW_THRESHOLD);
}

bool Mower::isBatteryCritical() {
    float voltage = batterySensor_.readVoltage();
    return (voltage < BATTERY_CRITICAL_THRESHOLD);
}

bool Mower::isRainDetected() {
    return rainSensor_.isRaining();
}

bool Mower::isLifted() {
    // Check if mower is lifted (using IMU or other sensors)
    return false; // Implement actual check
}

bool Mower::isBorderDetected() {
    return perimeterSensors_.isDetected();
}

bool Mower::isCollisionDetected() {
    return bumpSensors_.checkAny() || checkObstacles();
}

// State Machine Control
bool Mower::shouldStartMowing() {
    return !isBatteryLow() && !isRainDetected() && !isLifted();
}

// Helper function to normalize angle to [-PI, PI]
float Mower::normalizeAngle(float angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

void Mower::updateTargetPosition(const RobotPosition& currentPos) {
    // This is a simple implementation that just sets a target in front of the robot
    // You can replace this with more sophisticated path planning logic
    const float targetDistance = 1.0f; // 1 meter ahead
    
    targetX_ = currentPos.x + (targetDistance * cos(currentPos.theta));
    targetY_ = currentPos.y + (targetDistance * sin(currentPos.theta));
    
    // Make sure the target is within the lawn boundaries
    targetX_ = constrain(targetX_, 0.0f, LAWN_SIZE_X);
    targetY_ = constrain(targetY_, 0.0f, LAWN_SIZE_Y);
    
    #ifdef DEBUG_NAVIGATION
    if (SERIAL_DEBUG && millis() - lastDebugPrint_ > 1000) {
        SERIAL_DEBUG.print(F("New target: ("));
        SERIAL_DEBUG.print(targetX_);
        SERIAL_DEBUG.print(", ");
        SERIAL_DEBUG.print(targetY_);
        SERIAL_DEBUG.println(")");
    }
    #endif
}

void Mower::onTargetReached() {
    // This method is called when the robot reaches its target position
    // You can add logic here to decide the next move
    
    // For now, just set a new target in a random direction
    updateTargetPosition(positionManager_.getPosition());
    
    #ifdef DEBUG_NAVIGATION
    if (SERIAL_DEBUG) {
        SERIAL_DEBUG.println(F("Target reached, setting new target"));
    }
    #endif
}

void Mower::handleObstacle() {
    // Simple obstacle avoidance - rotate 90 degrees to the right
    static unsigned long lastObstacleTime = 0;
    unsigned long currentTime = millis();
    
    // Only react to obstacles every 500ms to avoid rapid changes
    if (currentTime - lastObstacleTime < 500) {
        return;
    }
    
    lastObstacleTime = currentTime;
    
    // Stop the robot
    stopMotors();
    
    // Get current orientation
    RobotPosition currentPos = positionManager_.getPosition();
    
    // Calculate new target angle (90 degrees to the right)
    float newAngle = normalizeAngle(currentPos.theta - HALF_PI);
    
    // Set the target angle
    targetAngle_ = newAngle;
    
    // Update the target position based on the new angle
    updateTargetPosition(currentPos);
    
    #ifdef DEBUG_NAVIGATION
    if (SERIAL_DEBUG) {
        SERIAL_DEBUG.print(F("Obstacle detected, turning to: "));
        SERIAL_DEBUG.print(newAngle * RAD_TO_DEG);
        SERIAL_DEBUG.println("°");
    }
    #endif
}

bool Mower::isNavigating() const {
    return navigationMode_ != Mower::NavigationMode::STOPPED && navigationMode_ != Mower::NavigationMode::MANUAL;
}

void Mower::setBladeSpeed(float speed) {
    // Imposta la velocità del motore della lama con un valore tra 0.0 e 1.0
    if (isEmergencyStop_) {
        bladeMotor_.setSpeed(0);
        return;
    }
    
    // Assicurati che la velocità sia nel range corretto
    speed = constrain(speed, 0.0f, 1.0f);
    bladeMotor_.setSpeed(static_cast<int>(speed * 100.0f));
}

void Mower::setMotorSpeed(float left, float right) {
    // Safety check - don't move if in emergency stop
    if (isEmergencyStop_) {
        leftMotor_.setSpeed(0);
        rightMotor_.setSpeed(0);
        return;
    }
    
    // Convert from -100..100 to motor control range
    left = constrain(left, -100, 100);
    right = constrain(right, -100, 100);
    
    // Apply motor speeds with direction
    leftMotor.setSpeed(left);
    rightMotor.setSpeed(right);
    
    #ifdef DEBUG_MOTORS
    static unsigned long lastPrint = 0;
    if (SERIAL_DEBUG && millis() - lastPrint > 500) {
        SERIAL_DEBUG.print(F("Motors: L="));
        SERIAL_DEBUG.print(left);
        SERIAL_DEBUG.print(F("%, R="));
        SERIAL_DEBUG.print(right);
        SERIAL_DEBUG.println(F("%"));
        lastPrint = millis();
    }
    #endif
}

void Mower::navigateRandom() {
    if (obstacleAvoidanceState_ > 0) {
        handleObstacle();
        return;
    }

    // Simple random movement pattern
    unsigned long currentTime = millis();
    if (currentTime - lastObstacleTime_ > 5000) { // Change direction every 5 seconds
        lastObstacleTime_ = currentTime;
        
        // Random angle between -45 and 45 degrees
        float angle = (random(91) - 45) * (PI / 180.0f);
        targetAngle_ = positionManager_->getYaw() + angle;
        
        // Normalize angle to -PI..PI
        while (targetAngle_ > PI) targetAngle_ -= 2 * PI;
        while (targetAngle_ < -PI) targetAngle_ += 2 * PI;
    }
    
    // Move forward with slight curve based on target angle
    float currentYaw = positionManager_->getYaw();
    float angleDiff = atan2(sin(targetAngle_ - currentYaw), cos(targetAngle_ - currentYaw));
    
    // Set motor speeds based on angle difference
    // Usa il 50% della velocità massima come base
    float baseSpeed = 0.5f; // 50% speed
    float turnFactor = 0.4f * (angleDiff / (PI / 4.0f)); // Fino al 40% di differenza
    
    // Imposta le velocità con limiti tra -1.0 e 1.0
    float leftSpeed = constrain(baseSpeed + turnFactor, -1.0f, 1.0f);
    float rightSpeed = constrain(baseSpeed - turnFactor, -1.0f, 1.0f);
    
    setMotorSpeed(leftSpeed, rightSpeed);
}

void Mower::navigateParallel() {
    if (obstacleAvoidanceState_ > 0) {
        handleObstacle();
        return;
    }

    // Simple parallel line pattern (simplified)
    static bool movingForward = true;
    static float lastTurnY = 0.0f;
    
    RobotPosition pos = positionManager_->getPosition();
    
    if (movingForward) {
        // Move forward until we reach the end of the area
        if (pos.y >= LAWN_SIZE_Y - 0.5f) {
            // Reached the end, turn around
            movingForward = false;
            lastTurnY = pos.y;
            // Turn 180 degrees
            targetAngle_ = positionManager_->getYaw() + PI;
            // Normalize angle to -PI..PI
            while (targetAngle_ > PI) targetAngle_ -= 2 * PI;
            while (targetAngle_ < -PI) targetAngle_ += 2 * PI;
            
            // Stop during turn
            setMotorSpeed(0, 0);
            delay(500); // Small pause before starting turn
        } else {
            // Move forward with slight correction to maintain straight line
            float currentYaw = positionManager_->getYaw();
            float angleDiff = atan2(sin(-currentYaw), cos(-currentYaw));
            
            // 60% speed forward with correction
            float baseSpeed = 0.6f;
            // Limit correction to 30% of base speed
            float correctionFactor = constrain(0.3f * angleDiff / (PI/4.0f), -0.3f, 0.3f);
            
            float leftSpeed = constrain(baseSpeed - correctionFactor, -1.0f, 1.0f);
            float rightSpeed = constrain(baseSpeed + correctionFactor, -1.0f, 1.0f);
            
            setMotorSpeed(leftSpeed, rightSpeed);
        }
    } else {
        // Move backward until we reach the start of the area
        if (pos.y <= 0.5f) {
            // Reached the start, turn around and shift over
            movingForward = true;
            // Shift over by blade width
            targetX_ = fmod(pos.x + BLADE_WIDTH, LAWN_SIZE_X);
            // Turn back to forward direction
            targetAngle_ = 0.0f;
            
            // Stop during turn
            setMotorSpeed(0, 0);
            delay(500); // Small pause before starting next line
        } else {
            // Move backward with slight correction
            float currentYaw = positionManager_->getYaw();
            float angleDiff = atan2(sin(PI - currentYaw), cos(PI - currentYaw));
            
            // 50% speed backward with correction
            float baseSpeed = -0.5f;
            // Limit correction to 25% of base speed
            float correctionFactor = constrain(0.25f * angleDiff / (PI/4.0f), -0.25f, 0.25f);
            
            float leftSpeed = constrain(baseSpeed - correctionFactor, -1.0f, 1.0f);
            float rightSpeed = constrain(baseSpeed + correctionFactor, -1.0f, 1.0f);
            
            setMotorSpeed(leftSpeed, rightSpeed);
        }
    }
}

void Mower::navigate() {
    // Skip navigation if not in a navigating state
    if (!isNavigating()) {
        stopMotors();
        return;
    }
    
    // Get current position from position manager
    RobotPosition currentPos = positionManager.getPosition();
    
    // Check if position is valid
    if (!currentPos.isValid) {
        #ifdef DEBUG_MODE
        if (SERIAL_DEBUG) {
            SERIAL_DEBUG.println(F("Warning: Invalid position data"));
        }
        #endif
        stopMotors();
        return;
    }
    
    // Update target position if needed (e.g., for pattern mowing)
    updateTargetPosition(currentPos);
    
    // Calculate vector to target
    float dx = targetX_ - currentPos.x;
    float dy = targetY_ - currentPos.y;
    float distance = sqrt(dx*dx + dy*dy);
    
    // Check if we've reached the target
    if (distance < POSITION_TOLERANCE) {
        // We've reached the target position
        stopMotors();
        onTargetReached();
        return;
    }
    
    // Calculate desired heading to target
    float targetHeading = atan2(dy, dx);
    
    // Calculate heading error (difference between current and target heading)
    float headingError = normalizeAngle(targetHeading - currentPos.theta);
    
    // Calculate turn rate using a simple P controller
    float turnRate = constrain(headingError * 0.8f, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    
    // Calculate forward speed - reduce speed when turning
    float forwardSpeed = constrain(distance * 0.3f, 0.0f, MAX_LINEAR_VELOCITY);
    forwardSpeed *= (1.0f - (fabs(headingError) / PI));  // Reduce speed when facing away from target
    
    // Apply motor speeds
    float leftSpeed = forwardSpeed - (turnRate * WHEEL_BASE / 2.0f);
    float rightSpeed = forwardSpeed + (turnRate * WHEEL_BASE / 2.0f);
    
    // Normalize speeds to maintain direction
    float maxSpeed = max(fabs(leftSpeed), fabs(rightSpeed));
    if (maxSpeed > 0.1f) {
        float scale = min(MAX_LINEAR_VELOCITY, maxSpeed) / maxSpeed;
        leftSpeed *= scale;
        rightSpeed *= scale;
    }
    
    // Set motor speeds
    setMotorSpeed(leftSpeed, rightSpeed);
    
    #ifdef DEBUG_NAVIGATION
    if (SERIAL_DEBUG && millis() - lastDebugPrint > 1000) {
        SERIAL_DEBUG.print(F("Pos: ("));
        SERIAL_DEBUG.print(currentPos.x);
        SERIAL_DEBUG.print(", ");
        SERIAL_DEBUG.print(currentPos.y);
        SERIAL_DEBUG.print(") Target: (");
        SERIAL_DEBUG.print(targetX_);
        SERIAL_DEBUG.print(", ");
        SERIAL_DEBUG.print(targetY_);
        SERIAL_DEBUG.print(") Dist: ");
        SERIAL_DEBUG.print(distance);
        SERIAL_DEBUG.print(" HdgErr: ");
        SERIAL_DEBUG.print(headingError * RAD_TO_DEG);
        SERIAL_DEBUG.println("°");
        lastDebugPrint = millis();
    }
    #endif
}

void Mower::navigateSpiral() {
    // Skip if not in a navigating state
    if (!isNavigating()) {
        stopMotors();
        return;
    }
    
    // Get current position from position manager
    RobotPosition currentPos = positionManager.getPosition();
    
    // Check if position is valid
    if (!currentPos.isValid) {
        #ifdef DEBUG_MODE
        if (SERIAL_DEBUG) {
            SERIAL_DEBUG.println(F("Warning: Invalid position data"));
        }
        #endif
        stopMotors();
        return;
    }
    
    // Simple spiral pattern
    static float radius = 0.0f;
    static float angle = 0.0f;
    
    // Increase radius as we go
    radius += 0.01f; // SPIRAL_STEP
    angle += 0.1f;
    
    // Reset after reaching maximum radius
    if (radius > 5.0f) {
        radius = 0.5f; // Start over with a small radius
    }
    
    // Calculate target point on spiral relative to start position
    float spiralX = radius * cos(angle);
    float spiralY = radius * sin(angle);
    
    // Convert to global coordinates (assuming start at 0,0)
    targetX_ = spiralX;
    targetY_ = spiralY;
    
    // Call the main navigation function to move to the target
    navigate();
    
    #ifdef DEBUG_NAVIGATION
    if (SERIAL_DEBUG && millis() - lastDebugPrint > 1000) {
        SERIAL_DEBUG.print(F("Spiral - Radius: "));
        SERIAL_DEBUG.print(radius);
        SERIAL_DEBUG.print(F("m, Target: ("));
        SERIAL_DEBUG.print(targetX_);
        SERIAL_DEBUG.print(F(","));
        SERIAL_DEBUG.print(targetY_);
        SERIAL_DEBUG.println(F(")"));
        lastDebugPrint = millis();
    }
    #endif
}

bool Mower::checkObstacles() {
    if (!ultrasonicSensors_ && !bumpSensors_) {
        return false;
    }
    
    bool obstacleDetected = false;
    
    // Check ultrasonic sensors
    if (ultrasonicSensors_) {
        for (int i = 0; i < ultrasonicSensors_->getSensorCount(); i++) {
            float distance = ultrasonicSensors_->getDistance(i);
            if (distance > 0 && distance < OBSTACLE_DISTANCE_THRESHOLD) {
                obstacleDetected = true;
                break;
            }
        }
    }
    
    // Check bump sensors
    if (!obstacleDetected && bumpSensors_) {
        obstacleDetected = bumpSensors_->isAnyPressed();
    }
    
    if (obstacleDetected) {
        handleObstacle();
        return true;
    }
    
    return false;
}

void Mower::handleObstacle() {
    unsigned long currentTime = millis();
    
    switch (obstacleAvoidanceState_) {
        case 0: // Start obstacle avoidance
            stopMotors();
            lastObstacleTime_ = currentTime;
            obstacleAvoidanceState_ = 1;
            break;
            
        case 1: // Move backward
            setMotorSpeed(-0.3f, -0.3f);
            if (currentTime - lastObstacleTime_ > 500) { // Back up for 500ms
                lastObstacleTime_ = currentTime;
                obstacleAvoidanceState_ = 2;
                
                // Choose a random direction to turn (left or right)
                targetAngle_ = positionManager_->getYaw() + (random(2) ? ROTATION_ANGLE : -ROTATION_ANGLE);
                // Normalize angle to -PI..PI
                while (targetAngle_ > PI) targetAngle_ -= 2 * PI;
                while (targetAngle_ < -PI) targetAngle_ += 2 * PI;
            }
            break;
            
        case 2: // Turn away from obstacle
            float currentYaw = positionManager_->getYaw();
            float angleDiff = atan2(sin(targetAngle_ - currentYaw), cos(targetAngle_ - currentYaw));
            
            if (fabs(angleDiff) < 0.1f) { // Within ~6 degrees
                obstacleAvoidanceState_ = 0;
                lastObstacleTime_ = currentTime + OBSTACLE_AVOIDANCE_DELAY;
            } else {
                float turnSpeed = constrain(angleDiff * 2.0f, -1.0f, 1.0f);
                setMotorSpeed(-turnSpeed, turnSpeed);
            }
            break;
    }
}

bool Mower::checkPerimeter() {
    if (!perimeterSensors_) {
        return false;
    }
    
    if (perimeterSensors_->isInside()) {
        return false;
    }
    
    // We're outside the perimeter
    handlePerimeter();
    return true;
}

void Mower::handlePerimeter() {
    // Simple perimeter handling: turn 90 degrees and move forward
    static bool handlingPerimeter = false;
    
    if (!handlingPerimeter) {
        handlingPerimeter = true;
        stopMotors();
        
        // Turn 90 degrees to the left
        targetAngle_ = positionManager_->getYaw() + (PI / 2.0f);
        // Normalize angle to -PI..PI
        while (targetAngle_ > PI) targetAngle_ -= 2 * PI;
        while (targetAngle_ < -PI) targetAngle_ += 2 * PI;
    } else {
        float currentYaw = positionManager_->getYaw();
        float angleDiff = atan2(sin(targetAngle_ - currentYaw), cos(targetAngle_ - currentYaw));
        
        if (fabs(angleDiff) < 0.1f) { // Within ~6 degrees
            // Done turning, move forward
            setMotorSpeed(0.5f, 0.5f);
            
            // Reset after moving forward a bit
            static unsigned long startMoveTime = millis();
            if (millis() - startMoveTime > 2000) { // Move forward for 2 seconds
                handlingPerimeter = false;
                startMoveTime = millis();
            }
        } else {
            // Continue turning
            float turnSpeed = constrain(angleDiff * 2.0f, -1.0f, 1.0f);
            setMotorSpeed(-turnSpeed, turnSpeed);
        }
    }
}

void Mower::updatePosition() {
    if (positionManager_) {
        positionManager_->update();
    }
}
