#include "MowerStateMachine.h"
#include "../handlers/Mower.h"
#include <Arduino.h>



MowerStateMachine::MowerStateMachine(Mower& mower)
    : mower_(mower),
      currentState_(MowerState::IDLE),
      previousState_(MowerState::IDLE),
      stateStartTime_(0),
      emergencyStopActive_(false) {
}

void MowerStateMachine::begin() {
    // Initialize the state machine
    currentState_ = MowerState::IDLE;
    previousState_ = MowerState::IDLE;
    stateStartTime_ = millis();
    emergencyStopActive_ = false;
    
    // Call the entry handler for the initial state
    onEnterState(currentState_);
}

void MowerStateMachine::update() {
    // Check for emergency stop condition
    if (isEmergencyStopActive() && currentState_ != MowerState::EMERGENCY_STOP) {
        requestStateChange(MowerState::EMERGENCY_STOP);
        return;
    }

    // Update the current state
    onUpdateState();
}

MowerState MowerStateMachine::getCurrentState() const {
    return currentState_;
}

bool MowerStateMachine::requestStateChange(MowerState newState) {
    if (currentState_ == newState) {
        return true; // Already in the requested state
    }

    if (!isTransitionAllowed(currentState_, newState)) {
        return false; // Invalid transition
    }

    // Exit current state
    onExitState(currentState_);
    
    // Update state
    previousState_ = currentState_;
    currentState_ = newState;
    stateStartTime_ = millis();
    
    // Enter new state
    onEnterState(currentState_);
    
    return true;
}

void MowerStateMachine::startMowing() {
    if (currentState_ == MowerState::IDLE || 
        currentState_ == MowerState::CHARGING) {
        requestStateChange(MowerState::MOWING);
    }
}

void MowerStateMachine::dock() {
    if (currentState_ == MowerState::IDLE || 
        currentState_ == MowerState::MOWING) {
        requestStateChange(MowerState::DOCKING);
    }
}

void MowerStateMachine::emergencyStop() {
    requestStateChange(MowerState::EMERGENCY_STOP);
}

void MowerStateMachine::manualControl() {
    if (currentState_ == MowerState::IDLE) {
        requestStateChange(MowerState::MANUAL_CONTROL);
    }
}

void MowerStateMachine::resetError() {
    if (currentState_ == MowerState::ERROR || 
        currentState_ == MowerState::EMERGENCY_STOP) {
        requestStateChange(previousState_);
    }
}

void MowerStateMachine::onEnterState(MowerState newState) {
    stateStartTime_ = millis();
    
    switch (newState) {
        case MowerState::IDLE:
            mower_.stopMotors();
            mower_.setBladeSpeed(0);
            mower_.setNavigationMode(Mower::NavigationMode::STOPPED);
            break;
            
        case MowerState::MOWING:
            mower_.startMowing();
            mower_.setNavigationMode(Mower::NavigationMode::RANDOM);
            break;
            
        case MowerState::DOCKING:
            mower_.startDocking();
            mower_.setNavigationMode(Mower::NavigationMode::MANUAL);
            break;
            
        case MowerState::CHARGING:
            mower_.enableCharging(true);
            mower_.setNavigationMode(Mower::NavigationMode::STOPPED);
            break;
            
        case MowerState::PAUSED:
            mower_.stopMotors();
            mower_.setBladeSpeed(0);
            mower_.setNavigationMode(Mower::NavigationMode::STOPPED);
            break;
            
        case MowerState::TESTING:
            mower_.stopMotors();
            mower_.setBladeSpeed(0);
            mower_.setNavigationMode(Mower::NavigationMode::STOPPED);
            // Additional test initialization can go here
            break;
            
        case MowerState::SLEEP:
            mower_.stopMotors();
            mower_.setBladeSpeed(0);
            mower_.setNavigationMode(Mower::NavigationMode::STOPPED);
            // TODO: Enter low power mode
            break;
            
        case MowerState::MANUAL_CONTROL:
            mower_.stopMotors();
            mower_.setBladeSpeed(0);
            mower_.setNavigationMode(Mower::NavigationMode::MANUAL);
            break;
            
        case MowerState::EMERGENCY_STOP:
            mower_.emergencyStop();
            mower_.setNavigationMode(Mower::NavigationMode::STOPPED);
            break;
            
        case MowerState::MAINTENANCE_NEEDED:
            mower_.stopMotors();
            mower_.setBladeSpeed(0);
            mower_.setNavigationMode(Mower::NavigationMode::STOPPED);
            break;
            
        case MowerState::ROS_CONTROL:
            mower_.stopMotors();
            mower_.setBladeSpeed(0);
            mower_.setNavigationMode(Mower::NavigationMode::MANUAL);
            break;
            
        default:
            // No special handling for other states
            break;
    }
    
    #ifdef DEBUG_MODE
    mower_.logStateChange(previousState_, newState);
    #endif
}

void MowerStateMachine::onExitState(MowerState oldState) {
    // Clean up resources from the old state if needed
    switch (oldState) {
        case MowerState::CHARGING:
            // Disable charging when leaving charging state
            mower_.enableCharging(false);
            break;
            
        // Add cleanup for other states as needed
        default:
            break;
    }
}

void MowerStateMachine::onUpdateState() {
    // Call the appropriate handler for the current state
    switch (currentState_) {
        case MowerState::IDLE:
            handleIdle();
            break;
        case MowerState::MOWING:
            handleMowing();
            break;
        case MowerState::DOCKING:
            handleDocking();
            break;
        case MowerState::CHARGING:
            handleCharging();
            break;
        case MowerState::MANUAL_CONTROL:
            handleManualControl();
            break;
        case MowerState::ERROR:
            handleError();
            break;
        case MowerState::EMERGENCY_STOP:
            handleEmergencyStop();
            break;
        case MowerState::BORDER_DETECTED:
            handleBorderDetected();
            break;
        case MowerState::LIFTED:
            handleLifted();
            break;
        case MowerState::RAIN_DELAY:
            handleRainDelay();
            break;
        case MowerState::MAINTENANCE_NEEDED:
            handleMaintenanceNeeded();
            break;
        case MowerState::TESTING:
            handleTesting();
            break;
    }
}

bool MowerStateMachine::isTransitionAllowed(MowerState from, MowerState to) const {
    // Allow all transitions from/to ERROR state for recovery
    if (from == MowerState::ERROR || to == MowerState::ERROR) {
        return true;
    }

    // Check valid transitions based on current state
    switch (from) {
        case MowerState::IDLE:
            return (to == MowerState::MOWING) ||
                   (to == MowerState::DOCKING) ||
                   (to == MowerState::TESTING) ||
                   (to == MowerState::ERROR) ||
                   (to == MowerState::MANUAL_CONTROL) ||
                   (to == MowerState::MAINTENANCE_NEEDED);
                   
        case MowerState::MOWING:
            return (to == MowerState::DOCKING) ||
                   (to == MowerState::PAUSED) ||
                   (to == MowerState::ERROR) ||
                   (to == MowerState::EMERGENCY_STOP);
                   
        case MowerState::DOCKING:
            return (to == MowerState::CHARGING) ||
                   (to == MowerState::ERROR) ||
                   (to == MowerState::IDLE) ||
                   (to == MowerState::EMERGENCY_STOP);
                   
        case MowerState::CHARGING:
            return (to == MowerState::IDLE) ||
                   (to == MowerState::MOWING) ||
                   (to == MowerState::ERROR) ||
                   (to == MowerState::EMERGENCY_STOP);
                   
        case MowerState::PAUSED:
            return (to == MowerState::MOWING) ||
                   (to == MowerState::IDLE) ||
                   (to == MowerState::DOCKING);
                   
        case MowerState::TESTING:
            return (to == MowerState::IDLE) ||
                   (to == MowerState::ERROR) ||
                   (to == MowerState::EMERGENCY_STOP);
                   
        case MowerState::SLEEP:
            return (to == MowerState::IDLE) ||
                   (to == MowerState::CHARGING);
                   
        case MowerState::MANUAL_CONTROL:
            return (to == MowerState::IDLE);
            
        case MowerState::EMERGENCY_STOP:
            return (to == MowerState::IDLE);
            
        case MowerState::MAINTENANCE_NEEDED:
            return (to == MowerState::IDLE);
            
        case MowerState::ROS_CONTROL:
            return (to == MowerState::IDLE);
            
        default:
            return false;
    }
}

bool MowerStateMachine::isEmergencyStopActive() const {
    // Check various emergency stop conditions
    return emergencyStopActive_ || 
           mower_.isCollisionDetected() || 
           mower_.isLifted() ||
           mower_.isBatteryCritical();
}

// State handler implementations
void MowerStateMachine::handleIdle() {
    // Check if we should start mowing
    if (mower_.shouldStartMowing()) {
        requestStateChange(MowerState::MOWING);
    }
    // Check if battery is low and we're not at the dock
    else if (mower_.isBatteryLow() && !mower_.isAtDockingStation()) {
        requestStateChange(MowerState::DOCKING);
    }
    // Check if it's raining
    else if (mower_.isRainDetected()) {
        requestStateChange(MowerState::DOCKING);
    }
}

void MowerStateMachine::handleDocking() {
    static bool isNavigatingToDock = false;
    static unsigned long lastDockAttempt = 0;
    static int dockAttempts = 0;
    
    unsigned long currentTime = millis();
    
    // Check if we've reached the dock
    if (mower_.isAtDockingStation()) {
        isNavigatingToDock = false;
        requestStateChange(MowerState::CHARGING);
        return;
    }
    
    // Start docking procedure if not already started
    if (!isNavigatingToDock) {
        isNavigatingToDock = true;
        dockAttempts = 0;
        mower_.startDocking();
        lastDockAttempt = currentTime;
        return;
    }
    
    // Check for timeout (2 minutes max to dock)
    if (currentTime - stateStartTime_ > 120000) {
        requestStateChange(MowerState::ERROR);
        return;
    }
    
    // Check if docking has failed
    if (mower_.isDockingFailed()) {
        // Reset attempts counter if last attempt was more than 1 minute ago
        if (currentTime - lastDockAttempt > 60000) {
            dockAttempts = 0;
        }
        
        if (dockAttempts >= 3) { // Too many failed attempts
            requestStateChange(MowerState::ERROR);
            return;
        }
        
        // Try a different approach
        dockAttempts++;
        lastDockAttempt = currentTime;
        
        // Back up and try again with a different angle
        mower_.stopMotors();
        delay(1000);
        mower_.moveToPosition(-50, 0); // Back up 50cm
        delay(2000);
        mower_.startDocking(); // Retry docking
    }
}

void MowerStateMachine::handleCharging() {
    static bool isChargingStarted = false;
    static const unsigned long MAX_CHARGING_TIME = 3600000; // 1 hour max charging time
    
    // Start charging if not already started
    if (!isChargingStarted) {
        if (mower_.isAtDockingStation()) {
            mower_.enableCharging(true);
            isChargingStarted = true;
            #ifdef DEBUG_MODE
            SERIAL_DEBUG.println("Charging started");
            #endif
        } else {
            // Not at docking station, go back to DOCKING state
            requestStateChange(MowerState::DOCKING);
            return;
        }
    }
    
    // Check if charging is complete
    if (mower_.isBatteryFull()) {
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println("Battery fully charged");
        #endif
        mower_.enableCharging(false);
        isChargingStarted = false;
        requestStateChange(MowerState::IDLE);
        return;
    }
    
    // Check for charging errors
    if (mower_.isChargingError()) {
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println("Charging error detected");
        #endif
        mower_.enableCharging(false);
        isChargingStarted = false;
        requestStateChange(MowerState::ERROR);
        return;
    }
    
    // Check if we're still connected to the dock
    if (!mower_.isAtDockingStation()) {
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println("Disconnected from dock while charging");
        #endif
        mower_.enableCharging(false);
        isChargingStarted = false;
        requestStateChange(MowerState::DOCKING);
        return;
    }
    
    // Check for maximum charging time
    if (millis() - stateStartTime_ > MAX_CHARGING_TIME) {
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println("Maximum charging time reached");
        #endif
        mower_.enableCharging(false);
        isChargingStarted = false;
        requestStateChange(MowerState::ERROR);
    }
}

void MowerStateMachine::handleMowing() {
    static const unsigned long MAX_MOWING_TIME = 14400000; // 4 hours max mowing time
    static unsigned long lastObstacleCheck = 0;
    
    // Check for emergency stop conditions first
    if (mower_.isBatteryCritical()) {
        requestStateChange(MowerState::DOCKING);
        return;
    }
    
    // Check for normal stop conditions
    if (mower_.isBatteryLow()) {
        requestStateChange(MowerState::DOCKING);
        return;
    }
    
    if (mower_.isRainDetected()) {
        requestStateChange(MowerState::RAIN_DELAY);
        return;
    }
    
    if (mower_.isLifted()) {
        requestStateChange(MowerState::LIFTED);
        return;
    }
    
    if (mower_.isBorderDetected()) {
        requestStateChange(MowerState::BORDER_DETECTED);
        return;
    }
    
    // Check for obstacles periodically (every 100ms)
    if (millis() - lastObstacleCheck > 100) {
        lastObstacleCheck = millis();
        if (mower_.checkObstacles()) {
            mower_.handleObstacle();
            return;
        }
    }
    
    // Check for maximum mowing time
    if (millis() - stateStartTime_ > MAX_MOWING_TIME) {
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println("Maximum mowing time reached");
        #endif
        requestStateChange(MowerState::DOCKING);
        return;
    }
    
    // Continue normal mowing operation
    mower_.navigate();
}

void MowerStateMachine::handleManualControl() {
    // In manual control, the mower responds to direct commands
    // but we still need to check for safety conditions
    
    // Emergency stop conditions still apply
    if (mower_.isBatteryCritical() || mower_.isLifted()) {
        mower_.stopMotors();
        mower_.setBladeSpeed(0);
        return;
    }
    
    // If battery is low, force return to dock
    if (mower_.isBatteryLow()) {
        requestStateChange(MowerState::DOCKING);
        return;
    }
    
    // Check for rain - if it starts raining, go to rain delay
    if (mower_.isRainDetected()) {
        requestStateChange(MowerState::RAIN_DELAY);
        return;
    }
    
    // Note: Manual control commands are handled by the Mower class
    // which calls the appropriate motor/sensor methods directly
}

void MowerStateMachine::handleError() {
    static bool errorReported = false;
    
    // Stop all motors when entering error state
    if (!errorReported) {
        mower_.stopMotors();
        mower_.setBladeSpeed(0);
        errorReported = true;
        
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println("ERROR: Entered error state");
        // TODO: Log error details if available
        #endif
    }
    
    // Stay in error state until reset
    // The error can be reset by calling resetError()
}

void MowerStateMachine::handleEmergencyStop() {
    // Stay in emergency stop until reset
    // All systems should be stopped
    mower_.stopMotors();
}

void MowerStateMachine::handleBorderDetected() {
    // Handle border detection (e.g., change direction)
    mower_.handleBorder();
    // After handling, return to mowing or go to docking if needed
    if (mower_.shouldReturnToDockAfterBorder()) {
        requestStateChange(MowerState::DOCKING);
    } else {
        requestStateChange(MowerState::MOWING);
    }
}

void MowerStateMachine::handleLifted() {
    // Handle lifted condition
    mower_.stopMotors();
    // If mower is put down again, return to previous state
    if (!mower_.isLifted() && millis() - stateStartTime_ > 2000) {
        requestStateChange(previousState_);
    }
}

void MowerStateMachine::handleRainDelay() {
    // Wait for rain to stop
    if (!mower_.isRainDetected()) {
        if (mower_.shouldReturnToDockInRain()) {
            requestStateChange(MowerState::DOCKING);
        } else {
            requestStateChange(MowerState::MOWING);
        }
    }
}

void MowerStateMachine::handleMaintenanceNeeded() {
    // Stay in maintenance state until reset
    mower_.stopMotors();
}

void MowerStateMachine::handleTesting() {
    // Run through a series of tests
    static int testStep = 0;
    static unsigned long lastTestTime = 0;
    
    unsigned long currentTime = millis();
    if (currentTime - lastTestTime < 2000) {
        return; // Wait between test steps
    }
    
    lastTestTime = currentTime;
    
    switch (testStep) {
        case 0: // Test motors
            mower_.setMotors(0.3f, 0.3f);
            break;
            
        case 1:
            mower_.setMotors(-0.3f, -0.3f);
            break;
            
        case 2:
            mower_.setMotors(0.3f, -0.3f); // Rotate
            break;
            
        case 3:
            mower_.setMotors(-0.3f, 0.3f); // Rotate other way
            break;
            
        case 4: // Test blade
            mower_.setBladeSpeed(0.5f);
            break;
            
        case 5:
            mower_.setBladeSpeed(0);
            break;
            
        case 6: // Test sensors
            // Just read and log sensor values
            #ifdef DEBUG_MODE
            SERIAL_DEBUG.print("Battery: ");
            SERIAL_DEBUG.println(mower_.batterySensor.readVoltage());
            SERIAL_DEBUG.print("Rain: ");
            SERIAL_DEBUG.println(mower_.rainSensor.isRaining() ? "Yes" : "No");
            #endif
            break;
            
        default:
            // Test complete
            mower_.stopMotors();
            mower_.setBladeSpeed(0);
            requestStateChange(MowerState::IDLE);
            return;
    }
    
    testStep++;
}
