/**
 * @file Mower.h
 * @brief Main controller class for the autonomous mower system.
 * 
 * This file contains the declaration of the Mower class which is the central
 * controller for the autonomous mower. It manages all hardware components,
 * state transitions, and system behaviors.
 * 
 * @section dependencies Dependencies
 * - Arduino.h: Core Arduino functionality
 * - LiquidCrystal_I2C.h: For LCD display control
 */

#ifndef MOWER_H
#define MOWER_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "../config.h"
#include "../pin_config.h"
#include "PositionManager.h"

// Forward declarations per evitare dipendenze circolari
class MowerState;
class IdleState;
class MowingState;
class DockingState;
class UndockingState;
class ChargingState;
class EmergencyStopState;
class ManualControlState;
class ErrorState;
class LiftedState;
class NavigationManager;
class NavigatorBase;
class RandomNavigator;
class LawnMowerNavigator;
class BorderNavigator;

// Includi solo gli header necessari
#include "../states/MowerState.h"
#include "../LCD/LCDMenu.h"

// Includi i moduli principali
#include "../sensors/IMUModule/IMUModule.h"
#include "../sensors/GPSModule/GPSModule.h"
#include "../sensors/UltrasonicSensors/UltrasonicSensors.h"
// Include navigator headers
#include "../navigation/RandomNavigator.h"
#include "../navigation/LawnMowerNavigator.h"
#include "../navigation/BorderNavigator.h"
#include "../navigation/NavigationManager.h"

// Include motor and sensor headers
#include "../motors/DriveMotor/DriveMotor.h"
#include "../motors/BladeMotor/BladeMotor.h"
#include "../sensors/BatterySensor/BatterySensor.h"
#include "../sensors/RainSensor/RainSensor.h"
#include "../sensors/BumpSensors/BumpSensors.h"
#include "../actuators/Buzzer/Buzzer.h"
#include "../actuators/Relay/Relay.h"
#include "PositionManager.h"

// Include NavigationManager after forward declarations
#include "../navigation/NavigationManager.h"

/**
 * @class Mower
 * @brief Main controller class for the autonomous mower system.
 * 
 * The Mower class is responsible for:
 * - Managing the state machine and state transitions
 * - Controlling all hardware components (motors, sensors, actuators)
 * - Implementing safety checks and emergency procedures
 * - Providing an interface for user interaction
 * - Managing navigation and mowing patterns
 */
class Mower {
    // Dichiarazioni friend per le classi stato
    friend class IdleState;
    friend class MowingState;
    friend class DockingState;
    friend class UndockingState;
    friend class ChargingState;
    friend class EmergencyStopState;
    friend class ManualControlState;
    friend class ErrorState;
    friend class BorderDetectedState;
    friend class LiftedState;
    friend class PausedState;
    friend class SleepState;
    friend class RainDelayState;
    friend class MaintenanceNeededState;
    friend class RosControlState;
    friend class RemoteCommand;
    friend class WiFiRemote;
    
public:
    // Metodi di navigazione
    bool saveHomePosition();
    bool hasValidHomePosition() const;
    float getDistanceToHome() const;
    float getBearingToHome() const;
    
    // GPS getters (delegati al PositionManager)
    double getGPSLatitude() const { return positionManager_->getGPSLatitude(); }
    double getGPSLongitude() const { return positionManager_->getGPSLongitude(); }
    float getGPSHDOP() const { return positionManager_->getGPSHDOP(); }
    uint8_t getGPSSatellites() const { return positionManager_->getGPSSatellites(); }
    bool hasGPSFix() const { return positionManager_->hasGPSFix(); }
    
    // Get the current position from PositionManager
    RobotPosition getCurrentPosition() const { 
        return positionManager_ ? positionManager_->getPosition() : RobotPosition(); 
    }
    
    /**
     * @brief Updates the daily work time counter
     * @param isWorking True if the mower is currently working (mowing)
     * @return True if daily work limit is reached
     */
    bool updateWorkTime(bool isWorking) {
        unsigned long currentTime = millis();
        
        // Reset counters at midnight
        if (currentTime - lastMidnightReset_ >= 24 * 60 * 60 * 1000UL) {
            totalWorkTimeToday_ = 0;
            lastMidnightReset_ = currentTime - (currentTime % (24 * 60 * 60 * 1000UL));
            DEBUG_PRINTLN(F("Resetting daily work time counter at midnight"));
        }
        
        // Update work time if mower is working
        if (isWorking && lastWorkTimeUpdate_ > 0) {
            unsigned long elapsed = currentTime - lastWorkTimeUpdate_;
            totalWorkTimeToday_ += elapsed;
            
            DEBUG_PRINT(F("Daily work time: "));
            DEBUG_PRINT(totalWorkTimeToday_ / (60 * 60 * 1000UL)); // Hours
            DEBUG_PRINT(F("h "));
            DEBUG_PRINT((totalWorkTimeToday_ % (60 * 60 * 1000UL)) / (60 * 1000UL)); // Minutes
            DEBUG_PRINTLN(F("m"));
            
            // Check if daily limit is reached
            if (totalWorkTimeToday_ >= MAX_DAILY_WORK_TIME) {
                DEBUG_PRINTLN(F("Daily work limit (8h) reached!"));
                return true;
            }
        }
        
        lastWorkTimeUpdate_ = currentTime;
        return false;
    }
    
    /**
     * @brief Resets the daily work time counter
     */
    void resetWorkTime() {
        totalWorkTimeToday_ = 0;
        lastWorkTimeUpdate_ = millis();
        lastMidnightReset_ = lastWorkTimeUpdate_ - (lastWorkTimeUpdate_ % (24 * 60 * 60 * 1000UL));
        DEBUG_PRINTLN(F("Work time counter reset"));
    }
    
    /**
     * @brief Get the distance from the left front ultrasonic sensor
     * @return Distance in centimeters
     */
    float getLeftSensorDistance() const { 
        return ultrasonicSensors.getFrontLeftDistance(); 
    }
    
    /**
     * @brief Get the distance from the right front ultrasonic sensor
     * @return Distance in centimeters
     */
    float getRightSensorDistance() const { 
        return ultrasonicSensors.getFrontRightDistance(); 
    }
    
    // Ottieni il PositionManager
    PositionManager* getPositionManager() { return positionManager_; }
    const PositionManager* getPositionManager() const { return positionManager_; }
    
    /**
     * @brief State type alias from MowerTypes.h
     */
    using State = ::State;
    
    /**
     * @brief Event type alias from MowerTypes.h
     */
    using Event = ::Event;
    
    // Metodi per ottenere gli stati (implementati in StateGetters.h)
    
    /**
     * @brief Sets the LCD menu reference and links it with this Mower instance
     * @param menu Reference to the LCDMenu instance
     */
    void setLCDMenu(LCDMenu& menu) {
        lcdMenu = &menu;
        if (lcdMenu) {
            lcdMenu->setMower(this);  // Imposta il puntatore a this nel menu
        }
    }
    
    /**
     * @name Initialization
     * @{
     */
    
    /**
     * @brief Get the Idle state instance
     * @return Reference to the IdleState singleton
     */
    MowerState& getIdleState();
    
    /**
     * @brief Get the Mowing state instance
     * @return Reference to the MowingState singleton
     */
    MowerState& getMowingState();
    
    /**
     * @brief Get the Docking state instance
     * @return Reference to the DockingState singleton
     */
    MowerState& getDockingState();
    
    /**
     * @brief Get the Undocking state instance
     * @return Reference to the UndockingState singleton
     */
    MowerState& getUndockingState();
    
    /**
     * @brief Get the Charging state instance
     * @return Reference to the ChargingState singleton
     */
    MowerState& getChargingState();
    
    /**
     * @brief Get the Emergency Stop state instance
     * @return Reference to the EmergencyStopState singleton
     */
    MowerState& getEmergencyStopState();
    
    /**
     * @brief Get the Lifted state instance
     * @return Reference to the LiftedState singleton
     */
    MowerState& getLiftedState();
    
    /**
     * @brief Get the Error state instance
     * @return Reference to the ErrorState singleton
     */
    MowerState& getErrorState();
    
    /** @} */
    
    /**
     * @name Utility Methods
     * @{
     */
    
    /**
     * @brief Get the current battery level percentage
     * @return Battery level as a percentage (0-100%)
     */
    float getBatteryPercentage() const { return batterySensor.getBatteryPercentage(); }
    
    /**
     * @brief Get the system uptime in seconds
     * @return Uptime in seconds since system start
     */
    unsigned long getUptime() const { return millis() / 1000; }
    
    /**
     * @brief Check if it's currently raining
     * @return true if rain is detected, false otherwise
     */
    bool isRaining() const { return rainSensor.isRaining(); }
    
    /**
     * @brief Check if an obstacle is detected by ultrasonic sensors
     * @return true if an obstacle is detected, false otherwise
     */
    bool isObstacleDetected() const { return ultrasonicSensors.isObstacleDetected(); }
    
    /**
     * @brief Get the battery voltage in volts
     * @return Battery voltage in volts
     */
    float getBatteryVoltage() const { return batterySensor.getVoltage(); }
    
    /**
     * @brief Get the battery current in amperes
     * @return Battery current in amperes (positive for discharge, negative for charge)
     */
    float getBatteryCurrent() const { return batterySensor.getCurrent(); }
    
    /**
     * @brief Get the IMU data
     * @return IMUData structure containing accelerometer and gyroscope data
     */
    IMUData getIMUData() const { return imu.getData(); }
    
    /**
     * @brief Get the current latitude
     * @return Latitude in degrees, or 0 if GPS is not available
     */
    float getLatitude() const { 
#ifdef ENABLE_GPS
        return gps.getLatitude(); 
#else
        return 0.0f;
#endif
    }
    
    /**
     * @brief Get the current longitude
     * @return Longitude in degrees, or 0 if GPS is not available
     */
    float getLongitude() const { 
#ifdef ENABLE_GPS
        return gps.getLongitude(); 
#else
        return 0.0f;
#endif
    }
    
    /**
     * @brief Get the number of visible GPS satellites
     * @return Number of visible satellites, or 0 if GPS is not available
     */
    uint8_t getSatellites() const { 
#ifdef ENABLE_GPS
        return gps.getSatellites(); 
#else
        return 0;
#endif
    }
    
    /**
     * @brief Check if the GPS has a valid fix
     * @return true if GPS has a valid fix, false otherwise
     */
    bool isGPSValid() const {
#ifdef ENABLE_GPS
        return gps.isValid() && gps.getHDOP() < MAX_HDOP_FOR_HOME_POSITION && 
               gps.getSatellites() >= MIN_SATELLITES_FOR_HOME_POSITION;
#else
        return false;
#endif
    }
    // Get the current compass heading
    // Returns: Heading in degrees (0-360), or 0 if compass is not available
    float getCompassHeading() const {
        // TODO: Implement this method to get the current compass heading
        // This is a placeholder - replace with actual compass reading
        return 0.0f;
    }
    
    /**
     * @brief Get the ultrasonic sensor distances
     * @param distances Array of floats to store the distances (in cm)
     * @param maxDistances Maximum number of distances to retrieve
     * @return Number of distances retrieved
     */
    uint8_t getUltrasonicDistances(float* distances, uint8_t maxDistances) const { 
        if (maxDistances >= 3) {
            // Assumendo che UltrasonicSensors abbia un metodo per ottenere le distanze
            ultrasonicSensors.getAllDistances(distances[0], distances[1], distances[2]);
            return 3;
        }
        return 0;
    }
    
    /**
     * @brief Set manual control for the mower
     * @param leftSpeed Speed for the left motor (-100.0 to 100.0)
     * @param rightSpeed Speed for the right motor (-100.0 to 100.0)
     */
    void setManualControl(float leftSpeed, float rightSpeed) {
        // Assicurati che le velocità siano nel range corretto (-100.0% a 100.0%)
        leftSpeed = constrain(leftSpeed, -100.0f, 100.0f);
        rightSpeed = constrain(rightSpeed, -100.0f, 100.0f);
        
        // Imposta le velocità dei motori utilizzando i metodi esistenti
        setLeftMotorSpeed(leftSpeed);
        setRightMotorSpeed(rightSpeed);
        
        // Se necessario, avvia i motori
        if (leftSpeed != 0.0f || rightSpeed != 0.0f) {
            startDriveMotors();
        } else {
            stopDriveMotors();
        }
    }
    
    /**
     * @brief Check if the mower is aligned with the charging dock
     * @details Uses front ultrasonic sensors to determine alignment
     * @return true if aligned (sensor difference < 5cm), false otherwise
     */
    bool isAlignedWithDock() const { 
        float leftDist = ultrasonicSensors.getFrontLeftDistance();
        float rightDist = ultrasonicSensors.getFrontRightDistance();
        return abs(leftDist - rightDist) < 5.0f;  // Consider aligned if difference is less than 5cm
    }
    
    /**
     * @brief Calculate the alignment correction needed to approach the dock
     * @details Uses front ultrasonic sensors to calculate required steering correction
     * @return Positive value means turn right, negative means turn left
     */
    float calculateDockAlignmentCorrection() const {
        float leftDist = ultrasonicSensors.getFrontLeftDistance();
        float rightDist = ultrasonicSensors.getFrontRightDistance();
        return rightDist - leftDist;  // Positive means turn right, negative means turn left
    }
    
    /** @} */
    
    /**
     * @name Error Handling
     * @{
     */
    
    /**
     * @brief Log an error message to the error system
     * @param error The error message to log
     */
    void logError(const String& error);
    
    /**
     * @brief Get the last error message
     * @return The most recent error message, or empty string if none
     */
    String getLastError() const;
    
    /**
     * @brief Check if the last error has been resolved
     * @return true if the last error is resolved, false otherwise
     */
    bool isErrorResolved() const;
    
    /** @} */
    
    /**
     * @name Motor Control
     * @{
     */
    
    /**
     * @brief Update motor states and apply speed changes
     * @details This should be called regularly to ensure smooth motor control
     */
    void updateMotors();
    
    /**
     * @brief Start the blade motor
     */
    void startBlades();
    
    /**
     * @brief Stop the blade motor
     */
    void stopBlades();
    
    /**
     * @brief Set the speed of the blade motor
     * @param speed Speed value from 0.0 to 100.0 (percentage of max speed)
     */
    void setBladeSpeed(float speed);
    
    /**
     * @brief Stop all motors (drive and blade)
     */
    void stopMotors();
    
    /**
     * @brief Start the mowing operation
     */
    void startMowing();
    
    /**
     * @brief Start the docking operation
     */
    void startDocking();
    
    /**
     * @brief Update all sensor readings
     */
    void updateSensors();
    
    /**
     * @brief Perform emergency stop
     */
    void emergencyStop();
    
    /**
     * @brief Reset emergency stop condition
     */
    void resetEmergencyStop();
    
    /**
     * @brief Handle border detection
     */
    void handleBorder();
    
    /**
     * @brief Handle obstacle detection
     */
    void handleObstacle();
    
    /**
     * @brief Set the current state of the mower
     * @param newState The new state to transition to
     */
    void setState(State newState);
    
    /**
     * @brief Set the current state of the mower using a state object reference
     * @param newState Reference to the new concrete state
     */
    void setState(MowerState& newState) { changeState(newState); }
    
    /**
     * @brief Set the speed of the left drive motor
     * @param speed Speed value from -100.0 to 100.0 (percentage of max speed)
     */
    void setLeftMotorSpeed(float speed) {
        if (leftMotor_) leftMotor_->setSpeed(speed);
    }
    
    /**
     * @brief Set the speed of the right drive motor
     * @param speed Speed value from -100.0 to 100.0 (percentage of max speed)
     */
    void setRightMotorSpeed(float speed) {
        if (rightMotor_) rightMotor_->setSpeed(speed);
    }
    
    /**
     * @brief Start both drive motors at their current speed settings
     */
    void startDriveMotors() {
        // The DriveMotor class doesn't have a start() method.
        // The motors are started by setting their speed.
        // The current speed should already be set, so we just need to ensure
        // the motors are enabled if needed.
        if (leftMotor_) leftMotor_->setLinearSpeed(leftMotorSpeed_);
        if (rightMotor_) rightMotor_->setLinearSpeed(rightMotorSpeed_);
    }
    
    /**
     * @brief Stop both drive motors
     */
    void stopDriveMotors() {
        if (leftMotor_) leftMotor_->setLinearSpeed(0);
        if (rightMotor_) rightMotor_->setLinearSpeed(0);
        leftMotorSpeed_ = 0;
        rightMotorSpeed_ = 0;
    }
    
    /**
     * @name Navigation Control
     * @{
     */
    
    /**
     * @brief Start random movement pattern for mowing
     * @details Initiates a random movement pattern for autonomous mowing
     */
    void startRandomMovement();
    
    /**
     * @brief Stop the current random movement
     */
    void stopRandomMovement();
    
    /**
     * @brief Follow the perimeter wire to navigate the mowing area
     * @details Uses perimeter wire sensors to follow the boundary
     */
    void followPerimeter();
    
    /** @} */
    
    /**
     * @name State Management
     * @{
     */
    
    /**
     * @brief Get the current system state
     * @return Current state of the mower state machine
     */
    State getState() const;
    
    /**
     * @brief Construct a new Mower object
     * @param lcdMenu Optional pointer to LCDMenu instance (default: nullptr)
     */
    explicit Mower(LCDMenu* lcdMenu = nullptr);
    
    /**
     * @brief Initialize the mower system
     * @details Sets up hardware components and initial state
     */
    void begin();
    
    /**
     * @brief Initialize system components
     * @note This is an alias for begin() for backward compatibility
     */
    void init();
    
    /**
     * @brief Main update loop
     * @details Should be called repeatedly from the main loop
     */
    void update();
    
    /**
     * @brief Handle a system event
     * @param event The event to handle
     */
    void handleEvent(Event event);
    
    /**
     * @brief Transition to a new state
     * @param newState Reference to the new state
     */
    void changeState(MowerState& newState);
    
    /**
     * @brief Convert state enum to string
     * @param state The state to convert
     * @return String representation of the state
     */
    const char* stateToString(State state) const;
    
    /**
     * @brief Convert event enum to string
     * @param event The event to convert
     * @return String representation of the event
     */
    const char* eventToString(Event event) const;
    
    /**
     * @brief Log current system status
     */
    void logStatus();
    
    /** @} */
    
    // Reference to the LCDMenu instance
    LCDMenu* lcdMenu = nullptr;  // Inizializzato a nullptr
    
    // Riferimento al PositionManager
    PositionManager* positionManager_;
    
    /**
     * @brief Clear the LCD display
     */
    void clearLcdDisplay();
    
    /**
     * @brief Set the LCD cursor position
     * @param col Column position (0-15)
     * @param row Row position (0-1)
     */
    void setLcdCursor(uint8_t col, uint8_t row);
    
    /**
     * @brief Print text to the LCD display
     * @param text The text to print
     */
    void printToLcd(const String &text);
    
    /**
     * @brief Print a number to the LCD display
     * @param number The number to print
     */
    void printToLcd(int number);
    
    /**
     * @brief Update the LCD display with new text
     * @param line1 First line of text
     * @param line2 Second line of text (optional)
     */
    void updateLcdDisplay(const String &line1, const String &line2 = "");
    
    /**
     * @brief Play a tone on the buzzer
     * @param frequency Tone frequency in Hz
     * @param duration Duration in milliseconds
     */
    void playBuzzerTone(unsigned int frequency, unsigned long duration);
    
    /**
     * @brief Stop the buzzer
     */
    void stopBuzzer();
    
    /** @} */
    /**
     * @name System Status Getters
     * @{
     */
    
    /**
     * @brief Get the current battery level
     * @return Battery level as a percentage (0.0 - 100.0)
     */
    float getBatteryLevel() const { return batteryLevel_; }
    
    /**
     * @brief Check if the mower is currently charging
     * @return true if charging, false otherwise
     */
    bool isCharging() const { return charging_; }
    
    /**
     * @brief Enable or disable battery charging
     * @param enable true to enable charging, false to disable
     * @return true if successful, false otherwise
     */
    bool enableCharging(bool enable); /* implemented in Mower.cpp */

    /* removed inline body to avoid duplicate definition */
    /* bool enableCharging(bool enable) {
        if (enable) {
            chargingRelay.on();
            charging_ = true;
            DEBUG_PRINTLN("Charging enabled");
        } else {
            chargingRelay.off();
            charging_ = false;
            DEBUG_PRINTLN("Charging disabled");
        }
        return true;
    }
    
    /**
     * @brief Check if the mower is lifted off the ground
     * @return true if lifted, false otherwise
     */
    bool isLifted() const { return lifted_; }
    
    /**
     * @brief Check if a border/perimeter wire is detected
     * @return true if border detected, false otherwise
     */
    bool isBorderDetected() const { return borderDetected_; }
    
    /**
     * @brief Check if a collision is detected by bump sensors
     * @return true if collision detected, false otherwise
     */
    bool isCollisionDetected() const { return collisionDetected_; }
    
    /**
     * @brief Check if battery level is critically low
     * @return true if battery is critically low, false otherwise
     */
    bool isBatteryCritical() const { return batteryCritical_; }
    
    /**
     * @brief Check if battery level is low
     * @return true if battery is low, false otherwise
     */
    bool isBatteryLow() const { return batteryLow_; }
    
    /**
     * @brief Check if battery is fully charged
     * @return true if battery is fully charged, false otherwise
     */
    bool isBatteryCharged() const { return batteryCharged_; }
    
    /**
     * @brief Check if battery is at maximum charge level
     * @return true if battery is at maximum charge, false otherwise
     */
    bool isBatteryFull() const { return batteryFull_; }
    
    /**
     * @brief Check if mower is docked at the charging station
     * @return true if docked, false otherwise
     */
    bool isDocked() const { return docked_; }
    
    /** @} */
    
    /**
     * @name System Status Setters
     * @{
     */
    
    /**
     * @brief Set the charging state
     * @param charging true if charging, false otherwise
     */
    void setCharging(bool charging) { charging_ = charging; }
    
    /**
     * @brief Set the lifted state
     * @param lifted true if lifted, false otherwise
     */
    void setLifted(bool lifted) { lifted_ = lifted; }
    
    /**
     * @brief Set the border detection state
     * @param detected true if border detected, false otherwise
     */
    void setBorderDetected(bool detected) { borderDetected_ = detected; }
    
    /**
     * @brief Set the collision detection state
     * @param detected true if collision detected, false otherwise
     */
    void setCollisionDetected(bool detected) { collisionDetected_ = detected; }
    
    /**
     * @brief Set the docked state
     * @param docked true if docked, false otherwise
     */
    void setDocked(bool docked) { docked_ = docked; }
    
    /** @} */
    
    /**
     * @brief Check for obstacles in the mower's path
     * @return true if obstacles detected, false otherwise
     */
    bool checkObstacles() const { return collisionDetected_; }
    
    /**
     * @brief Get the current state object
     * @return Pointer to the current MowerState
     */
    MowerState* getCurrentState() const { return currentState_; }
    
    /**
     * @name Navigation
     * @{
     */
    
    /**
     * @brief Navigation mode type definition
     */
    using NavigationMode = ::NavigationMode;
    
    /**
     * @brief Gets the current navigation mode
     * @return Current navigation mode
     */
    NavigationMode getNavigationMode() const { 
        return navigationManager_.getCurrentMode(); 
    }
    
    /**
     * @brief Sets the navigation mode
     * @param mode Navigation mode to set
     */
    void setNavigationMode(NavigationMode mode);
    
    /**
     * @brief Convert navigation mode to string
     * @param mode Navigation mode to convert
     * @return String representation of the navigation mode
     */
    const char* navigationModeToString(NavigationMode mode) const;
    
    // Navigator access methods
    RandomNavigator& getRandomNavigator();
    LawnMowerNavigator& getLawnMowerNavigator();
    BorderNavigator& getBorderNavigator();
    NavigatorBase* getCurrentNavigator();
    const char* getCurrentNavigatorName() const;
    
    /**
     * @brief Start navigation with the current navigation mode
     */
    void startNavigation();
    
    /**
     * @brief Stop navigation
     */
    void stopNavigation();
    
    /**
     * @brief Check if navigation is active
     * @return true if navigation is active, false otherwise
     */
    bool isNavigating() const { return isNavigating_; }
    
    /** @} */
    
private:
    // ... (rest of the code remains the same)

    // Motor controllers (using pointers for polymorphism if needed)
    DriveMotor* leftMotor_ = &leftMotor;  ///< Pointer to left drive motor controller
    DriveMotor* rightMotor_ = &rightMotor; ///< Pointer to right drive motor controller
    float leftMotorSpeed_ = 0.0f;         ///< Current left motor speed (-100.0 to 100.0)
    float rightMotorSpeed_ = 0.0f;        ///< Current right motor speed (-100.0 to 100.0)
    
    // Navigation
    NavigationManager navigationManager_;
    NavigationMode navigationMode_;
    bool isNavigating_ = false;
    
    // Safety check
    void checkSafety();

    // Navigation methods
    void updateNavigation();

    // Timers and timing
    unsigned long lastUpdateTime_;
    unsigned long lastSensorUpdate_;
    unsigned long lastSensorReadTime_ = 0;
    unsigned long lastDisplayUpdateTime_;
    unsigned long lastBatteryCheckTime_;
    unsigned long totalWorkTimeToday_ = 0;         // Tempo totale di lavoro oggi (ms)
    unsigned long lastWorkTimeUpdate_ = 0;         // Ultimo aggiornamento del tempo di lavoro
    unsigned long lastMidnightReset_ = 0;          // Timestamp dell'ultimo reset a mezzanotte
    static constexpr unsigned long MAX_DAILY_WORK_TIME = 8 * 60 * 60 * 1000UL; // 8 ore in millisecondi(ms)

    // Sensor states
    bool lifted_;                    ///< Flag indicating if mower is lifted
    bool borderDetected_;            ///< Flag indicating if border wire is detected
    bool collisionDetected_;         ///< Flag indicating if collision is detected
    bool docked_;                    ///< Flag indicating if mower is docked
    bool batteryCritical_;           ///< Flag indicating critically low battery
    bool batteryLow_;                ///< Flag indicating low battery
    bool batteryCharged_;            ///< Flag indicating battery is charged
    bool batteryFull_;               ///< Flag indicating battery is fully charged
    bool charging_;                  ///< Flag indicating if battery is charging
    bool emergencyStopActive_ = false; ///< Flag indicating if emergency stop is active
    bool bladesRunning_ = false;     ///< Flag indicating if blades are running
    float batteryLevel_;             ///< Current battery level (0-100%)
    float bladeSpeed_ = 0.0f;        ///< Current blade speed (0-100%)
    MowerState* currentState_;       ///< Current state of the mower
    MowerState* nextState_;          ///< Next state to transition to
    
    // Error handling
    String lastError_;               ///< Last error message
    String emergencyStopReason_;      ///< Reason for the last emergency stop
    unsigned long errorTimestamp_;    ///< Timestamp of last error (ms)
    static const unsigned long ERROR_RESET_TIMEOUT = 5000; ///< Time before error auto-reset (ms)
    
    // Navigation constants
    static const float OBSTACLE_DETECTION_DISTANCE = 30.0f; ///< Distance threshold for obstacle detection (cm)
    static const unsigned long DOCKING_CONFIRMATION_TIME = 2000; ///< Time to confirm docking (ms)
    
    // Backlight control
    unsigned long lastLcdActivity;     ///< Timestamp of last LCD activity for backlight timeout
    
    /**
     * @brief Ultrasonic sensor identifiers
     */
    enum UltrasonicSensor {
        ULTRASONIC_FRONT = 0,  ///< Front ultrasonic sensor
        ULTRASONIC_LEFT,        ///< Left ultrasonic sensor
        ULTRASONIC_RIGHT,       ///< Right ultrasonic sensor
        ULTRASONIC_REAR         ///< Rear ultrasonic sensor
    };
    
    // Navigation components
    RandomNavigator randomNavigator_;      ///< Random navigation algorithm
    LawnMowerNavigator lawnMowerNavigator_; ///< Lawn mower pattern navigation
    BorderNavigator borderNavigator_;      ///< Border following navigation
    
    // Hardware components
    DriveMotor leftMotor;            ///< Left drive motor controller
    DriveMotor rightMotor;           ///< Right drive motor controller
    BladeMotor bladeMotor;           ///< Blade motor controller
    Buzzer buzzer;                   ///< Buzzer for audio feedback
    Relay motorRelay;                ///< Motor power relay
    Relay chargingRelay;             ///< Battery charging relay
    
    // Sensor modules
    BatterySensor batterySensor;        ///< Battery monitoring
    BumpSensors bumpSensors;           ///< Bump/contact sensors
    UltrasonicSensors ultrasonicSensors; ///< Distance sensors
    IMUModule imu;                     ///< Inertial Measurement Unit
    GPSModule gps;                     ///< GPS module for positioning
    RainSensor rainSensor;             ///< Rain detection sensor
};

// Includi le implementazioni inline dei getter degli stati
#include "../states/StateGetters.h"

#endif // MOWER_H
