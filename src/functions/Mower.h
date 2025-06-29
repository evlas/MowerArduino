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

/**
 * @brief State and event type definitions for the mower state machine.
 */
#include "MowerTypes.h"

// Forward declarations to resolve circular dependencies
class MowerState;
class IdleState;
class MowingState;
class DockingState;
class UndockingState;
class ChargingState;
class EmergencyStopState;
class ManualControlState;
class ErrorState;
class BorderDetectedState;
class LiftedState;
class PausedState;
class SleepState;
class RainDelayState;
class MaintenanceNeededState;
class RosControlState;

// Hardware component includes
#include "../motors/DriveMotor/DriveMotor.h"
#include "../motors/BladeMotor/BladeMotor.h"
#include "../actuators/Buzzer/Buzzer.h"
#include "../actuators/Relay/Relay.h"

// Sensor includes
#include "../sensors/BatterySensor/BatterySensor.h"
#include "../sensors/BumpSensors/BumpSensors.h"
#include "../sensors/UltrasonicSensors/UltrasonicSensors.h"
#include "../sensors/IMUModule/IMUModule.h"
#ifdef ENABLE_GPS
#include "../sensors/GPSModule/GPSModule.h"
#endif
#include "../sensors/RainSensor/RainSensor.h"

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
public:
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
     * @name Initialization
     * @{
     */
    
    /**
     * @brief Initialize all hardware components and system state
     * @details This method initializes all sensors, motors, and system components.
     * It should be called once during system startup.
     */
    void initializeComponents();
    
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
     * @name Debugging
     * @{
     */
    
    /**
     * @brief Print debug information to the serial console
     * @details Outputs system status, sensor readings, and state information
     */
    void printDebugInfo() const;
    
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
     * @details Initializes all member variables to their default values
     */
    Mower();
    
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
    
    /**
     * @name User Interface
     * @{
     */
    
    /**
     * @brief Update the LCD display with new content
     * @param line1 Text for the first line
     * @param line2 Text for the second line (optional)
     */
    void updateLcdDisplay(const char* line1, const char* line2 = "");
    
    /**
     * @brief Clear the LCD display
     */
    void clearLcdDisplay();
    
    /**
     * @brief Set the LCD cursor position
     * @param col Column (0-15)
     * @param row Row (0-1)
     */
    void setLcdCursor(uint8_t col, uint8_t row);
    
    /**
     * @brief Print text to the LCD at current cursor position
     * @param text Text to print
     */
    void printToLcd(const char* text);
    
    /**
     * @brief Print a number to the LCD at current cursor position
     * @param number Integer number to print
     */
    void printToLcd(int number);
    
    /**
     * @brief Print a floating point number to the LCD
     * @param number Floating point number to print
     * @param decimals Number of decimal places to show (default: 2)
     */
    void printToLcd(float number, int decimals = 2);
    
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
     * @brief Get the current navigation mode
     * @return Current navigation mode
     */
    NavigationMode getNavigationMode() const { return navigationMode_; }
    
    /**
     * @brief Set the navigation mode
     * @param mode New navigation mode
     */
    void setNavigationMode(NavigationMode mode);
    
    /**
     * @brief Convert navigation mode to string
     * @param mode Navigation mode to convert
     * @return String representation of the navigation mode
     */
    const char* navigationModeToString(NavigationMode mode) const;
    
    /**
     * @name Blade Control
     * @{
     */
    
    /**
     * @brief Start the blade motor
     */
    void startBlades();
    
    /**
     * @brief Stop the blade motor
     */
    void stopBlades();
    
    /**
     * @brief Set the blade motor speed
     * @param speed Speed value (0.0 to 1.0)
     */
    void setBladeSpeed(float speed);
    
    /** @} */
    
    /**
     * @name Drive Motor Control
     * @{
     */
    
    /**
     * @brief Set the left drive motor speed
     * @param speed Speed value (-1.0 to 1.0, negative for reverse)
     */
    void setLeftMotorSpeed(float speed);
    
    /**
     * @brief Set the right drive motor speed
     * @param speed Speed value (-1.0 to 1.0, negative for reverse)
     */
    void setRightMotorSpeed(float speed);
    
    /**
     * @brief Stop both drive motors
     */
    void stopDriveMotors();
    
    /**
     * @brief Restart the drive motors with the last set speeds
     * 
     * This method reactivates the drive motors using the previously set speeds.
     * It's typically called after motors have been stopped temporarily.
     */
    void startDriveMotors();
    
    /**
     * @brief Stop all motors (drive and blade)
     */
    void stopMotors();
    
    /** @} */
    
    /**
     * @name Safety System
     * @{
     */
    
    /**
     * @brief Immediately stop all motors and enter emergency stop state
     */
    void emergencyStop();
    
    /**
     * @brief Reset the emergency stop condition
     */
    void resetEmergencyStop();
    
    /**
     * @brief Check if emergency stop is active
     * @return true if emergency stop is active, false otherwise
     */
    bool isEmergencyStopActive() const { return emergencyStopActive_; }
    
    /**
     * @brief Enable or disable battery charging
     * @param enable true to enable charging, false to disable
     * @return true if successful, false otherwise
     */
    bool enableCharging(bool enable);
    
    /**
     * @brief Update all sensor readings
     * @details Should be called regularly to keep sensor data current
     */
    void updateSensors();
    
    /**
     * @brief Handle border detection event
     * @details Called when the mower detects a boundary or perimeter wire
     */
    void handleBorder();
    
    /**
     * @brief Handle obstacle detection
     * @details Called when an obstacle is detected by sensors
     */
    void handleObstacle();
    
    /**
     * @brief Start mowing operation
     */
    void startMowing();
    
    /**
     * @brief Start docking procedure
     */
    void startDocking();
    
    /**
     * @brief Set the system state
     * @param newState New state to transition to
     */
    void setState(State newState);
    
    /**
     * @brief Set the system state using a state object
     * @param newState Reference to the new state object
     */
    void setState(MowerState& newState) { changeState(newState); }
    
    /**
     * @brief Perform safety checks
     * @details Verifies system safety and triggers emergency stop if needed
     */
    void checkSafety();
    
    /** @} */

private:
    // Current and next state pointers for state machine
    MowerState* currentState_;      ///< Current state of the mower state machine
    MowerState* nextState_;         ///< Next state to transition to
    
    // Internal state variables
    bool emergencyStopActive_;      ///< Flag indicating if emergency stop is active
    bool bladesRunning_;            ///< Flag indicating if blades are currently running
    float leftMotorSpeed_;          ///< Current speed of left motor (-1.0 to 1.0)
    float rightMotorSpeed_;         ///< Current speed of right motor (-1.0 to 1.0)
    float bladeSpeed_;              ///< Current blade speed (0.0 to 1.0)
    bool charging_;                 ///< Flag indicating if charging is active
    float batteryLevel_;             ///< Current battery level (0.0 to 100.0)
    NavigationMode navigationMode_;  ///< Current navigation mode
    unsigned long lastSensorUpdate_;  ///< Timestamp of last sensor update (ms)
    
    // Sensor states
    bool lifted_;                    ///< Flag indicating if mower is lifted
    bool borderDetected_;            ///< Flag indicating if border wire is detected
    bool collisionDetected_;         ///< Flag indicating if collision is detected
    bool docked_;                    ///< Flag indicating if mower is docked
    bool batteryCritical_;           ///< Flag indicating critically low battery
    bool batteryLow_;                ///< Flag indicating low battery
    bool batteryCharged_;            ///< Flag indicating battery is charged
    bool batteryFull_;               ///< Flag indicating battery is fully charged
    
    // Error handling
    String lastError_;               ///< Last error message
    unsigned long errorTimestamp_;    ///< Timestamp of last error (ms)
    static const unsigned long ERROR_RESET_TIMEOUT = 5000; ///< Time before error auto-reset (ms)
    
    // Navigation constants
    static const float OBSTACLE_DETECTION_DISTANCE = 30.0f; ///< Distance threshold for obstacle detection (cm)
    static const unsigned long DOCKING_CONFIRMATION_TIME = 2000; ///< Time to confirm docking (ms)
    
    /**
     * @brief Ultrasonic sensor identifiers
     */
    enum UltrasonicSensor {
        ULTRASONIC_FRONT = 0,  ///< Front ultrasonic sensor
        ULTRASONIC_LEFT,        ///< Left ultrasonic sensor
        ULTRASONIC_RIGHT,       ///< Right ultrasonic sensor
        ULTRASONIC_REAR         ///< Rear ultrasonic sensor
    };
    
    // Hardware components
    LiquidCrystal_I2C lcd;      ///< LCD display controller
    DriveMotor leftMotor;       ///< Left drive motor controller
    DriveMotor rightMotor;      ///< Right drive motor controller
    BladeMotor bladeMotor;      ///< Blade motor controller
    Buzzer buzzer;              ///< Buzzer for audio feedback
    Relay motorRelay;           ///< Motor power relay
    Relay chargingRelay;        ///< Battery charging relay
    
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
