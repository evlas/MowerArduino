#ifndef MOWER_H
#define MOWER_H

#include <Arduino.h>
#include <math.h>
#include "../../config.h"
#include "../../pin_config.h"

// Include necessary headers for member variables
#include "../sensors/RainSensor.h"
#include "../sensors/BatterySensor.h"
#include "../sensors/IMU.h"
#include "../sensors/GPS.h"
#include "../sensors/UltrasonicSensors.h"
#include "../sensors/BumpSensors.h"
#include "../sensors/PerimeterSensors.h"
#include "../motors/DriveMotor.h"
#include "../motors/BladeMotor.h"
#include "../actuators/Relay.h"

// Forward declarations
class MowerStateMachine;
class Maneuver;
class PositionManager;

// Forward declare enums and structs
enum class MowerState;
struct RobotPosition;
class UltrasonicSensors;
class BumpSensors;
class PerimeterSensors;
class RainSensor;
class BatterySensor;
class IMUModule;
class GPSModule;
class Relay;
class DriveMotor;
class BladeMotor;

// Math constants and conversion factors are provided by Arduino.h

// Forward declarations
class MowerStateMachine;

/**
 * @brief Main class representing the robotic lawn mower
 */
class Mower {
public:
    // Navigation modes
    enum class NavigationMode {
        RANDOM,      // Random movement
        PARALLEL,    // Parallel lines pattern
        SPIRAL,      // Spiral pattern
        STOPPED,     // Stopped
        MANUAL       // Manual control
    };
    friend class MowerStateMachine; // Allow state machine to access private members
    
private:
    // State machine instance
    MowerStateMachine* stateMachine_{nullptr};
    Maneuver* maneuver_{nullptr};  // Controller delle manovre
    
    // Position management
    PositionManager* positionManager_{nullptr};
    
    // Navigation state
    NavigationMode navigationMode_{NavigationMode::STOPPED};
    float targetX_{0.0f};
    float targetY_{0.0f};
    float targetAngle_{0.0f};
    float currentX_{0.0f};
    float currentY_{0.0f};
    float currentAngle_{0.0f};
    
    // Charging state
    bool isCharging_{false};
    bool chargingEnabled_{false};
    unsigned long chargingStartTime_{0};
    float lastChargeCurrent_{0.0f};
    
    // System states
    bool isManualControl_{false};
    bool isEmergencyStop_{false};
    bool isMowing_{false};
    bool dockingInProgress_{false};
    bool borderDetected_{false};
    bool lifted_{false};
    bool rainDetected_{false};
    bool atDockingStation_{false};
    bool collisionDetected_{false};
    bool chargingError_{false};
    bool testComplete_{false};
    
    // Timing
    unsigned long lastSensorUpdate_{0};
    unsigned long lastMotorUpdate_{0};
    unsigned long lastBladeUpdate_{0};
    unsigned long lastNavigationUpdate_{0};
    unsigned long lastObstacleTime_{0};
    unsigned long lastDebugPrint_{0};
    unsigned long lastLoopTime_{0};
    unsigned long lastTelemetryUpdate_{0};
    unsigned long lastBatteryUpdate_{0};
    unsigned long lastOdometryUpdate_{0};
    unsigned long lastSafetyCheckUpdate_{0};
    
    // Battery state
    float batteryVoltage_{0.0f};
    float batteryCurrent_{0.0f};
    
    // Obstacle avoidance
    int obstacleAvoidanceState_{0};
    
    // Timing intervals are defined in config.h
    // MAIN_LOOP_DELAY: 50ms
    // SENSOR_UPDATE_INTERVAL: 50ms (20Hz)
    // NAVIGATION_UPDATE_INTERVAL: 100ms (10Hz)
    // SAFETY_CHECK_INTERVAL: 50ms (20Hz)
    // ODOMETRY_UPDATE_INTERVAL: 10ms (100Hz)
    // TELEMETRY_UPDATE_INTERVAL: 1000ms (1Hz)
    // BATTERY_UPDATE_INTERVAL: 5000ms (0.2Hz)
    
    // Navigation constants from config.h
    static constexpr float ANGLE_TOLERANCE = HEADING_TOLERANCE_DEG * DEG_TO_RAD;
    
    // Update methods
    void updateSensors();
    void updateBattery();
    void updateTelemetry();
    void updateNavigationTimed();
    void checkSafety();
    
    // Navigation helpers
    void updatePosition();
    void updateNavigation();
    void setMotors(float leftSpeed, float rightSpeed);
    bool isAtTarget(float tolerance = 0.1f);
    void rotateToAngle(float targetAngle);
    void moveToPosition(float x, float y);
    void handleBorder();
    void handleRain();
    void handleLowBattery();
    float normalizeAngle(float angle);
    
    // Navigation patterns
    void navigateRandom();
    void navigateParallel();
    void navigateSpiral();
    
    // Navigation control
    void updateTargetPosition(const RobotPosition& currentPos);
    void onTargetReached();
    
    // Sensor checks
    bool checkObstacles();
    bool checkPerimeter();
    void handleObstacle();
    void handlePerimeter();
    
    // Battery management
    float readBatteryVoltage() const;
    float readBatteryCurrent() const;
    
public:
    /**
     * @brief Construct a new Mower object
     * 
     * @param leftMotorPin Pin for the left motor
     * @param rightMotorPin Pin for the right motor
     * @param bladeMotorPin Pin for the blade motor
     * @param motorRelayPin Pin for the motor relay
     */
    Mower(uint8_t leftMotorPin, uint8_t rightMotorPin, uint8_t bladeMotorPin, uint8_t motorRelayPin);
    
    /**
     * @brief Destroy the Mower object
     * 
     * Cleans up dynamically allocated memory and resources.
     */
    virtual ~Mower();
    
    // Prevent copying and assignment
    Mower(const Mower&) = delete;
    Mower& operator=(const Mower&) = delete;
    
    /**
     * @brief Initialize the mower and all its components
     */
    void begin();
    
    /**
     * @brief Main update function to be called in the main loop
     * @param currentTime Current time in milliseconds from millis()
     */
    void update(unsigned long currentTime = 0);
    
    /**
     * @brief Get the battery update interval
     * @return Battery update interval in milliseconds
     */
    unsigned long getBatteryUpdateInterval() const { return BATTERY_UPDATE_INTERVAL; }
    
    // ===== State Machine Interface =====
    
    /**
     * @brief Enable or disable battery charging
     * @param enable True to enable charging, false to disable
     */
    void enableCharging(bool enable);
    
    /**
     * @brief Start the docking procedure
     */
    void startDocking();
    
    /**
     * @brief Start the mowing operation
     */
    void startMowing();
    
    /**
     * @brief Emergency stop all operations
     */
    void emergencyStop();
    
    // ===== Sensor Checks =====
    
    /**
     * @brief Check if the mower is at the docking station
     * @return true if at docking station, false otherwise
     */
    bool isAtDockingStation();
    
    /**
     * @brief Check if docking has failed
     * @return true if docking failed, false otherwise
     */
    bool isDockingFailed();
    
    /**
     * @brief Check if the battery is fully charged
     * @return true if battery is full, false otherwise
     */
    bool isBatteryFull();
    
    /**
     * @brief Check if the battery is currently charging
     * @return true if battery is charging (negative current), false otherwise
     */
    bool isCharging();
    
    /**
     * @brief Check if the battery is low
     * @return true if battery is low, false otherwise
     */
    bool isBatteryLow();
    
    /**
     * @brief Check if the battery is critically low
     * @return true if battery is critically low, false otherwise
     */
    bool isBatteryCritical();
    
    /**
     * @brief Get the current state machine instance
     * @return Reference to the state machine
     */
    MowerStateMachine& getStateMachine() { return *stateMachine_; }
    
    /**
     * @brief Check if rain is detected
     * @return true if rain is detected, false otherwise
     */
    bool isRainDetected();
    
    /**
     * @brief Check if the mower is lifted
     * @return true if lifted, false otherwise
     */
    bool isLifted();
    
    /**
     * @brief Stops all motors (drive and blade)
     */
    void stopMotors();
    
    /**
     * @brief Check if a border is detected
     * @return true if border detected, false otherwise
     */
    bool isBorderDetected();
    
    /**
     * @brief Check if a collision is detected
     * @return true if collision detected, false otherwise
     */
    bool isCollisionDetected();
    
    /**
     * @brief Check if there is a charging error
     * @return true if charging error, false otherwise
     */
    bool isChargingError();
    
    // ===== Navigation Control =====
    
    /**
     * @brief Set the navigation mode
     * @param mode The navigation mode to set
     */
    void setNavigationMode(NavigationMode mode);
    
    /**
     * @brief Get the current navigation mode
     * @return The current navigation mode
     */
    NavigationMode getNavigationMode() const;
    
    /**
     * @brief Check if the mower is currently navigating
     * @return true if navigating, false otherwise
     */
    bool isNavigating() const;
    
    // ===== State Machine Control =====
    
    /**
     * @brief Check if the mower should start mowing
     * @return true if should start mowing, false otherwise
     */
    bool shouldStartMowing();
    
    /**
     * @brief Check if the mower should return to dock after border detection
     * @return true if should return to dock, false otherwise
     */
    bool shouldReturnToDockAfterBorder() const;
    
    /**
     * @brief Check if the mower should return to dock when rain is detected
     * @return true if should return to dock, false otherwise
     */
    bool shouldReturnToDockInRain() const;
    
    // ===== Logging =====
    
    /**
     * @brief Log a state change
     * @param oldState The previous state
     * @param newState The new state
     */
    void logStateChange(MowerState oldState, MowerState newState);
    
    // ===== Power Management =====
    
    /**
     * @brief Enter low-power sleep mode
     */
    void enterSleepMode();
    
    // ===== Test Mode =====
    
    /**
     * @brief Check if the test is complete
     * @return true if test is complete, false otherwise
     */
    bool isTestComplete() const;
    
private:
    // Motor control
    void setMotorSpeed(float left, float right);
    void setBladeSpeed(float speed);
    
    // Navigation
    void navigate();
    
    // Battery management
    bool isBatteryFull() const;
    bool isBatteryLow() const;
    bool isBatteryCritical() const;
    bool isAtDockingStation() const;
    bool isCharging() const;

    // ===== Member Variables =====
    
    // Hardware Components
    RainSensor rainSensor_;
    BatterySensor batterySensor_;
    IMUModule imu_;
    GPSModule gps_;
    
    // Motors
    DriveMotor leftMotor_;
    DriveMotor rightMotor_;
    BladeMotor bladeMotor_;
    Relay motorsRelay_;
    
    // Sensor Arrays
    UltrasonicSensors ultrasonicSensors_;
    BumpSensors bumpSensors_;
    PerimeterSensors perimeterSensors_;
};

#endif // MOWER_H
