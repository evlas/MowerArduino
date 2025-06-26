#ifndef MOWER_STATE_MACHINE_H
#define MOWER_STATE_MACHINE_H

#include <Arduino.h>

// Forward declaration of Mower class to avoid circular dependencies
class Mower;

/**
 * @brief Enumeration of all possible states for the mower
 */
enum class MowerState {
    IDLE,               // Initial state, mower is waiting for commands
    DOCKING,            // Returning to charging station
    CHARGING,           // Charging the battery
    MOWING,             // Actively mowing the lawn
    MANUAL_CONTROL,     // Under manual control
    ERROR,              // Error state that needs attention
    EMERGENCY_STOP,     // Emergency stop activated
    BORDER_DETECTED,    // Detected a border/obstacle
    LIFTED,             // Mower has been lifted
    RAIN_DELAY,         // Paused due to rain
    MAINTENANCE_NEEDED, // Maintenance required
    TESTING,            // Testing mode
    PAUSED,             // Paused state (manual pause)
    SLEEP,              // Low power sleep mode
    ROS_CONTROL         // Remote ROS control mode
};

/**
 * @brief State Machine class for controlling the mower's behavior
 */
class MowerStateMachine {
public:
    /**
     * @brief Construct a new Mower State Machine object
     * @param mower Reference to the main Mower class
     */
    explicit MowerStateMachine(Mower& mower);

    /**
     * @brief Initialize the state machine
     */
    void begin();

    /**
     * @brief Update the state machine (to be called in the main loop)
     */
    void update();

    /**
     * @brief Get the current state
     * @return MowerState Current state
     */
    MowerState getCurrentState() const;

    /**
     * @brief Request a state transition
     * @param newState The state to transition to
     * @return bool True if transition was successful
     */
    bool requestStateChange(MowerState newState);

    // State transition methods for external control
    void startMowing();
    void dock();
    void emergencyStop();
    void manualControl();
    void resetError();

private:
    Mower& mower_;
    MowerState currentState_;
    MowerState previousState_;
    unsigned long stateStartTime_;
    bool emergencyStopActive_;

    // State entry/exit/update handlers
    void onEnterState(MowerState newState);
    void onExitState(MowerState oldState);
    void onUpdateState();

    // State validation
    bool isTransitionAllowed(MowerState from, MowerState to) const;
    bool isEmergencyStopActive() const;

    // State handlers
    void handleIdle();
    void handleDocking();
    void handleCharging();
    void handleMowing();
    void handleManualControl();
    void handleError();
    void handleEmergencyStop();
    void handleBorderDetected();
    void handleLifted();
    void handleRainDelay();
    void handleMaintenanceNeeded();
    void handleTesting();
};

#endif // MOWER_STATE_MACHINE_H
