/**
 * @file EmergencyStopState.h
 * @brief Definition of the EmergencyStopState class for the autonomous mower.
 * 
 * This file contains the declaration of the EmergencyStopState class, which is
 * activated when an emergency stop condition is detected. This state immediately
 * halts all mower operations and requires manual intervention to resume normal
 * operation after the emergency condition is resolved.
 */

#ifndef EMERGENCY_STOP_STATE_H
#define EMERGENCY_STOP_STATE_H

#include "MowerState.h"
#include "../functions/MowerTypes.h"

/**
 * @class EmergencyStopState
 * @brief State representing when the mower has triggered an emergency stop.
 * 
 * The EmergencyStopState is responsible for:
 * - Immediately stopping all motors and moving parts
 * - Activating visual and audible alarms
 * - Preventing any further movement until manually reset
 * - Providing clear status about the emergency condition
 * - Requiring explicit user intervention to clear the emergency state
 * 
 * @note This class implements the Singleton pattern as there should only
 *       be one instance of the emergency stop state in the system.
 */
class EmergencyStopState : public MowerState {
public:
    /**
     * @brief Get the singleton instance of EmergencyStopState
     * @return Reference to the single instance of EmergencyStopState
     */
    static EmergencyStopState& getInstance() {
        static EmergencyStopState instance;
        return instance;
    }
    
    // ===== MowerState Interface Implementation =====
    
    /**
     * @brief Called when entering the Emergency Stop state
     * @param mower Reference to the Mower instance
     */
    void enter(Mower& mower) override;
    
    /**
     * @brief Update function called periodically while in this state
     * @param mower Reference to the Mower instance
     */
    void update(Mower& mower) override;
    
    /**
     * @brief Called when exiting the Emergency Stop state
     * @param mower Reference to the Mower instance
     */
    void exit(Mower& mower) override;
    
    /**
     * @brief Get the name of this state
     * @return String literal "EMERGENCY_STOP"
     */
    const char* getName() const override { return "EMERGENCY_STOP"; }
    
    /**
     * @brief Get the state type enum value
     * @return State::EMERGENCY_STOP enum value
     */
    State getStateType() const override { return State::EMERGENCY_STOP; }
    
    // ===== Event Handling =====
    
    /**
     * @brief Handle system events
     * @param mower Reference to the Mower instance
     * @param event The event to handle
     */
    void handleEvent(Mower& mower, Event event) override;

    // ===== Singleton Implementation =====
    
    /**
     * @brief Deleted copy constructor
     */
    EmergencyStopState(const EmergencyStopState&) = delete;
    
    /**
     * @brief Deleted assignment operator
     */
    void operator=(const EmergencyStopState&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern
     */
    EmergencyStopState() = default;
    
    // ===== Member Variables =====
    
    unsigned long lastBeepTime_ = 0;  ///< Timestamp of the last beep sound (ms)
    bool isBeeping_ = false;         ///< Flag indicating if the alarm is currently beeping
};

#endif // EMERGENCY_STOP_STATE_H
