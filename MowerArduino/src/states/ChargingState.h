/**
 * @file ChargingState.h
 * @brief Definition of the ChargingState class for the autonomous mower.
 * 
 * This file contains the declaration of the ChargingState class, which manages
 * the mower's behavior while it is connected to the charging station. This state
 * monitors the charging process, battery levels, and handles transitions to other
 * states when charging is complete or when conditions change.
 */

#ifndef CHARGING_STATE_H
#define CHARGING_STATE_H

#include "MowerState.h"

/**
 * @class ChargingState
 * @brief State representing when the mower is connected to the charging station.
 * 
 * The ChargingState is responsible for:
 * - Monitoring the charging process and battery levels
 * - Detecting when the battery is fully charged
 * - Managing the charging relay and power systems
 * - Handling transitions to other states when charging is complete
 * - Providing status updates through the user interface
 * 
 * @note This class implements the Singleton pattern as there should only
 *       be one instance of the charging state in the system.
 */
class ChargingState : public MowerState {
public:
    /**
     * @brief Get the singleton instance of ChargingState
     * @return Reference to the single instance of ChargingState
     */
    static ChargingState& getInstance() {
        static ChargingState instance;
        return instance;
    }
    
    // ===== MowerState Interface Implementation =====
    
    /**
     * @brief Called when entering the Charging state
     * @param mower Reference to the Mower instance
     */
    void enter(Mower& mower) override;
    
    /**
     * @brief Update function called periodically while in this state
     * @param mower Reference to the Mower instance
     */
    void update(Mower& mower) override;
    
    /**
     * @brief Called when exiting the Charging state
     * @param mower Reference to the Mower instance
     */
    void exit(Mower& mower) override;
    
    /**
     * @brief Get the name of this state
     * @return String literal "CHARGING"
     */
    const char* getName() const override { return "CHARGING"; }
    
    /**
     * @brief Get the state type enum value
     * @return State::CHARGING enum value
     */
    State getStateType() const override { return State::CHARGING; }
    
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
    ChargingState(const ChargingState&) = delete;
    
    /**
     * @brief Deleted assignment operator
     */
    void operator=(const ChargingState&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern
     */
    ChargingState() = default;
    
    // ===== Member Variables =====
    
    unsigned long lastBatteryCheck_;  ///< Timestamp of last battery level check (ms)
    bool isFullyCharged_;            ///< Flag indicating if battery is fully charged
};

#endif // CHARGING_STATE_H
