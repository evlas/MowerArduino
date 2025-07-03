/**
 * @file MowingState.h
 * @brief Definition of the MowingState class for the autonomous mower.
 * 
 * This file contains the declaration of the MowingState class, which represents
 * the state when the mower is actively cutting grass. This state manages
 * the mowing pattern, obstacle avoidance, and battery monitoring during operation.
 */

#ifndef MOWING_STATE_H
#define MOWING_STATE_H

#include "MowerState.h"
#include "../functions/MowerTypes.h"
#include <Arduino.h>

// Forward declaration
class Mower;

/**
 * @brief Display update interval in milliseconds
 */
constexpr unsigned long DISPLAY_UPDATE_INTERVAL = 1000;

/**
 * @brief Maximum inactivity time before returning to base (30 minutes)
 */
constexpr unsigned long MAX_INACTIVITY_TIME = 30 * 60 * 1000UL;

/**
 * @brief Default driving speed (0-255)
 */
constexpr uint8_t DEFAULT_DRIVE_SPEED = 180;

/**
 * @class MowingState
 * @brief State representing when the mower is actively cutting grass.
 * 
 * The MowingState is responsible for:
 * - Controlling the mowing pattern and movement
 * - Monitoring battery level and returning to base when needed
 * - Avoiding obstacles and borders
 * - Managing blade motor operation
 * - Updating the user interface with mowing status
 * 
 * @note This class implements the Singleton pattern as there should only
 *       be one instance of the mowing state in the system.
 */
class MowingState : public MowerState {
public:
    /**
     * @brief Get the singleton instance of MowingState
     * @return Reference to the single instance of MowingState
     */
    static MowingState& getInstance() {
        static MowingState instance;
        return instance;
    }
    
    // ===== MowerState Interface Implementation =====
    
    /**
     * @brief Called when entering the Mowing state
     * @param mower Reference to the Mower instance
     */
    void enter(Mower& mower) override;
    
    /**
     * @brief Update function called periodically while in this state
     * @param mower Reference to the Mower instance
     */
    void update(Mower& mower) override;
    
    /**
     * @brief Called when exiting the Mowing state
     * @param mower Reference to the Mower instance
     */
    void exit(Mower& mower) override;
    
    /**
     * @brief Get the name of this state
     * @return String literal "MOWING"
     */
    const char* getName() const override { return "MOWING"; }
    
    /**
     * @brief Get the state type enum value
     * @return State::MOWING enum value
     */
    State getStateType() const override { return State::MOWING; }
    
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
    MowingState(const MowingState&) = delete;
    
    /**
     * @brief Deleted assignment operator
     */
    void operator=(const MowingState&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern
     */
    MowingState() = default;
    
    // ===== Private Member Variables =====
    
    unsigned long lastActivityTime_ = 0;     ///< Timestamp of last activity (ms)
    unsigned long lastDisplayUpdate_ = 0;    ///< Timestamp of last display update (ms)
    
    // ===== Private Methods =====
    
    /**
     * @brief Update the display with current status information
     * @param mower Reference to the Mower instance
     */
    void updateDisplay(Mower& mower);
    
    /**
     * @brief Handle border detection and avoidance
     * @param mower Reference to the Mower instance
     */
    void handleBorderAvoidance(Mower& mower);
};

#endif // MOWING_STATE_H
