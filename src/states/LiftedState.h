/**
 * @file LiftedState.h
 * @brief Definition of the LiftedState class for the autonomous mower.
 * 
 * This file contains the declaration of the LiftedState class, which is
 * activated when the mower detects that it has been lifted off the ground.
 * This state ensures safety by stopping all moving parts and waiting for
 * the mower to be placed back on the ground.
 */

#ifndef LIFTED_STATE_H
#define LIFTED_STATE_H

#include "MowerState.h"
#include "../functions/MowerTypes.h"
#include <Arduino.h>

/**
 * @class LiftedState
 * @brief State representing when the mower has been lifted off the ground.
 * 
 * The LiftedState is responsible for:
 * - Detecting and confirming when the mower is lifted
 * - Stopping all motors and blades immediately
 * - Providing clear feedback that the mower is in a lifted state
 * - Monitoring for the mower to be placed back on the ground
 * - Handling timeouts if the mower remains lifted for too long
 * 
 * @note This class implements the Singleton pattern as there should only
 *       be one instance of the lifted state in the system.
 */
class LiftedState : public MowerState {
public:
    /**
     * @brief Get the singleton instance of LiftedState
     * @return Reference to the single instance of LiftedState
     */
    static LiftedState& getInstance() {
        static LiftedState instance;
        return instance;
    }
    
    // ===== MowerState Interface Implementation =====
    
    /**
     * @brief Called when entering the Lifted state
     * @param mower Reference to the Mower instance
     */
    void enter(Mower& mower) override;
    
    /**
     * @brief Update function called periodically while in this state
     * @param mower Reference to the Mower instance
     */
    void update(Mower& mower) override;
    
    /**
     * @brief Called when exiting the Lifted state
     * @param mower Reference to the Mower instance
     */
    void exit(Mower& mower) override;
    
    /**
     * @brief Get the name of this state
     * @return String literal "LIFTED"
     */
    const char* getName() const override { return "LIFTED"; }
    
    /**
     * @brief Get the state type enum value
     * @return State::LIFTED enum value
     */
    State getStateType() const override { return State::LIFTED; }
    
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
    LiftedState(const LiftedState&) = delete;
    
    /**
     * @brief Deleted assignment operator
     */
    void operator=(const LiftedState&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern
     */
    LiftedState() = default;
    
    // ===== Member Variables =====
    
    unsigned long liftStartTime_ = 0;  ///< Timestamp when the lift was first detected (ms)
    bool liftConfirmed_ = false;       ///< Flag indicating if the lift has been confirmed
    
    // ===== Constants =====
    
    /**
     * @brief Time required to confirm a lift condition (ms)
     * 
     * This prevents false positives from brief sensor fluctuations.
     */
    static constexpr unsigned long LIFT_CONFIRMATION_TIME = 2000;  // 2 seconds
    
    /**
     * @brief Maximum time the mower can remain lifted before taking action (ms)
     * 
     * After this time, the mower may enter an error state or take other
     * safety measures.
     */
    static constexpr unsigned long MAX_LIFTED_TIME = 5 * 60 * 1000UL;  // 5 minutes
};

#endif // LIFTED_STATE_H
