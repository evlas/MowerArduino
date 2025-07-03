/**
 * @file IdleState.h
 * @brief Definition of the IdleState class for the autonomous mower.
 * 
 * This file contains the declaration of the IdleState class, which represents
 * the state when the mower is powered on but not actively mowing or performing
 * other operations. It's a waiting state that responds to user commands.
 */

#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "MowerState.h"

/**
 * @class IdleState
 * @brief State representing when the mower is idle and waiting for commands.
 * 
 * The IdleState is the default state when the mower is powered on but not
 * actively performing any tasks. In this state, the mower:
 * - Waits for user input to start mowing or other operations
 * - Monitors system status and battery level
 * - Can transition to other states based on events or commands
 * 
 * @note This class implements the Singleton pattern as there should only
 *       be one instance of the idle state in the system.
 */
class IdleState : public MowerState {
public:
    /**
     * @brief Get the singleton instance of IdleState
     * @return Reference to the single instance of IdleState
     */
    static IdleState& getInstance() {
        static IdleState instance;
        return instance;
    }
    
    // ===== MowerState Interface Implementation =====
    
    /**
     * @brief Called when entering the Idle state
     * @param mower Reference to the Mower instance
     */
    void enter(Mower& mower) override;
    
    /**
     * @brief Update function called periodically while in this state
     * @param mower Reference to the Mower instance
     */
    void update(Mower& mower) override;
    
    /**
     * @brief Called when exiting the Idle state
     * @param mower Reference to the Mower instance
     */
    void exit(Mower& mower) override;
    
    /**
     * @brief Get the name of this state
     * @return String literal "IDLE"
     */
    const char* getName() const override { return "IDLE"; }
    
    /**
     * @brief Get the state type enum value
     * @return State::IDLE enum value
     */
    State getStateType() const override { return State::IDLE; }
    
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
    IdleState(const IdleState&) = delete;
    
    /**
     * @brief Deleted assignment operator
     */
    void operator=(const IdleState&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern
     */
    IdleState() = default;
};

#endif // IDLE_STATE_H
