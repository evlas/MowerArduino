/**
 * @file MowerState.h
 * @brief Abstract base class for all mower states in the state machine.
 * 
 * This file defines the MowerState interface that all concrete state classes must implement.
 * It follows the State design pattern to manage different behaviors of the mower.
 */

#ifndef MOWER_STATE_H
#define MOWER_STATE_H

#include "../functions/MowerTypes.h"

// Forward declaration for Mower class to avoid circular dependency
class Mower;

/**
 * @class MowerState
 * @brief Abstract base class defining the interface for all mower states.
 * 
 * The MowerState class serves as the base class for all states in the mower's
 * state machine. Each concrete state implements specific behavior for the
 * mower's operations in that particular state.
 * 
 * @note This is an abstract class and cannot be instantiated directly.
 */
class MowerState {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup in derived classes
     */
    virtual ~MowerState() = default;
    
    // ===== Pure Virtual Methods (must be implemented by derived classes) =====
    
    /**
     * @brief Called when entering this state
     * @param mower Reference to the Mower instance
     */
    virtual void enter(Mower& mower) = 0;
    
    /**
     * @brief Called repeatedly during the state's active period
     * @param mower Reference to the Mower instance
     */
    virtual void update(Mower& mower) = 0;
    
    /**
     * @brief Called when exiting this state
     * @param mower Reference to the Mower instance
     */
    virtual void exit(Mower& mower) = 0;
    
    /**
     * @brief Get the human-readable name of this state
     * @return String containing the state's name
     */
    virtual const char* getName() const = 0;
    
    /**
     * @brief Get the state type enum value
     * @return The State enum value corresponding to this state
     */
    virtual State getStateType() const = 0;
    
    // ===== Event Handling =====
    
    /**
     * @brief Handle a system event
     * @param mower Reference to the Mower instance
     * @param event The event to handle
     */
    virtual void handleEvent(Mower& mower, Event event) = 0;
    
    // ===== Error Handling =====
    
    /**
     * @brief Handle an error condition
     * @param mower Reference to the Mower instance
     * @param errorMsg Error message describing the issue
     */
    virtual void handleError(Mower& mower, const char* errorMsg);
};

#endif // MOWER_STATE_H
