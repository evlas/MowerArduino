/**
 * @file ErrorState.h
 * @brief Definition of the ErrorState class for the autonomous mower.
 * 
 * This file contains the declaration of the ErrorState class, which is
 * activated when an unrecoverable error is detected in the mower system.
 * This state ensures safe shutdown and provides diagnostic information
 * about the error condition.
 */

#ifndef ERROR_STATE_H
#define ERROR_STATE_H

#include "MowerState.h"

/**
 * @class ErrorState
 * @brief State representing when the mower has encountered an error condition.
 * 
 * The ErrorState is responsible for:
 * - Capturing and storing error information when an error occurs
 * - Safely stopping all mower operations
 * - Providing diagnostic information about the error
 * - Preventing further operation until the error is cleared
 * - Supporting error recovery procedures
 * 
 * @note This class implements the Singleton pattern as there should only
 *       be one instance of the error state in the system.
 */
class ErrorState : public MowerState {
public:
    /**
     * @brief Get the singleton instance of ErrorState
     * @return Reference to the single instance of ErrorState
     */
    static ErrorState& getInstance() {
        static ErrorState instance;
        return instance;
    }
    
    // ===== MowerState Interface Implementation =====
    
    /**
     * @brief Called when entering the Error state
     * @param mower Reference to the Mower instance
     */
    void enter(Mower& mower) override;
    
    /**
     * @brief Update function called periodically while in this state
     * @param mower Reference to the Mower instance
     */
    void update(Mower& mower) override;
    
    /**
     * @brief Called when exiting the Error state
     * @param mower Reference to the Mower instance
     */
    void exit(Mower& mower) override;
    
    /**
     * @brief Get the name of this state
     * @return String literal "ERROR"
     */
    const char* getName() const override { return "ERROR"; }
    
    /**
     * @brief Get the state type enum value
     * @return State::ERROR enum value
     */
    State getStateType() const override { return State::ERROR; }
    
    // ===== Event Handling =====
    
    /**
     * @brief Handle system events
     * @param mower Reference to the Mower instance
     * @param event The event to handle
     */
    void handleEvent(Mower& mower, Event event) override;

    // ===== Error Management =====
    
    /**
     * @brief Set the current error code
     * @param code The error code to set
     */
    void setErrorCode(int code) { errorCode_ = code; }
    
    /**
     * @brief Get the current error code
     * @return The current error code
     */
    int getErrorCode() const { return errorCode_; }
    
    /**
     * @brief Get a human-readable error message for the current error code
     * @return Pointer to a string describing the error, or "Unknown error"
     */
    const char* getErrorMessage() const;

    // ===== Singleton Implementation =====
    
    /**
     * @brief Deleted copy constructor
     */
    ErrorState(const ErrorState&) = delete;
    
    /**
     * @brief Deleted assignment operator
     */
    void operator=(const ErrorState&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern
     */
    ErrorState() = default;
    
    // ===== Member Variables =====
    
    int errorCode_ = 0;  ///< Numeric code identifying the current error condition
};

#endif // ERROR_STATE_H
