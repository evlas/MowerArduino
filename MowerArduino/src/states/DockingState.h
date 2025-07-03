/**
 * @file DockingState.h
 * @brief Definition of the DockingState class for the autonomous mower.
 * 
 * This file contains the declaration of the DockingState class, which manages
 * the process of returning the mower to its charging station. This state is
 * responsible for navigating to the dock, aligning with the charging contacts,
 * and initiating the charging process.
 */

#ifndef DOCKING_STATE_H
#define DOCKING_STATE_H

#include "MowerState.h"
#include "../functions/MowerTypes.h"
#include "../pin_config.h"
#include <Arduino.h>

// Forward declaration
class Mower;

/**
 * @class DockingState
 * @brief State representing when the mower is returning to its charging station.
 * 
 * The DockingState manages the complex process of navigating back to the charging
 * station, which involves multiple phases:
 * 1. Initial approach: Navigating to the general area of the charging station
 * 2. Alignment: Precisely aligning with the charging contacts
 * 3. Final approach: Moving straight into the charging station
 * 4. Docked: Successfully connected to the charging station
 * 
 * This state handles sensor inputs, motor control, and error recovery during
 * the docking process.
 * 
 * @note This class implements the Singleton pattern as there should only
 *       be one instance of the docking state in the system.
 */
class DockingState : public MowerState {
public:
    /**
     * @brief Get the singleton instance of DockingState
     * @return Reference to the single instance of DockingState
     */
    static DockingState& getInstance() {
        static DockingState instance;
        return instance;
    }
    
    // ===== MowerState Interface Implementation =====
    
    /**
     * @brief Called when entering the Docking state
     * @param mower Reference to the Mower instance
     */
    void enter(Mower& mower) override;
    
    /**
     * @brief Update function called periodically while in this state
     * @param mower Reference to the Mower instance
     */
    void update(Mower& mower) override;
    
    /**
     * @brief Called when exiting the Docking state
     * @param mower Reference to the Mower instance
     */
    void exit(Mower& mower) override;
    
    /**
     * @brief Get the name of this state
     * @return String literal "DOCKING"
     */
    const char* getName() const override { return "DOCKING"; }
    
    /**
     * @brief Get the state type enum value
     * @return State::DOCKING enum value
     */
    State getStateType() const override { return State::DOCKING; }
    
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
    DockingState(const DockingState&) = delete;
    
    /**
     * @brief Deleted assignment operator
     */
    void operator=(const DockingState&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern
     */
    DockingState() = default;
    
    // ===== Docking Phases =====
    
    /**
     * @brief Enumeration of docking phases
     */
    enum class DockingPhase {
        INITIAL_APPROACH,  ///< Navigating to the general area of the dock
        ALIGNMENT,         ///< Aligning with the charging contacts
        FINAL_APPROACH,    ///< Moving straight into the dock
        DOCKED             ///< Successfully connected to the dock
    };
    
    // ===== Member Variables =====
    
    DockingPhase dockingPhase_;           ///< Current docking phase
    unsigned long lastPhaseChangeTime_ = 0; ///< Timestamp of last phase change (ms)
    unsigned long dockingStartTime_ = 0;   ///< Timestamp when docking started (ms)
    unsigned long phaseStartTime_ = 0;     ///< Timestamp when current phase started (ms)
    
    // ===== Constants =====
    
    static constexpr float DOCK_DETECTION_DISTANCE = 50.0f;  ///< Maximum distance to detect dock (cm)
    static constexpr unsigned long DOCKING_TIMEOUT = 5 * 60 * 1000UL;  ///< Total timeout for docking (5 min)
    static constexpr unsigned long ALIGNMENT_TIMEOUT = 30000UL;  ///< Timeout for alignment phase (30s)
    static constexpr unsigned long DOCKING_CONFIRMATION_TIME = 3000UL;  ///< Time to confirm docked state (3s)
    
    // ===== Private Methods =====
    
    /**
     * @brief Handle the initial approach phase
     * @param mower Reference to the Mower instance
     */
    void handleInitialApproach(Mower& mower);
    
    /**
     * @brief Handle the alignment phase
     * @param mower Reference to the Mower instance
     */
    void handleAlignment(Mower& mower);
    
    /**
     * @brief Handle the final approach phase
     * @param mower Reference to the Mower instance
     */
    void handleFinalApproach(Mower& mower);
    
    /**
     * @brief Transition to a new docking phase
     * @param newPhase The phase to transition to
     */
    void changePhase(DockingPhase newPhase);
};

#endif // DOCKING_STATE_H
