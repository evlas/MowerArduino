// RandomNavigator.cpp
#include "RandomNavigator.h"
#include "../config.h"
#include "../functions/Mower.h"
#include "../functions/PositionManager.h"
#include <Arduino.h>

extern PositionManager positionManager;

RandomNavigator::RandomNavigator(Mower& mower) : mower_(mower), state_(State::DRIVING) {}

void RandomNavigator::init(Mower& mower) {
    // Initialize any required resources
    state_ = State::DRIVING;
}

void RandomNavigator::start(Mower& mower) {
    state_ = State::DRIVING;
    driveForward();
}

void RandomNavigator::stop(Mower& mower) {
    mower.stopDriveMotors();
    state_ = State::DRIVING; // Reset to default state
}

void RandomNavigator::driveForward() {
    mower_.setLeftMotorSpeed(DEFAULT_MOTOR_SPEED);
    mower_.setRightMotorSpeed(DEFAULT_MOTOR_SPEED);
}

void RandomNavigator::update(Mower& mower) {
    switch (state_) {
        case State::DRIVING:
            if (mower.isObstacleDetected() || mower.isBorderDetected()) {
                // Stop and reverse 0.5 m
                mower.stopDriveMotors();
                positionManager.moveStraight(-DEFAULT_MOTOR_SPEED, 0.5f); // reverse 0.5 m at default speed
                // Random turn between 30 and 150 degrees, either direction
                int deg = random(30, 151);
                if (random(0, 2) == 0) deg = -deg;
                float rad = deg * DEG_TO_RAD;
                positionManager.turn(rad, 0.4f);
                // Resume driving
                driveForward();
            }
            break;
        case State::REVERSING:
        case State::TURNING:
            // Blocking calls above handle these, so should not stay in these states
            state_ = State::DRIVING;
            break;
    }
}

bool RandomNavigator::handleEvent(Mower& mower, Event event) {
    // Handle events if needed
    return false; // Event not handled
}
