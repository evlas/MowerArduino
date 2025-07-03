// RandomNavigator.cpp
#include "RandomNavigator.h"
#include "../config.h"
#include "../functions/Mower.h"
#include "../functions/PositionManager.h"

extern PositionManager positionManager;

RandomNavigator::RandomNavigator(Mower& mower) : mower_(mower), state_(State::DRIVING) {}

void RandomNavigator::begin() {
    state_ = State::DRIVING;
    driveForward();
}

void RandomNavigator::driveForward() {
    mower_.setLeftMotorSpeed(DEFAULT_MOTOR_SPEED);
    mower_.setRightMotorSpeed(DEFAULT_MOTOR_SPEED);
}

void RandomNavigator::update() {
    switch (state_) {
        case State::DRIVING:
            if (mower_.isObstacleDetected() || mower_.isBorderDetected()) {
                // Stop and reverse 0.5 m
                mower_.stopDriveMotors();
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
