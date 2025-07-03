// LawnMowerNavigator.cpp
#include "LawnMowerNavigator.h"
#include "../config.h"
#include "../functions/Mower.h"
#include "../functions/PositionManager.h"

extern PositionManager positionManager;

LawnMowerNavigator::LawnMowerNavigator(Mower& mower) : mower_(mower), state_(State::STRAIGHT) {}

void LawnMowerNavigator::begin() {
    state_ = State::STRAIGHT;
    driveForward();
}

void LawnMowerNavigator::driveForward() {
    mower_.setLeftMotorSpeed(DEFAULT_MOTOR_SPEED);
    mower_.setRightMotorSpeed(DEFAULT_MOTOR_SPEED);
}

void LawnMowerNavigator::update() {
    switch (state_) {
        case State::STRAIGHT: {
            if (mower_.isObstacleDetected() || mower_.isBorderDetected()) {
                mower_.stopDriveMotors();
                state_ = State::TURN1;
                positionManager.turn(-PI/2, 0.4f); // 90° left
            }
            break;
        }
        case State::TURN1: {
            // After turn complete, check immediate obstacle
            if (mower_.isObstacleDetected() || mower_.isBorderDetected()) {
                // Corner situation -> 270° total left (already turned 90)
                positionManager.turn(-PI*3/2, 0.4f);
                state_ = State::STRAIGHT;
                driveForward();
            } else {
                state_ = State::OFFSET;
                positionManager.moveStraight(DEFAULT_MOTOR_SPEED, PARALLEL_LINE_SPACING);
            }
            break;
        }
        case State::OFFSET: {
            if (mower_.isObstacleDetected() || mower_.isBorderDetected()) {
                // Corner encountered during offset
                positionManager.turn(-PI*3/2, 0.4f);
                state_ = State::STRAIGHT;
                driveForward();
            } else {
                state_ = State::TURN2;
                positionManager.turn(-PI/2, 0.4f); // second 90° left
            }
            break;
        }
        case State::TURN2: {
            state_ = State::STRAIGHT;
            driveForward();
            break;
        }
        case State::CORNER_TURN: {
            // handled inline above
            break;
        }
    }
}
