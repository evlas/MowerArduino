#include "BorderNavigator.h"
#include "../functions/Mower.h"
#include "../functions/PositionManager.h"
#include <Arduino.h>

BorderNavigator::BorderNavigator() {
    // Initialize any BorderNavigator-specific members here
}

void BorderNavigator::init(Mower& mower) {
    state_ = State::FIND_BORDER;
    lastStateChange_ = millis();
    mower.stopDriveMotors();
}

void BorderNavigator::update(Mower& mower) {
    switch (state_) {
        case State::FIND_BORDER:
            // Ruota finché non trova il bordo
            mower.setLeftMotorSpeed(30.0f);
            mower.setRightMotorSpeed(-30.0f);
            
            if (mower.isBorderDetected()) {
                state_ = State::FOLLOW_BORDER;
                lastStateChange_ = millis();
            }
            break;
            
        case State::FOLLOW_BORDER:
            // Segui il bordo
            if (mower.isBorderDetected()) {
                // Se il bordo è rilevato, gira leggermente a destra
                mower.setLeftMotorSpeed(40.0f);
                mower.setRightMotorSpeed(10.0f);
            } else {
                // Altrimenti gira a sinistra per ritrovare il bordo
                mower.setLeftMotorSpeed(10.0f);
                mower.setRightMotorSpeed(40.0f);
            }
            
            // Ogni tanto fai una pausa per evitare di seguire falsi positivi
            if (millis() - lastStateChange_ > 5000) {
                state_ = State::TURN_AROUND;
                lastStateChange_ = millis();
            }
            break;
            
        case State::TURN_AROUND:
            // Ruota di 180 gradi
            mower.setLeftMotorSpeed(-40.0f);
            mower.setRightMotorSpeed(40.0f);
            
            if (millis() - lastStateChange_ > 1000) {
                state_ = State::FOLLOW_BORDER;
                lastStateChange_ = millis();
            }
            break;
    }
}

void BorderNavigator::start(Mower& mower) {
    state_ = State::FIND_BORDER;
    lastStateChange_ = millis();
    mower.stopDriveMotors();
}

void BorderNavigator::stop(Mower& mower) {
    mower.stopDriveMotors();
    state_ = State::FIND_BORDER; // Reset to initial state
}

bool BorderNavigator::handleEvent(Mower& mower, Event event) {
    switch (event) {
        case Event::BORDER_DETECTED:
            // Se stiamo cercando il bordo, passiamo alla modalità di inseguimento
            if (state_ == State::FIND_BORDER) {
                state_ = State::FOLLOW_BORDER;
                lastStateChange_ = millis();
            }
            return true;
            
        case Event::OBSTACLE_DETECTED:
            // Se rileviamo un ostacolo, giriamo
            state_ = State::TURN_AROUND;
            lastStateChange_ = millis();
            return true;
            
        default:
            return false;
    }
}
