#include "NavigationManager.h"
#include "NavigatorBase.h"
#include "RandomNavigator.h"
#include "LawnMowerNavigator.h"
#include "BorderNavigator.h"
#include "../functions/Mower.h"
#include <Arduino.h>

NavigationManager::NavigationManager() 
    : currentNavigator_(nullptr) {
}

NavigationManager::~NavigationManager() {
    // Navigators are owned by Mower; no deletion necessary
}

void NavigationManager::init(Mower& mower) {
    // Ottieni i navigatori dal Mower (gestiti dal Mower stesso)
    randomNavigator_ = &mower.getRandomNavigator();
    lawnMowerNavigator_ = &mower.getLawnMowerNavigator();
    borderNavigator_ = &mower.getBorderNavigator();

    // Inizializza tutti i navigatori
    randomNavigator_->init(mower);
    lawnMowerNavigator_->init(mower);
    borderNavigator_->init(mower);
    
    // Imposta il navigatore di default
    currentNavigator_ = randomNavigator_;
    currentMode_ = NavigationMode::RANDOM;
}

void NavigationManager::setNavigationMode(NavigationMode mode) {
    if (currentMode_ == mode) {
        return; // Già in questa modalità
    }
    
    // Aggiorna il navigatore corrente
    switch (mode) {
        case NavigationMode::RANDOM:
            currentNavigator_ = randomNavigator_;
            break;
        case NavigationMode::LAWN_MOWER:
            currentNavigator_ = lawnMowerNavigator_;
            break;
        case NavigationMode::BORDER:
            currentNavigator_ = borderNavigator_;
            break;
        default:
            // Modalità non valida, usa quella casuale
            currentNavigator_ = randomNavigator_;
            mode = NavigationMode::RANDOM;
            break;
    }
    
    currentMode_ = mode;
}

void NavigationManager::update(Mower& mower) {
    if (currentNavigator_) {
        currentNavigator_->update(mower);
    }
}

bool NavigationManager::handleEvent(Mower& mower, Event event) {
    if (currentNavigator_) {
        return currentNavigator_->handleEvent(mower, event);
    }
    return false;
}

const char* NavigationManager::getCurrentModeName() const {
    if (currentNavigator_) {
        return currentNavigator_->getName();
    }
    return "NONE";
}

void NavigationManager::startNavigation(Mower& mower) {
    if (currentNavigator_) {
        currentNavigator_->start(mower);
    }
}

void NavigationManager::stopNavigation(Mower& mower) {
    if (currentNavigator_) {
        currentNavigator_->stop(mower);
    }
    mower.stopDriveMotors();
}

const char* NavigationManager::getCurrentNavigatorName() const {
    if (currentNavigator_ == randomNavigator_) {
        return "Random";
    } else if (currentNavigator_ == lawnMowerNavigator_) {
        return "LawnMower";
    } else if (currentNavigator_ == borderNavigator_) {
        return "Border";
    }
    return "None";
}
