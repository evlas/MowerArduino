#ifndef HOME_POSITION_MANAGER_H
#define HOME_POSITION_MANAGER_H

#include "EEPROMManager.h"
#include <Arduino.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_TWOPI
#define M_TWOPI (2.0 * M_PI)
#endif

class HomePositionManager {
public:
    // Imposta la posizione home corrente
    static bool setHomePosition(float x, float y, float theta) {
        HomePosition homePos;
        homePos.x = x;
        homePos.y = y;
        homePos.theta = theta;
        homePos.isSet = true;
        
        EEPROMManager::saveHomePosition(homePos);
        return true;
    }
    
    // Ottiene la posizione home salvata
    static bool getHomePosition(float& x, float& y, float& theta) {
        HomePosition homePos;
        if (EEPROMManager::loadHomePosition(homePos) && homePos.isSet) {
            x = homePos.x;
            y = homePos.y;
            theta = homePos.theta;
            return true;
        }
        return false;
    }
    
    // Verifica se la posizione home è impostata
    static bool isHomePositionSet() {
        return EEPROMManager::isHomePositionSet();
    }
    
    // Cancella la posizione home
    static void clearHomePosition() {
        EEPROMManager::clearHomePosition();
    }
    
    // Calcola la distanza dalla posizione corrente alla home (in metri)
    // Restituisce -1 se la posizione home non è impostata
    static float calculateDistanceToHome(float currentX, float currentY) {
        float homeX, homeY, homeTheta;
        if (getHomePosition(homeX, homeY, homeTheta)) {
            float dx = homeX - currentX;
            float dy = homeY - currentY;
            return sqrtf(dx*dx + dy*dy);
        }
        return -1.0f;  // Posizione home non impostata
    }
    
    // Calcola l'angolo verso la home (in radianti, tra -PI e PI)
    // Restituisce 0 se la posizione home non è impostata
    static float calculateAngleToHome(float currentX, float currentY, float currentTheta) {
        float homeX, homeY, homeTheta;
        if (getHomePosition(homeX, homeY, homeTheta)) {
            float dx = homeX - currentX;
            float dy = homeY - currentY;
            float targetAngle = atan2f(dy, dx);
            
            // Normalizza l'angolo tra -PI e PI
            float angleDiff = targetAngle - currentTheta;
            while (angleDiff > M_PI) angleDiff -= M_TWOPI;
            while (angleDiff < -M_PI) angleDiff += M_TWOPI;
            
            return angleDiff;
        }
        return 0.0f;  // Posizione home non impostata
    }
    
    // Verifica se la posizione corrente è vicina alla home (entro una certa tolleranza)
    static bool isAtHome(float currentX, float currentY, float toleranceMeters = 0.3f) {
        float distance = calculateDistanceToHome(currentX, currentY);
        return (distance >= 0 && distance <= toleranceMeters);
    }
    
    // Verifica se il robot è orientato verso la home (entro una certa tolleranza)
    static bool isFacingHome(float currentX, float currentY, float currentTheta, float toleranceRadians = 0.26f) { // ~15 gradi
        float angleDiff = calculateAngleToHome(currentX, currentY, currentTheta);
        return (fabsf(angleDiff) <= toleranceRadians);
    }
};

#endif // HOME_POSITION_MANAGER_H
