#ifndef NAVIGATION_CONFIG_H
#define NAVIGATION_CONFIG_H

// Configurazioni per la navigazione
namespace NavigationConfig {
    // Velocità di navigazione standard (in percentuale: -100.0 a 100.0)
    constexpr float DEFAULT_NAVIGATION_SPEED = 50.0f;
    
    // Velocità di rotazione (in percentuale: -100.0 a 100.0)
    constexpr float ROTATION_SPEED = 30.0f;
    
    // Soglia per la rilevazione del bordo (in cm)
    constexpr float BORDER_DETECTION_THRESHOLD = 30.0f;
    
    // Tempo massimo per una singola fase di navigazione (in ms)
    constexpr unsigned long PHASE_TIMEOUT = 60000;
    
    // Distanza minima di sicurezza dagli ostacoli (in cm)
    constexpr float MIN_OBSTACLE_DISTANCE = 20.0f;
}

#endif // NAVIGATION_CONFIG_H
