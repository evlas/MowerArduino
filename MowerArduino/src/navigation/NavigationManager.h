#ifndef NAVIGATION_MANAGER_H
#define NAVIGATION_MANAGER_H

#include "../functions/MowerTypes.h"

// Forward declarations
class Mower;
class NavigatorBase;
class RandomNavigator;
class LawnMowerNavigator;
class BorderNavigator;

/**
 * @class NavigationManager
 * @brief Gestisce il passaggio tra le diverse modalità di navigazione
 */
class NavigationManager {
public:
    NavigationManager();
    ~NavigationManager();
    
    /**
     * @brief Inizializza il gestore di navigazione
     */
    void init(Mower& mower);
    
    /**
     * @brief Imposta la modalità di navigazione
     * @param mode Modalità di navigazione da attivare
     */
    void setNavigationMode(NavigationMode mode);
    
    /**
     * @brief Aggiorna la navigazione
     */
    void update(Mower& mower);
    
    /**
     * @brief Gestisce un evento
     * @param event Evento da gestire
     * @return true se l'evento è stato gestito, false altrimenti
     */
    bool handleEvent(Mower& mower, Event event);
    
    /**
     * @brief Ottiene la modalità di navigazione corrente
     * @return Modalità di navigazione corrente
     */
    NavigationMode getCurrentMode() const { return currentMode_; }
    
    /**
     * @brief Ottiene il nome della modalità di navigazione corrente
     * @return Nome della modalità
     */
    const char* getCurrentModeName() const;
    
    /**
     * @brief Avvia la navigazione con la modalità corrente
     * @param mower Riferimento all'oggetto Mower
     */
    void startNavigation(Mower& mower);
    
    /**
     * @brief Ferma la navigazione
     * @param mower Riferimento all'oggetto Mower
     */
    void stopNavigation(Mower& mower);
    
    /**
     * @brief Check if navigation is currently active
     * @return true if navigation is active, false otherwise
     */
    bool isNavigationActive() const { return currentNavigator_ != nullptr; }
    
    /**
     * @brief Get the current navigator instance
     * @return Pointer to the current navigator, or nullptr if none active
     */
    NavigatorBase* getCurrentNavigator() const { return currentNavigator_; }
    
    /**
     * @brief Get the name of the current navigator
     * @return Name of the current navigator, or "None" if no navigator is active
     */
    const char* getCurrentNavigatorName() const;

private:
    NavigationMode currentMode_ = NavigationMode::RANDOM;
    NavigatorBase* currentNavigator_ = nullptr;
    
    // Puntatori ai navigatori
    RandomNavigator* randomNavigator_ = nullptr;
    LawnMowerNavigator* lawnMowerNavigator_ = nullptr;
    BorderNavigator* borderNavigator_ = nullptr;
};

#endif // NAVIGATION_MANAGER_H
