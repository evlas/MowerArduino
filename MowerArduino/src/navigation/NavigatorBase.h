#ifndef NAVIGATOR_BASE_H
#define NAVIGATOR_BASE_H

// Forward declaration of Mower class to avoid circular dependency
class Mower;

// Include Event enum
#include "../functions/MowerTypes.h"

/**
 * @class NavigatorBase
 * @brief Interfaccia base per tutte le strategie di navigazione
 */
class NavigatorBase {
public:
    virtual ~NavigatorBase() = default;
    
    /**
     * @brief Inizializza il navigatore
     * @param mower Riferimento all'oggetto Mower
     */
    virtual void init(Mower& mower) = 0;
    
    /**
     * @brief Aggiorna la navigazione
     * @param mower Riferimento all'oggetto Mower
     */
    virtual void update(Mower& mower) = 0;
    
    /**
     * @brief Gestisce un evento
     * @param mower Riferimento all'oggetto Mower
     * @param event Evento da gestire
     * @return true se l'evento è stato gestito, false altrimenti
     */
    virtual bool handleEvent(Mower& mower, Event event) = 0;
    
    /**
     * @brief Ottiene il nome della modalità di navigazione
     * @return Nome della modalità
     */
    virtual const char* getName() const = 0;
    
    /**
     * @brief Avvia la navigazione
     * @param mower Riferimento all'oggetto Mower
     */
    virtual void start(Mower& mower) = 0;
    
    /**
     * @brief Ferma la navigazione
     * @param mower Riferimento all'oggetto Mower
     */
    virtual void stop(Mower& mower) = 0;
};

#endif // NAVIGATOR_BASE_H
