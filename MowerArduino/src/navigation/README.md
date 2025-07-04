# Sistema di Navigazione

Questo modulo gestisce le diverse strategie di navigazione del robot tagliaerba.

## Struttura

- `NavigatorBase.h` - Interfaccia base per tutti i navigatori
- `NavigationManager.h/cpp` - Gestisce il passaggio tra le diverse modalità di navigazione
- `RandomNavigator.h/cpp` - Navigazione casuale
- `LawnMowerNavigator.h/cpp` - Navigazione a strisce (tosaerba)
- `BorderNavigator.h/cpp` - Navigazione lungo il perimetro
- `NavigationConfig.h` - Configurazioni per la navigazione

## Aggiungere una nuova strategia di navigazione

1. Crea una nuova classe che eredita da `NavigatorBase`
2. Implementa i metodi richiesti:
   - `init()` - Inizializza il navigatore
   - `update()` - Aggiorna la logica di navigazione
   - `handleEvent()` - Gestisce gli eventi
   - `getName()` - Restituisce il nome della modalità

3. Aggiungi la nuova modalità all'enum `NavigationMode` in `MowerTypes.h`
4. Aggiungi il nuovo navigatore al `NavigationManager`

## Utilizzo

```cpp
// Inizializzazione
navigationManager_.init(mower);

// Impostare la modalità di navigazione
navigationManager_.setNavigationMode(NavigationMode::RANDOM);

// Aggiornamento
navigationManager_.update(mower);

// Gestione eventi
navigationManager_.handleEvent(mower, event);
```

## Convenzioni

- Le velocità dei motori sono espresse in percentuale (-100.0 a 100.0)
- Le distanze sono espresse in centimetri
- Gli angoli sono espressi in gradi

## Debug

Per abilitare i messaggi di debug, definire `DEBUG_NAVIGATION` in `config.h`
