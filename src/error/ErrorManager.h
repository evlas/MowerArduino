#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <Arduino.h>

// Dimensione massima degli errori gestiti
#define MAX_ERRORS 10

// Codici di errore
enum class ErrorCode {
    NO_ERROR = 0,
    BATTERY_LOW,
    BATTERY_CRITICAL,
    BATTERY_ERROR,
    GPS_ERROR,
    MOTOR_ERROR,
    SENSOR_ERROR,
    WIFI_ERROR,
    UNKNOWN_ERROR
};

// Struttura per memorizzare informazioni su un errore
struct ErrorInfo {
    ErrorCode code;
    String message;
    unsigned long timestamp;
    bool isCritical;
};

class ErrorManager {
public:
    // Inizializza il gestore degli errori
    static void begin();
    
    // Aggiungi un errore
    static void addError(ErrorCode code, const String& message = "", bool isCritical = false);
    
    // Rimuovi un errore specifico
    static bool removeError(ErrorCode code);
    
    // Verifica se esiste un errore specifico
    static bool hasError(ErrorCode code);
    
    // Verifica se ci sono errori critici
    static bool hasCriticalError();
    
    // Pulisci tutti gli errori
    static void clearAllErrors();
    
    // Ottieni il numero di errori attivi
    static uint8_t getErrorCount();
    
    // Ottieni un errore dall'indice
    static ErrorInfo getError(uint8_t index);
    
    // Ottieni la descrizione testuale di un codice di errore
    static String getErrorString(ErrorCode code);
    
    // Pulisci gli errori e restituisci true se tutti gli errori sono stati risolti
    static bool clearErrors();
    
private:
    static ErrorInfo _activeErrors[MAX_ERRORS];
    static uint8_t _errorCount;
    static bool _initialized;
    
    // Inizializza le descrizioni degli errori
    static void _initializeErrorDescriptions();
    
    // Struttura per le descrizioni degli errori
    struct ErrorDescription {
        ErrorCode code;
        const char* description;
    };
    static const ErrorDescription _errorDescriptions[9]; // Numero di errori definiti
    static const uint8_t ERROR_DESCRIPTIONS_COUNT;
};

#endif // ERROR_MANAGER_H
