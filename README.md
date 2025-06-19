# 🤖 MowerArduino - Robot Tagliaerba Autonomo

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Arduino](https://img.shields.io/badge/Arduino%20Mega%202560-Compatible-00979D?logo=arduino)](https://store.arduino.cc/products/arduino-mega-2560-rev3)
[![Version](https://img.shields.io/badge/Version-0.1.0-blue)](https://github.com/evlas/MowerArduino)

Un sistema avanzato per robot tagliaerba autonomo basato su **Arduino Mega 2560**, progettato per offrire un taglio preciso e intelligente del prato con il massimo dell'autonomia e della sicurezza.

## 🌟 Novità (v0.1.0)

- **Gestione del Relay** per il controllo dell'alimentazione dei motori
- **Documentazione Doxygen** completa per le classi principali
- **Miglioramento della StateMachine** per una gestione più robusta degli stati
- **Refactoring del codice** per ridurre l'accoppiamento tra i componenti

## 🏗 Architettura del Sistema

```mermaid
graph TD
    %% Nodo principale
    A[Arduino Mega 2560]
    
    %% Moduli principali
    B[Moduli]
    B1[State Machine]
    B2[Gestione Motori]
    B3[Controllo Lame]
    B4[Gestione Batteria]
    B5[Comunicazione WiFi]
    
    %% Sensori
    C[Sensori]
    C1[IMU]
    C2[GPS NEO-6M]
    C3[Ultrasuoni]
    C4[Urti]
    C5[Perimetrali]
    C6[Pioggia]
    C7[Batteria INA226]
    
    %% Attuatori
    D[Attuatori]
    D1[Motori di Traslazione]
    D2[Motore Lama]
    D3[LCD 16x2 I2C]
    D4[Buzzer]
    D5[Relè di Potenza]
    
    %% Collegamenti
    A --> B
    B --> B1 & B2 & B3 & B4 & B5
    A --> C & D
    C --> C1 & C2 & C3 & C4 & C5 & C6 & C7
    D --> D1 & D2 & D3 & D4 & D5
```

## 🚀 Funzionalità Principali

### 🧭 Navigazione e Controllo
- **Macchina a Stati** per la gestione delle operazioni
- **Controllo dei Motori** con supporto per motori brushless
- **Gestione del Relay** per la sicurezza dell'alimentazione
- **Navigazione Intelligente** con evitamento ostacoli

### ⚡ Gestione Energia
- Monitoraggio batteria con INA226
- **Gestione del Relay** per il risparmio energetico
- Spegnimento automatico in caso di bassa tensione

### 🔒 Sicurezza
- **Protezione da sovracorrente**
- **Arresto di emergenza**
- **Controlli di sicurezza** integrati
- **Gestione del Relay** per isolare i componenti quando non in uso

## 🛠 Struttura del Codice

```
src/
├── LCD/               # Gestione display LCD
├── actuators/         # Attuatori (relay, buzzer, ecc.)
├── battery/          # Gestione batteria e alimentazione
├── communications/    # Comunicazione WiFi e seriale
├── error/            # Gestione errori
├── functions/        # Funzionalità principali
│   └── StateMachine.cpp/h  # Macchina a stati principale
├── handler/          # Gestori di sistema
├── motors/           # Controllo motori e lame
├── position/         # Gestione posizione
├── safety/           # Funzioni di sicurezza
└── sensors/          # Gestione sensori
```

## 🔧 Installazione e Configurazione

1. **Prerequisiti**
   - Arduino IDE 1.8.x o successiva
   - **Librerie richieste**:
     ```bash
     Wire.h (inclusa nell'IDE Arduino)
     LiquidCrystal_I2C.h
     INA226_WE.h
     ArduinoJson.h
     DS1302.h
     ```

2. **Configurazione**
   - Clonare la repository
   - Aprire `MowerArduino.ino` con Arduino IDE
   - Installare le librerie richieste
   - Configurare `config.h` e `pin_config.h`
   - Selezionare "Arduino Mega or Mega 2560"
   - Compilare e caricare il firmware

## 📊 Stato del Progetto

| Componente           | Stato       | Note                                      |
|----------------------|-------------|------------------------------------------|
| Hardware            | ✅ Completato |                                          |
| Core Firmware       | ✅ Completato | Gestione base del sistema                |
| State Machine       | ✅ Completato | Gestione stati e transizioni             |
| Gestione Relay      | ✅ Completato | Controllo alimentazione motori           |
| Controllo Motori    | 🟡 In Sviluppo |                                          |
| Navigazione         | 🟡 In Sviluppo |                                          |
| Documentazione      | 🟡 In Corso  | In corso di completamento                 |


## 📋 Roadmap

### 🚩 Prossimi Passi
- [ ] Completamento documentazione Doxygen
- [ ] Test approfonditi del sistema di gestione relay
- [ ] Implementazione algoritmi di navigazione
- [ ] Interfaccia utente avanzata

### 🔮 Futuro
- Integrazione con sistemi domotici
- App mobile dedicata
- Ottimizzazione percorsi di taglio

## 🤝 Contributi

I contributi sono benvenuti! Per favore leggi le [linee guida per i contributi](CONTRIBUTING.md) prima di inviare una pull request.

## 📄 Licenza

Questo progetto è rilasciato sotto licenza MIT. Vedi il file `LICENSE` per i dettagli.

---

<div align="center">
  Creato con ❤️ per gli amanti del giardino perfetto
</div>
