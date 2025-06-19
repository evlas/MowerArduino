# 🤖 MowerArduino - Robot Tagliaerba Autonomo

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Arduino](https://img.shields.io/badge/Arduino%20Mega%202560-Compatible-00979D?logo=arduino)](https://store.arduino.cc/products/arduino-mega-2560-rev3)
[![Version](https://img.shields.io/badge/Version-0.0.1-blue)](https://github.com/evlas/MowerArduino)

Un sistema avanzato per robot tagliaerba autonomo basato su **Arduino Mega 2560**, progettato per offrire un taglio preciso e intelligente del prato con il massimo dell'autonomia e della sicurezza.

## 🌟 Caratteristiche Principali

- **Navigazione Intelligente** con sensori avanzati
- **Taglio Efficace** con gestione ottimizzata delle lame
- **Sicurezza Avanzata** con rilevamento ostacoli e protezioni
- **Controllo Remoto** via WiFi
- **Gestione Batteria** con ricarica automatica
- **Architettura Modulare** per facile personalizzazione

## 🏗 Architettura del Sistema

```mermaid
graph TD
    %% Firmware
    E[Firmware]
    E1[Inizializzazione]
    E2[Loop Principale]
    E3[Macchina a Stati]
    F1[IDLE]
    F2[MOWING]
    F3[DOCKING]
    F4[CHARGING]
    F5[ERROR]
    
    %% Collegamenti
    A --> B
    A --> C
    A --> D
    
    B --> B1 & B2 & B3 & B4 & B5 & B6 & B7
    C --> C1 & C2 & C3 & C4 & C5
    D --> D1 & D2
    
    E --> E1 & E2 & E3
    E3 --> F1 & F2 & F3 & F4 & F5
```

## 🚀 Funzionalità

### 🧭 Navigazione
- Mappatura dell'area di lavoro
- Evitamento ostacoli
- Percorsi efficienti
- Ritorno automatico alla base

### ⚡ Energia
- Monitoraggio batteria
- Ricarica automatica
- Gestione del consumo

### 🔒 Sicurezza
- Rilevamento ostacoli
- Protezione lama
- Arresto di emergenza
- Controlli integrità

## 🛠 Installazione

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
   - **Hardware richiesto**:
     - Scheda Arduino Mega 2560
     - Modulo WiFi ESP8266 per la connettività
     - Modulo GPS NEO-6M
     - Sensore IMU (es. MPU6050)
     - Modulo INA226 per il monitoraggio della batteria
     - Display LCD I2C 16x2
     - Motori brushless con controller a bordo a 5 fili (es: 42GP-4260)

2. **Configurazione**
   - Clonare la repository
   - Aprire il file `MowerArduino.ino` con Arduino IDE
   - Installare le librerie richieste tramite il Gestore Librerie
   - Configurare i parametri in `config.h` e `pin_config.h`
   - Selezionare la scheda "Arduino Mega or Mega 2560"
   - Selezionare la porta corretta
   - Compilare e caricare il firmware

3. **Configurazione WiFi**
   - Il modulo ESP8266 deve essere configurato
   - La comunicazione avviene tramite Serial2 (pin 16 RX2, 17 TX2)

## 📊 Stato del Progetto

| Categoria | Stato |
|-----------|-------|
| Hardware | ✅ Completato |
| Firmware Base | ✅ Completato |
| Navigazione | 🟡 In Sviluppo |
| Interfaccia Web | 🟡 In Sviluppo |
| Documentazione | 🟡 In Sviluppo |

## 📋 Roadmap

### 🎯 Prossime Versioni
- [x] Sistema base di navigazione
- [ ] Integrazione sensori avanzati
- [ ] Interfaccia web completa
- [ ] Supporto multi-lingua

### 🔮 Futuro
- Integrazione con sistemi domotici
- App mobile dedicata
- Machine learning per percorsi ottimizzati

## 🤝 Contributi

I contributi sono benvenuti! Per favore leggi le [linee guida per i contributi](CONTRIBUTING.md) prima di inviare una pull request.

## 📄 Licenza

Questo progetto è rilasciato sotto licenza MIT. Vedi il file `LICENSE` per i dettagli.

## 🙋 Supporto

Per domande o supporto, apri una [issue](https://github.com/tu-utente/MowerArduino/issues) sulla repository.

---

<div align="center">
  Creato con ❤️ per gli amanti del giardino perfetto
</div>
