# Test Interattivi per Sensori e Attuatori

Questa cartella contiene test interattivi per verificare il funzionamento dei sensori e degli attuatori del robot tagliaerba.

## Come utilizzare i test

1. Apri il test desiderato nell'IDE Arduino
2. Caricalo sulla scheda
3. Apri il Monitor Seriale (115200 baud)
4. Segui le istruzioni a schermo per interagire con il test

## Elenco dei test disponibili

### Sensori

1. **RainSensorTest**
   - Verifica il funzionamento del sensore di pioggia
   - Comandi:
     - `r`: Leggi lo stato del sensore
     - `h`: Mostra l'aiuto

2. **BumpSensorsTest**
   - Verifica i sensori di urto
   - Comandi:
     - `r`: Leggi tutti i sensori di urto
     - `h`: Mostra l'aiuto

3. **UltrasonicSensorsTest**
   - Verifica i sensori ad ultrasuoni
   - Comandi:
     - `r`: Leggi tutti i sensori ad ultrasuoni
     - `h`: Mostra l'aiuto

### Attuatori

1. **BuzzerTest**
   - Verifica il funzionamento del buzzer
   - Comandi:
     - `1`: Beep singolo
     - `2`: Doppio beep
     - `3`: Triplo beep
     - `4`: Sirena (3 cicli)
     - `t`: Attiva/disattiva tono continuo
     - `s`: Spegni il buzzer
     - `h`: Mostra l'aiuto

2. **RelayTest**
   - Verifica il funzionamento dei relay
   - Comandi:
     - `m`: Attiva/disattiva il relay motori
     - `b`: Attiva/disattiva il relay lama
     - `a`: Attiva/disattiva il relay accessori
     - `1`: Accendi TUTTI i relay
     - `0`: Spegni TUTTI i relay
     - `s`: Mostra lo stato attuale
     - `h`: Mostra l'aiuto

3. **BladeMotorTest**
   - Verifica il motore del tagliaerba
   - Comandi:
     - `s`: Ferma il motore
     - `1-4`: Imposta la velocità (25-100%)
     - `f`: Inverte la direzione
     - `h`: Mostra l'aiuto

4. **DriveMotorTest**
   - Verifica i motori di trazione
   - Comandi:
     - `s`: Ferma i motori
     - `f`: Avanti
     - `b`: Indietro
     - `l`: Gira a sinistra
     - `r`: Gira a destra
     - `1-4`: Imposta la velocità (25-100%)
     - `e`: Mostra i valori degli encoder
     - `h`: Mostra l'aiuto

## Requisiti

- IDE Arduino
- Tutte le librerie richieste dal progetto principale
- Connessione seriale a 115200 baud

## Note

- Assicurati che i pin definiti nei test corrispondano alla tua configurazione hardware
- Usa questi test per verificare il corretto funzionamento di ogni componente prima di utilizzarli nel codice principale
