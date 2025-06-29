#include <Arduino.h>
#include "Maneuver.h"
#include "PositionManager.h"
#include "IMUModule.h"
#include "GPSModule.h"
#include "config.h"
#include "pin_config.h"

// Dichiarazioni delle istanze globali
extern PositionManager positionManager;

// Crea le istanze dei motori
DriveMotor leftMotor(
    MOTOR_LEFT_PWM_PIN, 
    MOTOR_LEFT_DIR_PIN, 
    MOTOR_LEFT_ENCODER_PIN,
    WHEEL_DIAMETER,
    ENCODER_PULSES_PER_REV,
    true  // isLeft = true per il motore sinistro
);

DriveMotor rightMotor(
    MOTOR_RIGHT_PWM_PIN, 
    MOTOR_RIGHT_DIR_PIN, 
    MOTOR_RIGHT_ENCODER_PIN,
    WHEEL_DIAMETER,
    ENCODER_PULSES_PER_REV,
    false  // isLeft = false per il motore destro
);

// Crea l'istanza del gestore di manovre
Maneuver mowerManeuver(&leftMotor, &rightMotor, &positionManager, BLADE_WIDTH, WHEEL_BASE, MAX_LINEAR_SPEED);  // bladeWidth=0.44m, wheelBase=0.55m, maxLinearSpeed=1.0m/s

// Variabili per i test
unsigned long testStartTime = 0;
int currentTest = 0;
const int TEST_DURATION = 3000; // 3 secondi per test
bool firstLoop = true;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // Attendi la connessione seriale
    }

    Serial.println("=== Test di Manovra del Robot ===");
    Serial.print("Diametro ruota: ");
    Serial.print(WHEEL_DIAMETER * 100); // Converti in cm
    Serial.println(" cm");
    Serial.print("Impulsi encoder/giro: ");
    Serial.println(ENCODER_PULSES_PER_REV);
    
    // Inizializza i motori
    leftMotor.begin();
    rightMotor.begin();
    
    // I parametri di accelerazione sono già impostati nei valori di default della classe MotorBase
    // MAX_LINEAR_ACCEL e MAX_LINEAR_DECEL sono usati direttamente
    
    Serial.println("Motori inizializzati");
    delay(1000);
    
    testStartTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - testStartTime;
    
    // Esegui il test corrente
    switch (currentTest) {
        case 0: // Test 1: Avanti
            if (elapsedTime == 0) {
                Serial.println("\nTest 1: Avanti");
                mowerManeuver.forward(TEST_DURATION, 50.0f); // Avanti al 50% di potenza
            }
            if (elapsedTime >= TEST_DURATION) {
                mowerManeuver.stop();
                currentTest++;
                testStartTime = currentTime;
            }
            break;
            
        case 1: // Test 2: Indietro
            if (elapsedTime == 0) {
                Serial.println("\nTest 2: Indietro");
                mowerManeuver.backward(TEST_DURATION, 30.0f); // Indietro al 30% di potenza
            }
            if (elapsedTime >= TEST_DURATION) {
                mowerManeuver.stop();
                currentTest++;
                testStartTime = currentTime;
            }
            break;
            
        case 2: // Test 3: Rotazione a sinistra
            if (elapsedTime == 0) {
                Serial.println("\nTest 3: Rotazione a sinistra");
                mowerManeuver.turnLeft(90, 40.0f); // Rotazione a sinistra di 90° al 40%
            }
            if (elapsedTime >= TEST_DURATION) {
                mowerManeuver.stop();
                currentTest++;
                testStartTime = currentTime;
            }
            break;
            
        case 3: // Test 4: Rotazione a destra
            if (elapsedTime == 0) {
                Serial.println("\nTest 4: Rotazione a destra");
                mowerManeuver.rotateRight(30.0f); // Rotazione sul posto a destra al 30%
            }
            if (elapsedTime >= TEST_DURATION) {
                mowerManeuver.stop();
                currentTest++;
                testStartTime = currentTime;
            }
            break;
            
        case 4: // Test 5: Curva a sinistra
            if (elapsedTime == 0) {
                Serial.println("\nTest 5: Curva a sinistra");
                // Ruota a sinistra con velocità media di 50% (30 e 70 sono le velocità dei singoli motori)
                mowerManeuver.rotateLeft(30.0f); // Rotazione sul posto a sinistra al 30% di velocità
            }
            if (elapsedTime >= TEST_DURATION) {
                mowerManeuver.stop();
                currentTest++;
                testStartTime = currentTime;
            }
            break;
            
        case 5: // Test completato
            if (firstLoop) {
                firstLoop = false;
                Serial.println("\nTutti i test sono stati completati con successo!");
                Serial.println("Starting movement sequence...");
                
                // Avvia il taglio con la larghezza corretta della lama (44cm)
                // La distanza tra le linee verrà calcolata automaticamente
                mowerManeuver.startMowing(BLADE_WIDTH, 50.0f); // Inizia il taglio con la larghezza della lama specificata il movimento a zigzag
            }
            
            // Aggiorna il movimento a zigzag
            mowerManeuver.updateZigzag();
            
            // Stampa lo stato ogni 100ms
            static unsigned long lastPrintTime = 0;
            if (millis() - lastPrintTime > 100) {
                lastPrintTime = millis();
                
                // Stampa la velocità dei motori
                Serial.print("Left: ");
                Serial.print(leftMotor.getSpeed());
                Serial.print("%\tRight: ");
                Serial.print(rightMotor.getSpeed());
                Serial.println("%");
            }
            break;
    }
    
    // Aggiorna lo stato dei motori
    leftMotor.update();
    rightMotor.update();
    
    // Piccolo ritardo per non sovraccaricare la CPU
    delay(10);
}