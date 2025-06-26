/*
 * config.hpp - Configurazione Globale Robot Tagliaerba
 *
 * Questo file contiene tutte le configurazioni del sistema.
 * Modifica le impostazioni qui per abilitare/disabilitare componenti
 * e personalizzare il comportamento del robot.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ========================
// INFORMAZIONI FIRMWARE
// ========================
#define FIRMWARE_VERSION "0.0.1"
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

#include "pin_config.h"

// ========================
// PARAMETRI ROBOT
// ========================

// --- DIMENSIONI FISICHE ---
#define ROBOT_LENGTH 0.50f           // [m] Lunghezza totale del robot
#define ROBOT_WIDTH 0.40f            // [m] Larghezza totale del robot
#define ROBOT_HEIGHT 0.25f           // [m] Altezza totale del robot
#define CLEARANCE 0.05f              // [m] Altezza minima da terra del telaio

// --- CONFIGURAZIONE RUOTE ---
#define WHEEL_DIAMETER 0.30f         // [m] Diametro delle ruote motrici
#define WHEEL_BASE 0.55f             // [m] Distanza tra le ruote motrici (asse sinistro-destro)
#define WHEEL_WIDTH 0.08f            // [m] Larghezza delle ruote
#define WHEEL_FRICTION 0.8f          // [-] Coefficiente di attrito (0.0-1.0)

// --- CONFIGURAZIONE LAMA ---
#define BLADE_WIDTH 0.44f            // [m] Larghezza di taglio effettiva
#define BLADE_OFFSET_X 0.20f          // [m] Offset orizzontale dal centro (positivo = avanti)
#define BLADE_OFFSET_Y 0.0f          // [m] Offset laterale dal centro (positivo = destra)
#define BLADE_HEIGHT 0.03f           // [m] Altezza di taglio da terra

// --- PESI E BILANCIAMENTO ---
#define ROBOT_WEIGHT 35.0f           // [kg] Peso totale a vuoto
#define BATTERY_WEIGHT 2.5f          // [kg] Peso della batteria
#define MAX_SLOPE_ANGLE 25.0f        // [°] Massima pendenza superabile

// --- DIMENSIONI AREA DI LAVORO ---
#define LAWN_SIZE_X 20.0f            // [m] Lunghezza massima dell'area di lavoro
#define LAWN_SIZE_Y 15.0f            // [m] Larghezza massima dell'area di lavoro
#define DOCKING_STATION_X 0.0f       // [m] Posizione X della stazione di ricarica
#define DOCKING_STATION_Y 0.0f       // [m] Posizione Y della stazione di ricarica

// --- CENTRI DI ROTAZIONE ---
#define TURN_RADIUS 0.20f            // [m] Raggio di sterzata minimo
#define PIVOT_OFFSET 0.0f            // [m] Offset del punto di perno (positivo = avanti)

// --- POSIZIONE SENSORI (rispetto al centro del robot) ---
#define ULTRASONIC_FRONT_L 0.25f     // [m] Posizione L sensore anteriore
#define ULTRASONIC_FRONT_C 0.0f      // [m] Posizione C sensore anteriore
#define ULTRASONIC_FRONT_R 0.25f     // [m] Posizione R sensore anteriore
#define BUMPER_FRONT_L 0.25f    // [m] Sporgenza anteriore dei bumper
#define BUMPER_FRONT_R 0.25f    // [m] Sporgenza anteriore dei bumper
#define LIFT_SENSOR_OFFSET 0.0f      // [m] Offset sensore sollevamento

// ========================
// TIMING E INTERVALLI
// ========================
#define MAIN_LOOP_DELAY 50              // [ms] Delay loop principale
#define SENSOR_UPDATE_INTERVAL 50       // [ms] Intervallo aggiornamento sensori (20Hz)
#define NAVIGATION_UPDATE_INTERVAL 100  // [ms] Intervallo aggiornamento navigazione (10Hz)
#define SAFETY_CHECK_INTERVAL 50        // [ms] Intervallo controlli sicurezza (20Hz)
#define ODOMETRY_UPDATE_INTERVAL 10     // [ms] Intervallo aggiornamento odometria (100Hz)
#define TELEMETRY_UPDATE_INTERVAL 1000  // [ms] Intervallo aggiornamento telemetria (1Hz)
#define BATTERY_UPDATE_INTERVAL 5000    // [ms] Intervallo controllo batteria (0.2Hz)

// ========================
// SICUREZZA
// ========================
// Configurazione pulsanti
#define BUTTON_DEBOUNCE 50  // [ms] Tempo di debounce per tutti i pulsanti

// Soglie di sicurezza
#define TILT_THRESHOLD 30.0f        // [°] Soglia di inclinazione massima
#define LIFT_THRESHOLD 0.1f         // [g] Soglia di sollevamento (accelerazione verticale minima)

// Distanze di sicurezza
#define SAFETY_MARGIN 0.10f             // [m] Margine di sicurezza intorno al robot
#define OBSTACLE_CLEARANCE (SAFETY_MARGIN * 2.0f)  // [m] Distanza minima da ostacoli
#define PERIMETER_SAFETY_DISTANCE (SAFETY_MARGIN * 3.0f) // [m] Distanza di sicurezza dal perimetro
#define CLIFF_DETECTION_HEIGHT 0.05f    // [m] Altezza rilevamento dislivelli

// Timeout
#define EMERGENCY_STOP_TIMEOUT 1000           // [ms] Timeout arresto di emergenza
#define MOTOR_COMMAND_TIMEOUT (MAIN_LOOP_DELAY * 10)  // [ms] Timeout comandi motore (10 cicli)

// ========================
// MOTORI
// ========================
// Valori PWM assoluti ottenibili (timer 20kHz con ICR=799)
// Questi valori sono usati come riferimento per la conversione percentuale (-100%% .. 100%%)
#define MAX_MOTOR_SPEED 799      // 100%
#define MIN_MOTOR_SPEED -799     // -100%
#define DEFAULT_MOTOR_SPEED 750  // ~94% di MAX_MOTOR_SPEED
#define TURN_MOTOR_SPEED 300     // ~38% di MAX_MOTOR_SPEED
#define NEUTRAL_MOTOR_SPEED 0    // 0%

// ========================
// NAVIGAZIONE
// ========================
// Prestazioni
#define MAX_LINEAR_SPEED 1.1f           // [m/s] Velocità massima in avanti/indietro
#define MAX_ANGULAR_SPEED 1.0f          // [rad/s] Velocità angolare massima
#define MAX_LINEAR_ACCEL 0.5f           // [m/s²] Accelerazione lineare massima
#define MAX_LINEAR_DECEL 0.5f           // [m/s²] Decelerazione lineare massima
#define MAX_ANGULAR_ACCEL 1.0f          // [rad/s²] Accelerazione angolare massima

// Tolleranze
#define POSITION_TOLERANCE (SAFETY_MARGIN * 0.5f)  // [m] Tolleranza raggiungimento posizione (50% del margine di sicurezza)
#define HEADING_TOLERANCE_DEG 5.0f                 // [°] Tolleranza angolare in gradi

// Navigazione
#define OBSTACLE_DISTANCE_THRESHOLD 0.30f          // [m] Soglia di distanza ostacoli
#define OBSTACLE_AVOIDANCE_DELAY 2000              // [ms] Ritardo evitamento ostacoli
#define ROTATION_ANGLE_DEG 45.0f                   // [°] Angolo di rotazione in gradi
#define PARALLEL_DISTANCE 0.5f                     // [m] Distanza tra linee parallele
#define SPIRAL_STEP 0.1f                           // [m] Passo della spirale
#define MAX_SPEED 0.5f                             // [0-1] Velocità massima motori
#define ROTATION_SPEED 0.3f                        // [0-1] Velocità di rotazione
#define HEADING_TOLERANCE (HEADING_TOLERANCE_DEG * DEG_TO_RAD) // [rad] Tolleranza angolo (convertita in radianti)
#define PARALLEL_LINE_SPACING BLADE_WIDTH  // [m] Spaziatura linee navigazione parallela (uguale alla larghezza di taglio)

// Manovre
#define MIN_TURN_RADIUS (WHEEL_BASE * 0.55f)  // [m] Raggio di sterzata minimo (55% della base)
#define SPOT_TURN_RADIUS (WHEEL_BASE * 0.18f) // [m] Raggio per rotazione su se stanti
#define APPROACH_DISTANCE (ROBOT_LENGTH * 0.3f) // [m] Distanza di avvicinamento target (30% lunghezza robot)
#define REVERSE_DISTANCE (ROBOT_LENGTH * 0.4f)  // [m] Distanza retromarcia in caso di ostacolo (40% lunghezza robot)

// Mappatura
#define MAP_CELL_SIZE (BLADE_WIDTH * 0.45f)  // [m] Dimensione cella mappa (45% larghezza lama)
#define MAX_WORK_AREA 10000.0f               // [m²] Area massima di lavoro
#define MAP_WIDTH (int)(sqrt(MAX_WORK_AREA) / MAP_CELL_SIZE)  // [celle] Larghezza mappa
#define MAP_HEIGHT MAP_WIDTH                // [celle] Altezza mappa (quadrata)
#define MAP_UPDATE_RATE 1.0f            // [Hz] Frequenza aggiornamento mappa

// ========================
// SERIAL PORTS
// ========================
#define SERIAL_DEBUG Serial  // USB Serial for debugging
#define SERIAL_GPS Serial1   // Hardware Serial1 for GPS
#define SERIAL_WIFI Serial2  // Hardware Serial2 for ESP8266
#define SERIAL_ROS Serial3   // Hardware Serial3 available for future use

// ========================
// CONFIGURAZIONI DEBUG
// ========================
#define DEBUG_MODE  // Commenta per disabilitare debug
#define SERIAL_DEBUG_BAUD 115200

// ========================
// ABILITAZIONE COMPONENTI
// ========================

// --- SENSORI ---
#define ENABLE_GPS              // Sensore GPS (commenta se non presente)
#define SERIAL_GPS_BAUD 115200
//#define ENABLE_PERIMETER        // Sensore filo perimetrale
#define PERIMETER_SIGNAL_THRESHOLD 100  // Soglia di rilevamento del segnale del perimetro

// --- MOTORI ---
#define DRIVE_MOTOR_TIPO1  // Motori Brushless 5 fili
//#define DRIVE_MOTOR_TIPO2      // Motori Brushed
// Velocità massima dei motori
// Motor speed range is now 0-799 (for 20kHz PWM with ICR = 799)
#define MAX_MOTOR_SPEED 799
#define MIN_MOTOR_SPEED -799
#define DEFAULT_MOTOR_SPEED 750  // ~94% of max speed
#define TURN_MOTOR_SPEED 300     // ~37.5% of max speed for turns

#define ENABLE_BLADE_MOTORS
#define BLADE_MOTORS_NUM      2        //Numero di motori per le lame di taglio
#define BLADE_MOTOR_TIPO1    // Motori Brushless 5 fili
//#define BLADE_MOTOR_TIPO2  // Motori Brushed 
#define DEFAULT_BLADE_SPEED 200  // Velocità predefinita della lama (0-255)

// --- ATTUATORI ---
#define ENABLE_BUZZER   // Buzzer per segnalazioni acustiche
#define ENABLE_DISPLAY  // Display LCD/OLED (commenta se non presente)

// --- FUNZIONALITÀ ---
#define ENABLE_WIFI        // Comunicazione WiFi (commenta se non presente)
#define SERIAL_WIFI_BAUD 115200
//#define ENABLE_CHARGING    // Sistema di ricarica automatica
//#define ENABLE_SCHEDULE    // Scheduler programmazione orari

// Scheduler parameters
#define MAX_TIME_SLOTS 2
#define TIME_SLOT_FORMAT "HH:MM"
#define DEFAULT_START_TIME_1 "07:00"  // Default start time for first slot
#define DEFAULT_END_TIME_1 "10:00"    // Default end time for first slot
#define DEFAULT_START_TIME_2 "14:00"  // Default start time for second slot
#define DEFAULT_END_TIME_2 "17:00"    // Default end time for second slot

// Charging station parameters
#define CHARGING_STATION_SEARCH_RADIUS 1000     // mm
#define CHARGING_STATION_APPROACH_SPEED 50      // mm/s
#define CHARGING_STATION_ALIGN_TOLERANCE 10     // mm
#define CHARGING_STATION_CONNECT_TIMEOUT 30000  // ms

// --- SENSORI ANALOGICI ---
#define RAIN_THRESHOLD 500        // Valore soglia per rilevamento pioggia (0-1023)
#define RAIN_CHECK_INTERVAL 1000  // ms (1 secondo)

// LCD Configuration
#define LCD_I2C_ADDRESS 0x27
#define LCD_EN 2
#define LCD_RW 1
#define LCD_RS 0
#define LCD_D4 4
#define LCD_D5 5
#define LCD_D6 6
#define LCD_D7 7
#define LCD_BK 3

// IMU MPU6050
#define IMU_I2C_ADDRESS 0x68
#define IMU_GYRO_RANGE 2000  // ±2000 dps
#define IMU_ACCEL_RANGE 8    // ±8g

// ========================
// PARAMETRI OPERATIVI
// ========================

// --- SOGLIE SENSORI ---
#define MAX_ULTRASONIC_DISTANCE 200     // cm

// Configurazione INA226
#define BATTERY_MONITOR_ADDRESS 0x40
#define BATTERY_CHECK_INTERVAL 1000   // Intervallo di controllo batteria (ms)
#define BATTERY_SHUNT_RESISTOR  0.1f   // Ohm
#define BATTERY_SENSE_RESISTOR  BATTERY_SHUNT_RESISTOR  // Alias per compatibilità
#define BATTERY_MAX_CURRENT     10.0f  // A massimi misurati
#define BATTERY_VOLTAGE_DIVIDER 1.0f   // Rapporto partitore tensione (1:1 se non usato)
#define BATTERY_VOLTAGE_DIVIDER_RATIO BATTERY_VOLTAGE_DIVIDER  // Alias per compatibilità

// Configurazione batteria LiPo
#define BATTERY_CELLS          6
#define BATTERY_CAPACITY       5.0f  // Ampere-ora

// Parametri di tensione (per cella)
#define BATTERY_FULL_VOLTAGE    4.2f   // V per cella a piena carica
#define BATTERY_EMPTY_VOLTAGE   3.0f   // V per cella scarica
#define BATTERY_LOW_THRESHOLD   3.3f   // V per cella (soglia di avviso batteria bassa)
#define BATTERY_CRITICAL_THRESHOLD 3.2f  // V per cella (soglia di spegnimento)

// Parametri calcolati
#define BATTERY_VOLTAGE_MAX     (BATTERY_CELLS * BATTERY_FULL_VOLTAGE)      // 25.2V per 6S
#define BATTERY_VOLTAGE_MIN     (BATTERY_CELLS * BATTERY_EMPTY_VOLTAGE)     // 18.0V per 6S
#define BATTERY_VOLTAGE_CRITICAL (BATTERY_CELLS * BATTERY_CRITICAL_VOLTAGE) // 19.2V per 6S
#define BATTERY_LOW_PERCENT     20.0f  // 20% di carica
#define BATTERY_LOW_LEVEL       (BATTERY_VOLTAGE_MIN + (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) * (BATTERY_LOW_PERCENT / 100.0f))  // Soglia di batteria bassa (20%)

// --- TIMING MOVIMENTI ---
#define TURN_DURATION 1000        // ms per girata 90°
#define BACKUP_DURATION 800       // ms per retromarcia
#define PAUSE_AFTER_OBSTACLE 500  // ms pausa dopo ostacolo

// --- PARAMETRI DI NAVIGAZIONE ---
#define NAVIGATION_MODE_RANDOM      0
#define NAVIGATION_MODE_PARALLEL    1
#define NAVIGATION_MODE_SPIRAL      2

// Parametri per la navigazione casuale
#define RANDOM_TURN_MIN_ANGLE   10    // Minimo angolo di virata (10 gradi)
#define RANDOM_TURN_MAX_ANGLE   170   // Massimo angolo di virata (170 gradi)
#define RANDOM_MAX_DISTANCE     300   // Massima distanza in cm prima di virare

// Parametri per la navigazione a linee parallele
#define PARALLEL_TURN_ANGLE     90         // Angolo di virata per linee parallele

// Parametri per la spirale
#define SPIRAL_MAX_RINGS        8         // Numero massimo di anelli
#define SPIRAL_RADIUS_STEP      30        // Incremento del raggio in cm
#define SPIRAL_START_RADIUS     30        // Raggio iniziale in cm

// Parametri di sicurezza
#define MIN_DISTANCE_FROM_OBSTACLE   20  // Minima distanza dagli ostacoli in cm

// ========================
// CONFIGURAZIONE ODOMETRIA
// ========================
// Configurazione encoder
#define ENCODER_PULSES_PER_REV 12      // Impulsi per giro dell'encoder
#define MOTOR_GEAR_RATIO 185.0f        // Rapporto di riduzione del motore
  
// Calcolo impulsi per metro
// (impulsi/giro) * (giri motore/giro ruota) / (circonferenza ruota in metri)
// Es: (12 * 185) / (π * 0.15) ≈ 4712 impulsi/metro
#define PULSES_PER_METER ((ENCODER_PULSES_PER_REV * MOTOR_GEAR_RATIO) / (PI * WHEEL_DIAMETER))
   
// Fattore di correzione odometria (1.0 = nessuna correzione)
#define ODOMETRY_CORRECTION_LEFT 1.0f
#define ODOMETRY_CORRECTION_RIGHT 1.0f
    
// Timeout rilevamento fermo (ms)
#define ODOMETRY_STOP_TIMEOUT 1000

// ========================
// CONFIGURAZIONI AVANZATE
// ========================

// --- NAVIGAZIONE ---
#define RANDOM_WALK_ENABLED         // Abilita camminata casuale
#define SYSTEMATIC_PATTERN_ENABLED  // Abilita pattern sistematico
#define EDGE_FOLLOWING_ENABLED      // Abilita seguimento bordi

// --- SICUREZZA ---
#define EMERGENCY_STOP_ENABLED   // Arresto di emergenza
#define TILT_PROTECTION_ENABLED  // Protezione inclinazione
#define LIFT_PROTECTION_ENABLED  // Protezione sollevamento (usa LIFT_PIN per rilevamento contatto)
#define TEMPERATURE_MONITORING  // Monitoraggio temperatura

// Le soglie di sicurezza sono definite nella sezione SICUREZZA all'inizio del file

// --- SCHEDULER ---
#define MAX_SCHEDULE_SLOTS 7         // Numero massimo slot orari
#define DEFAULT_MOWING_DURATION 120  // Durata taglio default (minuti)

// --- COMUNICAZIONE ---
#define WIFI_TIMEOUT 10000  // Timeout connessione (ms)

// --- GPS CONFIGURATION ---
// Definisce la porta seriale da utilizzare per il GPS
// Su ESP32, Serial1 è tipicamente usato per i dispositivi esterni
#ifndef SERIAL_GPS
  #define SERIAL_GPS Serial1
  #define GPS_BAUDRATE 9600
  #define GPS_CONFIG SERIAL_8N1  // 8 bit, no parity, 1 stop bit
#endif
#define TELEMETRY_INTERVAL_MS 1000  // Intervallo minimo tra due invii di telemetria (ms)

// ========================
// COSTANTI SISTEMA
// ========================

// Codici errore
#define ERROR_NONE 0
#define ERROR_OBSTACLE_STUCK 1
#define ERROR_BATTERY_LOW 2
#define ERROR_MOTOR_FAULT 3
#define ERROR_SENSOR_FAULT 4
#define ERROR_COMMUNICATION_FAULT 5
#define ERROR_TEMPERATURE_HIGH 6
#define ERROR_TILT_DETECTED 7
#define ERROR_LIFT_DETECTED 8

// Comandi Bluetooth/WiFi
#define CMD_START "START"
#define CMD_STOP "STOP"
#define CMD_HOME "HOME"
#define CMD_MANUAL "MANUAL"
#define CMD_AUTO "AUTO"
#define CMD_STATUS "STATUS"
#define CMD_RESET "RESET"

// ========================
// VALIDAZIONE CONFIGURAZIONE
// ========================

#endif  // CONFIG_H
