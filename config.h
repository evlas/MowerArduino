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

// SERIAL PORTS
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
//#define ENABLE_IMU              // IMU per orientamento
#define ENABLE_RTC              // RTC per orologio
//#define ENABLE_BATTERY_MONITOR  // Monitor tensione/corrente batteria
//#define ENABLE_GPS              // Sensore GPS (commenta se non presente)
#define SERIAL_GPS_BAUD 115200
//#define ENABLE_PERIMETER        // Sensore filo perimetrale
#define ENABLE_ULTRASONIC         // Sensore ultrasonico per rilevamento ostacoli
#define ENABLE_BUMP_SENSORS       // Sensori di urto meccanici
//#define ENABLE_RAIN_SENSOR      // Sensore pioggia

// --- MOTORI ---
#define ENABLE_DRIVE_MOTORS
#define DRIVE_MOTOR_TIPO1  // Motori Brushless 5 fili
//#define DRIVE_MOTOR_TIPO2      // Motori Brushed

#define ENABLE_BLADE_MOTORS
#define BLADE_MOTOR_TIPO1    // Motori Brushless 5 fili
//#define BLADE_MOTOR_TIPO2  // Motori Brushed 

// --- ATTUATORI ---
#define ENABLE_BUZZER   // Buzzer per segnalazioni acustiche
//#define ENABLE_DISPLAY  // Display LCD/OLED (commenta se non presente)
#define ENABLE_RELAY    // Controller relay aggiuntivi

// --- FUNZIONALITÀ ---
//#define ENABLE_WIFI        // Comunicazione WiFi (commenta se non presente)
#define SERIAL_WIFI_BAUD 115200
//#define ENABLE_NAVIGATION  // Sistema di navigazione avanzato
//#define ENABLE_SAFETY      // Sistema di sicurezza
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

// Velocità massima dei motori
#define MAX_SPEED 255
#define MIN_SPEED 0
#define DEFAULT_SPEED 200

// ========================
// CONFIGURAZIONI TIMING
// ========================
#define MAIN_LOOP_DELAY 50              // Delay loop principale (ms)
#define SENSOR_UPDATE_INTERVAL 100      // Intervallo aggiornamento sensori (ms)
#define NAVIGATION_UPDATE_INTERVAL 200  // Intervallo navigazione (ms)
#define SAFETY_CHECK_INTERVAL 50        // Intervallo controlli sicurezza (ms)

// --- SENSORI DIGITALI ---
#ifdef ENABLE_RAIN_SENSOR
#define RAIN_THRESHOLD 500        // Valore soglia per rilevamento pioggia (0-1023)
#define RAIN_CHECK_INTERVAL 1000  // ms (1 secondo)
#endif

// --- SENSORI ANALOGICI ---


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
#ifdef ENABLE_IMU
#define IMU_I2C_ADDRESS 0x68
#define IMU_GYRO_RANGE 2000  // ±2000 dps
#define IMU_ACCEL_RANGE 8    // ±8g
#endif


// ========================
// PARAMETRI OPERATIVI
// ========================

// --- SOGLIE SENSORI ---
#ifdef ENABLE_ULTRASONIC
#define OBSTACLE_DISTANCE_THRESHOLD 30  // cm
#define MAX_ULTRASONIC_DISTANCE 200     // cm
#endif

#ifdef ENABLE_BATTERY_MONITOR
#define ADDR_INA226 0x40

#define BATTERY_SHUNT_RESISTOR 0.1  // 0.1 Ohm
#define BATTERY_MAX_VOLTAGE 7*BATTERY_FULL_VOLTAGE   // V
#define BATTERY_MAX_CURRENT 3.2     // A

#define LOW_BATTERY_THRESHOLD 7*BATTERY_LOW_THRESHOLD   // V
#define FULL_BATTERY_VOLTAGE 7*BATTERY_FULL_VOLTAGE    // V
#define BATTERY_VOLTAGE_DIVIDER 5.0  // Rapporto partitore tensione

// Battery parameters
#define BATTERY_FULL_VOLTAGE 4.2     // V
#define BATTERY_EMPTY_VOLTAGE 3.2    // V
#define BATTERY_LOW_THRESHOLD 3.5    // V
#define BATTERY_CHECK_INTERVAL 1000  // ms
#endif

#ifdef ENABLE_RAIN_SENSOR
#define RAIN_THRESHOLD 300  // Valore ADC soglia pioggia
#endif

// --- TIMING MOVIMENTI ---
#define TURN_DURATION 1000        // ms per girata 90°
#define BACKUP_DURATION 800       // ms per retromarcia
#define PAUSE_AFTER_OBSTACLE 500  // ms pausa dopo ostacolo

// ========================
// CONFIGURAZIONI AVANZATE
// ========================

// --- NAVIGAZIONE ---
#ifdef ENABLE_NAVIGATION
#define RANDOM_WALK_ENABLED         // Abilita camminata casuale
#define SYSTEMATIC_PATTERN_ENABLED  // Abilita pattern sistematico
#define EDGE_FOLLOWING_ENABLED      // Abilita seguimento bordi
#endif

// --- SICUREZZA ---
#ifdef ENABLE_SAFETY
#define EMERGENCY_STOP_ENABLED   // Arresto di emergenza
#define TILT_PROTECTION_ENABLED  // Protezione inclinazione
#define LIFT_PROTECTION_ENABLED  // Protezione sollevamento

#define TEMPERATURE_MONITORING  // Monitoraggio temperatura
#endif

// --- SCHEDULER ---
#ifdef ENABLE_SCHEDULE
#define MAX_SCHEDULE_SLOTS 7         // Numero massimo slot orari
#define DEFAULT_MOWING_DURATION 120  // Durata taglio default (minuti)
#endif

// --- COMUNICAZIONE ---
#ifdef ENABLE_WIFI
#define WIFI_TIMEOUT 10000  // Timeout connessione (ms)
#endif

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

// Verifica che almeno un sensore di ostacoli sia abilitato
#if !defined(ENABLE_ULTRASONIC) && !defined(ENABLE_BUMP_SENSORS)
#warning "Attenzione: Nessun sensore di ostacoli abilitato!"
#endif

// Verifica che i motori di trazione siano abilitati
#ifndef ENABLE_DRIVE_MOTORS
#error "Errore: I motori di trazione devono essere abilitati!"
#endif


#endif  // CONFIG_H
