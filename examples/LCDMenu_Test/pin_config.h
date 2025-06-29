#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <Arduino.h>

/*
 * pin_config.h - Definizione e descrizione dei PIN utilizzati dal robot tagliaerba
 * 
 * In questo file sono raccolte tutte le macro di configurazione dei PIN hardware.
 * Modifica questi valori in base al cablaggio del tuo progetto.
 *
 * SEZIONI:
 * - LED di stato
 * - Motori di trazione
 * - Motore lama
 * - Sensori (ultrasuoni, bump, pioggia, filo perimetrale, RTC)
 * - Attuatori (buzzer, relay)
 * - Altri PIN
 */

// ========================
// LED di stato
// ========================
#define pinLED LED_BUILTIN  // LED integrato sulla scheda

// ========================
// COMUNICAZIONE I2C
// ========================
#define I2C_SDA 20
#define I2C_SCL 21

// ========================
// MOTORI DI TRAZIONE
// ========================
#define MOTOR_LEFT_PWM_PIN 5    // PWM sinistro (PWM pin)
#define MOTOR_LEFT_DIR_PIN 4    // Direzione sinistro
#define MOTOR_LEFT_PPM_PIN 3    // PPM sinistro (opzionale)

#define MOTOR_RIGHT_PWM_PIN 7   // PWM destro (PWM pin)
#define MOTOR_RIGHT_DIR_PIN 6   // Direzione destro
#define MOTOR_RIGHT_PPM_PIN 2   // PPM destro (opzionale)

// ========================
// MOTORE LAMA
// ========================
#define MOTOR_BLADE_PWM_PIN 8      // PWM primo motore lama principale
//#define MOTOR_BLADE_PWM_PIN 11      // PWM secondo motore lama principale
//#define MOTOR_BLADE_PWM_PIN 10    // PWM motore lama principale
#define MOTOR_BLADE_DIR_PIN 9      // Direzione motore lama principale
// ========================
// SENSORI
// ========================
// Ultrasuoni (abilitare con ENABLE_ULTRASONIC)
// Front
#define ULTRASONIC_FRONT_ECHO_PIN 34    // Echo centrale
#define ULTRASONIC_FRONT_TRIG_PIN 35    // Trigger centrale
// Left
#define ULTRASONIC_LEFT_ECHO_PIN 36     // Echo sinistra
#define ULTRASONIC_LEFT_TRIG_PIN 37     // Trigger sinistra
// Right
#define ULTRASONIC_RIGHT_ECHO_PIN 38    // Echo destra
#define ULTRASONIC_RIGHT_TRIG_PIN 39    // Trigger destra

// Alias per compatibilità con il codice esistente
#define ULTRASONIC_ECHO_PIN ULTRASONIC_FRONT_ECHO_PIN
#define ULTRASONIC_TRIGGER_PIN ULTRASONIC_FRONT_TRIG_PIN
#define ULTRASONIC_MAX_DISTANCE 400     // Massima distanza in cm

// Enum per identificare i sensori a ultrasuoni
typedef enum {
    ULTRASONIC_FRONT = 0,
    ULTRASONIC_LEFT,
    ULTRASONIC_RIGHT,
    ULTRASONIC_COUNT
} UltrasonicSensor;

// Sensori di urto (abilitare con ENABLE_BUMP_SENSORS)
#define FRONT_RIGHT_BUMP_PIN 46      // Bump anteriore destro
#define FRONT_LEFT_BUMP_PIN 47       // Bump anteriore sinistro
#define REAR_RIGHT_BUMP_PIN 44       // Bump posteriore destro
#define REAR_LEFT_BUMP_PIN 45        // Bump posteriore sinistro

// Alias per compatibilità con il codice esistente
#define BUMP_FRONT_PIN FRONT_LEFT_BUMP_PIN
#define BUMP_REAR_PIN REAR_LEFT_BUMP_PIN
#define BUMP_LEFT_PIN FRONT_LEFT_BUMP_PIN
#define BUMP_RIGHT_PIN FRONT_RIGHT_BUMP_PIN

// Sensore filo perimetrale (abilitare con ENABLE_PERIMETER_WIRE)
#define RIGHT_PERIMETER_PIN A4     // Filo perimetrale destro
#define LEFT_PERIMETER_PIN A5      // Filo perimetrale sinistro

// Sensore pioggia (abilitare con ENABLE_RAIN_SENSOR)
#define RAIN_SENSOR_PIN A6           // Pin analogico per sensore pioggia


#define EMERG_STOP_BUTTON_PIN 10  // Pulsante Stop

// IMU (MPU6050/MPU9250)
#define IMU_SDA_PIN 20              // SDA per IMU (condiviso con I2C)
#define IMU_SCL_PIN 21              // SCL per IMU (condiviso con I2C)
#define IMU_INTERRUPT_PIN 30        // Interrupt pin per IMU (cambia se necessario)

// RTC (abilitare con ENABLE_RTC)
#define RTC_SCLK_PIN 31              // Clock RTC
#define RTC_IO_PIN 30                // Data RTC
#define RTC_RST_PIN 29               // Reset RTC

// Configurazione encoder motori
#define MOTOR_LEFT_ENCODER_PIN 2      // Pin encoder motore sinistro
#define MOTOR_RIGHT_ENCODER_PIN 3     // Pin encoder motore destro

// Direzione motori (true = invertito, false = normale)
#define MOTOR_LEFT_REVERSED false
#define MOTOR_RIGHT_REVERSED true

// Sensore sollevamento (abilitare con ENABLE_SAFETY)
#define LIFT_PIN 12                  // Sensore sollevamento

// ========================
// ATTUATORI
// ========================
#define BUZZER_PIN 48                // Buzzer per segnalazioni acustiche
#define RELAY_MOTORS_PIN 24          // Relay motori
#define RELAY_CHARGING_PIN 25        // Relay di ricarica

// ========================
// PULSANTI DI CONTROLLO
// ========================
#define START_BUTTON_PIN 50  // Pulsante Start
#define PLUS_BUTTON_PIN 51   // Pulsante +
#define MINUS_BUTTON_PIN 52  // Pulsante -
#define STOP_BUTTON_PIN 53  // Pulsante Stop

// ========================
// AGGIUNGI ALTRI PIN QUI SE NECESSARIO
// ========================

#endif // PIN_CONFIG_H
