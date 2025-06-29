/**
 * @file BatterySensor.h
 * @brief Battery monitoring interface for the autonomous mower.
 * 
 * This file contains the declaration of the BatterySensor class, which provides
 * an interface to monitor the mower's battery status using the INA226 power
 * monitoring IC. It allows reading voltage, current, power, and calculating
 * battery percentage.
 */

#ifndef BATTERY_SENSOR_H
#define BATTERY_SENSOR_H

#include <Wire.h>
#include <INA226_WE.h>

/**
 * @class BatterySensor
 * @brief Provides battery monitoring functionality using the INA226 power monitor.
 * 
 * This class interfaces with the INA226 power monitoring IC to provide
 * real-time battery information including voltage, current, power, and
 * state of charge. It handles the low-level I2C communication and provides
 * high-level battery status information.
 */
class BatterySensor {
public:
    // ===== Initialization =====
    
    /**
     * @brief Initialize the battery sensor
     * @return true if initialization was successful, false otherwise
     * 
     * This method initializes the I2C communication with the INA226 sensor
     * and configures it for battery monitoring. It should be called once
     * during system startup.
     */
    bool begin();
    
    // ===== Direct Sensor Readings =====
    
    /**
     * @brief Read the battery voltage
     * @return Battery voltage in volts (V)
     */
    float readVoltage();
    
    /**
     * @brief Read the battery current
     * @return Current in amperes (A), positive for discharge, negative for charge
     */
    float readCurrent();
    
    /**
     * @brief Read the battery power
     * @return Power in watts (W)
     */
    float readPower();
    
    // ===== Sensor Management =====
    
    /**
     * @brief Update the sensor readings
     * 
     * This method is called periodically to update the sensor readings.
     * The INA226 updates automatically, but this method allows for
     * implementing additional filtering or processing if needed.
     */
    void update() {
        // The INA226 sensor updates automatically
        // Additional filtering logic can be added here if needed
    }
    
    // ===== Status Methods =====
    
    /**
     * @brief Check if the sensor is connected and responding
     * @return true if the sensor is connected, false otherwise
     */
    bool isConnected() const { return connected; }
    
    // ===== Battery Information =====
    
    /**
     * @brief Get the current battery voltage
     * @return Battery voltage in volts (V)
     */
    float getVoltage() { return readVoltage(); }
    
    /**
     * @brief Get the current battery current
     * @return Current in amperes (A), positive for discharge, negative for charge
     */
    float getCurrent() { return readCurrent(); }
    
    /**
     * @brief Get the current battery power
     * @return Power in watts (W)
     */
    float getPower() { return readPower(); }
    
    /**
     * @brief Calculate the estimated battery percentage
     * @return Battery percentage (0-100%)
     * 
     * This method estimates the battery percentage based on voltage.
     * The calculation assumes a Li-ion battery with a nominal voltage
     * range of 3.0V (0%) to 4.2V (100%).
     */
    float getBatteryPercentage();
    
    /**
     * @brief Check if the battery is currently charging
     * @return true if charging, false otherwise
     */
    bool isCharging() { return connected && (readCurrent() < -0.1f); }  // Negative current = charging
    
    /**
     * @brief Check if the battery is currently discharging
     * @return true if discharging, false otherwise
     */
    bool isDischarging() { return connected && (readCurrent() > 0.1f); } // Positive current = discharging
    
    /**
     * @brief Check if the battery is fully charged
     * @param threshold Voltage threshold for considering the battery full (default: 4.2V)
     * @return true if the battery voltage is at or above the threshold, false otherwise
     */
    bool isBatteryFull(float threshold = 4.2f) { return connected && (readVoltage() >= threshold); }

private:
    INA226_WE ina226;       ///< INA226 power monitor instance
    bool connected = false; ///< Connection status of the sensor
};

/**
 * @brief Global BatterySensor instance
 * 
 * This is the main instance of the BatterySensor that should be used
 * throughout the application to access battery information.
 */
extern BatterySensor batterySensor;

#endif // BATTERY_SENSOR_H
