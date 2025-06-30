/**
 * @file BatterySensor.cpp
 * @brief Implementation of the BatterySensor class for monitoring battery status.
 * 
 * This file contains the implementation of the BatterySensor class methods that
 * interface with the INA226 power monitoring IC to provide battery information.
 */

#include "BatterySensor.h"
#include "../../config.h"
#include <INA226_WE.h>
#include <math.h>

/**
 * @brief Global BatterySensor instance
 * 
 * This is the main instance of the BatterySensor that is used throughout
 * the application to access battery information.
 */
BatterySensor batterySensor;

BatterySensor::BatterySensor() : 
    connected(false),
    voltageFiltered(0.0f),
    currentFiltered(0.0f),
    lastUpdate(0) {
    // Initialize with default values
}

bool BatterySensor::begin() {
    // Initialize the INA226 sensor
    connected = ina226.init();
    if (!connected) {
        DEBUG_PRINTLN(F("BatterySensor: INA226 init failed! Check wiring (SDA/SCL) and I2C address."));
        return false;
    } else {
        DEBUG_PRINTLN(F("BatterySensor: INA226 init successful."));

    // Configure INA226 with appropriate settings for battery monitoring
/*    ina226.setResistorRange(0.1, 10.0);  // 100mΩ shunt resistor, 10A max current
    ina226.setMeasureMode(CONTINUOUS);   // Continuous measurement mode
    ina226.setAverage(AVERAGE_16);       // Average over 16 samples for noise reduction
    ina226.setConversionTime(CONV_TIME_1100);  // 1.1ms conversion time (max speed)
    ina226.setCorrectionFactor(1.0);    // No additional correction needed
*/  
        return true;
    }

 
}

float BatterySensor::readVoltage() {
//    voltageFiltered = ina226.getBusVoltage_V();
    voltageFiltered = BATTERY_VOLTAGE_MAX;
    return voltageFiltered;
}

float BatterySensor::readCurrent() {    
//    currentFiltered = ina226.getCurrent_mA();
    currentFiltered = 1000.0;
    return currentFiltered;
}

float BatterySensor::readPower() {
    // Calculate power in watts: P = V * I
    // Convert current from mA to A by dividing by 1000
    return voltageFiltered * (currentFiltered / 1000.0f);
}

float BatterySensor::getBatteryPercentage() {
    float percentage = 0.0f;
    
    // Simple linear estimation (you might want to implement a more accurate curve)
    percentage = ((voltageFiltered - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN)) * 100.0f;
    
    percentage = 100.0f;

    // Clamp to 0-100%
    return constrain(percentage, 0.0f, 100.0f);
}

BatteryState BatterySensor::getBatteryState() {
/*    if (voltageFiltered <= BATTERY_VOLTAGE_CRITICAL) {
        return BATTERY_STATE_CRITICAL;
    } else if (voltageFiltered <= BATTERY_VOLTAGE_MIN) {
        return BATTERY_STATE_EMPTY;
    } else if (voltageFiltered >= BATTERY_VOLTAGE_MAX * 0.98f) {
        return BATTERY_STATE_FULL;
    } else if (currentFiltered < -0.1f) {  // Negative current means charging
        return BATTERY_STATE_CHARGING;
    } else if (currentFiltered > 0.1f) {   // Positive current means discharging
        return BATTERY_STATE_DISCHARGING;
    } else {
        return BATTERY_STATE_IDLE;
    }
*/
    return BATTERY_STATE_DISCHARGING;
}

// These methods are now defined inline in the header file
// bool BatterySensor::isCharging() {
//     return getBatteryState() == BATTERY_STATE_CHARGING;
// }

// bool BatterySensor::isCritical() {
//     return getBatteryState() == BATTERY_STATE_CRITICAL;
// }

// bool BatterySensor::isFullyCharged() {
//     return getBatteryState() == BATTERY_STATE_FULL;
// }

// bool BatterySensor::isConnected() {
//     return connected && (millis() - lastUpdate < BATTERY_UPDATE_INTERVAL);  // Consider disconnected if no updates for 5s
// }
