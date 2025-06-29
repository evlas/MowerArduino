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

/**
 * @brief Global BatterySensor instance
 * 
 * This is the main instance of the BatterySensor that is used throughout
 * the application to access battery information.
 */
BatterySensor batterySensor;

bool BatterySensor::begin() {
    // Initialize the INA226 sensor
    connected = ina226.init();
    if (!connected) {
        #ifdef DEBUG_MODE
        SERIAL_DEBUG.println(F("BatterySensor: INA226 init failed!"));
        #endif
        return false;
    }

    // Configure INA226 with appropriate settings for battery monitoring
    ina226.setResistorRange(0.1, 10.0);  // 100mΩ shunt resistor, 10A max current
    ina226.setMeasureMode(CONTINUOUS);   // Continuous measurement mode
    ina226.setAverage(AVERAGE_16);       // Average over 16 samples for noise reduction
    ina226.setConversionTime(CONV_TIME_1100);  // 1.1ms conversion time (max speed)
    ina226.setCorrectionFactor(1.0);    // No additional correction needed

    #ifdef DEBUG_MODE
    SERIAL_DEBUG.println(F("BatterySensor: INA226 initialized"));
    #endif
    
    return true;
}

float BatterySensor::readVoltage() {
    // Return 0.0 if not connected, otherwise read bus voltage
    return connected ? ina226.getBusVoltage_V() : 0.0f;
}

float BatterySensor::readCurrent() {
    // Return 0.0 if not connected, otherwise read current in mA
    // Note: Negative values indicate charging, positive values indicate discharging
    return connected ? ina226.getCurrent_mA() : 0.0f;
}

float BatterySensor::readPower() {
    // Calculate power in watts: P = V * I
    // Convert current from mA to A by dividing by 1000
    return connected ? ina226.getBusVoltage_V() * ina226.getCurrent_mA() / 1000.0f : 0.0f;
}

float BatterySensor::getBatteryPercentage() {
    // Return 0% if sensor is not connected
    if (!connected) return 0.0f;
    
    // Read current battery voltage
    float voltage = readVoltage();
    
    // Battery voltage thresholds (adjust based on your specific battery chemistry)
    const float minVoltage = 3.0f;  // Minimum safe voltage (0% charge)
    const float maxVoltage = 4.2f;  // Maximum voltage (100% charge)
    
    // Safety checks to avoid division by zero or out-of-bounds values
    if (voltage <= minVoltage) return 0.0f;    // At or below minimum voltage
    if (voltage >= maxVoltage) return 100.0f;  // At or above maximum voltage
    
    // Linear interpolation between min and max voltage to estimate percentage
    // This is a simple estimation - real battery discharge curves are not linear
    return (voltage - minVoltage) / (maxVoltage - minVoltage) * 100.0f;
}
