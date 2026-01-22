#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <Arduino.h>
#include "calibration_common.h"

// Sensor type definitions for temperature compensation
#define SENSOR_TYPE_ACCEL 0
#define SENSOR_TYPE_GYRO  1
#define SENSOR_TYPE_MAG   2

/**
 * Configure the temperature sensor registers for optimal reading
 * 
 * This function configures the ICM-20948 temperature sensor with settings optimized for
 * accurate temperature readings.
 */
void configureTempSensor();

/**
 * Read the raw temperature value from the ICM-20948 sensor
 * 
 * @return Raw temperature register value
 */
int16_t readRawTemperature();

/**
 * Convert raw temperature reading to degrees Celsius
 * 
 * @param rawTemp Raw temperature register value
 * @return Temperature in degrees Celsius
 */
float convertRawToCelsius(int16_t rawTemp);

/**
 * Read the current temperature in degrees Celsius
 * 
 * @param temp Reference to store the temperature value
 * @return True if temperature reading was successful, false otherwise
 */
bool readTemperature(float& temp);

/**
 * Calibrate the temperature sensor
 * 
 * This function calibrates the temperature sensor by comparing readings to a known reference
 * temperature. This improves the accuracy of temperature compensation for other sensors.
 * 
 * @param referenceTemp Optional reference temperature in Celsius (if known)
 * @return true if calibration was successful, false otherwise
 */
bool calibrateTemperatureSensor(float referenceTemp = NAN);

/**
 * Apply temperature compensation to sensor readings
 * 
 * This function applies temperature compensation to the specified sensor value
 * based on the current temperature and calibration parameters.
 * 
 * @param sensorValue The sensor value to compensate
 * @param sensorType The type of sensor (SENSOR_TYPE_ACCEL, SENSOR_TYPE_GYRO, SENSOR_TYPE_MAG)
 * @param axis The axis (0=X, 1=Y, 2=Z)
 * @param currentTemp The current temperature in Celsius
 * @return Temperature-compensated sensor value
 */
float applyTemperatureCompensation(float sensorValue, uint8_t sensorType, uint8_t axis, float currentTemp);

/**
 * Quick repeatability check run at boot.
 * Takes ~50 samples over 1 s and returns mean and σ.
 * Pass criterion: σ < 0.1 °C.
 * @return true if repeatability is good.
 */
bool validateTemperatureRepeatability(float &mean, float &sigma);

#endif // TEMP_SENSOR_H
