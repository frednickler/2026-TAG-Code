#ifndef ACCEL_CALIBRATION_H
#define ACCEL_CALIBRATION_H

#include <Arduino.h>
#include "calibration_common.h"

/**
 * Configure the accelerometer registers for optimal calibration
 * 
 * This function configures the ICM-20948 accelerometer with settings optimized for
 * accurate calibration:
 * - ±2g full-scale range for maximum sensitivity
 * - DLPF enabled with 24Hz bandwidth to reduce noise
 * - Sample rate of ~225Hz (1125Hz/(1+4))
 */
void configureAccelForCalibration();

/**
 * Apply accelerometer calibration to raw sensor readings
 * 
 * This function applies the appropriate calibration method to the raw accelerometer readings
 * based on the currently selected method in the calibration data.
 * 
 * @param ax_mg X-axis accelerometer reading in mg (in/out parameter)
 * @param ay_mg Y-axis accelerometer reading in mg (in/out parameter)
 * @param az_mg Z-axis accelerometer reading in mg (in/out parameter)
 * @param temperature Current temperature in degrees Celsius (for temperature compensation)
 */
void applyAccelCalibration(float& ax_mg, float& ay_mg, float& az_mg, float temperature = 0.0f);

/**
 * Perform simple bias-only accelerometer calibration
 * 
 * This basic calibration method collects accelerometer data while the device is
 * stationary and calculates the bias/offset for each axis.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateAccelSimple();

/**
 * Perform 6-position accelerometer calibration
 * 
 * This method implements the 6-position (cube) calibration method for accelerometers.
 * The device is placed in 6 different orientations (each face of a cube), and
 * accelerometer data is collected in each position. The final calibration parameters
 * include both bias/offset and scale factors.
 * 
 * @param withCrossAxis If true, also calculate cross-axis sensitivity correction
 * @return true if calibration was successful, false otherwise
 */
bool calibrateAccel6Position(bool withCrossAxis);

/**
 * Perform ellipsoid fitting calibration for accelerometer
 * 
 * This advanced calibration method collects accelerometer data while the device
 * is rotated through many different orientations. The collected data points form
 * an ellipsoid, which is then fitted to a sphere to determine calibration parameters.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateAccelEllipsoid();

/**
 * Perform temperature-compensated accelerometer calibration
 * 
 * This method calibrates the accelerometer at the current temperature and calculates
 * temperature coefficients to compensate for drift across different temperatures.
 * It measures accelerometer bias at different temperatures to establish the relationship
 * between temperature and sensor drift.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateAccelWithTempComp();

/**
 * Run the accelerometer calibration procedure with user-selectable methods
 * 
 * This function presents a menu to the user to select between different
 * accelerometer calibration methods:
 * 1. Simple bias-only calibration
 * 2. 6-position calibration with scale factors
 * 3. 6-position with cross-axis sensitivity correction
 * 4. Advanced ellipsoid fitting
 * 5. Temperature-compensated calibration
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateAccelerometer();

/**
 * Validate accelerometer calibration quality
 * 
 * This function performs a quick validation of the accelerometer calibration quality
 * by collecting samples while the device is stationary and checking:
 * 1. Magnitude is close to 1g (9.81 m/s²)
 * 2. Standard deviation is within acceptable limits
 * 3. Bias stability over short time period
 * 
 * @param meanMagnitude Output parameter for mean magnitude in g
 * @param stdDevMagnitude Output parameter for standard deviation of magnitude
 * @return true if calibration quality meets acceptance criteria, false otherwise
 */
bool validateAccelCalibration(float &meanMagnitude, float &stdDevMagnitude);

/**
 * Check if accelerometer is calibrated
 * 
 * @return true if accelerometer has valid calibration data
 */
bool isAccelCalibrated();

/**
 * Enable or disable dynamic accelerometer calibration
 * 
 * Dynamic calibration continuously refines the accelerometer bias estimation
 * during normal operation.
 * 
 * @param enabled true to enable dynamic calibration, false to disable
 */
void setDynamicAccelCalibrationEnabled(bool enabled);

/**
 * Check if dynamic accelerometer calibration is enabled
 * 
 * @return true if dynamic calibration is enabled
 */
bool isDynamicAccelCalibrationEnabled();

/**
 * Update dynamic accelerometer calibration with new sensor readings
 * 
 * This function should be called periodically with new accelerometer readings
 * to refine the calibration parameters over time.
 * 
 * @param ax X-axis accelerometer reading in g
 * @param ay Y-axis accelerometer reading in g
 * @param az Z-axis accelerometer reading in g
 */
void updateDynamicAccelCalibration(float ax, float ay, float az);

/**
 * Validate dynamic accelerometer calibration
 * 
 * This function checks the dynamic calibration parameters for validity.
 * 
 * @param scaleFactorError Output parameter for scale factor error
 * @param crossAxisError Output parameter for cross-axis error
 * @return true if dynamic calibration is valid, false otherwise
 */
bool validateAccelDynamic(float scaleFactorError[3], float crossAxisError[3]);

// Sensor-specific validation helpers (used by common validation)
/**
 * Read raw accelerometer sensor data
 * 
 * This function reads the raw accelerometer data from the sensor.
 * 
 * @param x Output parameter for X-axis reading
 * @param y Output parameter for Y-axis reading
 * @param z Output parameter for Z-axis reading
 * @return true if read was successful, false otherwise
 */
bool readAccelSensorData(float &x, float &y, float &z);

/**
 * Apply accelerometer sensor calibration
 * 
 * This function applies the calibration parameters to the raw accelerometer data.
 * 
 * @param x Input/output parameter for X-axis reading
 * @param y Input/output parameter for Y-axis reading
 * @param z Input/output parameter for Z-axis reading
 */
void applyAccelSensorCalibration(float &x, float &y, float &z);

/**
 * Check if accelerometer bias is acceptable
 * 
 * This function checks if the accelerometer bias is within acceptable limits.
 * 
 * @param x X-axis reading
 * @param y Y-axis reading
 * @param z Z-axis reading
 * @return true if bias is acceptable, false otherwise
 */
bool checkAccelBiasAcceptable(float x, float y, float z);

/**
 * Check if accelerometer noise is acceptable
 * 
 * This function checks if the accelerometer noise is within acceptable limits.
 * 
 * @param x X-axis reading
 * @param y Y-axis reading
 * @param z Z-axis reading
 * @return true if noise is acceptable, false otherwise
 */
bool checkAccelNoiseAcceptable(float x, float y, float z);

/**
 * Axis identification tool
 * 
 * Interactive tool to help identify the physical orientation of the accelerometer axes.
 * Displays real-time accelerometer readings and indicates which axis is dominant.
 */
void identifyAccelAxes();

#endif // ACCEL_CALIBRATION_H

