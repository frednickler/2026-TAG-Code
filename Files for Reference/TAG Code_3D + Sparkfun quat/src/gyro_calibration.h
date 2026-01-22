#ifndef GYRO_CALIBRATION_H
#define GYRO_CALIBRATION_H

#include <Arduino.h>
#include "calibration_common.h"

/**
 * Configure the gyroscope registers for optimal calibration
 * 
 * This function configures the ICM-20948 gyroscope with settings optimized for
 * accurate calibration:
 * - ±250 dps full-scale range for maximum sensitivity
 * - DLPF enabled with appropriate bandwidth to reduce noise
 * - Sample rate optimized for calibration
 */
void configureGyroForCalibration();

/**
 * Apply gyroscope calibration to raw sensor readings
 * 
 * This function applies the calibration parameters to the raw gyroscope readings,
 * including temperature compensation if enabled.
 * 
 * @param gx_dps X-axis gyroscope reading in degrees per second (in/out parameter)
 * @param gy_dps Y-axis gyroscope reading in degrees per second (in/out parameter)
 * @param gz_dps Z-axis gyroscope reading in degrees per second (in/out parameter)
 * @param temperature Current temperature in degrees Celsius (for temperature compensation)
 */
void applyGyroCalibration(float& gx_dps, float& gy_dps, float& gz_dps, float temperature);

/**
 * Perform simple gyroscope calibration (bias-only)
 * 
 * This basic calibration method collects gyroscope data while the device is
 * stationary and calculates the bias/offset for each axis.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateGyroSimple();

/**
 * Perform temperature-compensated gyroscope calibration
 * 
 * This method calibrates the gyroscope at the current temperature and calculates
 * temperature coefficients to compensate for drift across different temperatures.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateGyroWithTemp();

/**
 * Perform 6-position gyroscope calibration
 * 
 * This method implements the 6-position (cube) calibration method for gyroscopes.
 * The device is placed in 6 different orientations (each face of a cube), and
 * gyroscope data is collected in each position. This helps account for gravity-induced errors.
 * 
 * @param withTempComp If true, also perform temperature compensation
 * @return true if calibration was successful, false otherwise
 */
bool calibrateGyro6Position(bool withTempComp = false);

/**
 * Run the gyroscope calibration procedure with user-selectable methods
 * 
 * This function presents a menu to the user to select between different
 * gyroscope calibration methods:
 * 1. Simple bias-only calibration
 * 2. Temperature-compensated calibration
 * 3. 6-position calibration
 * 4. 6-position with temperature compensation
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateGyroscope();

/**
 * Validate gyroscope calibration quality
 * 
 * This function performs a comprehensive validation of the gyroscope calibration quality
 * by collecting samples while the device is stationary and checking:
 * 1. Bias stability (how close to zero after calibration)
 * 2. Noise characteristics (standard deviation)
 * 3. Temperature sensitivity (if temperature data available)
 * 
 * @param meanBias Output parameter for mean bias in dps
 * @param noiseSigma Output parameter for noise standard deviation in dps
 * @param tempSensitivity Output parameter for temperature sensitivity in dps/°C
 * @return true if calibration quality meets acceptance criteria, false otherwise
 */
bool validateGyroCalibration(float meanBias[3], float noiseSigma[3], float tempSensitivity[3]);

/**
 * Perform enhanced gyroscope calibration validation
 * 
 * This function validates the gyroscope calibration by collecting data
 * while the device is stationary and analyzing the stability, noise,
 * and temperature response characteristics.
 * 
 * @param driftRate Output parameter for gyro drift rate in degrees/hour
 * @param noiseFloor Output parameter for gyro noise floor in dps/√Hz
 * @param tempStability Output parameter for temperature stability in dps/°C
 * @return true if calibration meets quality thresholds, false otherwise
 */
bool validateGyroCalibrationEnhanced(float driftRate[3], float noiseFloor[3], float tempStability[3]);

/**
 * Perform dynamic gyroscope calibration validation
 * 
 * This function validates the gyroscope calibration during controlled rotation
 * to assess scale factor accuracy and linearity.
 * 
 * @param scaleFactorError Output parameter for scale factor error (percentage)
 * @param crossAxisError Output parameter for cross-axis sensitivity (percentage)
 * @return true if calibration meets quality thresholds, false otherwise
 */
bool validateGyroDynamic(float scaleFactorError[3], float crossAxisError[3]);

/**
 * Read raw gyroscope data
 * 
 * This function reads the raw gyroscope data from the sensor.
 * 
 * @param gx X-axis gyroscope reading (output parameter)
 * @param gy Y-axis gyroscope reading (output parameter)
 * @param gz Z-axis gyroscope reading (output parameter)
 * @return true if read was successful, false otherwise
 */
bool readGyro(float &gx, float &gy, float &gz);

/**
 * Read temperature data
 * 
 * This function reads the current temperature from the sensor.
 * 
 * @param temp Current temperature in degrees Celsius (output parameter)
 * @return true if read was successful, false otherwise
 */
bool readTemperature(float &temp);

// Sensor-specific validation helpers (used by common validation)
bool readGyroSensorData(float &x, float &y, float &z);
void applyGyroSensorCalibration(float &x, float &y, float &z);
bool checkGyroBiasAcceptable(float x, float y, float z);
bool checkGyroNoiseAcceptable(float x, float y, float z);

#endif // GYRO_CALIBRATION_H
