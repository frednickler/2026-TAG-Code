#ifndef MAG_CALIBRATION_H
#define MAG_CALIBRATION_H

#include <Arduino.h>
#include "calibration_common.h"

/**
 * Configure the magnetometer registers for optimal calibration
 * 
 * This function configures the ICM-20948 magnetometer with settings optimized for
 * accurate calibration:
 * - Maximum sensitivity
 * - Appropriate sample rate
 * - Continuous measurement mode
 */
void configureMagForCalibration();

/**
 * Apply magnetometer calibration to raw sensor readings
 * 
 * This function applies the calibration parameters to the raw magnetometer readings.
 * 
 * @param mx_uT X-axis magnetometer reading in μT (in/out parameter)
 * @param my_uT Y-axis magnetometer reading in μT (in/out parameter)
 * @param mz_uT Z-axis magnetometer reading in μT (in/out parameter)
 * @param temperature Current temperature in degrees Celsius (for temperature compensation)
 */
void applyMagCalibration(float& mx_uT, float& my_uT, float& mz_uT, float temperature = 0.0f);

/**
 * Perform simple bias-only magnetometer calibration
 * 
 * This basic calibration method collects magnetometer data while the device is
 * rotated through various orientations and calculates the bias/offset for each axis.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateMagSimple();

/**
 * Perform hard and soft iron calibration for magnetometer
 * 
 * This comprehensive calibration method collects magnetometer data while the device
 * is rotated through many different orientations. The collected data points form
 * an ellipsoid, which is then fitted to a sphere to determine calibration parameters
 * for both hard iron (offset) and soft iron (scaling and rotation) effects.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateMagHardSoftIron();

/**
 * Perform full 3-axis ellipsoid fitting calibration.
 * This method collects several hundred samples while the user rotates the
 * device through all orientations, then runs a least-squares algorithm to
 * determine both hard-iron bias and the full 3×3 soft-iron matrix (including
 * cross-axis terms).  The result is the highest-accuracy calibration but also
 * the slowest to compute.
 */
bool calibrateMagEllipsoid();

/**
 * Perform temperature-compensated magnetometer calibration
 * 
 * This method calibrates the magnetometer at the current temperature and calculates
 * temperature coefficients to compensate for drift across different temperatures.
 * It measures magnetometer bias at different temperatures to establish the relationship
 * between temperature and sensor drift.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateMagWithTempComp();

/**
 * Run the magnetometer calibration procedure with user-selectable methods
 * 
 * This function presents a menu to the user to select between different
 * magnetometer calibration methods:
 * 1. Simple bias-only calibration
 * 2. Hard and soft iron calibration
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateMagnetometer();

/**
 * Returns true if magnetometer has valid calibration data
 */
bool isMagCalibrated();

/**
 * Enable or disable dynamic (runtime) magnetometer calibration.
 * Dynamic calibration continuously refines hard/soft-iron parameters while the
 * firmware is running.  It consumes additional CPU time and therefore power,
 * so it is OFF by default.  Call this before flight if you need it.
 *
 * @param enabled  true ⇒ enable runtime calibration,  false ⇒ disable.
 */
void setDynamicMagCalibrationEnabled(bool enabled);

/**
 * Query whether dynamic magnetometer calibration is currently enabled.
 */
bool isDynamicMagCalibrationEnabled();

/**
 * Feed a new raw magnetometer sample into the dynamic calibration engine.
 * This is a no-op when the feature is disabled, so upstream code can call it
 * unconditionally without branching.
 */
void updateDynamicMagCalibration(float mx, float my, float mz);

/**
 * Quick validation of existing magnetometer calibration.
 * Collects a small set of samples (≈50), applies stored calibration, and
 * evaluates whether the corrected field magnitude is close to 1 with low
 * variance.  Intended for boot-time health check.
 *
 * @param meanField  Returns mean magnitude of corrected samples.
 * @param sigmaField Returns standard deviation of magnitude.
 * @return true if calibration passes threshold (|mean-1|<0.05 && σ/mean<0.05).
 */
bool validateMagCalibrationQuick(float &meanField, float &sigmaField);

/**
 * Validate magnetometer calibration quality
 * 
 * This function performs a comprehensive validation of the magnetometer calibration quality
 * by collecting samples while the device is rotated through different orientations and checking:
 * 1. Field magnitude consistency (should match Earth's magnetic field)
 * 2. Standard deviation of magnitude
 * 3. Sphericity of response (how close to a perfect sphere)
 * 
 * @param meanField Output parameter for mean field strength in μT
 * @param sigmaField Output parameter for standard deviation of field strength
 * @param sphereError Output parameter for deviation from perfect sphere (0-1, lower is better)
 * @return true if calibration quality meets acceptance criteria, false otherwise
 */
bool validateMagCalibration(float &meanField, float &sigmaField, float &sphereError);

/**
 * Enhanced quick validation of magnetometer calibration
 * 
 * This function performs a quick validation of the magnetometer calibration quality
 * by collecting samples while the device is stationary and checking field magnitude.
 * It's an enhanced version of validateMagCalibrationQuick with more detailed metrics.
 * 
 * @param meanField Output parameter for mean field strength in μT
 * @param sigmaField Output parameter for standard deviation of field strength
 * @param tempDrift Output parameter for temperature-related drift (if temp comp enabled)
 * @return true if calibration quality meets acceptance criteria, false otherwise
 */
bool validateMagCalibrationEnhanced(float &meanField, float &sigmaField, float &tempDrift);

/**
 * Read magnetometer calibration data
 * 
 * This function reads the magnetometer calibration data from storage.
 * 
 * @param mx_uT X-axis magnetometer reading in μT (in/out parameter)
 * @param my_uT Y-axis magnetometer reading in μT (in/out parameter)
 * @param mz_uT Z-axis magnetometer reading in μT (in/out parameter)
 * @return true if reading was successful, false otherwise
 */
bool readMagnetometerCalibration(float& mx_uT, float& my_uT, float& mz_uT);

/**
 * Helper function to check if recovery is needed and perform it if necessary
 * 
 * @param consecutive_failures Current count of consecutive failures
 * @param MAX_FAILURES Maximum allowed failures before recovery
 * @param last_success Reference to last successful read timestamp
 * @return false (indicating the read operation failed)
 */
bool checkRecovery(uint8_t& consecutive_failures, const uint8_t MAX_FAILURES, uint32_t& last_success);

/**
 * Query whether dynamic magnetometer calibration is currently enabled.
 */
bool isDynamicMagCalibrationEnabled();

/**
 * Feed a new raw magnetometer sample into the dynamic calibration engine.
 * This is a no-op when the feature is disabled, so upstream code can call it
 * unconditionally without branching.
 */
void updateDynamicMagCalibration(float mx, float my, float mz);

// Sensor-specific validation helpers (used by common validation)
bool readMagSensorData(float &x, float &y, float &z);
void applyMagSensorCalibration(float &x, float &y, float &z);
bool checkMagBiasAcceptable(float x, float y, float z);
bool checkMagNoiseAcceptable(float x, float y, float z);

#endif // MAG_CALIBRATION_H
