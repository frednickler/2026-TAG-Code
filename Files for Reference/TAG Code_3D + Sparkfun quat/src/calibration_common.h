#ifndef CALIBRATION_COMMON_H
#define CALIBRATION_COMMON_H

#include <Arduino.h>
#include "config.h"

/**
 * Enum defining the different accelerometer calibration methods
 */
enum AccelCalibrationMethod {
    ACCEL_CAL_SIMPLE = 0,       // Simple bias-only calibration
    ACCEL_CAL_SIX_POSITION = 1, // 6-position calibration with scale factors
    ACCEL_CAL_CROSS_AXIS = 2,   // 6-position with cross-axis sensitivity correction
    ACCEL_CAL_ELLIPSOID = 3     // Advanced ellipsoid fitting method
};

/**
 * Enum defining the different magnetometer calibration methods
 */
enum MagCalibrationMethod {
    MAG_CAL_SIMPLE = 0,         // Simple bias-only calibration
    MAG_CAL_HARD_SOFT_IRON = 1,  // Hard and soft iron calibration
    MAG_CAL_ELLIPSOID = 2       // Full 3D ellipsoid fit with cross-axis matrix
};

/**
 * Structure to hold calibration data for all sensors.
 */
struct CalibrationData {
    // Accelerometer calibration data
    float accelBias[3] = {0, 0, 0};     // Accelerometer bias/offset for each axis (mg)
    float accelScale[3] = {1, 1, 1};    // Accelerometer scale factors for each axis
    
    // Cross-axis sensitivity matrix (identity matrix by default)
    // This is a 3x3 matrix where:
    // - Diagonal elements (0,0), (1,1), (2,2) are the primary sensitivities (should be close to 1.0)
    // - Off-diagonal elements represent cross-axis sensitivity
    float accelCrossAxis[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    
    // Ellipsoid fitting parameters (advanced calibration)
    float accelEllipsoidCenter[3] = {0, 0, 0};  // Center of the fitted ellipsoid
    float accelEllipsoidRadii[3] = {1, 1, 1};   // Radii of the ellipsoid along principal axes
    float accelEllipsoidRotation[3][3] = {      // Rotation matrix for the ellipsoid
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    
    // Accelerometer calibration metadata
    bool accelCalibrated = false;                             // Whether accelerometer is calibrated
    bool accelDynamicCalEnabled = false;                      // Enable runtime (dynamic) calibration. Disabled by default to save power
    AccelCalibrationMethod accelCalMethod = ACCEL_CAL_SIMPLE;  // Current calibration method
    float accelCalTemp = 25.0f;                              // Temperature at calibration time
    bool accelTempCompEnabled = false;                       // Whether temperature compensation is enabled
    float accelTempCoeff[3] = {0, 0, 0};                     // Temperature coefficients for accel
    
    // Gyroscope calibration data
    float gyroBias[3] = {0, 0, 0};      // Gyroscope bias for each axis (dps)
    float referenceTemp = 25.0f;        // Reference temperature at which calibration was performed (°C)
    float gyroTempCoeff[3] = {0, 0, 0}; // Temperature coefficients for gyro (dps/°C) for each axis
    bool gyroCalibrated = false;        // Whether gyroscope is calibrated
    float gyroCalTemp = 25.0f;          // Temperature at gyroscope calibration time
    bool gyroTempCompEnabled = false;   // Whether temperature compensation is enabled for gyro
    
    // 6-position calibration data
    bool sixPositionCalibrated = false; // Whether 6-position calibration has been performed
    float gyroOffsets[6][3] = {0};      // Stores offsets from each of the 6 positions
    
    // 6-position accelerometer calibration data
    float accelOffsets[6][3] = {0};     // Stores accelerometer readings from each of the 6 positions
    
    // Magnetometer calibration data
    float magBias[3] = {0, 0, 0};       // Magnetometer hard-iron offset for each axis (uT)
    float magScale[3] = {1, 1, 1};      // Magnetometer soft-iron scale factor for each axis
    float magSoftIron[3][3] = {         // Soft iron correction matrix
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    float magDeclination = 0.0f;
    bool magCalibrated = false;         // Whether magnetometer is calibrated
    MagCalibrationMethod magCalMethod = MAG_CAL_SIMPLE;  // Current magnetometer calibration method
    bool magTempCompEnabled = false;    // Whether temperature compensation is enabled for mag
    float magTempCoeff[3] = {0, 0, 0};  // Temperature coefficients for magnetometer
    float magRmsError = 0.0f;          // RMS error of last ellipsoid fit (µT)
    float magFieldStrength = 0.0f;     // Estimated local field strength (µT)
    bool magDynamicCalEnabled = false; // Enable runtime (dynamic) calibration. Disabled by default to save power
    float magCalTemp = 25.0f;          // Temperature at which mag was calibrated

    // Temperature sensor calibration data
    float tempRoomOffset = 0.0f;        // Room temperature offset for temperature sensor
    float tempSensitivity = 333.87f;    // Temperature sensitivity (LSB/°C)
    float tempOffset = 21.0f;           // Temperature offset (°C)
    bool tempCalibrated = false;        // Whether temperature sensor is calibrated

    // Validation thresholds
    static constexpr float MAX_RESIDUAL_BIAS_GYRO = 0.1f;    // dps
    static constexpr float MAX_NOISE_GYRO = 0.05f;           // dps
    static constexpr float MAX_RESIDUAL_BIAS_ACCEL = 0.05f;  // g
    static constexpr float MAX_NOISE_ACCEL = 0.02f;          // g
    static constexpr float MAX_RESIDUAL_BIAS_MAG = 2.0f;     // uT
    static constexpr float MAX_NOISE_MAG = 1.0f;             // uT
};

/**
 * Enum defining the calibration process status
 */
enum CalibrationProcessStatus {
    CALIBRATION_NOT_STARTED = 0,
    CALIBRATION_IN_PROGRESS = 1,
    CALIBRATION_SUCCESSFUL = 2,
    CALIBRATION_FAILED = 3
};

// Global calibration data instance
extern CalibrationData calibration;
extern bool isCalibrating; // Flag to indicate if calibration is in progress

/**
 * Initialize the calibration process.
 */
void initCalibration();

/**
 * Load calibration data from EEPROM.
 *
 * @return True if calibration data was successfully loaded, false otherwise.
 */
bool loadCalibrationFromEEPROM();

/**
 * Save calibration data to EEPROM.
 * 
 * @return True if calibration data was successfully saved and verified, false otherwise.
 */
bool saveCalibrationToEEPROM();

/**
 * Clear calibration data from EEPROM.
 * 
 * @return True if calibration data was successfully cleared, false otherwise.
 */
bool clearCalibrationEEPROM();

/**
 * Reset the calibration data to its default values.
 */
void resetCalibration();

/**
 * Get a reference to the calibration data structure.
 *
 * @return A constant reference to the CalibrationData structure.
 */
const CalibrationData& getCalibrationData();

/**
 * Wait for user confirmation via serial input.
 * 
 * This function waits for the user to send 'y' or 'Y' over serial.
 * 
 * @param returnResult If true, returns a boolean result instead of void
 * @return True if user confirmed, false otherwise (only when returnResult is true)
 */
bool waitForUserConfirmation(bool returnResult = false);

/**
 * Print the current calibration values to the serial port.
 */
void printCalibrationValues();

/**
 * Collect bias data from accelerometer and gyroscope while device is stationary
 * 
 * @param sampleCount Number of samples to collect
 * @param axBias Reference to store X-axis accelerometer bias
 * @param ayBias Reference to store Y-axis accelerometer bias
 * @param azBias Reference to store Z-axis accelerometer bias
 * @param gxBias Reference to store X-axis gyroscope bias
 * @param gyBias Reference to store Y-axis gyroscope bias
 * @param gzBias Reference to store Z-axis gyroscope bias
 * @param enforceStillness If true, strictly enforce device stillness during calibration
 * @return true if bias collection was successful, false otherwise
 */
bool collectBias(uint16_t sampleCount,
                float& axBias, float& ayBias, float& azBias,
                float& gxBias, float& gyBias, float& gzBias,
                bool enforceStillness = true);

// Common validation functions
bool validateWithRetryOption();

// Helper functions for sensor-specific validation
bool checkBiasAcceptable(float x, float y, float z);
bool checkNoiseAcceptable(float x, float y, float z);
bool readSensorData(float &x, float &y, float &z);
void applySensorCalibration(float &x, float &y, float &z);

// User interaction helpers
void printCalibrationStatus();

// -----------------------------------------------------------------------------
// Validation callback mechanism
// -----------------------------------------------------------------------------

// Function pointer types for sensor-specific validation helpers
using ReadSensorDataFunc = bool (*)(float&, float&, float&);
using ApplySensorCalFunc = void (*)(float&, float&, float&);
using CheckBiasFunc      = bool (*)(float, float, float);
using CheckNoiseFunc     = bool (*)(float, float, float);

// Register callbacks before calling validateWithRetryOption()
void registerValidationCallbacks(ReadSensorDataFunc readFn,
                                 ApplySensorCalFunc applyFn,
                                 CheckBiasFunc biasFn,
                                 CheckNoiseFunc noiseFn);

// These will be called by validateWithRetryOption()
extern ReadSensorDataFunc   g_readSensorData;
extern ApplySensorCalFunc   g_applySensorCalibration;
extern CheckBiasFunc        g_checkBiasAcceptable;
extern CheckNoiseFunc       g_checkNoiseAcceptable;

// -----------------------------------------------------------------------------

#endif // CALIBRATION_COMMON_H
