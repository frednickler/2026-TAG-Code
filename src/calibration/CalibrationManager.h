#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include <Arduino.h>
#include "CalibrationData.h"

/**
 * @brief IMU Calibration Manager
 * 
 * Handles accelerometer and gyroscope calibration:
 * - Load/save to ESP32 Preferences
 * - Calibration routines (simple, 6-position)
 * - Quality validation
 * - Runtime application of calibration
 */
class CalibrationManager {
public:
    /**
     * @brief Initialize calibration manager
     * Loads calibration from Preferences if available
     * @return true if initialization successful
     */
    static bool init();
    
    /**
     * @brief Check if IMU is calibrated
     * @return true if valid calibration exists
     */
    static bool isCalibrated();
    
    /**
     * @brief Quick validation of current calibration (2 seconds)
     * Tests if loaded calibration is still within acceptable quality
     * @return true if calibration quality is acceptable
     */
    static bool validateQuick();
    
    /**
     * @brief Full validation of calibration (10 seconds)
     * Comprehensive quality check with detailed metrics
     * @param printDetails If true, print detailed results to serial
     * @return true if calibration quality is excellent
     */
    static bool validateFull(bool printDetails = true);
    
    // === Calibration Routines ===
    
    /**
     * @brief Calibrate gyroscope (bias only)
     * Device must be stationary during calibration
     * @return true if calibration successful
     */
    static bool calibrateGyro();
    
    /**
     * @brief Calibrate accelerometer (simple - bias only)
     * Device must be flat on stable surface (Z-up)
     * @return true if calibration successful
     */
    static bool calibrateAccelSimple();
    
    /**
     * @brief Calibrate accelerometer (6-position - bias + scale)
     * User places device in 6 orientations (cube faces)
     * @return true if calibration successful
     */
    static bool calibrateAccel6Position();
    
    /**
     * @brief Run full calibration (gyro + accel 6-position)
     * @return true if all calibrations successful
     */
    static bool calibrateFull();
    
    /**
     * @brief Calibrate magnetometer (simple min/max - hard iron only)
     * User rotates device in figure-8 pattern
     * @return true if calibration successful
     */
    static bool calibrateMagSimple();
    
    /**
     * @brief Calibrate magnetometer (ellipsoid fit - hard + soft iron)
     * User rotates device covering full sphere
     * @return true if calibration successful
     */
    static bool calibrateMagPrecision();
    
    // === Apply Calibration ===
    
    /**
     * @brief Apply accelerometer calibration to raw readings
     * @param ax X-axis acceleration (in/out) in g
     * @param ay Y-axis acceleration (in/out) in g
     * @param az Z-axis acceleration (in/out) in g
     */
    static void applyAccel(float& ax, float& ay, float& az);
    
    /**
     * @brief Apply gyroscope calibration to raw readings
     * @param gx X-axis rotation rate (in/out) in °/s
     * @param gy Y-axis rotation rate (in/out) in °/s
     * @param gz Z-axis rotation rate (in/out) in °/s
     */
    static void applyGyro(float& gx, float& gy, float& gz);
    
    /**
     * @brief Apply magnetometer calibration to raw readings
     * @param mx X-axis magnetic field (in/out) in µT
     * @param my Y-axis magnetic field (in/out) in µT
     * @param mz Z-axis magnetic field (in/out) in µT
     */
    static void applyMag(float& mx, float& my, float& mz);
    
    // === Data Access ===
    
    /**
     * @brief Get current calibration data
     * @return const reference to calibration data
     */
    static const IMUCalibration& getCalibration();
    
    /**
     * @brief Print current calibration values to serial
     */
    static void printCalibration();
    
    /**
     * @brief Reset calibration to defaults (uncalibrated state)
     * @param saveToPrefs If true, also clear saved data in Preferences
     * @return true if reset successful
     */
    static bool resetCalibration(bool saveToPrefs = true);

private:
    static IMUCalibration calibration;
    static bool initialized;
    
public:
    static CalibrationDiagnostics diagnostics;

private:
    // Helper to read mag with retry logic
    static bool readMagWithRetry(float& mx, float& my, float& mz);

    // Storage
    static bool loadFromPreferences();
    static bool saveToPreferences();
    
    // Helper functions
    static bool collectStationarySamples(
        float accelData[][3], 
        float gyroData[][3], 
        size_t sampleCount,
        const char* instruction = nullptr
    );
    
    static bool checkDeviceStationary(
        const float accelData[][3],
        const float gyroData[][3],
        size_t sampleCount
    );

    // Unified validation helper with range-adaptive thresholds
    static bool checkDataQuality(
        const float accelData[][3], 
        const float gyroData[][3], 
        size_t sampleCount, 
        bool checkStationary, 
        bool checkNoise,
        uint8_t accelRange = 1,  // Default: BMI2_ACC_RANGE_4G
        uint8_t gyroRange = 2    // Default: BMI2_GYR_RANGE_500
    );
    
    static void calculateMean(
        const float data[][3],
        size_t sampleCount,
        float mean[3]
    );
    
    static void calculateStdDev(
        const float data[][3],
        size_t sampleCount,
        const float mean[3],
        float stdDev[3]
    );
    
    static bool waitForStableOrientation(
        int axis, 
        float target, 
        float threshold = 0.8f,
        unsigned long timeout = 30000
    );
};

#endif // CALIBRATION_MANAGER_H
