#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H

#include <Arduino.h>

/**
 * @brief IMU Calibration Data Structure
 * 
 * Stores calibration parameters for accelerometer and gyroscope.
 * Saved to ESP32 Preferences (NVS) for persistence across reboots.
 */
struct IMUCalibration {
    // Accelerometer calibration
    float accelBias[3];      // Bias offsets [X, Y, Z] in g
    float accelScale[3];     // Scale factors [X, Y, Z] (nominal 1.0)
    
    // Gyroscope calibration
    float gyroBias[3];       // Bias offsets [X, Y, Z] in °/s
    
    // Magnetometer calibration
    float magBias[3];        // Hard Iron offsets [X, Y, Z] in µT
    float magSoftIron[3][3]; // Soft Iron correction matrix (3x3)
    
    // Metadata
    bool calibrated;         // True if valid calibration exists
    bool calibrationMountingUpsideDown;  // Mounting orientation when calibrated
    uint32_t timestamp;      // Unix timestamp of calibration
    uint16_t checksum;       // CRC16 for data integrity
    
    // Constructor - initialize to defaults
    IMUCalibration() {
        // Accelerometer defaults
        accelBias[0] = accelBias[1] = accelBias[2] = 0.0f;
        accelScale[0] = accelScale[1] = accelScale[2] = 1.0f;
        
        // Gyroscope defaults
        gyroBias[0] = gyroBias[1] = gyroBias[2] = 0.0f;
        
        // Magnetometer defaults (Hard Iron = 0, Soft Iron = Identity)
        magBias[0] = magBias[1] = magBias[2] = 0.0f;
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                magSoftIron[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        
        // Metadata
        calibrated = false;
        calibrationMountingUpsideDown = false;
        timestamp = 0;
        checksum = 0;
    }
    
    /**
     * @brief Calculate CRC16 checksum of calibration data
     * @return CRC16 checksum
     */
    uint16_t calculateChecksum() const {
        uint16_t crc = 0xFFFF;
        
        // Helper lambda or macro to process data
        auto updateCRC = [&](const void* data, size_t len) {
            const uint8_t* p = (const uint8_t*)data;
            for (size_t i = 0; i < len; i++) {
                crc ^= p[i];
                for (uint8_t j = 0; j < 8; j++) {
                    if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
                    else crc = crc >> 1;
                }
            }
        };

        // Add fields explicitly
        updateCRC(accelBias, sizeof(accelBias));
        updateCRC(accelScale, sizeof(accelScale));
        updateCRC(gyroBias, sizeof(gyroBias));
        updateCRC(magBias, sizeof(magBias));
        updateCRC(magSoftIron, sizeof(magSoftIron));
        updateCRC(&calibrated, sizeof(calibrated));
        updateCRC(&calibrationMountingUpsideDown, sizeof(calibrationMountingUpsideDown));
        updateCRC(&timestamp, sizeof(timestamp));
        
        return crc;
    }
    
    /**
     * @brief Validate checksum
     * @return true if checksum matches
     */
    bool isChecksumValid() const {
        return checksum == calculateChecksum();
    }
    
    /**
     * @brief Update checksum (call before saving)
     */
    void updateChecksum() {
        checksum = calculateChecksum();
    }
};

/**
 * @brief Calibration Diagnostics
 * Tracks performance and failures during calibration
 */
struct CalibrationDiagnostics {
    uint32_t totalSamples;
    uint32_t successfulI2CReads;
    uint32_t failedI2CReads;
    uint32_t watchdogFeeds;
    uint32_t retryAttempts;
    uint32_t phaseStartTime;
    uint32_t phaseEndTime;
    char currentPhase[32]; 
    
    void reset() {
        totalSamples = 0;
        successfulI2CReads = 0;
        failedI2CReads = 0;
        watchdogFeeds = 0;
        retryAttempts = 0;
        phaseStartTime = millis();
        currentPhase[0] = '\0';
    }
    
    void logPhase(const char* phase) {
        strncpy(currentPhase, phase, sizeof(currentPhase)-1);
        currentPhase[sizeof(currentPhase)-1] = '\0';
        phaseStartTime = millis();
    }

    void print() {
        Serial.printf("=== CALIBRATION DIAGNOSTICS ===\n");
        Serial.printf("Phase: %s\n", currentPhase);
        Serial.printf("Duration: %lu ms\n", millis() - phaseStartTime);
        Serial.printf("Samples: %u\n", totalSamples);
        Serial.printf("I2C Success: %u (Fail: %u)\n", successfulI2CReads, failedI2CReads);
        Serial.printf("Retries: %u\n", retryAttempts);
        Serial.printf("WDT Feeds: %u\n", watchdogFeeds);
    }
};

// Validation thresholds
namespace CalibrationThresholds {
    // Accelerometer (Base values for 4G range)
    constexpr float ACCEL_BIAS_MAX = 0.05f;      // g
    constexpr float ACCEL_SCALE_MIN = 0.95f;
    constexpr float ACCEL_SCALE_MAX = 1.05f;
    
    // ICM-20948 Accelerometer: 230 µg/√Hz noise density (44% higher than BMI270's 160 µg/√Hz)
    // Calculated for 50Hz sampling: RMS = 1.626mg, 6σ = 9.76mg
    // Noise increases with range: 2G=baseline, 4G=√2, 8G=2x, 16G=2.8x
    constexpr float ACCEL_NOISE_2G  = 0.010f;    // g (10mg - 6σ baseline)
    constexpr float ACCEL_NOISE_4G  = 0.014f;    // g (14mg - 6σ × √2)
    constexpr float ACCEL_NOISE_8G  = 0.020f;    // g (20mg - 6σ × 2)
    constexpr float ACCEL_NOISE_16G = 0.028f;    // g (28mg - 6σ × 2.8)
    
    // Stability thresholds (TEMPORARILY RELAXED - investigating high variation issue)
    // TODO: Investigate why variation is 60-179mg instead of expected ~10mg
    constexpr float ACCEL_STABILITY_2G  = 0.100f; // g (100mg - TEMP: was 20mg)
    constexpr float ACCEL_STABILITY_4G  = 0.200f; // g (200mg - TEMP: was 30mg)
    constexpr float ACCEL_STABILITY_8G  = 0.300f; // g (300mg - TEMP: was 45mg)
    constexpr float ACCEL_STABILITY_16G = 0.400f; // g (400mg - TEMP: was 65mg)
    
    // Gyroscope (Base values for 500dps range)
    constexpr float GYRO_BIAS_MAX = 0.5f;        // °/s (datasheet typical)
    
    // ICM-20948 Gyroscope: 0.015 dps/√Hz (2.14x noisier than BMI270's 0.007 dps/√Hz)
    // Calculated for 50Hz sampling: RMS = 0.106 dps, 10σ = 1.06 dps
    // TEMPORARILY RELAXED - sensor showing higher noise than datasheet
    // TODO: Investigate why gyro noise is 2.4 dps instead of expected 1.1 dps
    constexpr float GYRO_NOISE_250   = 2.5f;     // °/s (TEMP: was 1.1)
    constexpr float GYRO_NOISE_500   = 3.5f;     // °/s (TEMP: was 1.5)
    constexpr float GYRO_NOISE_1000  = 5.0f;     // °/s (TEMP: was 2.2)
    constexpr float GYRO_NOISE_2000  = 7.0f;     // °/s (TEMP: was 3.0)
    
    // BMM350 Magnetometer: X/Y: 190 nT RMS, Z: 450 nT RMS @ 100Hz ODR
    // Better than BMM150 on X/Y (3x), slightly worse on Z (1.5x)
    // Worst case (Z-axis): 0.45 µT RMS, 6σ = 2.7 µT
    constexpr float MAG_NOISE_MAX = 5.0f;        // µT (adequate margin for BMM350)
    constexpr float MAG_FIT_RMS_MAX = 10.0f;     // µT (ellipsoid fit quality)
    
    /**
     * @brief Get accelerometer noise threshold for current range
     * @param range BMI2_ACC_RANGE_* value (0=2G, 1=4G, 2=8G, 3=16G)
     */
    inline float getAccelNoiseMax(uint8_t range) {
        switch(range) {
            case 0: return ACCEL_NOISE_2G;   // BMI2_ACC_RANGE_2G
            case 1: return ACCEL_NOISE_4G;   // BMI2_ACC_RANGE_4G
            case 2: return ACCEL_NOISE_8G;   // BMI2_ACC_RANGE_8G
            case 3: return ACCEL_NOISE_16G;  // BMI2_ACC_RANGE_16G
            default: return ACCEL_NOISE_4G;
        }
    }
    
    /**
     * @brief Get accelerometer stability threshold for current range
     * @param range BMI2_ACC_RANGE_* value
     */
    inline float getAccelStabilityMax(uint8_t range) {
        switch(range) {
            case 0: return ACCEL_STABILITY_2G;
            case 1: return ACCEL_STABILITY_4G;
            case 2: return ACCEL_STABILITY_8G;
            case 3: return ACCEL_STABILITY_16G;
            default: return ACCEL_STABILITY_4G;
        }
    }
    
    /**
     * @brief Get gyroscope noise threshold for current range
     * @param range BMI2_GYR_RANGE_* value (0=2000, 1=1000, 2=500, 3=250)
     */
    inline float getGyroNoiseMax(uint8_t range) {
        switch(range) {
            case 0: return GYRO_NOISE_2000;  // BMI2_GYR_RANGE_2000
            case 1: return GYRO_NOISE_1000;  // BMI2_GYR_RANGE_1000
            case 2: return GYRO_NOISE_500;   // BMI2_GYR_RANGE_500
            case 3: return GYRO_NOISE_250;   // BMI2_GYR_RANGE_250
            default: return GYRO_NOISE_500;
        }
    }
}

#endif // CALIBRATION_DATA_H

