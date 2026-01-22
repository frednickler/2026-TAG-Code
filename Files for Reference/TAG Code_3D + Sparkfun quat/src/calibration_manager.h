#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include <Arduino.h>
#include <EEPROM.h>
#include "calibration_common.h"

// EEPROM layout constants
#define EEPROM_CAL_START_ADDR     0x100    // Start address for calibration data
#define EEPROM_CAL_MAGIC          0xCAFE   // Magic number to verify valid data
#define EEPROM_CAL_VERSION        0x0003   // Calibration data version
#define EEPROM_CAL_SIZE           640      // Total size allocated for calibration data (increased from 512 to accommodate ExtendedCalibrationData)

// Calibration status levels
enum CalibrationStatus {
    CAL_STATUS_NONE = 0,        // No calibration data found
    CAL_STATUS_GOOD,            // Calibration is valid and accurate
    CAL_STATUS_MARGINAL,        // Calibration needs quick update
    CAL_STATUS_POOR,            // Calibration needs robust recalibration
    CAL_STATUS_CORRUPTED        // Calibration data is corrupted
};

// Calibration quality thresholds (configurable)
struct CalibrationThresholds {
    // Accelerometer thresholds (deviation from 1g)
    float accel_good_threshold;      // < 2% deviation
    float accel_marginal_threshold;  // 2-5% deviation
    
    // Gyroscope thresholds (zero-rate output in dps)
    float gyro_good_threshold;       // < 0.5 dps
    float gyro_marginal_threshold;   // 0.5-2.0 dps
    
    // Magnetometer thresholds (field consistency %)
    float mag_good_threshold;        // < 3% variation
    float mag_marginal_threshold;    // 3-8% variation
    
    // Temperature change threshold (°C)
    float temp_change_threshold;     // > 10°C change since calibration
};

// Calibration history entry (simple logging)
struct CalibrationHistoryEntry {
    uint32_t timestamp;              // Unix timestamp or millis()
    CalibrationStatus status;        // Status at that time
    float temperature;               // Temperature during check
    uint8_t sensor_mask;            // Which sensors were checked (bit flags)
};

// Extended calibration data with metadata
struct ExtendedCalibrationData {
    uint16_t magic;                  // Magic number for validation
    uint16_t version;                // Data structure version
    uint32_t timestamp;              // When calibration was performed
    float calibration_temp;          // Temperature during calibration
    CalibrationData cal_data;        // The actual calibration data
    CalibrationThresholds thresholds; // Quality thresholds
    CalibrationHistoryEntry history[3]; // Last 3 status checks (simple logging)
    uint32_t last_cal_timestamp; // Timestamp of the last successful calibration
    float last_temp;             // Last known temperature
    uint16_t checksum;           // Data integrity checksum
};

// Color codes for user interface
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_RED     "\033[31m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_CYAN    "\033[36m"

// Function declarations

/**
 * Initialize the calibration manager system
 * Sets up EEPROM, loads existing calibration data, and initializes thresholds
 */
void initCalibrationManager();

/**
 * Perform comprehensive startup verification of all sensors
 * @return Overall calibration status
 */
CalibrationStatus performStartupVerification();

/**
 * Display calibration status with color-coded output
 * @param status Current calibration status
 * @param show_details Whether to show detailed sensor information
 */
void displayCalibrationStatus(CalibrationStatus status, bool show_details = true);

/**
 * Perform one-time robust calibration for all sensors
 * Includes EEPROM save/verify and fallback options
 * @return true if successful, false otherwise
 */
bool performRobustCalibration();

/**
 * Perform quick calibration update (bias-only for most sensors)
 * @return true if successful, false otherwise
 */
bool performQuickCalibration();

/**
 * Save calibration data to EEPROM with verification
 * @return true if save and verification successful
 */
bool saveCalibrationToEEPROM();

/**
 * Verify EEPROM data integrity
 * @return true if data is valid and uncorrupted
 */
bool verifyEEPROMData();

/**
 * Calculate checksum for calibration data
 * @param data Pointer to extended calibration data
 * @return Calculated checksum
 */
uint16_t calculateChecksum(const ExtendedCalibrationData* data);

/**
 * Check individual sensor calibration quality
 * @param sensor_type Type of sensor to check
 * @return Status of that specific sensor
 */
CalibrationStatus checkSensorCalibration(uint8_t sensor_type);

/**
 * Update calibration history with new entry
 * @param status Current status
 * @param temperature Current temperature
 * @param sensor_mask Which sensors were checked
 */
void updateCalibrationHistory(CalibrationStatus status, float temperature, uint8_t sensor_mask);

/**
 * Get user decision on calibration action
 * @param current_status Current calibration status
 * @param allow_bypass Whether to allow bypass option
 * @return User's chosen action (0=proceed, 1=quick cal, 2=robust cal, 3=bypass)
 */
int getUserCalibrationDecision(CalibrationStatus current_status, bool allow_bypass = true);

/**
 * Set adaptive thresholds based on application requirements
 * @param precision_level 0=relaxed, 1=normal, 2=high precision
 */
void setAdaptiveThresholds(int precision_level);

/**
 * Check if environmental conditions have changed significantly
 * @return true if recalibration recommended due to environment
 */
bool checkEnvironmentalChange();

/**
 * Display calibration menu and handle user interaction
 */
void showCalibrationMenu();

/**
 * Emergency fallback calibration (minimal quick calibration)
 * Used when robust calibration fails or is not available
 */
bool performEmergencyCalibration();

// Global variables
extern ExtendedCalibrationData g_extended_cal_data;
extern CalibrationThresholds g_default_thresholds;

// Sensor type bit flags for history logging
#define SENSOR_FLAG_ACCEL  (1 << 0)
#define SENSOR_FLAG_GYRO   (1 << 1)
#define SENSOR_FLAG_MAG    (1 << 2)
#define SENSOR_FLAG_TEMP   (1 << 3)

#endif // CALIBRATION_MANAGER_H
