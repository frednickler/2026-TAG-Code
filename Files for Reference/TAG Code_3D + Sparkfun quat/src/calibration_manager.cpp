/**
 * @file calibration_manager.cpp
 * @brief Implementation of the three-tier IMU calibration and verification system
 * 
 * This file implements a sophisticated calibration management system with three tiers:
 * 1. Robust One-Time Calibration: Comprehensive sensor calibration with EEPROM storage
 * 2. Startup Verification: Quick health checks and calibration validation
 * 3. Runtime Management: Adaptive calibration suggestions based on sensor conditions
 * 
 * The system provides fallback options, user guidance, and automatic adaptation to 
 * environmental changes while maintaining calibration quality metrics.
 */

#include "calibration_manager.h"
#include "accel_calibration.h"
#include "gyro_calibration.h"
#include "mag_calibration.h"
#include "temp_sensor.h"
#include "imu_validation.h"
#include "imu_config.h"
#include <stddef.h> // for offsetof

// Global calibration data with EEPROM backup
ExtendedCalibrationData g_extended_cal_data;

// Default thresholds for calibration quality assessment
// These values are based on empirical testing and can be adjusted
// for different applications using setAdaptiveThresholds()
CalibrationThresholds g_default_thresholds = {
    .accel_good_threshold = 0.02f,      // 2% deviation from 1g
    .accel_marginal_threshold = 0.05f,  // 5% deviation from 1g
    .gyro_good_threshold = 0.5f,        // 0.5 dps
    .gyro_marginal_threshold = 2.0f,    // 2.0 dps
    .mag_good_threshold = 0.03f,        // 3% field variation
    .mag_marginal_threshold = 0.08f,    // 8% field variation
    .temp_change_threshold = 10.0f      // 10°C temperature change
};

/**
 * @brief Initialize the calibration management system
 * 
 * This function sets up the first tier of the calibration system:
 * 1. Initializes EEPROM for calibration data storage
 * 2. Sets default quality thresholds
 * 3. Attempts to load existing calibration from EEPROM
 * 4. Falls back to defaults if no valid calibration exists
 * 
 * The system uses a magic number and version check to validate EEPROM data,
 * ensuring corrupted or incompatible calibration data is not used.
 */
void initCalibrationManager() {
    Serial.println(F("\n" COLOR_CYAN "=== Initializing Calibration Manager ===" COLOR_RESET));
    
    // Initialize EEPROM
    // Initialize EEPROM with enough space for Config (0x00-0xFF) and Calibration (0x100+)
    EEPROM.begin(1024);
    
    // Set default thresholds
    g_extended_cal_data.thresholds = g_default_thresholds;
    
    // Try to load existing calibration data
    if (loadCalibrationFromEEPROM()) {
        Serial.println(F(COLOR_GREEN "✓ Existing calibration data loaded" COLOR_RESET));
    } else {
        Serial.println(F(COLOR_YELLOW "⚠ No valid calibration data found" COLOR_RESET));
        // Initialize with defaults
        memset(&g_extended_cal_data, 0, sizeof(ExtendedCalibrationData));
        g_extended_cal_data.magic = EEPROM_CAL_MAGIC;
        g_extended_cal_data.version = EEPROM_CAL_VERSION;
        g_extended_cal_data.thresholds = g_default_thresholds;
    }
    
    Serial.println(F(COLOR_CYAN "=== Calibration Manager Ready ===" COLOR_RESET "\n"));
}

/**
 * @brief Perform startup verification of sensor calibration
 * 
 * This function implements the second tier of the calibration system:
 * 1. Checks current temperature and environmental conditions
 * 2. Verifies calibration status of each sensor
 * 3. Updates calibration history
 * 4. Provides color-coded status display
 * 
 * The overall status is determined by the worst-case sensor status,
 * and can be further degraded by significant environmental changes.
 * 
 * @return CalibrationStatus - Overall calibration status (GOOD/MARGINAL/POOR/CORRUPTED)
 */
CalibrationStatus performStartupVerification() {
    Serial.println(F("\n" COLOR_BLUE "=== Startup Calibration Verification ===" COLOR_RESET));
    
    float current_temp;
    if (!readTemperature(current_temp)) {
        Serial.println(F(COLOR_RED "✗ Failed to read temperature" COLOR_RESET));
        current_temp = 25.0f; // Default assumption
    }
    
    // Check environmental change
    bool env_changed = checkEnvironmentalChange();
    if (env_changed) {
        Serial.println(F(COLOR_YELLOW "⚠ Significant environmental change detected" COLOR_RESET));
    }
    
    // Check each sensor
    CalibrationStatus accel_status = checkSensorCalibration(SENSOR_TYPE_ACCEL);
    CalibrationStatus gyro_status = checkSensorCalibration(SENSOR_TYPE_GYRO);
    CalibrationStatus mag_status = checkSensorCalibration(SENSOR_TYPE_MAG);
    
    // Determine overall status (worst case)
    CalibrationStatus overall_status = CAL_STATUS_GOOD;
    if (accel_status > overall_status) overall_status = accel_status;
    if (gyro_status > overall_status) overall_status = gyro_status;
    if (mag_status > overall_status) overall_status = mag_status;
    
    // Environmental change can bump status up one level
    if (env_changed && overall_status == CAL_STATUS_GOOD) {
        overall_status = CAL_STATUS_MARGINAL;
    }
    
    // Update history
    uint8_t sensor_mask = SENSOR_FLAG_ACCEL | SENSOR_FLAG_GYRO | SENSOR_FLAG_MAG | SENSOR_FLAG_TEMP;
    updateCalibrationHistory(overall_status, current_temp, sensor_mask);
    
    // Display results
    displayCalibrationStatus(overall_status, true);
    
    return overall_status;
}

/**
 * @brief Display calibration status with optional detailed information
 * 
 * Provides user feedback through color-coded serial output:
 * - Green: Good calibration, no action needed
 * - Yellow: Marginal calibration, quick update recommended
 * - Red: Poor calibration, robust recalibration needed
 * 
 * @param status Current calibration status
 * @param show_details If true, shows detailed per-sensor information
 */
void displayCalibrationStatus(CalibrationStatus status, bool show_details) {
    Serial.println(F("\n" COLOR_CYAN "=== Calibration Status Report ===" COLOR_RESET));
    
    // Overall status
    const char* status_text;
    const char* color_code;
    const char* recommendation;
    
    switch (status) {
        case CAL_STATUS_GOOD:
            status_text = "GOOD";
            color_code = COLOR_GREEN;
            recommendation = "No action needed - proceed with normal operation";
            break;
        case CAL_STATUS_MARGINAL:
            status_text = "MARGINAL";
            color_code = COLOR_YELLOW;
            recommendation = "Quick calibration recommended";
            break;
        case CAL_STATUS_POOR:
            status_text = "POOR";
            color_code = COLOR_RED;
            recommendation = "Robust calibration required";
            break;
        case CAL_STATUS_CORRUPTED:
            status_text = "CORRUPTED";
            color_code = COLOR_RED;
            recommendation = "Emergency calibration required";
            break;
        default:
            status_text = "NONE";
            color_code = COLOR_RED;
            recommendation = "Initial calibration required";
            break;
    }
    
    Serial.printf("Overall Status: %s%s%s\n", color_code, status_text, COLOR_RESET);
    Serial.printf("Recommendation: %s\n", recommendation);
    
    if (show_details && g_extended_cal_data.timestamp > 0) {
        // Convert seconds to hours and minutes for better readability
        unsigned long seconds_ago = (millis() - g_extended_cal_data.timestamp) / 1000;
        unsigned long hours = seconds_ago / 3600;
        unsigned long minutes = (seconds_ago % 3600) / 60;
        
        if (hours > 0) {
            Serial.printf("Last Calibration: %02lu:%02lu ago\n", hours, minutes);
        } else {
            Serial.printf("Last Calibration: %02lu minutes ago\n", minutes);
        }
        Serial.printf("Calibration Temp: %.1f°C\n", g_extended_cal_data.calibration_temp);
    }
    
    Serial.println();
}

/**
 * @brief Update calibration history
 * 
 * This function updates the calibration history with the latest status,
 * temperature, and sensor mask.
 * 
 * @param status Current calibration status
 * @param temperature Current temperature
 * @param sensor_mask Sensor mask (bitfield)
 */
void updateCalibrationHistory(CalibrationStatus status, float temperature, uint8_t sensor_mask) {
    // Shift existing entries
    for (int i = 2; i > 0; i--) {
        g_extended_cal_data.history[i] = g_extended_cal_data.history[i-1];
    }
    
    // Add new entry
    g_extended_cal_data.history[0].timestamp = millis();
    g_extended_cal_data.history[0].status = status;
    g_extended_cal_data.history[0].temperature = temperature;
    g_extended_cal_data.history[0].sensor_mask = sensor_mask;
}

/**
 * @brief Perform emergency calibration when all else fails
 * 
 * This is a minimal calibration that provides basic functionality:
 * - Simple accelerometer bias correction
 * - Basic gyroscope zero-rate calibration
 * - No magnetometer calibration (too complex for emergency mode)
 * 
 * Used when:
 * - Robust calibration fails completely
 * - Quick calibration fails
 * - EEPROM data is corrupted
 * - User needs immediate basic functionality
 * 
 * @return true if emergency calibration successful, false for critical failure
 */
bool performEmergencyCalibration() {
    Serial.println(F("\n" COLOR_RED "=== EMERGENCY CALIBRATION ===" COLOR_RESET));
    Serial.println(F("Performing minimal calibration to ensure basic functionality..."));
    
    // Minimal accelerometer bias
    if (!calibrateAccelSimple()) {
        Serial.println(F(COLOR_RED "✗ Emergency accelerometer calibration failed" COLOR_RESET));
        return false;
    }
    
    // Minimal gyroscope bias
    if (!calibrateGyroSimple()) {
        Serial.println(F(COLOR_RED "✗ Emergency gyroscope calibration failed" COLOR_RESET));
        return false;
    }
    
    Serial.println(F(COLOR_YELLOW "⚠ Emergency calibration complete - recommend full calibration when possible" COLOR_RESET));
    return true;
}

/**
 * @brief Check for environmental changes
 * 
 * This function checks if the current temperature has changed significantly
 * since the last calibration.
 * 
 * @return true if environmental change detected, false otherwise
 */
bool checkEnvironmentalChange() {
    float current_temp;
    if (!readTemperature(current_temp)) {
        return false; // Can't determine change
    }
    
    float temp_diff = fabs(current_temp - g_extended_cal_data.calibration_temp);
    return temp_diff > g_extended_cal_data.thresholds.temp_change_threshold;
}

/**
 * @brief Set adaptive thresholds
 * 
 * This function sets the adaptive thresholds for calibration quality assessment.
 * 
 * @param precision_level Precision level (0-2)
 */
void setAdaptiveThresholds(int precision_level) {
    switch (precision_level) {
        case 0: // Relaxed
            g_extended_cal_data.thresholds.accel_good_threshold = 0.05f;
            g_extended_cal_data.thresholds.accel_marginal_threshold = 0.10f;
            break;
        case 1: // Normal (default)
            g_extended_cal_data.thresholds = g_default_thresholds;
            break;
        case 2: // High precision
            g_extended_cal_data.thresholds.accel_good_threshold = 0.01f;
            g_extended_cal_data.thresholds.accel_marginal_threshold = 0.03f;
            break;
    }
}

/**
 * @brief Check calibration status for a specific sensor
 * 
 * This function verifies the calibration status of a specific sensor type
 * by comparing current readings against expected values and thresholds.
 * 
 * @param sensor_type Type of sensor to check (SENSOR_TYPE_ACCEL, SENSOR_TYPE_GYRO, SENSOR_TYPE_MAG)
 * @return CalibrationStatus Status of the sensor's calibration
 */
CalibrationStatus checkSensorCalibration(uint8_t sensor_type) {
    // Read sensor data
    float x, y, z;
    bool read_ok = false;
    
    switch (sensor_type) {
        case SENSOR_TYPE_ACCEL:
            read_ok = readAccel(x, y, z);
            break;
        case SENSOR_TYPE_GYRO:
            read_ok = readGyro(x, y, z);
            break;
        case SENSOR_TYPE_MAG:
            read_ok = readMagnetometer(x, y, z);
            break;
        default:
            Serial.printf("[CAL] Error: Invalid sensor type %u\n", sensor_type);
            return CAL_STATUS_CORRUPTED;
    }
    
    if (!read_ok) {
        Serial.printf("[CAL] Error reading sensor %u\n", sensor_type);
        return CAL_STATUS_CORRUPTED;
    }
    
    // Calculate magnitude for basic sanity check
    float magnitude = sqrt(x*x + y*y + z*z);
    
    // Check based on sensor type
    switch (sensor_type) {
        case SENSOR_TYPE_ACCEL: {
            float expected_mag = 1.0f;  // 1g
            float deviation = fabs(magnitude - expected_mag);
            if (deviation <= g_extended_cal_data.thresholds.accel_good_threshold) return CAL_STATUS_GOOD;
            if (deviation <= g_extended_cal_data.thresholds.accel_marginal_threshold) return CAL_STATUS_MARGINAL;
            return CAL_STATUS_POOR;
        }
            
        case SENSOR_TYPE_GYRO: {
            // Gyro should be near zero at rest
            float deviation = magnitude; 
            if (deviation <= g_extended_cal_data.thresholds.gyro_good_threshold) return CAL_STATUS_GOOD;
            if (deviation <= g_extended_cal_data.thresholds.gyro_marginal_threshold) return CAL_STATUS_MARGINAL;
            return CAL_STATUS_POOR;
        }
            
        case SENSOR_TYPE_MAG: {
            // Magnetometer returns uT. Earth's field is typically 25-65 uT.
            // We verify it's within a reasonable range for a valid reading.
            // 20-70 uT is a safe "Good" range. 10-90 uT is "Marginal".
            if (magnitude >= 20.0f && magnitude <= 70.0f) {
                return CAL_STATUS_GOOD;
            } else if (magnitude >= 10.0f && magnitude <= 90.0f) {
                return CAL_STATUS_MARGINAL;
            } else {
                // If magnitude is 1.0f (normalized), it's definitely wrong for uT
                return CAL_STATUS_POOR; 
            }
        }
            
        default:
            return CAL_STATUS_CORRUPTED;
    }
}

/**
 * @brief Get user's decision on calibration
 * 
 * @param status The current calibration status
 * @param auto_mode If true, runs in non-interactive mode
 * @return UserCalibrationDecision The user's choice
 */
int getUserCalibrationDecision(CalibrationStatus status, bool auto_mode) {
    if (auto_mode) {
        if (status == CAL_STATUS_POOR || status == CAL_STATUS_CORRUPTED) {
            return 3;
        } else if (status == CAL_STATUS_MARGINAL) {
            return 2;
        }
        return 1;
    }

    Serial.println(F("\nPlease choose an action:"));
    Serial.println(F("  1. Quick Calibration (~2 mins)"));
    Serial.println(F("  2. Robust Calibration (~10 mins)"));
    Serial.println(F("  3. Individual Sensor Menu"));
    Serial.println(F("  4. Skip (use existing calibration)"));
    Serial.print(F("Enter your choice (1-4): "));

    while (true) {
        if (Serial.available() > 0) {
            char choice = Serial.read();
            if (choice >= '1' && choice <= '4') {
                Serial.println(choice);
                return (choice - '0');
            }
        }
        delay(100);
    }
}

/**
 * @brief Perform a robust calibration of all sensors
 * 
 * @return true if calibration was successful, false otherwise
 */
bool performRobustCalibration() {
    Serial.println(F("\n=== Starting Robust Calibration ==="));
    bool success = true;

    Serial.println(F("\nCalibrating accelerometer (6-position)..."));
    if (!calibrateAccel6Position(true)) { // with temp compensation
        Serial.println(F("Robust accelerometer calibration failed"));
        success = false;
    }

    // Step 2: Calibrate gyroscope (simple - device flat on surface)
    Serial.println(F("\nCalibrating gyroscope (simple)..."));
    if (!calibrateGyroSimple()) {
        Serial.println(F("Gyroscope calibration failed!"));
        return false;
    }
    Serial.println(F("✓ Gyroscope calibration completed"));

    Serial.println(F("\nCalibrating magnetometer (hard/soft iron)..."));
    if (!calibrateMagHardSoftIron()) {
        Serial.println(F("Robust magnetometer calibration failed"));
        success = false;
    }

    if (success) {
        Serial.println(F("\nRobust calibration completed successfully."));
        saveCalibrationToEEPROM();
    } else {
        Serial.println(F("\nRobust calibration failed."));
    }

    return success;
}

/**
 * @brief Perform a quick calibration of all sensors
 * 
 * This function performs a quick calibration of all sensors (accelerometer,
 * gyroscope, and magnetometer) using basic calibration methods. It's designed
 * to be relatively fast (about 2 minutes) while still providing reasonable
 * calibration quality.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool performQuickCalibration() {
    Serial.println(F("\n" COLOR_CYAN "=== Starting Quick Calibration ===" COLOR_RESET));
    Serial.println(F("This will take about 2 minutes. Please keep the device still."));
    
    bool success = true;
    
    // 1. Calibrate accelerometer (simple bias removal)
    Serial.println(F("\nCalibrating accelerometer..."));
    if (!calibrateAccelSimple()) {
        Serial.println(F(COLOR_RED "✗ Accelerometer calibration failed" COLOR_RESET));
        success = false;
    } else {
        Serial.println(F(COLOR_GREEN "✓ Accelerometer calibration complete" COLOR_RESET));
    }
    
    // 2. Calibrate gyroscope (zero-rate offset)
    Serial.println(F("\nCalibrating gyroscope..."));
        if (!calibrateGyroscope()) {
        Serial.println(F(COLOR_RED "✗ Gyroscope calibration failed" COLOR_RESET));
        success = false;
    } else {
        Serial.println(F(COLOR_GREEN "✓ Gyroscope calibration complete" COLOR_RESET));
    }
    
    // 3. Calibrate magnetometer (basic hard/soft iron)
    Serial.println(F("\nCalibrating magnetometer..."));
    Serial.println(F("Please rotate the device slowly in all directions"));
        if (!calibrateMagSimple()) {
        Serial.println(F(COLOR_RED "✗ Magnetometer calibration failed" COLOR_RESET));
        success = false;
    } else {
        Serial.println(F(COLOR_GREEN "✓ Magnetometer calibration complete" COLOR_RESET));
    }
    
    // 4. Read current temperature for compensation
    float temp;
    if (readTemperature(temp)) {
        g_extended_cal_data.last_temp = temp;
        g_extended_cal_data.calibration_temp = temp;
        Serial.printf("Calibration temperature: %.1f°C\n", temp);
    }
    
    // 5. Save calibration to EEPROM
    if (success) {
        if (saveCalibrationToEEPROM()) {
            Serial.println(F(COLOR_GREEN "\n✓ Calibration saved to EEPROM" COLOR_RESET));
        } else {
            Serial.println(F(COLOR_RED "\n✗ Failed to save calibration to EEPROM" COLOR_RESET));
            success = false;
        }
    }
    
    // 6. Verify calibration
    if (success) {
        Serial.println(F("\nVerifying calibration..."));
        CalibrationStatus status = performStartupVerification();
        if (status == CAL_STATUS_GOOD || status == CAL_STATUS_MARGINAL) {
            Serial.println(F(COLOR_GREEN "✓ Quick calibration completed successfully" COLOR_RESET));
        } else {
            Serial.println(F(COLOR_YELLOW "⚠ Quick calibration completed with warnings" COLOR_RESET));
        }
    } else {
        Serial.println(F(COLOR_RED "\n✗ Quick calibration failed" COLOR_RESET));
    }
    
    return success;
}

// -----------------------------------------------------------------------------
// Checksum Utility
// -----------------------------------------------------------------------------

/**
 * @brief Calculate a simple 16-bit additive checksum over the calibration data
 *
 * The checksum is computed over every byte of the ExtendedCalibrationData
 * structure EXCEPT the checksum field itself. This provides a fast integrity
 * check while keeping implementation simple for embedded targets.
 *
 * @param data Pointer to the calibration structure to checksum
 * @return 16-bit checksum value
 */
uint16_t calculateChecksum(const ExtendedCalibrationData* data) {
    if (!data) return 0;
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
    const size_t len = offsetof(ExtendedCalibrationData, checksum); // exclude checksum field
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += bytes[i];
    }
    return sum;
}
