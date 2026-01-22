#include "CalibrationManager.h"
#include <Preferences.h>
#include "../sensors/IMUManager.h"
#include "../system/Watchdog.h"
#include <Arduino.h>
#include "../config/SystemConfig.h"
#include "../config/SystemSettings.h"
#include "ellipsoid_fit.h"
#include <LittleFS.h>

// Static member initialization
IMUCalibration CalibrationManager::calibration;
bool CalibrationManager::initialized = false;
CalibrationDiagnostics CalibrationManager::diagnostics;

// Preferences namespace
static Preferences prefs;
static const char* PREFS_NAMESPACE = "imu_cal";

// ============================================================================
// INITIALIZATION & STORAGE
// ============================================================================

bool CalibrationManager::init() {
    DEBUG_INFO("Initializing CalibrationManager...");
    
    if (!loadFromPreferences()) {
        DEBUG_WARN("No saved calibration found - device needs calibration");
        // Only reset if NO valid data was found (loadFromPreferences handles validity)
        if (!isCalibrated()) {
             calibration = IMUCalibration(); // Reset to defaults
        }
        initialized = true;
        return false; // No full calibration loaded
    }
    
    DEBUG_INFO("Calibration loaded successfully");
    initialized = true;
    return true;
}

bool CalibrationManager::isCalibrated() {
    return calibration.calibrated && calibration.isChecksumValid();
}

bool CalibrationManager::loadFromPreferences() {
    if (!prefs.begin(PREFS_NAMESPACE, true)) { // Read-only mode
        DEBUG_ERROR("Failed to open Preferences");
        return false;
    }
    
    // Check if calibration exists - we check for checksum as minimum requirement
    if (!prefs.isKey("checksum")) {
        prefs.end();
        return false;
    }
    
    // Load all data
    calibration.calibrated = prefs.getBool("calibrated", false);
    calibration.calibrationMountingUpsideDown = prefs.getBool("calMountFlip", false);
    calibration.timestamp = prefs.getUInt("timestamp", 0);
    
    // Load accelerometer calibration
    calibration.accelBias[0] = prefs.getFloat("accelBiasX", 0.0f);
    calibration.accelBias[1] = prefs.getFloat("accelBiasY", 0.0f);
    calibration.accelBias[2] = prefs.getFloat("accelBiasZ", 0.0f);
    
    calibration.accelScale[0] = prefs.getFloat("accelScaleX", 1.0f);
    calibration.accelScale[1] = prefs.getFloat("accelScaleY", 1.0f);
    calibration.accelScale[2] = prefs.getFloat("accelScaleZ", 1.0f);
    
    // Load gyroscope calibration
    calibration.gyroBias[0] = prefs.getFloat("gyroBiasX", 0.0f);
    calibration.gyroBias[1] = prefs.getFloat("gyroBiasY", 0.0f);
    calibration.gyroBias[2] = prefs.getFloat("gyroBiasZ", 0.0f);
    
    // Load magnetometer calibration
    calibration.magBias[0] = prefs.getFloat("magBiasX", 0.0f);
    calibration.magBias[1] = prefs.getFloat("magBiasY", 0.0f);
    calibration.magBias[2] = prefs.getFloat("magBiasZ", 0.0f);
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            char key[16];
            snprintf(key, sizeof(key), "magSI%d%d", i, j);
            calibration.magSoftIron[i][j] = prefs.getFloat(key, (i == j) ? 1.0f : 0.0f);
        }
    }
    
    // Load checksum
    calibration.checksum = prefs.getUShort("checksum", 0);
    
    prefs.end();
    
    // Validate checksum
    if (!calibration.isChecksumValid()) {
        DEBUG_ERROR("Calibration checksum invalid - data corrupted");
        return false;
    }
    
    // Return true if checksum is valid, regardless of "calibrated" flag
    // This allows partial calibration to be loaded
    return calibration.isChecksumValid();
}

bool CalibrationManager::saveToPreferences() {
    if (!prefs.begin(PREFS_NAMESPACE, false)) { // Read-write mode
        DEBUG_ERROR("Failed to open Preferences for writing");
        return false;
    }
    
    // Update timestamp and checksum
    calibration.timestamp = millis() / 1000; // Seconds since boot (approximation)
    
    // Record current mounting orientation when calibration is saved
    calibration.calibrationMountingUpsideDown = SystemSettings::getConfig().mountUpsideDown;
    
    calibration.updateChecksum();
    
    // Save all data
    prefs.putBool("calibrated", calibration.calibrated);
    prefs.putBool("calMountFlip", calibration.calibrationMountingUpsideDown);
    prefs.putUInt("timestamp", calibration.timestamp);
    
    // Save accelerometer calibration
    prefs.putFloat("accelBiasX", calibration.accelBias[0]);
    prefs.putFloat("accelBiasY", calibration.accelBias[1]);
    prefs.putFloat("accelBiasZ", calibration.accelBias[2]);
    
    prefs.putFloat("accelScaleX", calibration.accelScale[0]);
    prefs.putFloat("accelScaleY", calibration.accelScale[1]);
    prefs.putFloat("accelScaleZ", calibration.accelScale[2]);
    
    // Save gyroscope calibration
    prefs.putFloat("gyroBiasX", calibration.gyroBias[0]);
    prefs.putFloat("gyroBiasY", calibration.gyroBias[1]);
    prefs.putFloat("gyroBiasZ", calibration.gyroBias[2]);
    
    // Save magnetometer calibration
    prefs.putFloat("magBiasX", calibration.magBias[0]);
    prefs.putFloat("magBiasY", calibration.magBias[1]);
    prefs.putFloat("magBiasZ", calibration.magBias[2]);
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            char key[16];
            snprintf(key, sizeof(key), "magSI%d%d", i, j);
            prefs.putFloat(key, calibration.magSoftIron[i][j]);
        }
    }
    
    // Save checksum
    prefs.putUShort("checksum", calibration.checksum);
    
    prefs.end();
    
    DEBUG_INFO("Calibration saved to Preferences");
    return true;
}

bool CalibrationManager::resetCalibration(bool saveToPrefs) {
    DEBUG_INFO("Resetting calibration to defaults...");
    
    calibration = IMUCalibration(); // Reset to defaults
    
    if (saveToPrefs) {
        if (!prefs.begin(PREFS_NAMESPACE, false)) {
            DEBUG_ERROR("Failed to clear Preferences");
            return false;
        }
        prefs.clear();
        prefs.end();
        DEBUG_INFO("Calibration cleared from Preferences");
    }
    
    return true;
}

// ============================================================================
// APPLY CALIBRATION
// ============================================================================

void CalibrationManager::applyAccel(float& ax, float& ay, float& az) {
    // Apply bias correction (Bias is in G, input is G)
    ax -= calibration.accelBias[0];
    ay -= calibration.accelBias[1];
    az -= calibration.accelBias[2];
    
    // Apply scale factor correction (unitless)
    ax *= calibration.accelScale[0];
    ay *= calibration.accelScale[1];
    az *= calibration.accelScale[2];
}

void CalibrationManager::applyGyro(float& gx, float& gy, float& gz) {
    // Apply bias correction (Bias is in deg/s, input is deg/s)
    gx -= calibration.gyroBias[0];
    gy -= calibration.gyroBias[1];
    gz -= calibration.gyroBias[2];
}

void CalibrationManager::applyMag(float& mx, float& my, float& mz) {
    // Apply even if not fully calibrated (mag can be calibrated separately)
    
    // Step 1: Apply Hard Iron (bias) correction
    float x = mx - calibration.magBias[0];
    float y = my - calibration.magBias[1];
    float z = mz - calibration.magBias[2];
    
    // Step 2: Apply Soft Iron (matrix) correction
    mx = calibration.magSoftIron[0][0] * x + 
         calibration.magSoftIron[0][1] * y + 
         calibration.magSoftIron[0][2] * z;
    my = calibration.magSoftIron[1][0] * x + 
         calibration.magSoftIron[1][1] * y + 
         calibration.magSoftIron[1][2] * z;
    mz = calibration.magSoftIron[2][0] * x + 
         calibration.magSoftIron[2][1] * y + 
         calibration.magSoftIron[2][2] * z;
}

// ============================================================================
// DATA ACCESS
// ============================================================================

const IMUCalibration& CalibrationManager::getCalibration() {
    return calibration;
}

void CalibrationManager::printCalibration() {
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.println("   Current IMU Calibration");
    DEBUG_SERIAL.println("========================================");
    
    if (!isCalibrated() && !calibration.isChecksumValid()) {
        DEBUG_SERIAL.println("STATUS: NOT CALIBRATED");
        DEBUG_SERIAL.println("========================================");
        return;
    }
    
    if (calibration.calibrated) {
        DEBUG_SERIAL.println("STATUS: FULLY CALIBRATED");
    } else {
        DEBUG_SERIAL.println("STATUS: PARTIALLY CALIBRATED");
    }
    DEBUG_SERIAL.printf("Timestamp: %lu seconds\n", calibration.timestamp);
    DEBUG_SERIAL.printf("Checksum: 0x%04X (valid: %s)\n", 
                       calibration.checksum, 
                       calibration.isChecksumValid() ? "YES" : "NO");
    DEBUG_SERIAL.println();
    
    // Accelerometer
    DEBUG_SERIAL.println("ACCELEROMETER:");
    DEBUG_SERIAL.printf("  Bias (g):   X=%.4f  Y=%.4f  Z=%.4f\n",
                       calibration.accelBias[0],
                       calibration.accelBias[1],
                       calibration.accelBias[2]);
    DEBUG_SERIAL.printf("  Scale:      X=%.4f  Y=%.4f  Z=%.4f\n",
                       calibration.accelScale[0],
                       calibration.accelScale[1],
                       calibration.accelScale[2]);
                       
    // Accel Quality Assessment
    bool accelGood = true;
    for(int i=0; i<3; i++) {
        if(fabs(calibration.accelScale[i] - 1.0f) > 0.05f) accelGood = false; // >5% scale error
        if(fabs(calibration.accelBias[i]) > 0.1f) accelGood = false; // >100mg bias
    }
    DEBUG_SERIAL.printf("  Quality:    %s\n", accelGood ? "GOOD (within typical range)" : "WARN (bias/scale large)");
    DEBUG_SERIAL.println();
    
    // Gyroscope
    DEBUG_SERIAL.println("GYROSCOPE:");
    DEBUG_SERIAL.printf("  Bias (°/s): X=%.4f  Y=%.4f  Z=%.4f\n",
                       calibration.gyroBias[0],
                       calibration.gyroBias[1],
                       calibration.gyroBias[2]);
                       
    // Gyro Quality Grading (based on BMI270 datasheet: ±500 mdps typical zero-rate offset)
    float maxBias = 0.0f;
    for(int i=0; i<3; i++) if(fabs(calibration.gyroBias[i]) > maxBias) maxBias = fabs(calibration.gyroBias[i]);
    
    const char* grade = "UNKNOWN";
    // Datasheet-aligned thresholds:
    if(maxBias < 0.1f) grade = "EXCELLENT (< 0.1 dps, well below datasheet)";
    else if(maxBias < 0.3f) grade = "GOOD (< 0.3 dps, better than typical)";
    else if(maxBias < 0.5f) grade = "FAIR (< 0.5 dps, within datasheet spec)";
    else grade = "POOR (> 0.5 dps, exceeds datasheet typical)";
    
    DEBUG_SERIAL.printf("  Quality:    %s\n", grade);
    
    // Magnetometer
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("MAGNETOMETER:");
    DEBUG_SERIAL.printf("  Hard Iron:  X=%.2f  Y=%.2f  Z=%.2f µT\n",
                       calibration.magBias[0],
                       calibration.magBias[1],
                       calibration.magBias[2]);
                       
    // Check if soft iron is identity (default) or calibrated
    bool siCalibrated = false;
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            float expected = (i==j) ? 1.0f : 0.0f;
            if (fabs(calibration.magSoftIron[i][j] - expected) > 0.001f) {
                siCalibrated = true;
                break;
            }
        }
        if(siCalibrated) break;
    }
    
    if (siCalibrated) {
        DEBUG_SERIAL.println("  Soft Iron:  CALIBRATED (Ellipsoid fit applied)");
        DEBUG_SERIAL.println("  Matrix:");
        for(int i=0; i<3; i++) {
            DEBUG_SERIAL.printf("    [%.3f  %.3f  %.3f]\n",
                               calibration.magSoftIron[i][0],
                               calibration.magSoftIron[i][1],
                               calibration.magSoftIron[i][2]);
        }
    } else {
        DEBUG_SERIAL.println("  Soft Iron:  DEFAULT (Identity matrix)");
    }

    DEBUG_SERIAL.println("========================================");
    return;
}

// ============================================================================
// CALIBRATION ROUTINES (Stubs - to be implemented in next phases)
// ============================================================================
// CALIBRATION ROUTINES (Stubs - to be implemented in next phases)
// ============================================================================

bool CalibrationManager::calibrateGyro() {
    if (!IMUManager::isAvailable()) {
        DEBUG_ERROR("IMU not available - cannot calibrate");
        return false;
    }
    
    // Reset gyro calibration in memory before starting
    calibration.gyroBias[0] = 0; calibration.gyroBias[1] = 0; calibration.gyroBias[2] = 0;

    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   GYROSCOPE CALIBRATION");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.println("This calibration measures gyroscope bias");
    DEBUG_SERIAL.println("while the device is completely stationary.");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("Instructions:");
    DEBUG_SERIAL.println("1. Place device on FLAT, STABLE surface");
    DEBUG_SERIAL.println("2. Do NOT touch or move the device");
    DEBUG_SERIAL.println("3. Keep surroundings vibration-free");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("Press ENTER when ready...");
    
    // Wait for user confirmation
    while (!DEBUG_SERIAL.available()) {
        Watchdog::feed();
        delay(10);
    }
    while (DEBUG_SERIAL.available()) {
        DEBUG_SERIAL.read(); // Clear buffer
    }
    DEBUG_SERIAL.println(" OK\n");
    
    // Collect samples - USE HEAP to avoid stack overflow
    const size_t SAMPLE_COUNT = 500;
    const uint32_t SAMPLE_INTERVAL_MS = 20; // 50Hz sampling
    
    // Allocate on heap (12KB total)
    float (*accelData)[3] = new float[SAMPLE_COUNT][3];
    float (*gyroData)[3] = new float[SAMPLE_COUNT][3];
    
    if (!accelData || !gyroData) {
        DEBUG_ERROR("Failed to allocate memory for calibration!");
        delete[] accelData;
        delete[] gyroData;
        return false;
    }
    
    DEBUG_SERIAL.println("Collecting 500 samples (10 seconds)...");
    DEBUG_SERIAL.println("Keep device STILL!");
    
    for (size_t i = 0; i < SAMPLE_COUNT; i++) {
        // Update IMU to get fresh data
        IMUManager::update();
        
        // Read raw data (uncalibrated)
        accelData[i][0] = IMUManager::getAccelX();
        accelData[i][1] = IMUManager::getAccelY();
        accelData[i][2] = IMUManager::getAccelZ();
        
        gyroData[i][0] = IMUManager::getGyroX() * RAD_TO_DEG;
        gyroData[i][1] = IMUManager::getGyroY() * RAD_TO_DEG;
        gyroData[i][2] = IMUManager::getGyroZ() * RAD_TO_DEG;
        
        // Progress indicator
        if ((i + 1) % 50 == 0) {
            DEBUG_SERIAL.printf("  %d/%d samples...\n", i + 1, SAMPLE_COUNT);
            Watchdog::feed(); // Feed during long collection
        }
        
        delay(SAMPLE_INTERVAL_MS);
    }
    
    DEBUG_SERIAL.println("✓ Data collection complete\n");
    
    DEBUG_SERIAL.println("✓ Data collection complete\n");
    
    // Validate Data Quality (Stationarity and Noise)
    if (!checkDataQuality(accelData, gyroData, SAMPLE_COUNT, true, true,
                          IMUManager::getCurrentAccelRange(), 
                          IMUManager::getCurrentGyroRange())) {
        DEBUG_ERROR("Calibration failed quality check!");
        DEBUG_ERROR("Please retry with device on stable surface");
        delete[] accelData;
        delete[] gyroData;
        
        // Prompt to retry or skip
        DEBUG_SERIAL.print("\n[R]etry or [S]kip calibration? ");
        while (!DEBUG_SERIAL.available()) {
            Watchdog::feed();
            delay(10);
        }
        char choice = DEBUG_SERIAL.read();
        while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
        DEBUG_SERIAL.println(choice);
        
        if (choice == 'r' || choice == 'R') {
            return calibrateGyro(); // Recursive retry
        } else {
            DEBUG_INFO("Gyroscope calibration skipped");
            return false;
        }
    }
    
    // Calculate gyro bias (mean)
    float gyroBias[3];
    calculateMean(gyroData, SAMPLE_COUNT, gyroBias);
    
    // Calculate noise (std dev) - purely for reporting now, validation already done
    float gyroStdDev[3];
    calculateStdDev(gyroData, SAMPLE_COUNT, gyroBias, gyroStdDev);
    
    // Save calibration
    calibration.gyroBias[0] = gyroBias[0];
    calibration.gyroBias[1] = gyroBias[1];
    calibration.gyroBias[2] = gyroBias[2];
    
    // Mark as calibrated (but may need accel too for full calibration)
    if (calibration.accelScale[0] == 1.0f && 
        calibration.accelBias[0] == 0.0f) {
        // Accel not calibrated yet - partial calibration
        DEBUG_INFO("Gyroscope calibrated (accelerometer still needs calibration)");
    } else {
        calibration.calibrated = true;
    }
    
    // Save to Preferences
    if (!saveToPreferences()) {
        DEBUG_ERROR("Failed to save calibration!");
        return false;
    }
    
    // Display results
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   GYROSCOPE CALIBRATION COMPLETE");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.printf("Bias (°/s):  X=%.4f  Y=%.4f  Z=%.4f\n",
                       gyroBias[0], gyroBias[1], gyroBias[2]);
    DEBUG_SERIAL.printf("Noise (°/s): X=%.4f  Y=%.4f  Z=%.4f\n",
                       gyroStdDev[0], gyroStdDev[1], gyroStdDev[2]);
    DEBUG_SERIAL.println("========================================\n");
    
    return true;
}

bool CalibrationManager::calibrateAccelSimple() {
    if (!IMUManager::isAvailable()) {
        DEBUG_ERROR("IMU not available - cannot calibrate");
        return false;
    }
    
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   ACCELEROMETER CALIBRATION (Simple)");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.println("This calibration measures accelerometer bias");
    DEBUG_SERIAL.println("while the device is flat (Z-axis pointing up).");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("Instructions:");
    DEBUG_SERIAL.println("1. Place device FLAT on stable surface");
    DEBUG_SERIAL.println("2. Z-axis should point UP (screen/components facing up)");
    DEBUG_SERIAL.println("3. Do NOT touch or move the device");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("Press ENTER when ready...");
    
    // Wait for user confirmation
    while (!DEBUG_SERIAL.available()) {
        Watchdog::feed();
        delay(10);
    }
    while (DEBUG_SERIAL.available()) {
        DEBUG_SERIAL.read();
    }
    DEBUG_SERIAL.println(" OK\n");
    
    // Reset accelerometer calibration in memory before starting
    calibration.accelBias[0] = 0; calibration.accelBias[1] = 0; calibration.accelBias[2] = 0;
    calibration.accelScale[0] = 1; calibration.accelScale[1] = 1; calibration.accelScale[2] = 1;

    // Collect samples - USE HEAP to avoid stack overflow
    const size_t SAMPLE_COUNT = 500;
    const uint32_t SAMPLE_INTERVAL_MS = 20;
    
    float (*accelData)[3] = new float[SAMPLE_COUNT][3];
    float (*gyroData)[3] = new float[SAMPLE_COUNT][3];
    
    if (!accelData || !gyroData) {
        DEBUG_ERROR("Failed to allocate memory!");
        delete[] accelData;
        delete[] gyroData;
        return false;
    }
    
    DEBUG_SERIAL.println("Collecting 500 samples (10 seconds)...");
    DEBUG_SERIAL.println("Keep device STILL!");
    
    for (size_t i = 0; i < SAMPLE_COUNT; i++) {
        IMUManager::update();
        
        // IMUManager returns m/s², convert back to G for calibration math
        accelData[i][0] = IMUManager::getAccelX() / 9.80665f;
        accelData[i][1] = IMUManager::getAccelY() / 9.80665f;
        accelData[i][2] = IMUManager::getAccelZ() / 9.80665f;
        
        gyroData[i][0] = IMUManager::getGyroX() * RAD_TO_DEG;
        gyroData[i][1] = IMUManager::getGyroY() * RAD_TO_DEG;
        gyroData[i][2] = IMUManager::getGyroZ() * RAD_TO_DEG;
        
        if ((i + 1) % 50 == 0) {
            DEBUG_SERIAL.printf("  %d/%d samples...\n", i + 1, SAMPLE_COUNT);
            Watchdog::feed();
        }
        
        delay(SAMPLE_INTERVAL_MS);
    }
    
    DEBUG_SERIAL.println("✓ Data collection complete\n");
    
    DEBUG_SERIAL.println("✓ Data collection complete\n");
    
    // Validate Data Quality (Stationarity and Noise)
    if (!checkDataQuality(accelData, gyroData, SAMPLE_COUNT, true, true,
                          IMUManager::getCurrentAccelRange(),
                          IMUManager::getCurrentGyroRange())) {
        DEBUG_ERROR("Calibration failed quality check!");
        delete[] accelData;
        delete[] gyroData;
        
        // Prompt to retry or skip
        DEBUG_SERIAL.print("\n[R]etry or [S]kip calibration? ");
        while (!DEBUG_SERIAL.available()) {
            Watchdog::feed();
            delay(10);
        }
        char choice = DEBUG_SERIAL.read();
        while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
        DEBUG_SERIAL.println(choice);
        
        if (choice == 'r' || choice == 'R') {
            return calibrateAccelSimple(); // Recursive retry
        } else {
            DEBUG_INFO("Accelerometer calibration skipped");
            return false;
        }
    }
    
    // Calculate mean
    float accelMean[3];
    calculateMean(accelData, SAMPLE_COUNT, accelMean);
    
    // Free arrays
    delete[] gyroData;
    delete[] accelData;
    
    // Auto-detect gravity axis (Simple Calibration)
    // Find axis with largest absolute value
    int gAxis = 0;
    float maxVal = 0;
    for(int i=0; i<3; i++) {
        if(fabs(accelMean[i]) > maxVal) {
            maxVal = fabs(accelMean[i]);
            gAxis = i;
        }
    }
    
    float target[3] = {0, 0, 0};
    // Round to nearest 1.0g (1.0 or -1.0)
    target[gAxis] = (accelMean[gAxis] > 0) ? 1.0f : -1.0f;
    
    DEBUG_INFO("Detected Gravity Axis: %c = %.2f g", 'X'+gAxis, target[gAxis]);
    
    // Calculate bias (Mean - Target)
    calibration.accelBias[0] = accelMean[0] - target[0];
    calibration.accelBias[1] = accelMean[1] - target[1];
    calibration.accelBias[2] = accelMean[2] - target[2];
    
    // Keep scale factors at 1.0 (not calibrated in simple method)
    calibration.accelScale[0] = 1.0f;
    calibration.accelScale[1] = 1.0f;
    calibration.accelScale[2] = 1.0f;
    
    // Validate magnitude is close to 1g
    float magnitude = sqrt(accelMean[0]*accelMean[0] + 
                          accelMean[1]*accelMean[1] + 
                          accelMean[2]*accelMean[2]);
    
    if (fabs(magnitude - 1.0f) > 0.1f) {
        DEBUG_WARN("Accelerometer magnitude unexpected: %.4f g (expected ~1.0g)", magnitude);
        DEBUG_WARN("Device may not be level or sensor may have issues");
        DEBUG_SERIAL.print("\nContinue anyway? [y/N]: ");
        
        while (!DEBUG_SERIAL.available()) {
            Watchdog::feed();
            delay(10);
        }
        char response = DEBUG_SERIAL.read();
        while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
        DEBUG_SERIAL.println(response);
        
        if (response != 'y' && response != 'Y') {
            DEBUG_INFO("Calibration cancelled");
            return false;
        }
    }
    
    // Mark as calibrated if gyro is also calibrated
    if (calibration.gyroBias[0] != 0.0f || 
       calibration.gyroBias[1] != 0.0f || 
        calibration.gyroBias[2] != 0.0f) {
        calibration.calibrated = true;
    }
    
    // Save to Preferences
    if (!saveToPreferences()) {
        DEBUG_ERROR("Failed to save calibration!");
        return false;
    }
    
    // Display results
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   ACCEL CALIBRATION COMPLETE (Simple)");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.printf("Bias (g):      X=%.4f  Y=%.4f  Z=%.4f\n",
                       calibration.accelBias[0], 
                       calibration.accelBias[1], 
                       calibration.accelBias[2]);
    DEBUG_SERIAL.printf("Magnitude:     %.4f g\n", magnitude);
    DEBUG_SERIAL.println("Note: Scale factors not calibrated (use 6-pos for scale)");
    DEBUG_SERIAL.println("========================================\n");
    
    return true;
}

bool CalibrationManager::calibrateAccel6Position() {
    if (!IMUManager::isAvailable()) {
        DEBUG_ERROR("IMU not available - cannot calibrate");
        return false;
    }
    
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   ACCELEROMETER CALIBRATION (6-Position)");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.println("This calibration measures accelerometer");
    DEBUG_SERIAL.println("bias AND scale factors using 6 orientations.");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("You will place the device in 6 positions:");
    DEBUG_SERIAL.println("  1. Z-up   (flat, screen up)");
    DEBUG_SERIAL.println("  2. Z-down (upside down)");
    DEBUG_SERIAL.println("  3. X-up   (left side up)");
    DEBUG_SERIAL.println("  4. X-down (right side up)");
    DEBUG_SERIAL.println("  5. Y-up   (top edge up)");
    DEBUG_SERIAL.println("  6. Y-down (bottom edge up)");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("Press ENTER to begin...");
    
    // Wait for user
    while (!DEBUG_SERIAL.available()) {
        Watchdog::feed();
        delay(10);
    }
    while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
    DEBUG_SERIAL.println();
    
    // Position labels and expected gravity direction
    // Position labels (generic, no physical assumptions)
    const char* positions[6] = {
        "Z-up   (+1g on Z)",
        "Z-down (-1g on Z)",
        "X-up   (+1g on X)",
        "X-down (-1g on X)",
        "Y-up   (+1g on Y)",
        "Y-down (-1g on Y)"
    };
    
    // Expected gravity vector for each position (±1g on one axis)
    float expectedGravity[6][3] = {
        { 0,  0,  1},  // Z-up
        { 0,  0, -1},  // Z-down
        { 1,  0,  0},  // X-up
        {-1,  0,  0},  // X-down
        { 0,  1,  0},  // Y-up
        { 0, -1,  0}   // Y-down
    };
    
    // ADJUST FOR MOUNTING ORIENTATION
    // When upside-down, Y and Z are inverted in getAccel() output
    // So we must invert Y and Z in expected values to match
    if (SystemSettings::getConfig().mountUpsideDown) {
        for (int i = 0; i < 6; i++) {
            expectedGravity[i][1] = -expectedGravity[i][1];  // Invert Y
            expectedGravity[i][2] = -expectedGravity[i][2];  // Invert Z
        }
        DEBUG_INFO("Calibration adjusted for UPSIDE DOWN mounting");
    }
    
    // Measured accelerations at each position
    float measurements[6][3];
    
    // Reset accelerometer calibration in memory before starting
    calibration.accelBias[0] = 0; calibration.accelBias[1] = 0; calibration.accelBias[2] = 0;
    calibration.accelScale[0] = 1; calibration.accelScale[1] = 1; calibration.accelScale[2] = 1;

    // Collect data for each position
    for (int pos = 0; pos < 6; pos++) {
        DEBUG_SERIAL.printf("\n--- Position %d/6: %s ---\n", pos + 1, positions[pos]);
        DEBUG_SERIAL.println("Place device in position and hold STABLE.");
        
        // Determine target axis and sign from expectedGravity array
        int targetAxis = 0;
        float targetSign = 0;
        
        if (expectedGravity[pos][0] != 0) { targetAxis = 0; targetSign = expectedGravity[pos][0]; }
        else if (expectedGravity[pos][1] != 0) { targetAxis = 1; targetSign = expectedGravity[pos][1]; }
        else if (expectedGravity[pos][2] != 0) { targetAxis = 2; targetSign = expectedGravity[pos][2]; }
        
        // STRICT CHECK: Wait for stable orientation before continuing
        if (!waitForStableOrientation(targetAxis, targetSign, 0.85f, 60000)) { // 60s timeout
            DEBUG_ERROR("Failed to detect correct position. Calibration aborted.");
            return false;
        }
        
        DEBUG_SERIAL.println(" OK\n");
        
        // Collect samples - USE HEAP
        const size_t SAMPLE_COUNT = 200;
        float (*accelData)[3] = new float[SAMPLE_COUNT][3];
        float (*gyroData)[3] = new float[SAMPLE_COUNT][3];
        
        if (!accelData || !gyroData) {
            DEBUG_ERROR("Memory allocation failed!");
            delete[] accelData;
            delete[] gyroData;
            return false;
        }
        
        DEBUG_SERIAL.println("Collecting data (4 seconds)...");
        
        for (size_t i = 0; i < SAMPLE_COUNT; i++) {
            IMUManager::update();
            
            // IMUManager returns m/s², convert back to G for calibration math
            // Calibration expects ~1.0 for 1g, not ~9.81
            accelData[i][0] = IMUManager::getAccelX() / 9.80665f;
            accelData[i][1] = IMUManager::getAccelY() / 9.80665f;
            accelData[i][2] = IMUManager::getAccelZ() / 9.80665f;
            
            gyroData[i][0] = IMUManager::getGyroX() * RAD_TO_DEG;
            gyroData[i][1] = IMUManager::getGyroY() * RAD_TO_DEG;
            gyroData[i][2] = IMUManager::getGyroZ() * RAD_TO_DEG;
            
            Watchdog::feed();  // Feed watchdog EVERY sample to prevent timeout during I2C errors
            delay(20); // 50Hz
        }
        
        // Validate Data Quality (Stationarity and Noise)
        // Validate Data Quality (Stationarity and Noise)
        if (!checkDataQuality(accelData, gyroData, SAMPLE_COUNT, true, true,
                              IMUManager::getCurrentAccelRange(),
                              IMUManager::getCurrentGyroRange())) {
            DEBUG_ERROR("Calibration failed quality check!");
            
            // Prompt to retry, ignore/use anyway, or cancel
            DEBUG_SERIAL.print("\n[R]etry position, [I]gnore (use data), or [C]ancel? ");
            while (!DEBUG_SERIAL.available()) {
                Watchdog::feed();
                delay(10);
            }
            char choice = DEBUG_SERIAL.read();
            while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
            DEBUG_SERIAL.println(choice);
            
            if (choice == 'r' || choice == 'R') {
                delete[] accelData;
                delete[] gyroData;
                pos--; // Retry same position
                continue;
            } else if (choice == 'c' || choice == 'C') {
                DEBUG_INFO("Calibration cancelled.");
                delete[] accelData;
                delete[] gyroData;
                return false;
            } else {
                DEBUG_WARN("Ignoring quality check warning - using data.");
                // Fall through to use data
            }
        }
        
        // Calculate mean for this position
        calculateMean(accelData, SAMPLE_COUNT, measurements[pos]);
        
        // Free arrays for this position
        delete[] accelData;
        delete[] gyroData;
        
        DEBUG_SERIAL.printf("✓ Measured: X=%.3f, Y=%.3f, Z=%.3f g\n",
                           measurements[pos][0],
                           measurements[pos][1],
                           measurements[pos][2]);
    }
    
    DEBUG_SERIAL.println("\n✓ All 6 positions collected!");
    DEBUG_SERIAL.println("Calculating calibration parameters...\n");
    
    // Solve for bias and scale using least squares
    // For each axis independently:
    // measured = bias + scale * actual
    // We have 2 measurements per axis (±1g), solve for bias and scale
    
    for (int axis = 0; axis < 3; axis++) {
        // Find measurements where this axis should be +1g and -1g
        float posReading = 0.0f, negReading = 0.0f;
        
        for (int pos = 0; pos < 6; pos++) {
            if (expectedGravity[pos][axis] == 1.0f) {
                posReading = measurements[pos][axis];
            } else if (expectedGravity[pos][axis] == -1.0f) {
                negReading = measurements[pos][axis];
            }
        }
        
        // Solve:
        // posReading = bias + scale * (+1)
        // negReading = bias + scale * (-1)
        // Adding: posReading + negReading = 2 * bias
        // Subtracting: posReading - negReading = 2 * scale
        
        calibration.accelBias[axis] = (posReading + negReading) / 2.0f;
        float rawScale = (posReading - negReading) / 2.0f;
        calibration.accelScale[axis] = 1.0f / rawScale; // Invert for correction
    }
    
    // Validate scale factors are reasonable (0.95 - 1.05)
    bool scaleAcceptable = true;
    for (int i = 0; i < 3; i++) {
        if (calibration.accelScale[i] < CalibrationThresholds::ACCEL_SCALE_MIN ||
            calibration.accelScale[i] > CalibrationThresholds::ACCEL_SCALE_MAX) {
            scaleAcceptable = false;
            break;
        }
    }
    
    if (!scaleAcceptable) {
        DEBUG_WARN("Scale factors outside expected range:");
        DEBUG_WARN("  X: %.4f  Y: %.4f  Z: %.4f",
                   calibration.accelScale[0],
                   calibration.accelScale[1],
                   calibration.accelScale[2]);
        DEBUG_WARN("  Expected: %.2f - %.2f",
                   CalibrationThresholds::ACCEL_SCALE_MIN,
                   CalibrationThresholds::ACCEL_SCALE_MAX);
        DEBUG_SERIAL.print("\nContinue anyway? [y/N]: ");
        
        while (!DEBUG_SERIAL.available()) {
            Watchdog::feed();
            delay(10);
        }
        char response = DEBUG_SERIAL.read();
        while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
        DEBUG_SERIAL.println(response);
        
        if (response != 'y' && response != 'Y') {
            DEBUG_INFO("Calibration cancelled");
            return false;
        }
    }
    
    // Mark as calibrated if gyro is also calibrated
    if (calibration.gyroBias[0] != 0.0f ||
        calibration.gyroBias[1] != 0.0f ||
        calibration.gyroBias[2] != 0.0f) {
        calibration.calibrated = true;
    }
    
    // Save to Preferences
    if (!saveToPreferences()) {
        DEBUG_ERROR("Failed to save calibration!");
        return false;
    }
    
    // Display results
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   ACCEL CALIBRATION COMPLETE (6-Pos)");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.printf("Bias (g):   X=%.4f  Y=%.4f  Z=%.4f\n",
                       calibration.accelBias[0],
                       calibration.accelBias[1],
                       calibration.accelBias[2]);
    DEBUG_SERIAL.printf("Scale:      X=%.4f  Y=%.4f  Z=%.4f\n",
                       calibration.accelScale[0],
                       calibration.accelScale[1],
                       calibration.accelScale[2]);
    DEBUG_SERIAL.println("========================================\n");
    
    return true;
}

bool CalibrationManager::calibrateFull() {
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   FULL IMU CALIBRATION");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.println("This will calibrate gyroscope and");
    DEBUG_SERIAL.println("accelerometer (6-position method).");
    DEBUG_SERIAL.println();
    
    // Step 1: Gyroscope
    if (!calibrateGyro()) {
        DEBUG_ERROR("Gyroscope calibration failed!");
        return false;
    }
    
    DEBUG_SERIAL.println("\n✓ Gyroscope calibration complete");
    DEBUG_SERIAL.println("Press ENTER to continue to accelerometer...");
    while (!DEBUG_SERIAL.available()) {
        Watchdog::feed();
        delay(10);
    }
    while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
    
    // Step 2: Accelerometer 6-position
    if (!calibrateAccel6Position()) {
        DEBUG_ERROR("Accelerometer calibration failed!");
        return false;
    }
    
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   FULL CALIBRATION COMPLETE!");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.println("Calibration data successfully SAVED to NVS.");
    DEBUG_SERIAL.println("New calibration has been loaded and is ACTIVE.");
    DEBUG_SERIAL.println("========================================\n");
    
    return true;
}

bool CalibrationManager::validateQuick() {
    if (!initialized || !IMUManager::isAvailable()) {
        return false;
    }
    
    DEBUG_INFO("Quick calibration validation (2 sec)...");
    
    // Collect 100 samples (2 seconds at 50Hz) - USE HEAP
    const size_t SAMPLE_COUNT = 100;
    float (*accelData)[3] = new float[SAMPLE_COUNT][3];
    float (*gyroData)[3] = new float[SAMPLE_COUNT][3];
    
    if (!accelData || !gyroData) {
        return false;
    }
    
    for (size_t i = 0; i < SAMPLE_COUNT; i++) {
        IMUManager::update();
        
        // Read calibrated data (m/s^2 and rad/s)
        accelData[i][0] = IMUManager::getAccelX();
        accelData[i][1] = IMUManager::getAccelY();
        accelData[i][2] = IMUManager::getAccelZ();
        
        gyroData[i][0] = IMUManager::getGyroX();
        gyroData[i][1] = IMUManager::getGyroY();
        gyroData[i][2] = IMUManager::getGyroZ();
        
        // Convert to Calibration Units (g and deg/s) for validaton
        const float MS2_TO_G = 1.0f / 9.80665f;
        accelData[i][0] *= MS2_TO_G;
        accelData[i][1] *= MS2_TO_G;
        accelData[i][2] *= MS2_TO_G;
        
        gyroData[i][0] *= RAD_TO_DEG;
        gyroData[i][1] *= RAD_TO_DEG;
        gyroData[i][2] *= RAD_TO_DEG;
        
        delay(20);
    }
    
    // Calculate stats
    float accelMean[3], accelStdDev[3];
    float gyroMean[3], gyroStdDev[3];
    
    calculateMean(accelData, SAMPLE_COUNT, accelMean);
    calculateStdDev(accelData, SAMPLE_COUNT, accelMean, accelStdDev);
    calculateMean(gyroData, SAMPLE_COUNT, gyroMean);
    calculateStdDev(gyroData, SAMPLE_COUNT, gyroMean, gyroStdDev);
    
    // Check accel magnitude close to 1g
    float accelMag = sqrt(accelMean[0]*accelMean[0] + 
                          accelMean[1]*accelMean[1] + 
                          accelMean[2]*accelMean[2]);
    bool accelMagOk = fabs(accelMag - 1.0f) < 0.1f;
    
    // Check gyro bias near zero
    bool gyroBiasOk = (fabs(gyroMean[0]) < CalibrationThresholds::GYRO_BIAS_MAX &&
                       fabs(gyroMean[1]) < CalibrationThresholds::GYRO_BIAS_MAX &&
                       fabs(gyroMean[2]) < CalibrationThresholds::GYRO_BIAS_MAX);
    
    // Check noise levels (use range-adaptive thresholds)
    float accelNoiseMax = CalibrationThresholds::getAccelNoiseMax(IMUManager::getCurrentAccelRange());
    float gyroNoiseMax = CalibrationThresholds::getGyroNoiseMax(IMUManager::getCurrentGyroRange());
    
    bool accelNoiseOk = (accelStdDev[0] < accelNoiseMax &&
                         accelStdDev[1] < accelNoiseMax &&
                         accelStdDev[2] < accelNoiseMax);
    
    bool gyroNoiseOk = (gyroStdDev[0] < gyroNoiseMax &&
                        gyroStdDev[1] < gyroNoiseMax &&
                        gyroStdDev[2] < gyroNoiseMax);
    
    bool passed = accelMagOk && gyroBiasOk && accelNoiseOk && gyroNoiseOk;
    
    // Free memory
    delete[] accelData;
    delete[] gyroData;
    
    if (passed) {
        DEBUG_INFO("✓ Calibration quality: GOOD");
    } else {
        DEBUG_WARN("⚠ Calibration quality degraded:");
        if (!accelMagOk) DEBUG_WARN("  Accel magnitude: %.3f g (expected ~1.0)", accelMag);
        if (!gyroBiasOk) DEBUG_WARN("  Gyro bias high: X=%.3f Y=%.3f Z=%.3f °/s", 
                                     gyroMean[0], gyroMean[1], gyroMean[2]);
    }
    
    return passed;
}

bool CalibrationManager::validateFull(bool printDetails) {
    // For now, just call validateQuick with details
    // Can be enhanced later with more comprehensive checks
    bool result = validateQuick();
    
    if (printDetails) {
        printCalibration();
    }
    
    return result;
}

// ============================================================================
// HELPER FUNCTIONS (Stubs - to be implemented in next phases)
// ============================================================================

bool CalibrationManager::collectStationarySamples(
    float accelData[][3], 
    float gyroData[][3], 
    size_t sampleCount,
    const char* instruction
) {
    if (!IMUManager::isAvailable()) {
        return false;
    }
    
    if (instruction != nullptr) {
        DEBUG_SERIAL.println(instruction);
    }
    
    const uint32_t SAMPLE_INTERVAL_MS = 20; // 50Hz
    
    for (size_t i = 0; i < sampleCount; i++) {
        IMUManager::update();
        
        // Get data (m/s^2 and rad/s)
        accelData[i][0] = IMUManager::getAccelX();
        accelData[i][1] = IMUManager::getAccelY();
        accelData[i][2] = IMUManager::getAccelZ();
        
        gyroData[i][0] = IMUManager::getGyroX();
        gyroData[i][1] = IMUManager::getGyroY();
        gyroData[i][2] = IMUManager::getGyroZ();
        
        // Convert to Calibration Units (g and deg/s)
        const float MS2_TO_G = 1.0f / 9.80665f;
        accelData[i][0] *= MS2_TO_G;
        accelData[i][1] *= MS2_TO_G;
        accelData[i][2] *= MS2_TO_G;
        
        gyroData[i][0] *= RAD_TO_DEG;
        gyroData[i][1] *= RAD_TO_DEG;
        gyroData[i][2] *= RAD_TO_DEG;
        
        delay(SAMPLE_INTERVAL_MS);
    }
    
    return true;
}

bool CalibrationManager::checkDeviceStationary(
    const float accelData[][3],
    const float gyroData[][3],
    size_t sampleCount
) {
    // Deprecated wrapper - redirects to new helper
    return checkDataQuality(accelData, gyroData, sampleCount, true, false); 
}

bool CalibrationManager::checkDataQuality(
    const float accelData[][3], 
    const float gyroData[][3], 
    size_t sampleCount,
    bool checkStationary,
    bool checkNoise,
    uint8_t accelRange,
    uint8_t gyroRange
) {
    if (checkStationary) {
        // Calculate variance of accelerometer magnitude
        float magSum = 0.0f;
        float magSqSum = 0.0f;
        
        for (size_t i = 0; i < sampleCount; i++) {
            float mag = sqrt(accelData[i][0] * accelData[i][0] +
                            accelData[i][1] * accelData[i][1] +
                            accelData[i][2] * accelData[i][2]);
            magSum += mag;
            magSqSum += mag * mag;
        }
        
        float magMean = magSum / sampleCount;
        float magVar = (magSqSum / sampleCount) - (magMean * magMean);
        float magStdDev = sqrt(magVar);
        
        // Get range-adaptive stability threshold
        float stabilityThreshold = CalibrationThresholds::getAccelStabilityMax(accelRange);
        
        if (magStdDev > stabilityThreshold) {
            DEBUG_WARN("Stability Check Failed: Accel variation %.4f g > %.4f (range %d)", 
                      magStdDev, stabilityThreshold, accelRange);
            return false;
        }
    }
    
    if (checkNoise) {
        float gyroMean[3], gyroStdDev[3];
        calculateMean(gyroData, sampleCount, gyroMean);
        calculateStdDev(gyroData, sampleCount, gyroMean, gyroStdDev);
        
        // Get range-adaptive gyro noise threshold
        float gyroNoiseMax = CalibrationThresholds::getGyroNoiseMax(gyroRange);
        
        // Check gyro noise
        for (int i = 0; i < 3; i++) {
            if (gyroStdDev[i] > gyroNoiseMax) {
                DEBUG_WARN("Noise Check Failed: Gyro axis %d noise %.4f > %.4f (range %d)", 
                           i, gyroStdDev[i], gyroNoiseMax, gyroRange);
                return false;
            }
        }
    }
    
    return true;
}

void CalibrationManager::calculateMean(
    const float data[][3],
    size_t sampleCount,
    float mean[3]
) {
    mean[0] = mean[1] = mean[2] = 0.0f;
    
    for (size_t i = 0; i < sampleCount; i++) {
        mean[0] += data[i][0];
        mean[1] += data[i][1];
        mean[2] += data[i][2];
    }
    
    mean[0] /= sampleCount;
    mean[1] /= sampleCount;
    mean[2] /= sampleCount;
}

void CalibrationManager::calculateStdDev(
    const float data[][3],
    size_t sampleCount,
    const float mean[3],
    float stdDev[3]
) {
    stdDev[0] = stdDev[1] = stdDev[2] = 0.0f;
    
    for (size_t i = 0; i < sampleCount; i++) {
        stdDev[0] += (data[i][0] - mean[0]) * (data[i][0] - mean[0]);
        stdDev[1] += (data[i][1] - mean[1]) * (data[i][1] - mean[1]);
        stdDev[2] += (data[i][2] - mean[2]) * (data[i][2] - mean[2]);
    }
    
    stdDev[0] = sqrt(stdDev[0] / sampleCount);
    stdDev[1] = sqrt(stdDev[1] / sampleCount);
    stdDev[2] = sqrt(stdDev[2] / sampleCount);
}

// Helper to wait for stable orientation on a specific axis
bool CalibrationManager::waitForStableOrientation(int axis, float target, float threshold, unsigned long timeout) {
    unsigned long startTime = millis();
    int stableCount = 0;
    const int REQUIRED_STABLE_SAMPLES = 20; // 0.4s @ 20ms delay
    
    DEBUG_SERIAL.println("   Waiting for correct orientation...");
    // Fix print to be mathematically correct (e.g. Z < -0.85)
    DEBUG_SERIAL.printf("   Target: %s %c %.2f g\n", 
                       (axis==0)?"X":(axis==1)?"Y":"Z", 
                       target > 0 ? '>' : '<',
                       target > 0 ? threshold : -threshold);
    
    while (millis() - startTime < timeout) {
        Watchdog::feed();
        IMUManager::update();
        
        float val = 0;
        if (axis == 0) val = IMUManager::getAccelX();
        else if (axis == 1) val = IMUManager::getAccelY();
        else if (axis == 2) val = IMUManager::getAccelZ();
        
        // Convert m/s² to G for threshold comparison
        val /= 9.80665f;
        
        // Check if value matches target sign and magnitude
        bool match = false;
        if (target > 0 && val > threshold) match = true;
        if (target < 0 && val < -threshold) match = true;
        
        // Check stability (simple check - is it in the zone?)
        if (match) {
            stableCount++;
            if (stableCount % 5 == 0) DEBUG_SERIAL.print(".");
            
            if (stableCount >= REQUIRED_STABLE_SAMPLES) {
                DEBUG_SERIAL.println(" STABLE!");
                return true;
            }
        } else {
            if (stableCount > 0) DEBUG_SERIAL.print("!"); // Lost stability
            stableCount = 0;
            
            // Print live feedback occasionally
            static long lastPrint = 0;
            if (millis() - lastPrint > 500) {
                 DEBUG_SERIAL.printf("\r   Adjust: X=%.2f Y=%.2f Z=%.2f   ", 
                                    IMUManager::getAccelX(), 
                                    IMUManager::getAccelY(), 
                                    IMUManager::getAccelZ());
                 lastPrint = millis();
            }
        }
        
        delay(20);
    }
    
    DEBUG_SERIAL.println("\n   Timeout waiting for orientation.");
    return false;
}
// ============================================================================
// MAGNETOMETER CALIBRATION
// ============================================================================

// ============================================================================
// ROBUSTNESS HELPERS
// ============================================================================

bool CalibrationManager::readMagWithRetry(float& mx, float& my, float& mz) {
    const int MAX_RETRIES = 5;
    const int RETRY_DELAY_MS = 20;
    
    // Use IMUManager's I2C error tracking to detect read failures
    // Note: getMagX/Y/Z returns cached values if update() fails, so we must check error count
    
    uint32_t startErrors = IMUManager::getI2CErrorCount();
    IMUManager::update();
    uint32_t endErrors = IMUManager::getI2CErrorCount();
    
    bool i2cSuccess = (endErrors == startErrors);
    
    // Also check for suspicious zero data
    mx = IMUManager::getMagX();
    my = IMUManager::getMagY();
    mz = IMUManager::getMagZ();
    bool notZero = (mx != 0.0f || my != 0.0f || mz != 0.0f);
    
    if (i2cSuccess && notZero) {
        diagnostics.successfulI2CReads++;
        return true;
    }
    
    // First attempt failed - count it
    diagnostics.failedI2CReads++;
    
    // Retry Loop
    for (int i = 0; i < MAX_RETRIES; i++) {
        diagnostics.retryAttempts++;
        
        Watchdog::feed();
        delay(RETRY_DELAY_MS);
        
        startErrors = IMUManager::getI2CErrorCount();
        IMUManager::update();
        endErrors = IMUManager::getI2CErrorCount();
        
        mx = IMUManager::getMagX();
        my = IMUManager::getMagY();
        mz = IMUManager::getMagZ();
        
        if (endErrors == startErrors && (mx != 0.0f || my != 0.0f || mz != 0.0f)) {
            diagnostics.successfulI2CReads++;
            return true;
        }
        
        // Retry also failed
        diagnostics.failedI2CReads++;
    }
    
    return false;
}

// Structure for saving/resuming chunked calibration
struct CalibrationProgress {
    static const int MAX_SAMPLES = 3000;
    int samplesCollected;
    float samples[MAX_SAMPLES][3];
    bool isValid;
    uint32_t saveCount;  // Increments each save, used instead of timestamp
    
    void save() {
        unsigned long saveStart = millis();
        Watchdog::feed(); // Feed before potentially slow flash write
        
        File f = LittleFS.open("/cal_progress.bin", "w");
        if (f) {
            // Write in chunks to allow watchdog feeding during large writes
            const size_t CHUNK_SIZE = 4096;
            uint8_t* data = (uint8_t*)this;
            size_t remaining = sizeof(CalibrationProgress);
            size_t offset = 0;
            
            while (remaining > 0) {
                size_t toWrite = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
                f.write(data + offset, toWrite);
                offset += toWrite;
                remaining -= toWrite;
                Watchdog::feed(); // Feed during write
            }
            
            f.close();
            unsigned long saveDuration = millis() - saveStart;
            DEBUG_INFO("Calibration progress saved: %d samples (%lu ms)", samplesCollected, saveDuration);
            if (saveDuration > 1000) {
                DEBUG_WARN("Slow save detected: %lu ms", saveDuration);
            }
        } else {
            DEBUG_ERROR("Failed to save calibration progress");
        }
    }
    
    bool load() {
        if (!LittleFS.exists("/cal_progress.bin")) return false;
        
        File f = LittleFS.open("/cal_progress.bin", "r");
        if (!f) return false;
        
        if (f.size() != sizeof(CalibrationProgress)) {
            f.close();
            return false;
        }
        
        f.read((uint8_t*)this, sizeof(CalibrationProgress));
        f.close();
        
        // Check validity - simple flag check
        if (!isValid) return false;
        
        // Sanity check on sample count
        if (samplesCollected < 0 || samplesCollected >= MAX_SAMPLES) {
            return false;
        }
        
        return true;
    }
    
    void clear() {
        LittleFS.remove("/cal_progress.bin");
    }
};


bool CalibrationManager::calibrateMagSimple() {
    if (!IMUManager::isAvailable()) {
        DEBUG_ERROR("IMU not available - cannot calibrate");
        return false;
    }
    
    // CRITICAL: Extend watchdog timeout during calibration
    // BMI270 I2C operations can block for up to 15 seconds during errors
    Watchdog::setExtendedTimeout(30);
    
    // Reset Diagnostics
    diagnostics.reset();
    diagnostics.logPhase("mag_simple_init");
    
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   MAGNETOMETER CALIBRATION (Simple)");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.println("This calibration determines Hard Iron offsets");
    DEBUG_SERIAL.println("(bias) using min/max method.");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("Instructions:");
    DEBUG_SERIAL.println("1. Rotate device slowly in FIGURE-8 patterns");
    DEBUG_SERIAL.println("2. Cover all orientations (pitch, roll, yaw)");
    DEBUG_SERIAL.println("3. Keep moving for 30 seconds");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("Press ENTER when ready...");
    
    while (!DEBUG_SERIAL.available()) {
        Watchdog::feed();
        delay(10);
    }
    while (DEBUG_SERIAL.available()) {
        DEBUG_SERIAL.read();
    }
    DEBUG_SERIAL.println(" OK\n");
    
    calibration.magBias[0] = calibration.magBias[1] = calibration.magBias[2] = 0.0f;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            calibration.magSoftIron[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    DEBUG_SERIAL.println("Collecting samples for 30 seconds...");
    DEBUG_SERIAL.println("Keep rotating the device!");
    
    diagnostics.logPhase("mag_simple_collection");
    
    float minMag[3] = {999999, 999999, 999999};
    float maxMag[3] = {-999999, -999999, -999999};
    
    unsigned long startTime = millis();
    const unsigned long DURATION_MS = 30000;
    int sampleCount = 0;
    unsigned long lastRecoveryTime = millis();
    const unsigned long RECOVERY_INTERVAL_MS = 10000; // Recovery pause every 10 seconds
    const int MAX_CONSECUTIVE_FAILURES = 50;
    int consecutiveFailures = 0;
    
    while (millis() - startTime < DURATION_MS) {
        Watchdog::feed(); // Feed every loop
        diagnostics.watchdogFeeds++;
        
        // CRITICAL: Give magnetometer periodic recovery time to prevent I2C lockup
        if (millis() - lastRecoveryTime > RECOVERY_INTERVAL_MS) {
            DEBUG_SERIAL.println("  [Sensor recovery pause...]");
            delay(100); // 100ms rest for BMM150
            lastRecoveryTime = millis();
        }
        
        float mx, my, mz;
        if (readMagWithRetry(mx, my, mz)) {
            consecutiveFailures = 0;
            
            if(mx < minMag[0]) minMag[0] = mx;
            if(mx > maxMag[0]) maxMag[0] = mx;
            if(my < minMag[1]) minMag[1] = my;
            if(my > maxMag[1]) maxMag[1] = my;
            if(mz < minMag[2]) minMag[2] = mz;
            if(mz > maxMag[2]) maxMag[2] = mz;
            
            sampleCount++;
            diagnostics.totalSamples++;
        } else {
            consecutiveFailures++;
            if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                DEBUG_ERROR("CRITICAL: I2C bus lockup detected! Aborting calibration.");
                diagnostics.print();
                Watchdog::restoreDefaultTimeout();
                return false;
            }
        }
        
        if(sampleCount > 0 && sampleCount % 50 == 0) { // Log every second (50 * 20ms = 1000ms)
            int elapsed = (millis() - startTime) / 1000;
            DEBUG_SERIAL.printf("  %d/%d seconds...\n", elapsed, DURATION_MS/1000);
        }
        
        delay(20);
    }
    
    DEBUG_SERIAL.println("✓ Data collection complete\n");
    
    // Check if ANY samples were collected
    if (sampleCount == 0) {
        DEBUG_ERROR("CRITICAL: All magnetometer reads failed!");
        DEBUG_ERROR("Check I2C connections and sensor health");
        diagnostics.print();
        Watchdog::restoreDefaultTimeout();
        return false;
    }
    
    diagnostics.print(); // Show diagnostics
    
    // Check for sufficient motion (Range check)
    // 20uT is a conservative minimum range (earth field is ~25-65uT)
    bool motionDetected = true;
    for(int i = 0; i < 3; i++) {
        if((maxMag[i] - minMag[i]) < 20.0f) {
            motionDetected = false;
            break;
        }
    }

    if (!motionDetected) {
        DEBUG_WARN("Insufficient motion detected!");
        DEBUG_WARN("Ensure you rotate the device covering all axes.");
        
         // Prompt to retry or skip
        DEBUG_SERIAL.print("\n[R]etry or [S]kip calibration? ");
        while (!DEBUG_SERIAL.available()) {
            Watchdog::feed();
            delay(10);
        }
        char choice = DEBUG_SERIAL.read();
        while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
        DEBUG_SERIAL.println(choice);
        
        if (choice == 'r' || choice == 'R') {
            return calibrateMagSimple(); // Recursive retry (timeout already extended)
        } else {
            DEBUG_INFO("Magnetometer calibration skipped");
            Watchdog::restoreDefaultTimeout();
            return false;
        }
    }

    for(int i = 0; i < 3; i++) {
        calibration.magBias[i] = (maxMag[i] + minMag[i]) / 2.0f;
    }
    
    if (!saveToPreferences()) {
        DEBUG_ERROR("Failed to save calibration!");
        Watchdog::restoreDefaultTimeout();
        return false;
    }
    
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   MAG CALIBRATION COMPLETE (Simple)");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.printf("Hard Iron (µT):  X=%.2f  Y=%.2f  Z=%.2f\n",
                       calibration.magBias[0],
                       calibration.magBias[1],
                       calibration.magBias[2]);
    DEBUG_SERIAL.printf("Range (µT):      X=%.0f  Y=%.0f  Z=%.0f\n",
                       maxMag[0] - minMag[0],
                       maxMag[1] - minMag[1],
                       maxMag[2] - minMag[2]);
    DEBUG_SERIAL.println("Soft Iron: Not calibrated (use Precision for soft iron)");
    DEBUG_SERIAL.println("========================================\n");
    
    Watchdog::restoreDefaultTimeout();
    return true;
}

bool CalibrationManager::calibrateMagPrecision() {
    if (!IMUManager::isAvailable()) {
        DEBUG_ERROR("IMU not available - cannot calibrate");
        return false;
    }
    
    // CRITICAL: Extend watchdog timeout during calibration
    // BMI270 I2C operations can block for up to 15 seconds during errors
    Watchdog::setExtendedTimeout(30);
    
    // Reset Diagnostics
    diagnostics.reset();
    diagnostics.logPhase("mag_precision_init");
    
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   MAGNETOMETER CALIBRATION (Precision)");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.println("This calibration determines Hard Iron (bias)");
    DEBUG_SERIAL.println("AND Soft Iron (correction matrix) using");
    DEBUG_SERIAL.println("ellipsoid least-squares fit.");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("Instructions:");
    DEBUG_SERIAL.println("1. Rotate device SLOWLY covering full sphere");
    DEBUG_SERIAL.println("2. Try to get uniform coverage of all orientations");
    DEBUG_SERIAL.println("3. Collection takes 60 seconds");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("Press ENTER when ready...");
    
    while (!DEBUG_SERIAL.available()) {
        Watchdog::feed();
        delay(10);
    }
    while (DEBUG_SERIAL.available()) {
        DEBUG_SERIAL.read();
    }
    DEBUG_SERIAL.println(" OK\n");
    
    calibration.magBias[0] = calibration.magBias[1] = calibration.magBias[2] = 0.0f;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            calibration.magSoftIron[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Allocate progress structure on HEAP to avoid stack overflow (36KB+)
    CalibrationProgress* progress = new CalibrationProgress();
    if (!progress) {
        DEBUG_ERROR("Failed to allocate memory for calibration!");
        return false;
    }
    
    // Check for saved progress
    bool isResuming = false;
    if (progress->load()) {
        isResuming = true;
        DEBUG_SERIAL.printf("Resuming calibration from %d samples...\n", progress->samplesCollected);
        DEBUG_SERIAL.printf("Need %d more samples\n", CalibrationProgress::MAX_SAMPLES - progress->samplesCollected);
        delay(1000);
    } else {
        progress->samplesCollected = 0;
        progress->isValid = true;
        progress->saveCount = 0;
        DEBUG_SERIAL.println("Collecting samples for 60 seconds...");
        DEBUG_SERIAL.println("Keep rotating slowly!");
    }
    
    diagnostics.logPhase("mag_precision_collection");
    
    unsigned long startTime = millis();
    int remainingSamples = CalibrationProgress::MAX_SAMPLES - progress->samplesCollected;
    // Calculate time needed for remaining samples at ~20ms/sample + some margin
    unsigned long neededTime = remainingSamples * 30; // 30ms per sample (includes delays)
    unsigned long adjustedDuration = (neededTime < 60000) ? 60000 : neededTime;  // Min 60s
    
    unsigned long lastRecoveryTime = millis();
    const unsigned long RECOVERY_INTERVAL_MS = 10000; 
    const int SAVE_CHUNK_SIZE = 100; // Save every 100 samples
    const int MAX_CONSECUTIVE_FAILURES = 50; // Abort if 50 reads fail in a row
    int consecutiveFailures = 0;
    float lastMx = -9999.0f, lastMy = -9999.0f, lastMz = -9999.0f;
    
    while (progress->samplesCollected < CalibrationProgress::MAX_SAMPLES && 
           (millis() - startTime) < adjustedDuration) {
        
        Watchdog::feed(); 
        diagnostics.watchdogFeeds++;
        
        // Periodic recovery pause
        if (millis() - lastRecoveryTime > RECOVERY_INTERVAL_MS) {
            DEBUG_SERIAL.println("  [Sensor recovery pause...]");
            delay(100); 
            lastRecoveryTime = millis();
        }
        
        float mx, my, mz;

        if (readMagWithRetry(mx, my, mz)) {
            // DEDUPLICATION: Check if this is exactly the same as the last sample
            // This happens because Mag ODR (12.5Hz) < Loop Rate (50Hz)
            if (mx == lastMx && my == lastMy && mz == lastMz) {
                delay(10); // Wait for fresh data
                continue; 
            }
            lastMx = mx; lastMy = my; lastMz = mz;

            consecutiveFailures = 0; // Reset on success
            
            progress->samples[progress->samplesCollected][0] = mx;
            progress->samples[progress->samplesCollected][1] = my;
            progress->samples[progress->samplesCollected][2] = mz;
            
            progress->samplesCollected++;
            diagnostics.totalSamples++;
            
            // Chunked Saving
            if (progress->samplesCollected % SAVE_CHUNK_SIZE == 0) {
                progress->saveCount++;
                progress->save();
                Watchdog::feed(); // Feed after file I/O
            }
        } else {
            consecutiveFailures++;
            if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                DEBUG_ERROR("CRITICAL: I2C bus lockup detected! Aborting calibration.");
                DEBUG_ERROR("Consecutive failures: %d", consecutiveFailures);
                progress->save(); // Save what we have
                delete progress;
                Watchdog::restoreDefaultTimeout();
                return false;
            }
        }
        
        if(progress->samplesCollected > 0 && progress->samplesCollected % 50 == 0) { 
            int elapsed = (millis() - startTime) / 1000;
            DEBUG_SERIAL.printf("  %d samples (%d%%)...\n", 
                progress->samplesCollected, 
                (progress->samplesCollected * 100) / CalibrationProgress::MAX_SAMPLES);
        }
        
        delay(20);
    }
    
    DEBUG_SERIAL.printf("✓ Collected %d samples\n\n", progress->samplesCollected);
    
    if (progress->samplesCollected < 500) {
        DEBUG_ERROR("Too few samples collected (<500). Calibration aborted.");
        progress->clear();
        delete progress;
        Watchdog::restoreDefaultTimeout();
        return false;
    }

    diagnostics.logPhase("mag_precision_compute");
    DEBUG_SERIAL.println("Calculating ellipsoid fit... (this may take a moment)");
    Watchdog::feed(); // Feed before heavy computation
    
    EllipsoidFitResult fitResult;
    // Pass the array pointer from the struct
    if (!fitEllipsoidLS(progress->samples, progress->samplesCollected, fitResult)) {
        DEBUG_ERROR("Ellipsoid fit failed!");
        // Keep progress file in case they want to retry just the fit? 
        // For now, clear it to start over, as bad data might be the cause.
        progress->clear(); 
        delete progress;
        Watchdog::restoreDefaultTimeout();
        return false;
    }
    
    // Clear progress file on success
    progress->clear();
    delete progress;
    
    calibration.magBias[0] = fitResult.center[0];
    calibration.magBias[1] = fitResult.center[1];
    calibration.magBias[2] = fitResult.center[2];
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            calibration.magSoftIron[i][j] = fitResult.transform[i][j];
        }
    }
    
    diagnostics.print();
    
    if (fitResult.rms > CalibrationThresholds::MAG_FIT_RMS_MAX) {
        DEBUG_WARN("Fit quality poor (RMS: %.2f µT > %.2f µT threshold)",
                   fitResult.rms, CalibrationThresholds::MAG_FIT_RMS_MAX);
        
        // Prompt to retry or skip
        DEBUG_SERIAL.print("\n[R]etry or [A]ccept anyway? ");
        while (!DEBUG_SERIAL.available()) {
            Watchdog::feed();
            delay(10);
        }
        char choice = DEBUG_SERIAL.read();
        while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
        DEBUG_SERIAL.println(choice);
        
        if (choice == 'r' || choice == 'R') {
            return calibrateMagPrecision(); // Recursive retry (timeout already extended)
        }
        // else: Accept the calibration anyway and continue to save
        DEBUG_INFO("Accepting calibration with higher RMS error");
    }
    
    if (!saveToPreferences()) {
        DEBUG_ERROR("Failed to save calibration!");
        Watchdog::restoreDefaultTimeout();
        return false;
    }
    
    DEBUG_SERIAL.println("\n========================================");
    DEBUG_SERIAL.println("   MAG CALIBRATION COMPLETE (Precision)");
    DEBUG_SERIAL.println("========================================");
    DEBUG_SERIAL.printf("Hard Iron (µT):  X=%.2f  Y=%.2f  Z=%.2f\n",
                       calibration.magBias[0],
                       calibration.magBias[1],
                       calibration.magBias[2]);
    DEBUG_SERIAL.println("Soft Iron Matrix:");
    for(int i = 0; i < 3; i++) {
        DEBUG_SERIAL.printf("  [%.4f  %.4f  %.4f]\n",
                           calibration.magSoftIron[i][0],
                           calibration.magSoftIron[i][1],
                           calibration.magSoftIron[i][2]);
    }
    DEBUG_SERIAL.printf("Fit RMS Error:   %.2f µT\n", fitResult.rms);
    DEBUG_SERIAL.println("========================================\n");
    
    Watchdog::restoreDefaultTimeout();
    return true;
}
