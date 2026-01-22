#include "calibration_common.h"
#include "calibration_manager.h"  // For ExtendedCalibrationData
#include <EEPROM.h>
#include "imu_validation.h"

// Forward declarations of sensor read helpers defined in main.cpp
bool readAccel(float& ax, float& ay, float& az);
bool readGyro(float& gx, float& gy, float& gz);
bool readMagnetometer(float& mx, float& my, float& mz);

// Global calibration data instance
CalibrationData calibration;

// Size (in bytes) needed to store ExtendedCalibrationData
static const size_t EEPROM_EXTENDED_CAL_SIZE = sizeof(ExtendedCalibrationData);
static bool g_eeprom_initialized = false;

// Ensure EEPROM emulation is started once with an adequate size
static void ensureEepromInitialized() {
    if (!g_eeprom_initialized) {
        // On ESP32 the size must be <= 4096 and a multiple of 4
        // CRITICAL FIX: Must allocate enough for START_ADDR + DATA_SIZE
        // Struct is ~552 bytes, Start Addr is 256. Total needed > 808 bytes.
        // Old code only allocated 552 bytes, truncating data at offset 600+!
        const size_t eepromSize = 1024; 
        if (EEPROM.begin(eepromSize)) {
            g_eeprom_initialized = true;
        } else {
            Serial.println("[CAL] Error: EEPROM.begin() failed – calibration data will not persist");
        }
    }
}

void initCalibration() {
    // Initialize EEPROM first
    ensureEepromInitialized();
    // Initialize calibration data with default values
    resetCalibration();
}

bool loadCalibrationFromEEPROM() {
    ensureEepromInitialized();
    
    // Read ExtendedCalibrationData from EEPROM byte-by-byte for consistency with save
    extern ExtendedCalibrationData g_extended_cal_data;
    
    // Read data byte-by-byte to match the save function approach
    uint8_t* dataPtr = (uint8_t*)&g_extended_cal_data;
    size_t struct_size = sizeof(ExtendedCalibrationData);
    
    for (size_t i = 0; i < struct_size; i++) {
        dataPtr[i] = EEPROM.read(EEPROM_CAL_START_ADDR + i);
    }
    
    // Debug: Show what was loaded
    Serial.printf("[CAL DEBUG] Loaded from EEPROM: magic=0x%04X, version=0x%04X\n", 
                  g_extended_cal_data.magic, g_extended_cal_data.version);
    Serial.printf("[CAL DEBUG] Loaded accelBias: %.4f, %.4f, %.4f\n", 
                  g_extended_cal_data.cal_data.accelBias[0],
                  g_extended_cal_data.cal_data.accelBias[1], 
                  g_extended_cal_data.cal_data.accelBias[2]);
    Serial.printf("[CAL DEBUG] Loaded magBias: %.4f, %.4f, %.4f\n", 
                  g_extended_cal_data.cal_data.magBias[0],
                  g_extended_cal_data.cal_data.magBias[1], 
                  g_extended_cal_data.cal_data.magBias[2]);
    Serial.printf("[CAL DEBUG] Loaded magScale: %.4f, %.4f, %.4f\n", 
                  g_extended_cal_data.cal_data.magScale[0],
                  g_extended_cal_data.cal_data.magScale[1], 
                  g_extended_cal_data.cal_data.magScale[2]);
    
    // CRITICAL DEBUG: Inspect raw bytes at magBias location
    size_t magBiasOffset = offsetof(ExtendedCalibrationData, cal_data) + offsetof(CalibrationData, magBias);
    Serial.printf("[CAL DEBUG] Raw EEPROM bytes at magBias offset %d: ", magBiasOffset);
    for (int i = 0; i < 12; i++) {  // 3 floats = 12 bytes
        Serial.printf("%02X ", EEPROM.read(EEPROM_CAL_START_ADDR + magBiasOffset + i));
    }
    Serial.println();
    
    // Check if EEPROM has valid data (look for magic number)
    if (g_extended_cal_data.magic != EEPROM_CAL_MAGIC) {
        Serial.println("[CAL] No valid calibration data found in EEPROM");
        return false;
    }
    
    // Copy the nested calibration data to the global calibration structure
    calibration = g_extended_cal_data.cal_data;
    
    // CRITICAL FIX: Write the loaded data BACK to the EEPROM buffer
    // This ensures that if any other code (like ConfigStore) calls EEPROM.commit() later,
    // it won't overwrite our calibration data with zeros.
    // ESP32 EEPROM uses a shared buffer. If we rely only on loading into RAM, 
    // a subsequent commit of the shared buffer would wipe our data if the buffer logic is inconsistent.
    for (size_t i = 0; i < sizeof(ExtendedCalibrationData); i++) {
        EEPROM.write(EEPROM_CAL_START_ADDR + i, dataPtr[i]);
    }
    // NOTE: We do NOT call EEPROM.commit() here - just populate the buffer
    
    Serial.println("[CAL] Calibration data loaded from EEPROM");
    printCalibrationValues();
    
    return true;
}

bool saveCalibrationToEEPROM() {
    // Ensure initialized with FULL SIZE to prevent truncation
    if (!EEPROM.begin(1024)) {
        Serial.println("[CAL] Error: EEPROM.begin(1024) failed");
        // Try to continue anyway
    }
    
    ensureEepromInitialized();
    
    // Get reference to global extended calibration data
    extern ExtendedCalibrationData g_extended_cal_data;
    
    // CRITICAL DEBUG: Show what's in the GLOBAL calibration variable BEFORE copying
    Serial.printf("[CAL DEBUG] GLOBAL calibration.magBias BEFORE copy: %.4f, %.4f, %.4f\n",
                  calibration.magBias[0], calibration.magBias[1], calibration.magBias[2]);
    Serial.printf("[CAL DEBUG] GLOBAL calibration.magScale BEFORE copy: %.4f, %.4f, %.4f\n",
                  calibration.magScale[0], calibration.magScale[1], calibration.magScale[2]);
    Serial.printf("[CAL DEBUG] GLOBAL calibration.magCalibrated: %s\n",
                  calibration.magCalibrated ? "YES" : "NO");
    
    // CRITICAL: Synchronize local calibration data to global structure before saving
    g_extended_cal_data.cal_data = calibration;
    
    // CRITICAL DEBUG: Show what's in g_extended_cal_data AFTER copying
    Serial.printf("[CAL DEBUG] g_extended_cal_data.cal_data.magBias AFTER copy: %.4f, %.4f, %.4f\n",
                  g_extended_cal_data.cal_data.magBias[0],
                  g_extended_cal_data.cal_data.magBias[1],
                  g_extended_cal_data.cal_data.magBias[2]);
    
    // Ensure magic number and version are set
    g_extended_cal_data.magic = EEPROM_CAL_MAGIC;
    g_extended_cal_data.version = EEPROM_CAL_VERSION;
    
    // Debug: Check structure size
    size_t struct_size = sizeof(ExtendedCalibrationData);
    Serial.printf("[CAL DEBUG] ExtendedCalibrationData size: %d bytes (EEPROM allocated: %d bytes)\n", 
                  struct_size, EEPROM_CAL_SIZE);
    
    if (struct_size > EEPROM_CAL_SIZE) {
        Serial.printf("[CAL ERROR] Structure too large! Size: %d > Allocated: %d\n", struct_size, EEPROM_CAL_SIZE);
        return false;
    }
    
    // Debug: Show what we're about to save
    Serial.printf("[CAL DEBUG] Saving to EEPROM: magic=0x%04X, version=0x%04X\n", 
                  g_extended_cal_data.magic, g_extended_cal_data.version);
    
    // Debug: Show accelerometer calibration data
    Serial.printf("[CAL DEBUG] Accel - Bias: %.4f, %.4f, %.4f | Scale: %.4f, %.4f, %.4f | Calibrated: %s\n", 
                  g_extended_cal_data.cal_data.accelBias[0],
                  g_extended_cal_data.cal_data.accelBias[1], 
                  g_extended_cal_data.cal_data.accelBias[2],
                  g_extended_cal_data.cal_data.accelScale[0],
                  g_extended_cal_data.cal_data.accelScale[1],
                  g_extended_cal_data.cal_data.accelScale[2],
                  g_extended_cal_data.cal_data.accelCalibrated ? "YES" : "NO");
    
    // Debug: Show gyroscope calibration data
    Serial.printf("[CAL DEBUG] Gyro - Bias: %.4f, %.4f, %.4f | Temp: %.1f°C | Calibrated: %s\n", 
                  g_extended_cal_data.cal_data.gyroBias[0],
                  g_extended_cal_data.cal_data.gyroBias[1], 
                  g_extended_cal_data.cal_data.gyroBias[2],
                  g_extended_cal_data.cal_data.gyroCalTemp,
                  g_extended_cal_data.cal_data.gyroCalibrated ? "YES" : "NO");
    
    // Debug: Show magnetometer calibration data
    Serial.printf("[CAL DEBUG] Mag - Bias: %.4f, %.4f, %.4f | Scale: %.4f, %.4f, %.4f | Calibrated: %s\n", 
                  g_extended_cal_data.cal_data.magBias[0],
                  g_extended_cal_data.cal_data.magBias[1], 
                  g_extended_cal_data.cal_data.magBias[2],
                  g_extended_cal_data.cal_data.magScale[0],
                  g_extended_cal_data.cal_data.magScale[1],
                  g_extended_cal_data.cal_data.magScale[2],
                  g_extended_cal_data.cal_data.magCalibrated ? "YES" : "NO");
    
    // CRITICAL DEBUG: Show struct offsets
    Serial.printf("[CAL DEBUG] Struct offsets:\n");
    Serial.printf("[CAL DEBUG]   accelBias offset: %d\n", offsetof(CalibrationData, accelBias));
    Serial.printf("[CAL DEBUG]   magBias offset: %d\n", offsetof(CalibrationData, magBias));
    Serial.printf("[CAL DEBUG]   magScale offset: %d\n", offsetof(CalibrationData, magScale));
    Serial.printf("[CAL DEBUG]   CalibrationData size: %d\n", sizeof(CalibrationData));
    Serial.printf("[CAL DEBUG]   ExtendedCalibrationData size: %d\n", sizeof(ExtendedCalibrationData));
    Serial.printf("[CAL DEBUG]   cal_data offset in ExtendedCalibrationData: %d\n", 
                  offsetof(ExtendedCalibrationData, cal_data));
    
    // Update timestamp and checksum
    g_extended_cal_data.timestamp = millis();
    g_extended_cal_data.checksum = calculateChecksum(&g_extended_cal_data);
    
    // Write ExtendedCalibrationData to EEPROM
    uint8_t* dataPtr = (uint8_t*)&g_extended_cal_data;
    
    // Explicitly write byte-by-byte
    for (size_t i = 0; i < struct_size; i++) {
        EEPROM.write(EEPROM_CAL_START_ADDR + i, dataPtr[i]);
    }
    
    // Commit changes to EEPROM
    bool success = EEPROM.commit();
    
    if (success) {
        Serial.println("[CAL] Calibration data saved to EEPROM");
    } else {
        Serial.println("[CAL] Error: Failed to save calibration data to EEPROM");
        return false;
    }
    
    // Small delay for stability
    delay(100);
    
    // Verify magic and version
    ExtendedCalibrationData verifyData;
    EEPROM.get(EEPROM_CAL_START_ADDR, verifyData);
    
    bool magic_ok = (verifyData.magic == EEPROM_CAL_MAGIC);
    bool version_ok = (verifyData.version == EEPROM_CAL_VERSION);
    
    bool verified = magic_ok && version_ok;
    
    if (!verified) {
        Serial.println("[CAL] Warning: EEPROM verification failed");
        if (!magic_ok) Serial.println("[CAL] - Magic number mismatch");
        if (!version_ok) Serial.println("[CAL] - Version mismatch");
    } else {
        Serial.println("[CAL] EEPROM verification passed");
    }
    
    return success && verified;
}

void resetCalibration() {
    // Reset to default values (already set in the struct initialization)
    calibration = CalibrationData();
    
    Serial.println("[CAL] Calibration data reset to defaults");
}

const CalibrationData& getCalibrationData() {
    return calibration;
}

void printCalibrationValues() {
    Serial.println("[CAL] Current calibration values:");
    
    Serial.println("[CAL] Accelerometer:");
    Serial.print("[CAL]   Bias (mg): ");
    Serial.print(calibration.accelBias[0], 4); Serial.print(", ");
    Serial.print(calibration.accelBias[1], 4); Serial.print(", ");
    Serial.println(calibration.accelBias[2], 4);
    
    Serial.print("[CAL]   Scale: ");
    Serial.print(calibration.accelScale[0], 4); Serial.print(", ");
    Serial.print(calibration.accelScale[1], 4); Serial.print(", ");
    Serial.println(calibration.accelScale[2], 4);
    
    Serial.print("[CAL]   Method: ");
    switch (calibration.accelCalMethod) {
        case ACCEL_CAL_SIMPLE: Serial.println("Simple bias-only"); break;
        case ACCEL_CAL_SIX_POSITION: Serial.println("6-position with scale"); break;
        case ACCEL_CAL_CROSS_AXIS: Serial.println("6-position with cross-axis"); break;
        case ACCEL_CAL_ELLIPSOID: Serial.println("Ellipsoid fitting"); break;
        default: Serial.println("Unknown"); break;
    }
    
    Serial.println("[CAL] Gyroscope:");
    Serial.print("[CAL]   Bias (dps): ");
    Serial.print(calibration.gyroBias[0], 4); Serial.print(", ");
    Serial.print(calibration.gyroBias[1], 4); Serial.print(", ");
    Serial.println(calibration.gyroBias[2], 4);
    
    Serial.print("[CAL]   Temp Coefficients (dps/°C): ");
    Serial.print(calibration.gyroTempCoeff[0], 6); Serial.print(", ");
    Serial.print(calibration.gyroTempCoeff[1], 6); Serial.print(", ");
    Serial.println(calibration.gyroTempCoeff[2], 6);
    
    Serial.println("[CAL] Magnetometer:");
    Serial.print("[CAL]   Bias (uT): ");
    Serial.print(calibration.magBias[0], 4); Serial.print(", ");
    Serial.print(calibration.magBias[1], 4); Serial.print(", ");
    Serial.println(calibration.magBias[2], 4);
    
    Serial.print("[CAL]   Scale: ");
    Serial.print(calibration.magScale[0], 4); Serial.print(", ");
    Serial.print(calibration.magScale[1], 4); Serial.print(", ");
    Serial.println(calibration.magScale[2], 4);
    
    Serial.print("[CAL]   Declination: ");
    Serial.println(calibration.magDeclination, 4);
}

/**
 * Wait for user confirmation via serial input.
 * 
 * This function waits for the user to send 'y' or 'Y' over serial.
 * 
 * @param returnResult If true, returns a boolean result instead of void
 * @return True if user confirmed, false otherwise (only when returnResult is true)
 */
bool waitForUserConfirmation(bool returnResult) {
    while (true) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'y' || c == 'Y') {
                if (returnResult) return true;
                return true;  // Always return true with new signature
            } else if (c == 'n' || c == 'N') {
                if (returnResult) return false;
                return false;  // Always return false with new signature
            }
        }
        delay(10);  // Small delay to prevent CPU hogging
    }
}

bool collectBias(uint16_t sampleCount,
                float& axBias, float& ayBias, float& azBias,
                float& gxBias, float& gyBias, float& gzBias,
                bool enforceStillness) {
    
    // Reduce sample count to avoid memory issues
    const uint16_t actualSampleCount = min(sampleCount, (uint16_t)300);
    
    Serial.print("[CAL] Collecting bias data (");
    Serial.print(actualSampleCount);
    Serial.println(" samples)...");
    
    // Add a settling period before starting data collection
    Serial.println("[CAL] Waiting for sensor to settle (2 seconds)...");
    delay(2000);
    
    // Check if device is still before starting (only if enforceStillness is true)
    float accel[3], gyro[3];
    if (readAccel(accel[0], accel[1], accel[2]) && readGyro(gyro[0], gyro[1], gyro[2])) {
        if (enforceStillness) {
            // Use more tolerant thresholds for gyro calibration (1.0 dps for stability)
            bool isStill = isDeviceStill(accel, gyro, 0.5f, 1.0f);
            
            // Debug output for stillness detection
            if (!isStill) {
                Serial.println("[CAL] Device is not still. Place on a stable surface.");
                return false;
            }
        }
    } else {
        Serial.println("[CAL] Failed to read sensor data.");
        return false;
    }
    
    // Variables for streaming statistics
    // We'll use Welford's online algorithm for computing mean and variance
    float ax_mean = 0, ay_mean = 0, az_mean = 0;
    float gx_mean = 0, gy_mean = 0, gz_mean = 0;
    float ax_m2 = 0, ay_m2 = 0, az_m2 = 0;  // For variance calculation
    float gx_m2 = 0, gy_m2 = 0, gz_m2 = 0;
    uint16_t validSamples = 0;
    
    // Collect samples
    for (uint16_t i = 0; i < actualSampleCount; i++) {
        float ax, ay, az;
        float gx, gy, gz;
        
        // Read accelerometer and gyroscope data
        bool accelOk = readAccel(ax, ay, az);
        bool gyroOk = readGyro(gx, gy, gz);
        
        // Debug: Print every 20th sample
        if (i % 20 == 0) {
            Serial.print("[DBG] Sample "); Serial.print(i); Serial.print(": ");
            if (accelOk) {
                Serial.print("Accel: ");
                Serial.print(ax, 4); Serial.print(", ");
                Serial.print(ay, 4); Serial.print(", ");
                Serial.print(az, 4); Serial.print(" | ");
            } else {
                Serial.print("Accel FAIL | ");
            }
            if (gyroOk) {
                Serial.print("Gyro: ");
                Serial.print(gx, 4); Serial.print(", ");
                Serial.print(gy, 4); Serial.print(", ");
                Serial.print(gz, 4);
            } else {
                Serial.print("Gyro FAIL");
            }
            Serial.println();
        }
        
        // Only use the sample if both readings were successful
        if (accelOk && gyroOk) {
            // Update streaming statistics using Welford's algorithm
            validSamples++;
            
            // For accelerometer
            float ax_delta = ax - ax_mean;
            ax_mean += ax_delta / validSamples;
            float ax_delta2 = ax - ax_mean;
            ax_m2 += ax_delta * ax_delta2;
            
            float ay_delta = ay - ay_mean;
            ay_mean += ay_delta / validSamples;
            float ay_delta2 = ay - ay_mean;
            ay_m2 += ay_delta * ay_delta2;
            
            float az_delta = az - az_mean;
            az_mean += az_delta / validSamples;
            float az_delta2 = az - az_mean;
            az_m2 += az_delta * az_delta2;
            
            // For gyroscope
            float gx_delta = gx - gx_mean;
            gx_mean += gx_delta / validSamples;
            float gx_delta2 = gx - gx_mean;
            gx_m2 += gx_delta * gx_delta2;
            
            float gy_delta = gy - gy_mean;
            gy_mean += gy_delta / validSamples;
            float gy_delta2 = gy - gy_mean;
            gy_m2 += gy_delta * gy_delta2;
            
            float gz_delta = gz - gz_mean;
            gz_mean += gz_delta / validSamples;
            float gz_delta2 = gz - gz_mean;
            gz_m2 += gz_delta * gz_delta2;
        }
        
        // Check if device is still moving during calibration (only if enforceStillness is true)
        if (enforceStillness && (i % 10 == 0)) { // Only check every 10 samples to save time
            float accel_check[3], gyro_check[3];
            if (readAccel(accel_check[0], accel_check[1], accel_check[2]) && 
                readGyro(gyro_check[0], gyro_check[1], gyro_check[2])) {
                if (!isDeviceStill(accel_check, gyro_check, 0.5f, 1.0f)) { // Use relaxed thresholds here too
                    Serial.println("[CAL] Device moved during calibration. Try again.");
                    return false;
                }
            }
        }
        
        // Show progress every 10% of samples
        if (i % (actualSampleCount / 10) == 0) {
            Serial.print(".");
        }
        
        // Small delay between samples
        delay(5);
    }
    
    Serial.println();
    
    // Check if we collected enough valid samples
    if (validSamples < actualSampleCount * 0.5) { // Reduced threshold to 50%
        Serial.println("[CAL] Not enough valid samples collected. Aborting.");
        return false;
    }
    
    // Calculate variance
    float ax_var = (validSamples > 1) ? (ax_m2 / (validSamples - 1)) : 0;
    float ay_var = (validSamples > 1) ? (ay_m2 / (validSamples - 1)) : 0;
    float az_var = (validSamples > 1) ? (az_m2 / (validSamples - 1)) : 0;
    float gx_var = (validSamples > 1) ? (gx_m2 / (validSamples - 1)) : 0;
    float gy_var = (validSamples > 1) ? (gy_m2 / (validSamples - 1)) : 0;
    float gz_var = (validSamples > 1) ? (gz_m2 / (validSamples - 1)) : 0;
    
    // Calculate standard deviation
    float ax_std = sqrt(ax_var);
    float ay_std = sqrt(ay_var);
    float az_std = sqrt(az_var);
    float gx_std = sqrt(gx_var);
    float gy_std = sqrt(gy_var);
    float gz_std = sqrt(gz_var);
    
    // Set bias values to the calculated means
    axBias = ax_mean;
    ayBias = ay_mean;
    azBias = az_mean;
    gxBias = gx_mean;
    gyBias = gy_mean;
    gzBias = gz_mean;
    
    // Calculate accelerometer magnitude for quality check
    float accelMagnitude = sqrt(axBias*axBias + ayBias*ayBias + azBias*azBias);
    
    // Quality verification - check if accelerometer magnitude is reasonable
    if (fabs(accelMagnitude - 1.0f) > 0.3f) {
        Serial.print("[CAL] Warning: Accel magnitude (");
        Serial.print(accelMagnitude);
        Serial.println("g) is far from expected 1g. Calibration may be inaccurate.");
        // We don't fail here, just warn the user
    }
    
    // Print statistics for diagnostics
    Serial.print("[CAL] Accel StdDev (mg): ");
    Serial.print(ax_std, 3); Serial.print(", ");
    Serial.print(ay_std, 3); Serial.print(", ");
    Serial.println(az_std, 3);
    
    Serial.print("[CAL] Gyro StdDev (dps): ");
    Serial.print(gx_std, 3); Serial.print(", ");
    Serial.print(gy_std, 3); Serial.print(", ");
    Serial.println(gz_std, 3);
    
    Serial.print("[CAL] Successfully collected ");
    Serial.print(validSamples);
    Serial.println(" valid samples.");
    
    Serial.println("[CAL] Bias data collected successfully");
    return true;
}

/**
 * @brief Clear all calibration data from EEPROM
 * This function erases the calibration data area in EEPROM, removing any
 * dummy test data or corrupted calibration values.
 */
bool clearCalibrationEEPROM() {
    ensureEepromInitialized();
    
    Serial.println("[CAL] Clearing calibration data from EEPROM...");
    
    // Write zeros to the entire calibration data area
    for (int i = 0; i < sizeof(ExtendedCalibrationData); i++) {
        EEPROM.write(EEPROM_CAL_START_ADDR + i, 0);
    }
    
    if (!EEPROM.commit()) {
        Serial.println("[CAL] ERROR: Failed to clear EEPROM");
        return false;
    }
    
    // Reset global calibration data to defaults
    memset(&g_extended_cal_data, 0, sizeof(ExtendedCalibrationData));
    initCalibration(); // Reset to default values
    
    Serial.println("[CAL] EEPROM calibration data cleared successfully");
    return true;
}

CalibrationData calData;
bool isCalibrating = false; // Initialize flag

/**
 * @brief Get the string representation of a calibration status
 * 
 * @param status Calibration status
 * @return String representation of the status
 */
const char* getCalibrationStatusString(CalibrationProcessStatus status) {
    switch (status) {
        case CALIBRATION_NOT_STARTED: return "Not started";
        case CALIBRATION_IN_PROGRESS: return "In progress";
        case CALIBRATION_SUCCESSFUL: return "Successful";
        case CALIBRATION_FAILED: return "Failed";
        default: return "Unknown";
    }
}

bool validateWithRetryOption() {
    Serial.println("[CAL] === Validation Results ===");
    
    // Collect validation data
    const int numSamples = 100;
    const float MAX_ACCEPTABLE_BIAS = 1.0f;      // Relaxed from 0.5f to 1.0f for handheld use
    const float MAX_ACCEPTABLE_NOISE = 0.5f;     // Relaxed from 0.2f to 0.5f
    const float MAX_ACCEPTABLE_TEMP_SENS = 0.1f; // Relaxed from 0.05f
    float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;
    float x_var = 0.0f, y_var = 0.0f, z_var = 0.0f;
    float magnitude_sum = 0.0f, magnitude_var = 0.0f;
    int validSamples = 0;
    
    // First pass - collect mean
    for (int i = 0; i < numSamples && validSamples < numSamples; i++) {
        float x, y, z;
        if (readSensorData(x, y, z)) {
            // Apply current calibration
            applySensorCalibration(x, y, z);
            
            x_sum += x;
            y_sum += y;
            z_sum += z;
            
            // Calculate magnitude for accelerometer validation
            float magnitude = sqrt(x*x + y*y + z*z);
            magnitude_sum += magnitude;
            
            validSamples++;
        }
        delay(10);
    }
    
    if (validSamples < numSamples * 0.9) {
        Serial.println("[CAL] Validation failed - not enough valid samples");
        return false;
    }
    
    float x_mean = x_sum / validSamples;
    float y_mean = y_sum / validSamples;
    float z_mean = z_sum / validSamples;
    float magnitude_mean = magnitude_sum / validSamples;
    
    // Second pass - collect variance
    validSamples = 0;
    for (int i = 0; i < numSamples && validSamples < numSamples; i++) {
        float x, y, z;
        if (readSensorData(x, y, z)) {
            // Apply current calibration
            applySensorCalibration(x, y, z);
            
            x_var += (x - x_mean) * (x - x_mean);
            y_var += (y - y_mean) * (y - y_mean);
            z_var += (z - z_mean) * (z - z_mean);
            
            float magnitude = sqrt(x*x + y*y + z*z);
            magnitude_var += (magnitude - magnitude_mean) * (magnitude - magnitude_mean);
            
            validSamples++;
        }
        delay(10);
    }
    
    float x_std = sqrt(x_var / validSamples);
    float y_std = sqrt(y_var / validSamples);
    float z_std = sqrt(z_var / validSamples);
    float magnitude_std = sqrt(magnitude_var / validSamples);
    
    // Determine sensor type and appropriate validation criteria
    bool isAccelerometer = false;
    bool isGyroscope = false;
    
    // Add debugging to see what magnitude we're dealing with
    Serial.printf("[CAL DEBUG] Magnitude mean: %.3f, checking sensor type...\n", magnitude_mean);
    
    // Check for accelerometer: magnitude should be ~1g, ~1000mg, or ~16384 LSB (depending on units)
    // Expanded ranges to handle edge cases
    if ((magnitude_mean > 0.5 && magnitude_mean < 1.5) ||           // g units
        (magnitude_mean > 200 && magnitude_mean < 2000) ||          // mg units (expanded range)
        (magnitude_mean > 8000 && magnitude_mean < 25000)) {        // LSB units (~16384 LSB/g)
        isAccelerometer = true;
        Serial.printf("[CAL DEBUG] Detected as accelerometer (magnitude: %.3f)\n", magnitude_mean);
    }
    // Check for gyroscope: magnitude should be ~0 when stationary
    else if (magnitude_mean < 10.0) {  // Allow for some bias in any units
        isGyroscope = true;
        Serial.printf("[CAL DEBUG] Detected as gyroscope (magnitude: %.3f)\n", magnitude_mean);
    } else {
        Serial.printf("[CAL DEBUG] Detected as magnetometer/unknown (magnitude: %.3f)\n", magnitude_mean);
    }
    
    // Display results based on sensor type
    if (isAccelerometer) {
        // Accelerometer validation - normalize magnitude to g units for display
        float magnitude_g = magnitude_mean;
        float magnitude_std_g = magnitude_std;
        
        // Convert to g units if needed
        if (magnitude_mean > 8000) {
            // Likely LSB units, convert to g (assuming ±2g range, 16384 LSB/g)
            magnitude_g = magnitude_mean / 16384.0f;
            magnitude_std_g = magnitude_std / 16384.0f;
        } else if (magnitude_mean > 100) {  // Changed from 500 to 100 to catch typical accel readings
            // Likely mg units, convert to g
            magnitude_g = magnitude_mean / 1000.0f;
            magnitude_std_g = magnitude_std / 1000.0f;
        }
        
        Serial.printf("[CAL] Mean magnitude: %.3fg (ideal: 1.0g, max error: ±0.05g)\n", magnitude_g);
        Serial.printf("[CAL] Std deviation: %.3fg (threshold: 0.02g)\n", magnitude_std_g);
        
        bool magnitudeOk = (abs(magnitude_g - 1.0f) < 0.05f);
        bool stabilityOk = (magnitude_std_g < 0.02f);
        
        if (magnitudeOk && stabilityOk) {
            Serial.println("[CAL] Status: GOOD");
            Serial.println("[CAL] - Magnitude within 5% of 1.0g");
            Serial.println("[CAL] - Device stability excellent");
        } else if (magnitudeOk && magnitude_std_g < 0.05f) {
            Serial.println("[CAL] Status: MARGINAL");
            Serial.println("[CAL] - Magnitude acceptable");
            Serial.println("[CAL] - Device stability marginal");
        } else {
            Serial.println("[CAL] Status: POOR");
            if (!magnitudeOk) Serial.println("[CAL] - Magnitude error too high");
            if (!stabilityOk) Serial.println("[CAL] - Device stability poor");
        }
    } else if (isGyroscope) {
        // Gyroscope validation (convert to dps for display)
        float bias_magnitude = sqrt(x_mean*x_mean + y_mean*y_mean + z_mean*z_mean);
        float noise_magnitude = sqrt(x_std*x_std + y_std*y_std + z_std*z_std);
        
        Serial.printf("[CAL] Residual bias: %.3f dps (threshold: 0.5 dps)\n", bias_magnitude);
        Serial.printf("[CAL] Noise level: %.3f dps (threshold: 0.1 dps)\n", noise_magnitude);
        
        bool biasOk = (bias_magnitude < 0.5f);
        bool noiseOk = (noise_magnitude < 0.1f);
        
        if (biasOk && noiseOk) {
            Serial.println("[CAL] Status: GOOD");
            Serial.println("[CAL] - Bias compensation excellent");
            Serial.println("[CAL] - Noise level low");
        } else if (bias_magnitude < 1.0f && noise_magnitude < 0.2f) {
            Serial.println("[CAL] Status: MARGINAL");
            Serial.println("[CAL] - Bias compensation acceptable");
            Serial.println("[CAL] - Noise level acceptable");
        } else {
            Serial.println("[CAL] Status: POOR");
            if (!biasOk) Serial.println("[CAL] - Residual bias too high");
            if (!noiseOk) Serial.println("[CAL] - Noise level too high");
        }
    } else {
        // Magnetometer or unknown sensor
        float total_magnitude = sqrt(x_mean*x_mean + y_mean*y_mean + z_mean*z_mean);
        float total_noise = sqrt(x_std*x_std + y_std*y_std + z_std*z_std);
        
        Serial.printf("[CAL] Signal magnitude: %.3f µT\n", total_magnitude);
        Serial.printf("[CAL] Noise level: %.3f µT (threshold: 2.0 µT)\n", total_noise);
        
        bool signalOk = (total_magnitude > 10.0f); // Reasonable magnetic field strength
        bool noiseOk = (total_noise < 2.0f);
        
        if (signalOk && noiseOk) {
            Serial.println("[CAL] Status: GOOD");
            Serial.println("[CAL] - Signal strength adequate");
            Serial.println("[CAL] - Noise level acceptable");
        } else if (total_magnitude > 5.0f && total_noise < 5.0f) {
            Serial.println("[CAL] Status: MARGINAL");
            Serial.println("[CAL] - Signal strength marginal");
            Serial.println("[CAL] - Noise level marginal");
        } else {
            Serial.println("[CAL] Status: POOR");
            if (!signalOk) Serial.println("[CAL] - Signal strength too low");
            if (!noiseOk) Serial.println("[CAL] - Noise level too high");
        }
    }
    
    Serial.println();
    Serial.println("[CAL] Options:");
    Serial.println("[CAL] 'y' - Accept and save this calibration");
    Serial.println("[CAL] 'n' - Discard and restart calibration");
    
    return waitForUserConfirmation();
}

// -----------------------------------------------------------------------------
// Validation callback mechanism implementation
// -----------------------------------------------------------------------------

// Global function pointers (defined in header as extern)
ReadSensorDataFunc   g_readSensorData        = nullptr;
ApplySensorCalFunc   g_applySensorCalibration = nullptr;
CheckBiasFunc        g_checkBiasAcceptable    = nullptr;
CheckNoiseFunc       g_checkNoiseAcceptable   = nullptr;

void registerValidationCallbacks(ReadSensorDataFunc readFn,
                                 ApplySensorCalFunc applyFn,
                                 CheckBiasFunc biasFn,
                                 CheckNoiseFunc noiseFn) {
    g_readSensorData        = readFn;
    g_applySensorCalibration = applyFn;
    g_checkBiasAcceptable    = biasFn;
    g_checkNoiseAcceptable   = noiseFn;
}

// Wrapper helpers expected by validateWithRetryOption()
// These wrappers call the registered callbacks if available, otherwise fail.
bool readSensorData(float &x, float &y, float &z) {
    if (g_readSensorData) {
        return g_readSensorData(x, y, z);
    }
    // No callback registered
    return false;
}

void applySensorCalibration(float &x, float &y, float &z) {
    if (g_applySensorCalibration) {
        g_applySensorCalibration(x, y, z);
    }
}

bool checkBiasAcceptable(float x, float y, float z) {
    if (g_checkBiasAcceptable) {
        return g_checkBiasAcceptable(x, y, z);
    }
    // Default: always false (not acceptable)
    return false;
}

bool checkNoiseAcceptable(float x, float y, float z) {
    if (g_checkNoiseAcceptable) {
        return g_checkNoiseAcceptable(x, y, z);
    }
    return false;
}

// -----------------------------------------------------------------------------
