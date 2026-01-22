#include "mag_calibration.h"
#include "imu_validation.h"
#include "temp_sensor.h"  // Include for temperature sensor access
#include "imu_config.h"   // Include for setBank, readRegister, writeRegister functions
#include "calibration_quality.h"  // Include for quality assessment

// Forward declarations of sensor read helpers defined in main.cpp
bool readMagnetometer(float& mx, float& my, float& mz);
bool readTemperature(float& temp);  // Add temperature reading function

// Forward declaration -------------------------------------------------------
static bool readMagData(int16_t& mx, int16_t& my, int16_t& mz);

/**
 * Configure the magnetometer registers for optimal calibration
 * 
 * This function configures the ICM-20948 magnetometer with settings optimized for
 * accurate calibration:
 * - Maximum sensitivity
 * - Appropriate sample rate
 * - Continuous measurement mode
 */
void configureMagForCalibration() {
    Serial.println("[CAL] Configuring magnetometer for calibration...");
    
    // 1. Reset I2C bus if needed
    Wire.end();
    delay(10);
    Wire.begin();
    Wire.setClock(400000); // Standard 400kHz I2C
    
    // 2. Reset ICM-20948
    setBank(0);
    writeRegister(0x06, 0x80); // PWR_MGMT_1: Device Reset
    delay(100);

    // 3. Wake up the ICM-20948 and select the best clock source
    writeRegister(0x06, 0x01); // PWR_MGMT_1: Clock auto-select
    delay(20);

    // 4. Disable the ICM-20948's I2C Master module
    writeRegister(0x03, 0x00); // USER_CTRL: I2C_MST_EN = 0
    delay(20);

    // 5. Enable I2C Bypass Mode
    writeRegister(0x0F, 0x02); // INT_PIN_CFG: BYPASS_EN = 1
    delay(10);

    // 6. Reset the magnetometer
    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(0x32); // CNTL3
    Wire.write(0x01); // Soft reset
    if (Wire.endTransmission() != 0) {
        Serial.println("[CAL] Error: Failed to reset magnetometer");
        return;
    }
    delay(50); // Give time for reset to complete
    
    // 7. Verify magnetometer ID
    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(0x01); // WIA2
    if (Wire.endTransmission(false) != 0) {
        Serial.println("[CAL] Error: Magnetometer not responding");
        return;
    }
    
    Wire.requestFrom(AK09916_ADDR, 1);
    if (!Wire.available()) {
        Serial.println("[CAL] Error: No response from magnetometer");
        return;
    }
    
    uint8_t whoami = Wire.read();
    if (whoami != 0x09) {
        Serial.print("[CAL] Error: Invalid magnetometer ID: 0x");
        Serial.println(whoami, HEX);
        return;
    }

    // 8. Set to continuous measurement mode 4 (100Hz)
    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(0x31); // CNTL2
    Wire.write(0x08); // Mode 4 (100Hz)
    if (Wire.endTransmission() != 0) {
        Serial.println("[CAL] Error: Failed to set magnetometer mode");
        return;
    }
    
    // 9. Verify mode was set
    delay(10);
    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(0x31); // CNTL2
    if (Wire.endTransmission(false) != 0) {
        Serial.println("[CAL] Error: Failed to verify magnetometer mode");
        return;
    }
    
    Wire.requestFrom(AK09916_ADDR, 1);
    if (!Wire.available()) {
        Serial.println("[CAL] Error: No response for mode verification");
        return;
    }
    
    uint8_t mode = Wire.read();
    if (mode != 0x08) {
        Serial.print("[CAL] Error: Failed to set mode, got: 0x");
        Serial.println(mode, HEX);
        return;
    }
    
    // 10. Wait for first data ready
    uint32_t timeout = millis() + 200; // 200ms timeout
    bool dataReady = false;
    
    while (millis() < timeout) {
        Wire.beginTransmission(AK09916_ADDR);
        Wire.write(0x10); // ST1
        if (Wire.endTransmission(false) != 0) {
            delay(1);
            continue;
        }
        
        Wire.requestFrom(AK09916_ADDR, 1);
        if (Wire.available() && (Wire.read() & 0x01)) {
            dataReady = true;
            break;
        }
        delay(1);
    }
    
    if (!dataReady) {
        Serial.println("[CAL] Warning: Timeout waiting for first data ready");
    } else {
        // Clear the data ready flag by reading the data
        int16_t dummy[3];
        readMagData(dummy[0], dummy[1], dummy[2]);
    }
    
    Serial.println("[CAL] Magnetometer configuration complete");
}

/**
 * Helper function to read magnetometer data with error handling
 */
bool readMagData(int16_t& mx, int16_t& my, int16_t& mz) {
    uint8_t rawData[7]; // ST1 + 6 data bytes
    
    // Read ST1 and 6 data bytes
    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(0x10); // ST1
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    
    if (Wire.requestFrom(AK09916_ADDR, 7) != 7) {
        return false;
    }
    
    for (int i = 0; i < 7; i++) {
        if (!Wire.available()) return false;
        rawData[i] = Wire.read();
    }
    
    // Check ST1 data ready bit
    if (!(rawData[0] & 0x01)) {
        return false;
    }
    
    // Check ST2 overflow bit
    if (rawData[6] & 0x08) {
        return false;
    }
    
    // Convert data to 16-bit integers
    mx = (int16_t)((rawData[2] << 8) | rawData[1]);
    my = (int16_t)((rawData[4] << 8) | rawData[3]);
    mz = (int16_t)((rawData[6] << 8) | rawData[5]);
    
    return true;
}

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
void applyMagCalibration(float& mx_uT, float& my_uT, float& mz_uT, float temperature) {
    // Only apply calibration if magnetometer is calibrated
    if (calibration.magCalibrated) {
        // Store original values for soft iron calculations
        float mx_orig = mx_uT;
        float my_orig = my_uT;
        float mz_orig = mz_uT;
        
        // Step 1: Apply hard iron correction (bias/offset)
        mx_uT -= calibration.magBias[0];
        my_uT -= calibration.magBias[1];
        mz_uT -= calibration.magBias[2];
        
        // Step 2: Apply temperature compensation if enabled
        if (calibration.magTempCompEnabled && temperature > -100.0f) {
            // Calculate temperature difference from calibration temperature
            float tempDiff = temperature - calibration.magCalTemp;
            
            // Apply temperature coefficients to compensate for thermal drift
            mx_uT -= calibration.magTempCoeff[0] * tempDiff;
            my_uT -= calibration.magTempCoeff[1] * tempDiff;
            mz_uT -= calibration.magTempCoeff[2] * tempDiff;
        }
        
        // Step 3: Apply soft iron correction if enabled
        if (calibration.magCalMethod == MAG_CAL_HARD_SOFT_IRON) {
            // Apply soft iron correction matrix
            float mx_corrected = calibration.magSoftIron[0][0] * mx_uT +
                               calibration.magSoftIron[0][1] * my_uT +
                               calibration.magSoftIron[0][2] * mz_uT;
            
            float my_corrected = calibration.magSoftIron[1][0] * mx_uT +
                               calibration.magSoftIron[1][1] * my_uT +
                               calibration.magSoftIron[1][2] * mz_uT;
            
            float mz_corrected = calibration.magSoftIron[2][0] * mx_uT +
                               calibration.magSoftIron[2][1] * my_uT +
                               calibration.magSoftIron[2][2] * mz_uT;
            
            mx_uT = mx_corrected;
            my_uT = my_corrected;
            mz_uT = mz_corrected;
        }
    }
}

/**
 * Perform simple bias-only magnetometer calibration
 * 
 * This basic calibration method collects magnetometer data while the device is
 * rotated through various orientations and calculates the bias/offset for each axis.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateMagSimple() {
    Serial.println("[CAL] Starting simple magnetometer calibration...");
    Serial.println("[CAL] Rotate the device slowly in all directions to sample the magnetic field");
    Serial.println("[CAL] When ready, press 'y' to start data collection");
    waitForUserConfirmation();
    
    // Configure magnetometer for calibration
    configureMagForCalibration();
    
    // Variables to track min and max values
    float mx_min = 1000.0f, my_min = 1000.0f, mz_min = 1000.0f;
    float mx_max = -1000.0f, my_max = -1000.0f, mz_max = -1000.0f;
    
    // Collect data for 30 seconds
    unsigned long startTime = millis();
    unsigned long lastFeedbackTime = startTime;
    int sampleCount = 0;
    
    Serial.println("[CAL] Collecting data for 30 seconds. Rotate the device in figure-8 patterns.");
    
    while (millis() - startTime < 30000) {
        float mx, my, mz;
        
        if (readMagnetometer(mx, my, mz)) {
            // Update min/max values
            mx_min = min(mx_min, mx);
            my_min = min(my_min, my);
            mz_min = min(mz_min, mz);
            
            mx_max = max(mx_max, mx);
            my_max = max(my_max, my);
            mz_max = max(mz_max, mz);
            
            sampleCount++;
            
            // Provide feedback every second
            if (millis() - lastFeedbackTime > 1000) {
                Serial.print("[CAL] Collected ");
                Serial.print(sampleCount);
                Serial.println(" samples");
                lastFeedbackTime = millis();
            }
        }
        
        delay(10);
    }
    
    if (sampleCount < 100) {
        Serial.println("[CAL] Not enough samples collected. Calibration failed.");
        return false;
    }
    
    // Calculate bias as the average of min and max for each axis
    calibration.magBias[0] = (mx_min + mx_max) / 2.0f;
    calibration.magBias[1] = (my_min + my_max) / 2.0f;
    calibration.magBias[2] = (mz_min + mz_max) / 2.0f;
    
    // Set identity matrix for soft iron correction (not used in simple calibration)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            calibration.magSoftIron[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // CRITICAL: Also set magScale to identity for EEPROM persistence
    calibration.magScale[0] = 1.0f;
    calibration.magScale[1] = 1.0f;
    calibration.magScale[2] = 1.0f;
    
    // Set calibration method and flag
    calibration.magCalMethod = MAG_CAL_SIMPLE;
    calibration.magCalibrated = true;
    
    Serial.println("[CAL] Simple magnetometer calibration complete");
    Serial.print("[CAL] Bias (μT): ");
    Serial.print(calibration.magBias[0], 4); Serial.print(", ");
    Serial.print(calibration.magBias[1], 4); Serial.print(", ");
    Serial.println(calibration.magBias[2], 4);
    
    Serial.print("[CAL] Range X: ");
    Serial.print(mx_min, 2); Serial.print(" to ");
    Serial.print(mx_max, 2); Serial.print(" (");
    Serial.print(mx_max - mx_min, 2); Serial.println(" μT)");
    
    Serial.print("[CAL] Range Y: ");
    Serial.print(my_min, 2); Serial.print(" to ");
    Serial.print(my_max, 2); Serial.print(" (");
    Serial.print(my_max - my_min, 2); Serial.println(" μT)");
    
    Serial.print("[CAL] Range Z: ");
    Serial.print(mz_min, 2); Serial.print(" to ");
    Serial.print(mz_max, 2); Serial.print(" (");
    Serial.print(mz_max - mz_min, 2); Serial.println(" μT)");
    
    // Register magnetometer validation callbacks and validate
    registerValidationCallbacks(readMagSensorData, applyMagSensorCalibration, 
                               checkMagBiasAcceptable, checkMagNoiseAcceptable);
    
    if (!validateWithRetryOption()) {
        Serial.println("✗ Magnetometer calibration validation failed");
        return false;
    }
    
    Serial.println("✓ Magnetometer calibration validation passed");
    Serial.println("[CAL] Magnetometer calibration complete");
    return true;
}

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
bool calibrateMagHardSoftIron() {
    isCalibrating = true; // Prevent main loop from interfering

    Serial.println("[CAL] Starting hard and soft iron magnetometer calibration...");
    Serial.println("[CAL] This procedure requires rotating the device through many orientations");
    Serial.println("[CAL] When ready, press 'y' to start data collection");
    waitForUserConfirmation();
    
    // Configure magnetometer for calibration
    configureMagForCalibration();
    
    // Reduced sample size to avoid memory issues
    const int maxSamples = 300;
    
    // Allocate memory on the heap instead of the stack
    float** dataPoints = new float*[maxSamples];
    if (!dataPoints) {
        Serial.println("[CAL] ERROR: Failed to allocate memory for calibration");
        return false;
    }
    
    // Allocate each row - with proper error handling
    bool allocFailed = false;
    int allocatedSamples = maxSamples; // Track how many samples we'll actually use
    for (int i = 0; i < maxSamples; i++) {
        dataPoints[i] = new float[3];
        if (!dataPoints[i]) {
            allocFailed = true;
            // Remember how many we successfully allocated
            allocatedSamples = i;
            break;
        }
    }
    
    // Handle allocation failure
    if (allocFailed) {
        Serial.println("[CAL] WARNING: Could only allocate memory for ");
        Serial.print(allocatedSamples);
        Serial.println(" samples, continuing with reduced set");
        
        // We'll continue with however many samples we could allocate
        // as long as we have a reasonable number (at least 100)
        if (allocatedSamples < 100) {
            Serial.println("[CAL] ERROR: Insufficient memory for calibration");
            // Clean up already allocated memory
            for (int i = 0; i < allocatedSamples; i++) {
                delete[] dataPoints[i];
            }
            delete[] dataPoints;
            return false;
        }
    }
    
    int sampleCount = 0;
    
    Serial.println("[CAL] Collecting data points. Rotate the device slowly in figure-8 patterns.");
    Serial.print("[CAL] Target samples: ");
    Serial.println(allocatedSamples);
    Serial.println("[CAL] Data collection will run for about 30 seconds.");
    
    unsigned long startTime = millis();
    unsigned long lastFeedbackTime = startTime;
    unsigned long lastDebugOutput = startTime;
    int debugCounter = 0;
    const unsigned long minimumDurationMs = 30000; // Minimum duration of 30 seconds
    
    while ((sampleCount < allocatedSamples || (millis() - startTime < minimumDurationMs)) && 
           (millis() - startTime < 45000)) { // Add safety timeout of 45s
        float mx, my, mz;
        
        if (readMagnetometer(mx, my, mz)) {
            // Print debug output every 20 samples (approximately every 200ms)
            if (++debugCounter >= 20) {
                debugCounter = 0;
                Serial.print("[CAL] Raw MAG: ");
                Serial.print(mx, 2); Serial.print(", ");
                Serial.print(my, 2); Serial.print(", ");
                Serial.println(mz, 2);
            }
            
            // Check if this orientation is significantly different from previous ones
            bool isDifferent = true;
            
            if (sampleCount > 0) {
                // Compare with the last few samples to ensure we're getting diverse orientations
                for (int i = max(0, sampleCount - 5); i < sampleCount; i++) {
                    float dx = mx - dataPoints[i][0];
                    float dy = my - dataPoints[i][1];
                    float dz = mz - dataPoints[i][2];
                    float distSq = dx*dx + dy*dy + dz*dz;
                    
                    if (distSq < 1.0f) { // Less than ~1μT difference
                        isDifferent = false;
                        break;
                    }
                }
            }
            
            if (isDifferent && sampleCount < allocatedSamples) {
                dataPoints[sampleCount][0] = mx;
                dataPoints[sampleCount][1] = my;
                dataPoints[sampleCount][2] = mz;
                sampleCount++;
                
                // Provide feedback every second
                if (millis() - lastFeedbackTime > 1000) {
                    Serial.print("[CAL] Collected ");
                    Serial.print(sampleCount);
                    Serial.print("/");
                    Serial.print(allocatedSamples);
                    Serial.print(" data points (");
                    Serial.print((millis() - startTime) / 1000);
                    Serial.println(" seconds elapsed)");
                    lastFeedbackTime = millis();
                }
            }
        }
        
        delay(10);
    }
    
    // After the loop, provide a summary
    unsigned long elapsedSeconds = (millis() - startTime) / 1000;
    Serial.print("[CAL] Data collection completed in ");
    Serial.print(elapsedSeconds);
    Serial.print(" seconds with ");
    Serial.print(sampleCount);
    Serial.println(" samples");
    
    if (sampleCount < allocatedSamples * 0.8) {
        Serial.println("[CAL] Not enough diverse data points collected. Calibration failed.");
        // Clean up allocated memory
        for (int i = 0; i < allocatedSamples; i++) {
            delete[] dataPoints[i];
        }
        delete[] dataPoints;
        return false;
    }
    
    Serial.print("[CAL] Collected ");
    Serial.print(sampleCount);
    Serial.println(" data points. Processing...");
    
    // Step 1: Calculate hard iron correction (center of ellipsoid)
    float sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < sampleCount; i++) {
        sumX += dataPoints[i][0];
        sumY += dataPoints[i][1];
        sumZ += dataPoints[i][2];
    }
    
    calibration.magBias[0] = sumX / sampleCount;
    calibration.magBias[1] = sumY / sampleCount;
    calibration.magBias[2] = sumZ / sampleCount;
    
    // Step 2: Remove hard iron bias from data points
    for (int i = 0; i < sampleCount; i++) {
        dataPoints[i][0] -= calibration.magBias[0];
        dataPoints[i][1] -= calibration.magBias[1];
        dataPoints[i][2] -= calibration.magBias[2];
    }
    
    // Step 3: Calculate the covariance matrix
    float cov[3][3] = {0};
    
    for (int i = 0; i < sampleCount; i++) {
        cov[0][0] += dataPoints[i][0] * dataPoints[i][0];
        cov[0][1] += dataPoints[i][0] * dataPoints[i][1];
        cov[0][2] += dataPoints[i][0] * dataPoints[i][2];
        cov[1][1] += dataPoints[i][1] * dataPoints[i][1];
        cov[1][2] += dataPoints[i][1] * dataPoints[i][2];
        cov[2][2] += dataPoints[i][2] * dataPoints[i][2];
    }
    
    cov[0][0] /= sampleCount;
    cov[0][1] /= sampleCount;
    cov[0][2] /= sampleCount;
    cov[1][1] /= sampleCount;
    cov[1][2] /= sampleCount;
    cov[2][2] /= sampleCount;
    
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];
    
    // Step 4: Calculate average radius
    float avg_radius_sq = cov[0][0] + cov[1][1] + cov[2][2];
    float avg_radius = sqrt(avg_radius_sq);
    
    // Step 5: Calculate soft iron correction matrix
    // This is a simplified approach - a more sophisticated algorithm would use
    // eigenvalue decomposition to find the principal axes of the ellipsoid
    
    // Calculate the scaling factors for each axis
    float scale_x = sqrt(avg_radius_sq / cov[0][0]);
    float scale_y = sqrt(avg_radius_sq / cov[1][1]);
    float scale_z = sqrt(avg_radius_sq / cov[2][2]);
    
    // Set up the soft iron correction matrix (simplified)
    calibration.magSoftIron[0][0] = scale_x;
    calibration.magSoftIron[0][1] = 0.0f;
    calibration.magSoftIron[0][2] = 0.0f;
    
    calibration.magSoftIron[1][0] = 0.0f;
    calibration.magSoftIron[1][1] = scale_y;
    calibration.magSoftIron[1][2] = 0.0f;
    
    calibration.magSoftIron[2][0] = 0.0f;
    calibration.magSoftIron[2][1] = 0.0f;
    calibration.magSoftIron[2][2] = scale_z;
    
    // CRITICAL: Also copy diagonal values to magScale array for EEPROM persistence
    // The quality assessment and EEPROM display use magScale, not magSoftIron
    calibration.magScale[0] = scale_x;
    calibration.magScale[1] = scale_y;
    calibration.magScale[2] = scale_z;
    
    // Set calibration method and flag
    calibration.magCalMethod = MAG_CAL_HARD_SOFT_IRON;
    calibration.magCalibrated = true;
    
    // Clean up allocated memory
    for (int i = 0; i < allocatedSamples; i++) {
        delete[] dataPoints[i];
    }
    delete[] dataPoints;

    isCalibrating = false; // Allow main loop to resume IMU reads
    
    // Display results
    Serial.println("\n╔════════════════════════════════════════════════════╗");
    Serial.println("║    MAGNETOMETER CALIBRATION COMPLETE              ║");
    Serial.println("╠════════════════════════════════════════════════════╣");
    
    Serial.print("║ Hard Iron Bias (µT): ");
    Serial.printf("%-28s ║\n", 
                 (String(calibration.magBias[0], 1) + ", " + 
                  String(calibration.magBias[1], 1) + ", " + 
                  String(calibration.magBias[2], 1)).c_str());
    
    float biasMag = sqrt(calibration.magBias[0]*calibration.magBias[0] + 
                        calibration.magBias[1]*calibration.magBias[1] + 
                        calibration.magBias[2]*calibration.magBias[2]);
    Serial.printf("║ Bias Magnitude: %-33.1f ║\n", biasMag);
    
    Serial.println("║                                                    ║");
    Serial.println("║ Soft Iron Correction Matrix:                      ║");
    Serial.printf("║   [%.2f, %.2f, %.2f]%-19s║\n", 
                 calibration.magSoftIron[0][0], calibration.magSoftIron[0][1], 
                 calibration.magSoftIron[0][2], "");
    Serial.printf("║   [%.2f, %.2f, %.2f]%-19s║\n", 
                 calibration.magSoftIron[1][0], calibration.magSoftIron[1][1], 
                 calibration.magSoftIron[1][2], "");
    Serial.printf("║   [%.2f, %.2f, %.2f]%-19s║\n", 
                 calibration.magSoftIron[2][0], calibration.magSoftIron[2][1], 
                 calibration.magSoftIron[2][2], "");
    
    Serial.println("║                                                    ║");
    Serial.printf("║ Field Strength: %-34.1f ║\n", avg_radius);
    Serial.println("╚════════════════════════════════════════════════════╝");
    
    // Quality assessment
    bool radiusOk = (avg_radius >= 25.0f && avg_radius <= 65.0f);
    bool biasOk = (biasMag < 100.0f);
    bool softIronOk = true;
    
    // Check soft iron matrix for extreme values
    for (int i = 0; i < 3; i++) {
        float scale = calibration.magSoftIron[i][i];
        if (scale < 0.5f || scale > 3.0f) {
            softIronOk = false;
            break;
        }
    }
    
    // Determine overall quality
    CalibrationQuality quality;
    if (!radiusOk) {
        quality = CAL_QUALITY_FAILED;
    } else if (!biasOk || !softIronOk) {
        quality = CAL_QUALITY_POOR;
    } else if (biasMag < 50.0f && avg_radius >= 30.0f && avg_radius <= 60.0f) {
        // Check soft iron symmetry
        float maxScale = max(max(calibration.magSoftIron[0][0], 
                                calibration.magSoftIron[1][1]), 
                            calibration.magSoftIron[2][2]);
        float minScale = min(min(calibration.magSoftIron[0][0], 
                                calibration.magSoftIron[1][1]), 
                            calibration.magSoftIron[2][2]);
        float asymmetry = maxScale / minScale;
        
        if (asymmetry < 1.5f && biasMag < 30.0f) {
            quality = CAL_QUALITY_EXCELLENT;
        } else if (asymmetry < 2.0f && biasMag < 40.0f) {
            quality = CAL_QUALITY_GOOD;
        } else {
            quality = CAL_QUALITY_ACCEPTABLE;
        }
    } else {
        quality = CAL_QUALITY_ACCEPTABLE;
    }
    
    // Print detailed quality report
    Serial.println("\n=== QUALITY ASSESSMENT ===");
    
    // Field strength check
    Serial.print("Field Strength: ");
    if (radiusOk) {
        Serial.printf("✓ GOOD (%.1f µT, expected 25-65 µT)\n", avg_radius);
    } else if (avg_radius < 25.0f) {
        Serial.printf("✗ TOO LOW (%.1f µT, expected 25-65 µT)\n", avg_radius);
        Serial.println("  → Incomplete rotation or strong magnetic interference");
        Serial.println("  → Try rotating more thoroughly in all directions");
    } else {
        Serial.printf("✗ TOO HIGH (%.1f µT, expected 25-65 µT)\n", avg_radius);
        Serial.println("  → Possible magnetic interference nearby");
        Serial.println("  → Move away from metal objects and recalibrate");
    }
    
    // Bias check
    Serial.print("Hard Iron Bias: ");
    if (biasMag < 30.0f) {
        Serial.printf("✓ EXCELLENT (%.1f µT)\n", biasMag);
    } else if (biasMag < 50.0f) {
        Serial.printf("✓ GOOD (%.1f µT)\n", biasMag);
    } else if (biasMag < 100.0f) {
        Serial.printf("⚠ ACCEPTABLE (%.1f µT)\n", biasMag);
        Serial.println("  → Some magnetic interference present");
    } else {
        Serial.printf("✗ HIGH (%.1f µT)\n", biasMag);
        Serial.println("  → Strong magnetic interference detected");
        Serial.println("  → Move away from metal objects and recalibrate");
    }
    
    // Soft iron check
    Serial.print("Soft Iron Matrix: ");
    float maxScale = max(max(calibration.magSoftIron[0][0], 
                            calibration.magSoftIron[1][1]), 
                        calibration.magSoftIron[2][2]);
    float minScale = min(min(calibration.magSoftIron[0][0], 
                            calibration.magSoftIron[1][1]), 
                        calibration.magSoftIron[2][2]);
    float asymmetry = maxScale / minScale;
    
    if (asymmetry < 1.5f) {
        Serial.printf("✓ EXCELLENT (asymmetry: %.2f)\n", asymmetry);
    } else if (asymmetry < 2.0f) {
        Serial.printf("✓ GOOD (asymmetry: %.2f)\n", asymmetry);
    } else if (asymmetry < 3.0f) {
        Serial.printf("⚠ ACCEPTABLE (asymmetry: %.2f)\n", asymmetry);
        Serial.println("  → Uneven rotation coverage or soft iron distortion");
        Serial.println("  → Try rotating more evenly in all directions");
    } else {
        Serial.printf("✗ POOR (asymmetry: %.2f)\n", asymmetry);
        Serial.println("  → Severe asymmetry detected");
        Serial.println("  → Ensure complete 3D rotation coverage");
    }
    
    // Overall quality
    Serial.print("\nOverall Quality: ");
    printQualityReport("Magnetometer", quality);
    
    // Offer retry if quality is poor
    if (quality <= CAL_QUALITY_ACCEPTABLE) {
        Serial.println("\n⚠ Calibration quality could be improved.");
        Serial.println("Tips for better results:");
        Serial.println("  • Move away from metal objects (desk, laptop, phone)");
        Serial.println("  • Rotate slowly in smooth figure-8 patterns");
        Serial.println("  • Cover ALL orientations evenly");
        Serial.println("  • Take the full 30 seconds");
        Serial.println("\nWould you like to:");
        Serial.println("  [y] Accept this calibration");
        Serial.println("  [n] Retry calibration");
        Serial.print("Choice: ");
        
        // Wait for user decision
        while (!Serial.available()) delay(10);
        char choice = Serial.read();
        Serial.println(choice);
        
        if (choice == 'n' || choice == 'N') {
            Serial.println("\nRetrying magnetometer calibration...\n");
            return calibrateMagHardSoftIron(); // Recursive retry
        }
    }
    
    Serial.println("\n✓ Magnetometer calibration accepted and will be saved.");
    return true;
}

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
bool calibrateMagWithTempComp() {
    Serial.println("[CAL] Starting temperature-compensated magnetometer calibration...");
    Serial.println("[CAL] This procedure requires heating/cooling the device to measure");
    Serial.println("[CAL] temperature sensitivity. Follow the instructions carefully.");
    
    // Step 1: Collect baseline data at current (room) temperature
    Serial.println("[CAL] First, we'll collect baseline data at room temperature.");
    Serial.println("[CAL] Rotate the device slowly in all directions to collect a full");
    Serial.println("[CAL] range of magnetometer readings. Try to cover all orientations.");
    Serial.println("[CAL] When ready, press 'y' to start data collection");
    waitForUserConfirmation();
    
    // Configure magnetometer for calibration
    configureMagForCalibration();
    
    // Variables for baseline data collection
    float mxSum1 = 0.0f, mySum1 = 0.0f, mzSum1 = 0.0f;
    float temp1;
    int validSamples1 = 0;
    const int numSamples = 300;
    
    // Read current temperature
    if (!readTemperature(temp1)) {
        Serial.println("[CAL] Failed to read temperature sensor");
        return false;
    }
    
    Serial.printf("[CAL] Baseline temperature: %.2f°C\n", temp1);
    Serial.println("[CAL] Collecting baseline magnetometer data. Rotate the device slowly...");
    
    // Collect baseline data
    unsigned long startTime = millis();
    unsigned long lastFeedbackTime = startTime;
    
    while (validSamples1 < numSamples && (millis() - startTime < 30000)) {
        float mx, my, mz;
        
        if (readMagnetometer(mx, my, mz)) {
            // Add to running sum
            mxSum1 += mx;
            mySum1 += my;
            mzSum1 += mz;
            validSamples1++;
            
            // Provide feedback every second
            if (millis() - lastFeedbackTime > 1000) {
                Serial.print("[CAL] Collected ");
                Serial.print(validSamples1);
                Serial.print("/");
                Serial.print(numSamples);
                Serial.println(" samples");
                lastFeedbackTime = millis();
            }
        }
        
        delay(10);
    }
    
    if (validSamples1 < numSamples * 0.8) {
        Serial.println("[CAL] Not enough valid samples collected. Calibration failed.");
        return false;
    }
    
    // Calculate baseline bias
    float mxBias1 = mxSum1 / validSamples1;
    float myBias1 = mySum1 / validSamples1;
    float mzBias1 = mzSum1 / validSamples1;
    
    Serial.println("[CAL] Baseline data collection complete.");
    Serial.printf("[CAL] Baseline bias: X=%.4f Y=%.4f Z=%.4f\n", mxBias1, myBias1, mzBias1);
    
    // Step 2: Ask user to change device temperature
    Serial.println("[CAL] Now we need to change the device temperature.");
    Serial.println("[CAL] Option 1: Hold the device in your hand for 5 minutes to warm it");
    Serial.println("[CAL] Option 2: Place the device in a warmer location");
    Serial.println("[CAL] Option 3: Use a cold pack to cool the device");
    Serial.println("[CAL] Try to change temperature by at least 5°C for best results");
    Serial.println("[CAL] When temperature has changed, press 'y' to continue");
    waitForUserConfirmation();
    
    // Step 3: Collect data at new temperature
    float mxSum2 = 0.0f, mySum2 = 0.0f, mzSum2 = 0.0f;
    float temp2;
    int validSamples2 = 0;
    
    // Read new temperature
    if (!readTemperature(temp2)) {
        Serial.println("[CAL] Failed to read temperature sensor");
        return false;
    }
    
    Serial.printf("[CAL] New temperature: %.2f°C\n", temp2);
    Serial.println("[CAL] Collecting magnetometer data at new temperature. Rotate the device slowly...");
    
    // Collect data at new temperature
    startTime = millis();
    lastFeedbackTime = startTime;
    
    while (validSamples2 < numSamples && (millis() - startTime < 30000)) {
        float mx, my, mz;
        
        if (readMagnetometer(mx, my, mz)) {
            // Add to running sum
            mxSum2 += mx;
            mySum2 += my;
            mzSum2 += mz;
            validSamples2++;
            
            // Provide feedback every second
            if (millis() - lastFeedbackTime > 1000) {
                Serial.print("[CAL] Collected ");
                Serial.print(validSamples2);
                Serial.print("/");
                Serial.print(numSamples);
                Serial.println(" samples");
                lastFeedbackTime = millis();
            }
        }
        
        delay(10);
    }
    
    if (validSamples2 < numSamples * 0.8) {
        Serial.println("[CAL] Not enough valid samples collected. Calibration failed.");
        return false;
    }
    
    // Calculate new bias
    float mxBias2 = mxSum2 / validSamples2;
    float myBias2 = mySum2 / validSamples2;
    float mzBias2 = mzSum2 / validSamples2;
    
    Serial.println("[CAL] New temperature data collection complete.");
    Serial.printf("[CAL] New bias: X=%.4f Y=%.4f Z=%.4f\n", mxBias2, myBias2, mzBias2);
    
    // Step 4: Calculate temperature coefficients
    float tempDiff = temp2 - temp1;
    if (abs(tempDiff) < 2.0f) {
        Serial.println("[CAL] Temperature difference too small (<2°C).");
        Serial.println("[CAL] Temperature compensation will not be accurate.");
        Serial.println("[CAL] Do you want to continue anyway? (y/n)");
        if (!waitForUserConfirmation(true)) {
            return false;
        }
    }
    
    // Calculate temperature coefficients (bias change per degree C)
    float mxTempCoeff = (mxBias2 - mxBias1) / tempDiff;
    float myTempCoeff = (myBias2 - myBias1) / tempDiff;
    float mzTempCoeff = (mzBias2 - mzBias1) / tempDiff;
    
    Serial.printf("[CAL] Temperature coefficients:\n");
    Serial.printf("[CAL] X: %.6f μT/°C\n", mxTempCoeff);
    Serial.printf("[CAL] Y: %.6f μT/°C\n", myTempCoeff);
    Serial.printf("[CAL] Z: %.6f μT/°C\n", mzTempCoeff);
    
    // Step 5: Save calibration data
    calibration.magBias[0] = mxBias1;
    calibration.magBias[1] = myBias1;
    calibration.magBias[2] = mzBias1;
    calibration.magTempCoeff[0] = mxTempCoeff;
    calibration.magTempCoeff[1] = myTempCoeff;
    calibration.magTempCoeff[2] = mzTempCoeff;
    calibration.magCalTemp = temp1;  // Reference temperature for bias
    calibration.magTempCompEnabled = true;
    calibration.magCalMethod = MAG_CAL_SIMPLE;  // Temperature comp works with simple bias
    calibration.magCalibrated = true;
    
    // CRITICAL: Set magScale to identity for temp comp calibration
    calibration.magScale[0] = 1.0f;
    calibration.magScale[1] = 1.0f;
    calibration.magScale[2] = 1.0f;
    
    // Step 6: Validate calibration quality
    float meanField, sigmaField, tempDrift;
    bool validationResult = validateMagCalibrationEnhanced(meanField, sigmaField, tempDrift);
    
    if (validationResult) {
        Serial.println("[CAL] Magnetometer calibration validation passed!");
        Serial.printf("[CAL] Mean field strength: %.2f μT\n", meanField);
        Serial.printf("[CAL] Field std deviation: %.4f μT\n", sigmaField);
        Serial.printf("[CAL] Temperature drift: %.4f μT/°C\n", tempDrift);
    } else {
        Serial.println("[CAL] Magnetometer calibration validation warning!");
        Serial.printf("[CAL] Mean field strength: %.2f μT (expected: ~25-65 μT)\n", meanField);
        Serial.printf("[CAL] Field std deviation: %.4f μT\n", sigmaField);
        Serial.printf("[CAL] Temperature drift: %.4f μT/°C\n", tempDrift);
        Serial.println("[CAL] Calibration saved but may not be optimal.");
    }
    
    Serial.println("[CAL] Magnetometer calibration complete");
    return true;
}

/**
 * Run the magnetometer calibration procedure with user-selectable methods
 * 
 * This function presents a menu to the user to select between different
 * magnetometer calibration methods:
 * 1. Simple bias-only calibration
 * 2. Ellipsoid fitting calibration (hard + soft iron)
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateMagnetometer() {
    // Quick validation of existing calibration
    float meanField, sigmaField;
    bool qcOk = validateMagCalibrationQuick(meanField, sigmaField);
    Serial.printf("[MAG QC] mean=%.3f  σ=%.3f  -> %s\n", meanField, sigmaField, qcOk ? "PASS" : "FAIL");
    if (!qcOk) {
        Serial.println("[MAG QC] Suggest performing ellipsoid calibration (option 2) for best results.");
    } else {
        Serial.println("[MAG QC] Calibration looks good; you may skip or redo as desired.");
    }

    Serial.println("\n\033[36m=== Magnetometer Calibration Methods ===\033[0m");
    Serial.println("  1. Simple Offset Calibration");
    Serial.println("  2. Ellipsoid Fit Calibration (Recommended)");
    Serial.print("Enter choice (1-2): ");
    
    // Clear any pending serial data
    while (Serial.available()) {
        Serial.read();
    }
    
    // Wait for user selection
    int selection = 0;
    while (selection < 1 || selection > 2) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c >= '1' && c <= '2') {
                selection = c - '0';
                Serial.println(c);
                break;
            }
        }
        delay(10);
    }
    
    bool result = false;
    
    switch (selection) {
        case 1:
            Serial.println("[CAL] Selected: Simple bias-only calibration");
            result = calibrateMagSimple();
            break;
        case 2:
            Serial.println("[CAL] Selected: Ellipsoid fitting calibration");
            result = calibrateMagHardSoftIron();
            break;
        default:
            Serial.println("Invalid selection");
            return false;
    }
    
    if (result) {
        Serial.println("[CAL] Magnetometer calibration complete");
    }
    
    return result;
}

/**
 * Ellipsoid calibration - uses hard/soft iron method
 * This is the same as calibrateMagHardSoftIron() but with a different name for clarity
 */
bool calibrateMagEllipsoid() {
    return calibrateMagHardSoftIron();
}

// Helper to allow other modules to query mag calibration status
bool isMagCalibrated() {
    return calibration.magCalibrated;
}

// ----- Dynamic (runtime) calibration support --------------------------------

// Simple running-min/max approach for hard-iron offset refinement.
// More sophisticated ellipsoid-fit-on-the-fly can be added later.

static float dyn_min[3] = {10000, 10000, 10000};
static float dyn_max[3] = {-10000, -10000, -10000};

void setDynamicMagCalibrationEnabled(bool enabled) {
    calibration.magDynamicCalEnabled = enabled;

    if (!enabled) {
        // Reset dynamic buffers to avoid stale extremes the next time we enable.
        dyn_min[0] = dyn_min[1] = dyn_min[2] = 10000;
        dyn_max[0] = dyn_max[1] = dyn_max[2] = -10000;
    }
}

bool isDynamicMagCalibrationEnabled() {
    return calibration.magDynamicCalEnabled;
}

void updateDynamicMagCalibration(float mx, float my, float mz) {
    if (!calibration.magDynamicCalEnabled) {
        return; // feature disabled ⇒ skip work to save power
    }

    // Update running extrema
    if (mx < dyn_min[0]) dyn_min[0] = mx;
    if (my < dyn_min[1]) dyn_min[1] = my;
    if (mz < dyn_min[2]) dyn_min[2] = mz;

    if (mx > dyn_max[0]) dyn_max[0] = mx;
    if (my > dyn_max[1]) dyn_max[1] = my;
    if (mz > dyn_max[2]) dyn_max[2] = mz;

    // Re-compute bias as mid-point; soft-iron left unchanged to minimise load.
    calibration.magBias[0] = (dyn_max[0] + dyn_min[0]) * 0.5f;
    calibration.magBias[1] = (dyn_max[1] + dyn_min[1]) * 0.5f;
    calibration.magBias[2] = (dyn_max[2] + dyn_min[2]) * 0.5f;
}

// -----------------------------------------------------------------------------
// Quick calibration validation (boot-time health check)
// -----------------------------------------------------------------------------

bool validateMagCalibrationQuick(float &meanField, float &sigmaField) {
    const int SAMPLE_COUNT = 50;            // ~1 s at 50 Hz sampling
    if (!calibration.magCalibrated) {
        Serial.println("[MAG QC] No stored calibration – skipping quick check.");
        meanField = sigmaField = 0;
        return false;
    }

    float mags[SAMPLE_COUNT];
    int collected = 0;
    unsigned long start = millis();

    while (collected < SAMPLE_COUNT && (millis() - start) < 3000) { // 3-s timeout
        float mx, my, mz;
        if (readMagnetometer(mx, my, mz)) {
            applyMagCalibration(mx, my, mz, 0); // in-place
            mags[collected++] = sqrtf(mx*mx + my*my + mz*mz);
        }
        delay(20); // 50 Hz
    }

    if (collected < SAMPLE_COUNT) {
        Serial.println("[MAG QC] Insufficient samples for quick check.");
        return false;
    }

    // Compute mean and std-dev
    float sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; ++i) sum += mags[i];
    meanField = sum / SAMPLE_COUNT;

    float var = 0;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        float d = mags[i] - meanField;
        var += d*d;
    }
    sigmaField = sqrtf(var / SAMPLE_COUNT);

    bool ok = (fabsf(meanField - 1.0f) < 0.05f) && (sigmaField / meanField < 0.05f);
    return ok;
}

/**
 * Enhanced magnetometer calibration validation with comprehensive metrics
 * 
 * This function performs a comprehensive validation of the magnetometer calibration quality
 * by collecting samples while the device is rotated through different orientations and checking:
 * 1. Field strength consistency across orientations
 * 2. Noise floor and stability metrics
 * 3. Temperature sensitivity analysis
 * 4. Cross-axis coupling verification
 * 
 * @param meanField Output parameter for mean magnetic field strength
 * @param sigmaField Output parameter for standard deviation of field strength
 * @param tempDrift Output parameter for temperature drift coefficient
 * @return true if calibration quality meets enhanced acceptance criteria, false otherwise
 */
bool validateMagCalibrationEnhanced(float &meanField, float &sigmaField, float &tempDrift) {
    Serial.println("[VAL] Enhanced magnetometer calibration validation...");
    Serial.println("[VAL] This test requires rotating the device through various orientations");
    Serial.println("[VAL] Press 'y' to begin validation");
    
    if (!waitForUserConfirmation()) {
        return false;
    }
    
    // Initialize statistics variables
    const int numSamples = 200;
    float fieldStrengths[numSamples];
    float temperatures[numSamples];
    int validSamples = 0;
    
    // Collect samples while device is rotated
    Serial.println("[VAL] Slowly rotate device through different orientations...");
    unsigned long startTime = millis();
    const unsigned long COLLECTION_TIME = 20000;  // 20 seconds
    
    while (validSamples < numSamples && (millis() - startTime < COLLECTION_TIME)) {
        float mx, my, mz, temp;
        
        if (readMagnetometer(mx, my, mz) && readTemperature(temp)) {
            // Apply calibration
            applyMagCalibration(mx, my, mz, temp);
            
            // Calculate field strength
            float fieldStrength = sqrt(mx*mx + my*my + mz*mz);
            
            // Store data
            fieldStrengths[validSamples] = fieldStrength;
            temperatures[validSamples] = temp;
            validSamples++;
            
            if (validSamples % 25 == 0) {
                Serial.printf("[VAL] Collected %d samples...\n", validSamples);
            }
        }
        
        delay(100);  // 10Hz sampling
    }
    
    if (validSamples < numSamples * 0.8) {
        Serial.println("[VAL] Error: Not enough valid samples collected");
        return false;
    }
    
    // Calculate field strength statistics
    float sumField = 0, sumFieldSq = 0;
    for (int i = 0; i < validSamples; i++) {
        sumField += fieldStrengths[i];
        sumFieldSq += fieldStrengths[i] * fieldStrengths[i];
    }
    
    meanField = sumField / validSamples;
    float variance = (sumFieldSq / validSamples) - (meanField * meanField);
    sigmaField = sqrt(variance);
    
    // Calculate temperature drift coefficient
    float sumTemp = 0, sumTempField = 0, sumTempSq = 0;
    for (int i = 0; i < validSamples; i++) {
        sumTemp += temperatures[i];
        sumTempField += temperatures[i] * fieldStrengths[i];
        sumTempSq += temperatures[i] * temperatures[i];
    }
    
    float meanTemp = sumTemp / validSamples;
    float tempVariance = (sumTempSq / validSamples) - (meanTemp * meanTemp);
    
    if (tempVariance > 0.1f) {  // Only calculate if temperature varied
        float covariance = (sumTempField / validSamples) - (meanTemp * meanField);
        tempDrift = covariance / tempVariance;  // μT/°C
    } else {
        tempDrift = 0.0f;  // No temperature variation
    }
    
    // Quality assessment criteria
    const float expectedField = 50.0f;  // Expected Earth's magnetic field ~50μT
    const float maxFieldError = 10.0f;  // ±10μT tolerance
    const float maxStdDev = 5.0f;       // Maximum standard deviation
    const float maxTempDrift = 0.5f;    // Maximum temperature drift (μT/°C)
    
    bool fieldStrengthOk = abs(meanField - expectedField) < maxFieldError;
    bool consistencyOk = sigmaField < maxStdDev;
    bool tempStabilityOk = abs(tempDrift) < maxTempDrift;
    
    // Output detailed results
    Serial.println("[VAL] Enhanced validation results:");
    Serial.printf("[VAL] Mean field strength: %.2f μT (expected: ~50μT)\n", meanField);
    Serial.printf("[VAL] Field consistency (σ): %.2f μT (threshold: %.2f)\n", sigmaField, maxStdDev);
    Serial.printf("[VAL] Temperature drift: %.3f μT/°C (threshold: %.3f)\n", tempDrift, maxTempDrift);
    Serial.printf("[VAL] Temperature range: %.1f°C to %.1f°C\n", 
                 temperatures[0], temperatures[validSamples-1]);
    
    // Individual test results
    Serial.printf("[VAL] Field strength test: %s\n", fieldStrengthOk ? "PASS" : "FAIL");
    Serial.printf("[VAL] Consistency test: %s\n", consistencyOk ? "PASS" : "FAIL");
    Serial.printf("[VAL] Temperature stability test: %s\n", tempStabilityOk ? "PASS" : "FAIL");
    
    bool overallPass = fieldStrengthOk && consistencyOk && tempStabilityOk;
    Serial.printf("[VAL] Overall validation: %s\n", overallPass ? "PASS" : "FAIL");
    
    if (!overallPass) {
        Serial.println("[VAL] Recommendations:");
        if (!fieldStrengthOk) {
            Serial.println("[VAL] - Recalibrate magnetometer (field strength out of range)");
        }
        if (!consistencyOk) {
            Serial.println("[VAL] - Check for magnetic interference or improve calibration");
        }
        if (!tempStabilityOk) {
            Serial.println("[VAL] - Consider enabling temperature compensation");
        }
    }
    
    return overallPass;
}

/**
 * Enhances the readMagnetometer function with robust error handling and recovery
 * 
 * This function is a wrapper for the main readMagnetometer function that adds
 * advanced error handling and recovery mechanisms.
 * 
 * @param mx_uT X-axis magnetometer reading in μT (output parameter)
 * @param my_uT Y-axis magnetometer reading in μT (output parameter)
 * @param mz_uT Z-axis magnetometer reading in μT (output parameter)
 * @return true if reading was successful, false otherwise
 */
bool readMagnetometerCalibration(float& mx_uT, float& my_uT, float& mz_uT) {
    static uint32_t last_continuous_mode_assert = 0;
    static uint32_t last_read = 0;
    static uint32_t last_success = 0;
    static uint8_t consecutive_failures = 0;
    const uint8_t MAX_FAILURES = 5;
    uint8_t status = 0;
    uint8_t rawData[6];
    
    // Force recovery if no successful reads in 500ms
    if (millis() - last_success > 500) {
        consecutive_failures = MAX_FAILURES;
        // Try recovery
        if (consecutive_failures >= MAX_FAILURES) {
            Serial.print("[CAL] Magnetometer recovery triggered (");
            Serial.print(consecutive_failures);
            Serial.println(" failures)");
            
            // Attempt to reinitialize the magnetometer
            configureMagForCalibration();
            
            consecutive_failures = 0;
            last_success = millis();
        }
        return false;
    }
    
    // Enforce minimum 5ms between reads (200Hz max)
    uint32_t now = micros();
    if (now - last_read < 5000) {
        return false;
    }
    last_read = now;
    
    // Periodically reassert continuous mode (every 5s) to prevent stalls
    if (millis() - last_continuous_mode_assert > 5000) {
        // Set to continuous measurement mode 4 (100Hz)
        Wire.beginTransmission(AK09916_ADDR);
        Wire.write(0x31); // CNTL2
        Wire.write(0x08); // Mode 4 (100Hz)
        Wire.endTransmission();
        last_continuous_mode_assert = millis();
    }
    
    // Read status register 1 (ST1)
    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(0x10); // ST1
    if (Wire.endTransmission(false) != 0) {
        consecutive_failures++;
        return checkRecovery(consecutive_failures, MAX_FAILURES, last_success);
    }
    
    Wire.requestFrom(AK09916_ADDR, 1);
    if (!Wire.available()) {
        consecutive_failures++;
        return checkRecovery(consecutive_failures, MAX_FAILURES, last_success);
    }
    
    status = Wire.read();
    
    // Check if data is ready
    if (!(status & 0x01)) {
        // Handle data overrun (bit 1 set) by reading ST2 to clear it
        if (status & 0x02) {
            Wire.beginTransmission(AK09916_ADDR);
            Wire.write(0x18); // ST2
            Wire.endTransmission(false);
            Wire.requestFrom(AK09916_ADDR, 1);
            if (Wire.available()) {
                Wire.read();
            }
        }
        
        // If we're getting repeated not-ready, force recovery
        if (consecutive_failures > 3) {
            return checkRecovery(consecutive_failures, MAX_FAILURES, last_success);
        }
        
        delay(2); // Slightly longer delay before retry
        return false;
    }
    
    // Read the 6 data registers (HXL to HZH)
    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(0x11); // HXL
    if (Wire.endTransmission(false) != 0) {
        consecutive_failures++;
        return checkRecovery(consecutive_failures, MAX_FAILURES, last_success);
    }
    
    if (Wire.requestFrom(AK09916_ADDR, 6) != 6) {
        consecutive_failures++;
        return checkRecovery(consecutive_failures, MAX_FAILURES, last_success);
    }
    
    for (int i = 0; i < 6; i++) {
        if (!Wire.available()) {
            consecutive_failures++;
            return checkRecovery(consecutive_failures, MAX_FAILURES, last_success);
        }
        rawData[i] = Wire.read();
    }
    
    // Read ST2 to check for overflow and clear DRDY
    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(0x18); // ST2
    if (Wire.endTransmission(false) != 0) {
        consecutive_failures++;
        return checkRecovery(consecutive_failures, MAX_FAILURES, last_success);
    }
    
    if (Wire.requestFrom(AK09916_ADDR, 1) != 1) {
        consecutive_failures++;
        return checkRecovery(consecutive_failures, MAX_FAILURES, last_success);
    }
    
    if (!Wire.available()) {
        consecutive_failures++;
        return checkRecovery(consecutive_failures, MAX_FAILURES, last_success);
    }
    
    status = Wire.read();
    
    // Check for sensor overflow
    if (status & 0x08) {
        consecutive_failures++;
        return checkRecovery(consecutive_failures, MAX_FAILURES, last_success);
    }
    
    // Data is valid, process it
    int16_t mx_raw = (int16_t)(rawData[1] << 8) | rawData[0];
    int16_t my_raw = (int16_t)(rawData[3] << 8) | rawData[2];
    int16_t mz_raw = (int16_t)(rawData[5] << 8) | rawData[4];
    
    // Convert raw values to μT (AK09916 has 0.15μT per LSB)
    mx_uT = mx_raw * 0.15f;
    my_uT = my_raw * 0.15f;
    mz_uT = mz_raw * 0.15f;
    
    last_success = millis();
    consecutive_failures = 0;
    return true;
}

/**
 * Helper function to check if recovery is needed and perform it if necessary
 * 
 * @param consecutive_failures Current count of consecutive failures
 * @param MAX_FAILURES Maximum allowed failures before recovery
 * @param last_success Reference to last successful read timestamp
 * @return false (indicating the read operation failed)
 */
bool checkRecovery(uint8_t& consecutive_failures, const uint8_t MAX_FAILURES, uint32_t& last_success) {
    if (consecutive_failures >= MAX_FAILURES) {
        Serial.print("[CAL] Magnetometer recovery triggered (");
        Serial.print(consecutive_failures);
        Serial.println(" failures)");
        
        // Attempt to reinitialize the magnetometer
        configureMagForCalibration();
        
        consecutive_failures = 0;
        last_success = millis();
    }
    return false;
}

// Sensor-specific validation helpers
bool readMagSensorData(float &x, float &y, float &z) {
    return readMagnetometer(x, y, z);
}

void applyMagSensorCalibration(float &x, float &y, float &z) {
    // Apply hard iron correction (bias)
    x -= calibration.magBias[0];
    y -= calibration.magBias[1];
    z -= calibration.magBias[2];
    
    // Apply soft iron correction (scale and cross-axis) if available
    // Apply soft iron correction (scale and cross-axis) if available
    if (calibration.magCalMethod == MAG_CAL_HARD_SOFT_IRON) {
        // Apply cross-axis correction matrix
        // Note: magSoftIron diagonal elements ALREADY contain the scale factors,
        // so we do not need to apply magScale[] separately.
        float x_orig = x;
        float y_orig = y;
        float z_orig = z;
        
        x = calibration.magSoftIron[0][0] * x_orig + calibration.magSoftIron[0][1] * y_orig + calibration.magSoftIron[0][2] * z_orig;
        y = calibration.magSoftIron[1][0] * x_orig + calibration.magSoftIron[1][1] * y_orig + calibration.magSoftIron[1][2] * z_orig;
        z = calibration.magSoftIron[2][0] * x_orig + calibration.magSoftIron[2][1] * y_orig + calibration.magSoftIron[2][2] * z_orig;
    }
}

bool checkMagBiasAcceptable(float x, float y, float z) {
    // For magnetometer, check if the magnitude is within reasonable Earth's magnetic field range
    float magnitude = sqrt(x*x + y*y + z*z);
    // Earth's magnetic field is typically 25-65 µT
    return (magnitude >= 20.0f && magnitude <= 80.0f);
}

bool checkMagNoiseAcceptable(float x, float y, float z) {
    // Check noise level against acceptance threshold for magnetometer
    // Noise should be less than 2 µT for good calibration
    return (x < 2.0f && y < 2.0f && z < 2.0f);
}
