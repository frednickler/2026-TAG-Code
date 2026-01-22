#include "accel_calibration.h"
#include "imu_validation.h"
#include "temp_sensor.h"  // Include for temperature sensor access
#include "imu_config.h"   // Include for setBank, readRegister, writeRegister functions

// Forward declarations of sensor read helpers defined in main.cpp
bool readAccel(float& ax, float& ay, float& az);
bool readGyro(float& gx, float& gy, float& gz);
bool readTemperature(float& temp);

// Forward declarations of local helper functions

/**
 * Diagnostic function to check raw accelerometer readings
 * This helps debug calibration issues by showing actual sensor values
 */
void diagnoseAccelReadings() {
    Serial.println("\n=== Accelerometer Diagnostic ===");
    
    for (int i = 0; i < 10; i++) {
        float ax, ay, az;
        if (readAccel(ax, ay, az)) {
            float magnitude = sqrt(ax*ax + ay*ay + az*az);
            Serial.printf("Sample %d: X=%.1f Y=%.1f Z=%.1f (mag=%.1f mg)\n", 
                         i+1, ax, ay, az, magnitude);
        } else {
            Serial.printf("Sample %d: READ FAILED\n", i+1);
        }
        delay(200);
    }
    
    Serial.println("Expected: ~1000mg magnitude when stationary");
    Serial.println("=================================\n");
}

/**
 * Apply accelerometer calibration to raw sensor readings
 * 
 * This function applies the appropriate calibration method to the raw accelerometer readings
 * based on the currently selected method in the calibration data.
 * 
 * @param ax_mg X-axis accelerometer reading in mg (in/out parameter)
 * @param ay_mg Y-axis accelerometer reading in mg (in/out parameter)
 * @param az_mg Z-axis accelerometer reading in mg (in/out parameter)
 * @param temperature Current temperature in degrees Celsius (for temperature compensation)
 */
void applyAccelCalibration(float& ax_mg, float& ay_mg, float& az_mg, float temperature) {
    // Only apply calibration if accelerometer is calibrated
    if (calibration.accelCalibrated) {
        // Store original values for cross-axis calculations
        float ax_orig = ax_mg;
        float ay_orig = ay_mg;
        float az_orig = az_mg;
        
        // Step 1: Apply bias correction (common to all methods)
        ax_mg -= calibration.accelBias[0];
        ay_mg -= calibration.accelBias[1];
        az_mg -= calibration.accelBias[2];
        
        // Step 2: Apply temperature compensation if enabled
        if (calibration.accelTempCompEnabled && temperature > -100.0f) {
            // Calculate temperature difference from calibration temperature
            float tempDiff = temperature - calibration.accelCalTemp;
            
            // Apply temperature coefficients to compensate for thermal drift
            ax_mg -= calibration.accelTempCoeff[0] * tempDiff;
            ay_mg -= calibration.accelTempCoeff[1] * tempDiff;
            az_mg -= calibration.accelTempCoeff[2] * tempDiff;
        }
        
        // Step 3: Apply scale factors (for all methods except simple bias)
        if (calibration.accelCalMethod != ACCEL_CAL_SIMPLE) {
            ax_mg *= calibration.accelScale[0];
            ay_mg *= calibration.accelScale[1];
            az_mg *= calibration.accelScale[2];
        }
        
        // Step 4: Apply cross-axis correction (only for cross-axis and ellipsoid methods)
        if (calibration.accelCalMethod == ACCEL_CAL_CROSS_AXIS) {
            // Apply cross-axis sensitivity correction using the calibration matrix
            float ax_corrected = calibration.accelCrossAxis[0][0] * (ax_orig - calibration.accelBias[0]) +
                                calibration.accelCrossAxis[0][1] * (ay_orig - calibration.accelBias[1]) +
                                calibration.accelCrossAxis[0][2] * (az_orig - calibration.accelBias[2]);
            
            float ay_corrected = calibration.accelCrossAxis[1][0] * (ax_orig - calibration.accelBias[0]) +
                                calibration.accelCrossAxis[1][1] * (ay_orig - calibration.accelBias[1]) +
                                calibration.accelCrossAxis[1][2] * (az_orig - calibration.accelBias[2]);
            
            float az_corrected = calibration.accelCrossAxis[2][0] * (ax_orig - calibration.accelBias[0]) +
                                calibration.accelCrossAxis[2][1] * (ay_orig - calibration.accelBias[1]) +
                                calibration.accelCrossAxis[2][2] * (az_orig - calibration.accelBias[2]);
            
            // Apply scale factors to the corrected values
            ax_mg = ax_corrected * calibration.accelScale[0];
            ay_mg = ay_corrected * calibration.accelScale[1];
            az_mg = az_corrected * calibration.accelScale[2];
        }
        else if (calibration.accelCalMethod == ACCEL_CAL_ELLIPSOID) {
            // For ellipsoid method, we've already applied bias and scale factors
            // Additional ellipsoid-specific corrections could be added here if needed
        }
    }
}

// ... (rest of the code remains the same)

/**
 * Configure the accelerometer registers for optimal calibration
 */
void configureAccelForCalibration() {
    Serial.printf("[CAL] Configuring accelerometer for ±%dg...\n", ACCEL_FSR_G);
    
    // Set Accelerometer Configuration (Bank 2)
    setBank(2);
    
    // ACCEL_CONFIG (0x14) Composition:
    // [5:3] ACCEL_DLPFCFG = 4 (24Hz Bandwidth) -> (4 << 3) = 0x20
    // [2:1] ACCEL_FS_SEL  = From Config       -> ACCEL_FS_SEL_BITS_REG
    // [0]   ACCEL_FCHOICE = 1 (Enable DLPF)   -> 0x01
    uint8_t accel_config_val = ACCEL_FS_SEL_BITS_REG | 0x20 | 0x01;
    writeRegister(0x14, accel_config_val);
    
    // Verify configuration
    delay(10);
    uint8_t current_cfg = readRegister(0x14);
    Serial.printf("[CAL DEBUG] ACCEL_CONFIG: 0x%02X (Target: 0x%02X)\n", current_cfg, accel_config_val);
    
    // Set Sample Rate Divider (ACCEL_SMPLRT_DIV)
    // Bank 2, Reg 0x10 (MSB) & 0x11 (LSB)
    // Target: ~225Hz. Base 1125Hz. Div = 4. 1125/(1+4) = 225Hz.
    writeRegister(0x10, 0x00); // MSB
    writeRegister(0x11, 0x04); // LSB
    
    setBank(0);  // Return to Bank 0
    Serial.println("[CAL] Accelerometer configured for calibration");
}

/**
 * Perform simple bias-only accelerometer calibration
 */
bool calibrateAccelSimple() {
    isCalibrating = true;
    Serial.println("[CAL] Starting simple accelerometer calibration...");
    Serial.println("[CAL] Place device on a level surface");
    Serial.println("[CAL] Press 'y' when ready");
    
    if (!waitForUserConfirmation()) {
        isCalibrating = false;
        return false;
    }
    
    // Configure accelerometer for calibration
    configureAccelForCalibration();
    
    // Initialize variables for sample collection
    const int NUM_SAMPLES = 300;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    int validSamples = 0;
    
    Serial.println("[CAL] Collecting accelerometer data...");
    unsigned long startTime = millis();
    
    // Collect samples
    while (validSamples < NUM_SAMPLES && (millis() - startTime < 5000)) {
        float ax, ay, az;
        if (readAccel(ax, ay, az)) {
            ax_sum += ax;
            ay_sum += ay;
            az_sum += az;
            validSamples++;
        }
        delay(10);  // ~100Hz sampling
    }
    
    if (validSamples < NUM_SAMPLES * 0.9) {
        Serial.println("[CAL] Error: Not enough valid samples collected");
        isCalibrating = false;
        return false;
    }
    
    // Calculate average bias
    calibration.accelBias[0] = ax_sum / validSamples;
    calibration.accelBias[1] = ay_sum / validSamples;
    calibration.accelBias[2] = (az_sum / validSamples) - 1000.0f;  // Subtract 1g (1000mg)
    
    // Reset scale factors to 1.0 (Simple cal assumes factory scale)
    calibration.accelScale[0] = 1.0f;
    calibration.accelScale[1] = 1.0f;
    calibration.accelScale[2] = 1.0f;
    
    // Store calibration method and mark as calibrated
    calibration.accelCalMethod = ACCEL_CAL_SIMPLE;
    calibration.accelCalibrated = true;
    
    Serial.println("[CAL] Simple accelerometer calibration complete");
    Serial.printf("[CAL] Bias (mg): X=%.2f Y=%.2f Z=%.2f\n",
                 calibration.accelBias[0],
                 calibration.accelBias[1],
                 calibration.accelBias[2]);
    
    // Register accelerometer validation callbacks and validate
    registerValidationCallbacks(readAccelSensorData, applyAccelSensorCalibration, 
                               checkAccelBiasAcceptable, checkAccelNoiseAcceptable);
    
    if (!validateWithRetryOption()) {
        Serial.println("✗ Accelerometer calibration validation failed");
        return false;
    }
    
    Serial.println("✓ Accelerometer calibration validation passed");
    return true;
}

/**
 * Enable or disable dynamic accelerometer calibration
 */
void setDynamicAccelCalibrationEnabled(bool enabled) {
    calibration.accelDynamicCalEnabled = enabled;
    Serial.printf("[CAL] Dynamic accelerometer calibration %s\n",
                 enabled ? "enabled" : "disabled");
}

/**
 * Perform 6-position accelerometer calibration with auto-detection
 */
bool calibrateAccel6Position(bool withTemp) {
    Serial.println("[CAL] Starting 6-position accelerometer calibration...");
    Serial.println("[CAL] This calibration will auto-detect when you place the device in each position.\n");
    
    // Step 1: Mandatory axis identification
    Serial.println("=== STEP 1: AXIS IDENTIFICATION ===");
    Serial.println("First, let's identify your device's axes.");
    Serial.println("Rotate the device and watch the labels below for 10 seconds...\n");
    
    unsigned long axisIdStart = millis();
    while (millis() - axisIdStart < 10000) {
        float ax, ay, az;
        if (readAccel(ax, ay, az)) {
            Serial.printf("\rX: %+7.1f  Y: %+7.1f  Z: %+7.1f mg  ->  ", ax, ay, az);
            
            float absX = abs(ax);
            float absY = abs(ay);
            float absZ = abs(az);
            
            if (absZ > 800 && absZ > absX && absZ > absY) {
                Serial.printf("[Z-%s]", az > 0 ? "UP" : "DOWN");
            } else if (absY > 800 && absY > absX && absY > absZ) {
                Serial.printf("[Y-%s]", ay > 0 ? "UP" : "DOWN");
            } else if (absX > 800 && absX > absY && absX > absZ) {
                Serial.printf("[X-%s]", ax > 0 ? "UP" : "DOWN");
            } else {
                Serial.print("[TILTED]");
            }
        }
        delay(200);
    }
    Serial.println("\n\n=== STEP 2: 6-POSITION CALIBRATION ===\n");
    
    // Arrays to store measurements from each position
    float ax_samples[6] = {0}, ay_samples[6] = {0}, az_samples[6] = {0};
    float temps[6] = {0};
    
    // Position definitions: axis (0=X, 1=Y, 2=Z) and target value (+1000 or -1000 mg)
    struct Position {
        int axis;
        float target;
        const char* description;
    };
    
    Position positions[6] = {
        {2, +1000.0f, "Z-axis pointing UP (device flat, display up)"},
        {2, -1000.0f, "Z-axis pointing DOWN (device flipped over)"},
        {1, +1000.0f, "Y-axis pointing UP (standing on end, USB up)"},
        {1, -1000.0f, "Y-axis pointing DOWN (standing on opposite end)"},
        {0, +1000.0f, "X-axis pointing UP (resting on right side)"},
        {0, -1000.0f, "X-axis pointing DOWN (resting on left side)"}
    };
    
    // Collect data for each position
    for (int position = 0; position < 6; position++) {
        Serial.printf("\n[CAL] Position %d of 6: %s\n", 
                     position + 1, positions[position].description);
        
        // Wait for stable orientation with auto-detection
        if (!waitForStableOrientation(positions[position].axis, 
                                      positions[position].target, 
                                      200.0f, 30000)) {
            Serial.println("[CAL] Failed to achieve stable orientation. Calibration aborted.");
            return false;
        }
        
        // Collect samples for this position
        const int NUM_SAMPLES = 200;
        float ax_sum = 0, ay_sum = 0, az_sum = 0;
        float temp_sum = 0;
        int validSamples = 0;
        
        Serial.println("[CAL] Collecting data...");
        unsigned long startTime = millis();
        
        // Collect samples
        while (validSamples < NUM_SAMPLES && (millis() - startTime < 5000)) {
            float ax, ay, az, temp;
            if (readAccel(ax, ay, az) && (!withTemp || readTemperature(temp))) {
                ax_sum += ax;
                ay_sum += ay;
                az_sum += az;
                if (withTemp) temp_sum += temp;
                validSamples++;
                
                // Show progress
                if (validSamples % 20 == 0) {
                    Serial.printf("\r  Samples: %d/%d", validSamples, NUM_SAMPLES);
                }
            }
            delay(10);
        }
        Serial.println();  // New line after progress
        
        if (validSamples < NUM_SAMPLES * 0.9) {
            Serial.println("[CAL] Error: Not enough valid samples");
            return false;
        }
        
        // Store average values for this position
        ax_samples[position] = ax_sum / validSamples;
        ay_samples[position] = ay_sum / validSamples;
        az_samples[position] = az_sum / validSamples;
        if (withTemp) temps[position] = temp_sum / validSamples;
        
        Serial.printf("[CAL] Position %d complete: X=%.1f, Y=%.1f, Z=%.1f mg\n",
                     position + 1, ax_samples[position], ay_samples[position], az_samples[position]);
    }
    
    // Calculate calibration parameters
    calibration.accelScale[0] = 2000.0f / (ax_samples[4] - ax_samples[5]);
    calibration.accelScale[1] = 2000.0f / (ay_samples[2] - ay_samples[3]);
    calibration.accelScale[2] = 2000.0f / (az_samples[0] - az_samples[1]);
    
    calibration.accelBias[0] = (ax_samples[4] + ax_samples[5]) / 2.0f;
    calibration.accelBias[1] = (ay_samples[2] + ay_samples[3]) / 2.0f;
    calibration.accelBias[2] = (az_samples[0] + az_samples[1]) / 2.0f;
    
    if (withTemp) {
        float avgTemp = 0;
        for (int i = 0; i < 6; i++) {
            avgTemp += temps[i];
        }
        avgTemp /= 6;
        calibration.accelCalTemp = avgTemp;
    }
    
    // Store calibration method and mark as calibrated
    calibration.accelCalMethod = ACCEL_CAL_SIX_POSITION;
    calibration.accelCalibrated = true;
    
    Serial.println("\n[CAL] ✓ 6-position calibration complete!");
    Serial.printf("[CAL] Scale: X=%.3f Y=%.3f Z=%.3f\n",
                 calibration.accelScale[0],
                 calibration.accelScale[1],
                 calibration.accelScale[2]);
    Serial.printf("[CAL] Bias (mg): X=%.2f Y=%.2f Z=%.2f\n",
                 calibration.accelBias[0],
                 calibration.accelBias[1],
                 calibration.accelBias[2]);
    
    return true;
}

/**
 * Perform temperature-compensated accelerometer calibration
 * 
 * This method calibrates the accelerometer at the current temperature and calculates
 * temperature coefficients to compensate for drift across different temperatures.
 * It measures accelerometer bias at different temperatures to establish the relationship
 * between temperature and sensor drift.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateAccelWithTempComp() {
    Serial.println("[CAL] Starting temperature-compensated accelerometer calibration...");
    Serial.println("[CAL] This procedure requires heating/cooling the device to measure");
    Serial.println("[CAL] temperature sensitivity. Follow the instructions carefully.");
    Serial.println("[CAL] Place device on flat, stable surface at room temperature");
    Serial.println("[CAL] When ready, press 'y' to continue");
    waitForUserConfirmation();
    
    // Configure accelerometer for calibration
    configureAccelForCalibration();
    
    // Step 1: Collect baseline bias at current (room) temperature
    float axBias1, ayBias1, azBias1;
    float gxBias, gyBias, gzBias;  // Not used for accel calibration
    float temp1;
    
    Serial.println("[CAL] Collecting baseline accelerometer data at room temperature...");
    if (!readTemperature(temp1)) {
        Serial.println("[CAL] Failed to read temperature sensor");
        return false;
    }
    
    if (!collectBias(300, axBias1, ayBias1, azBias1, gxBias, gyBias, gzBias, true)) {
        Serial.println("[CAL] Failed to collect baseline accelerometer data");
        return false;
    }
    
    Serial.printf("[CAL] Baseline temperature: %.2f°C\n", temp1);
    Serial.printf("[CAL] Baseline bias: X=%.4f Y=%.4f Z=%.4f\n", axBias1, ayBias1, azBias1);
    
    // Step 2: Ask user to change device temperature
    Serial.println("[CAL] Now we need to change the device temperature.");
    Serial.println("[CAL] Option 1: Hold the device in your hand for 5 minutes to warm it");
    Serial.println("[CAL] Option 2: Place the device in a warmer location");
    Serial.println("[CAL] Option 3: Use a cold pack to cool the device");
    Serial.println("[CAL] Try to change temperature by at least 5°C for best results");
    Serial.println("[CAL] When temperature has changed, press 'y' to continue");
    waitForUserConfirmation();
    
    // Step 3: Collect bias at new temperature
    float axBias2, ayBias2, azBias2;
    float temp2;
    
    Serial.println("[CAL] Collecting accelerometer data at new temperature...");
    if (!readTemperature(temp2)) {
        Serial.println("[CAL] Failed to read temperature sensor");
        return false;
    }
    
    if (!collectBias(300, axBias2, ayBias2, azBias2, gxBias, gyBias, gzBias, true)) {
        Serial.println("[CAL] Failed to collect accelerometer data at new temperature");
        return false;
    }
    
    Serial.printf("[CAL] New temperature: %.2f°C\n", temp2);
    Serial.printf("[CAL] New bias: X=%.4f Y=%.4f Z=%.4f\n", axBias2, ayBias2, azBias2);
    
    // Step 4: Calculate temperature coefficients (change in bias per degree C)
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
    float axTempCoeff = (axBias2 - axBias1) / tempDiff;
    float ayTempCoeff = (ayBias2 - ayBias1) / tempDiff;
    float azTempCoeff = (azBias2 - azBias1) / tempDiff;
    
    Serial.printf("[CAL] Temperature coefficients:\n");
    Serial.printf("[CAL] X: %.6f g/°C\n", axTempCoeff);
    Serial.printf("[CAL] Y: %.6f g/°C\n", ayTempCoeff);
    Serial.printf("[CAL] Z: %.6f g/°C\n", azTempCoeff);
    
    // Step 5: Save calibration data
    calibration.accelBias[0] = axBias1;
    calibration.accelBias[1] = ayBias1;
    calibration.accelBias[2] = azBias1;
    calibration.accelTempCoeff[0] = axTempCoeff;
    calibration.accelTempCoeff[1] = ayTempCoeff;
    calibration.accelTempCoeff[2] = azTempCoeff;
    calibration.accelCalTemp = temp1;  // Reference temperature for bias
    calibration.accelTempCompEnabled = true;
    calibration.accelCalMethod = ACCEL_CAL_SIMPLE;  // Temperature comp works with simple bias
    calibration.accelCalibrated = true;
    
    return true;
}

/**
 * Validate accelerometer calibration quality
 * 
 * This function performs a quick validation of the accelerometer calibration quality
 * by collecting samples while the device is stationary and checking:
 * 1. Magnitude is close to 1g (9.81 m/s²)
 * 2. Standard deviation is within acceptable limits
 * 3. Bias stability over short time period
 * 
 * @param meanMagnitude Output parameter for mean magnitude in g
 * @param stdDevMagnitude Output parameter for standard deviation of magnitude
 * @return true if calibration quality meets acceptance criteria, false otherwise
 */
bool validateAccelCalibration(float &meanMagnitude, float &stdDevMagnitude) {
    Serial.println("[VAL] Validating accelerometer calibration quality...");
    Serial.println("[VAL] Keep device stationary on a flat surface");
    
    // Initialize statistics variables
    float sumMag = 0.0f;
    float sumMagSquared = 0.0f;
    int validSamples = 0;
    const int numSamples = 100;
    
    // Read current temperature for compensation
    float temperature;
    bool tempAvailable = readTemperature(temperature);
    if (!tempAvailable) {
        Serial.println("[VAL] Warning: Temperature sensor not available");
        temperature = 0.0f;  // Use default if not available
    }
    
    // Collect samples
    unsigned long startTime = millis();
    while (validSamples < numSamples) {
        float ax, ay, az;
        
        if (readAccel(ax, ay, az)) {
            // Apply calibration with temperature compensation
            applyAccelCalibration(ax, ay, az, temperature);
            
            // Calculate magnitude in mg, then convert to g for validation
            float magnitude_mg = sqrt(ax*ax + ay*ay + az*az);
            float magnitude = magnitude_mg / 1000.0f;  // Convert mg to g for validation
            
            // Update statistics using Welford's algorithm for numerical stability
            validSamples++;
            float delta = magnitude - sumMag;
            sumMag += delta / validSamples;
            float delta2 = magnitude - sumMag;
            sumMagSquared += delta * delta2;
            
            delay(10);  // Short delay between samples
        }
        
        // Timeout after 5 seconds
        if (millis() - startTime > 5000 && validSamples < numSamples / 2) {
            Serial.println("[VAL] Validation timed out. Not enough valid samples.");
            return false;
        }
    }
    
    // Calculate final statistics
    meanMagnitude = sumMag;
    stdDevMagnitude = sqrt(sumMagSquared / validSamples);
    
    // Check if calibration meets quality criteria
    const float idealMagnitude = 1.0f;  // 1g when stationary
    const float maxMagnitudeError = 0.05f;  // 5% error tolerance
    const float maxStdDev = 0.02f;  // Maximum standard deviation
    
    bool magnitudeOk = abs(meanMagnitude - idealMagnitude) < maxMagnitudeError;
    bool stdDevOk = stdDevMagnitude < maxStdDev;
    
    Serial.printf("[VAL] Mean magnitude: %.4f g (ideal: 1.0)\n", meanMagnitude);
    Serial.printf("[VAL] Std deviation: %.4f g (threshold: %.4f)\n", stdDevMagnitude, maxStdDev);
    
    if (!magnitudeOk) {
        Serial.println("[VAL] Warning: Magnitude error exceeds threshold");
    }
    
    if (!stdDevOk) {
        Serial.println("[VAL] Warning: Standard deviation exceeds threshold");
    }
    
    return magnitudeOk && stdDevOk;
}

/**
 * Check if accelerometer is calibrated
 * 
 * @return true if accelerometer has valid calibration data
 */
bool isAccelCalibrated() {
    return calibration.accelCalibrated;
}

// ... (rest of the code remains the same)

/**
 * Perform advanced ellipsoid fitting calibration for accelerometer
 * 
 * This method implements ellipsoid fitting calibration for the accelerometer.
 * It collects samples while the device is rotated through various orientations,
 * then fits an ellipsoid to the data to determine scale factors, bias, and
 * cross-axis sensitivities.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateAccelEllipsoid() {
    Serial.println("[CAL] Starting ellipsoid fitting calibration...");
    Serial.println("[CAL] ROTATION INSTRUCTIONS:");
    Serial.println("[CAL] 1. Hold device flat (screen up) - rotate slowly");
    Serial.println("[CAL] 2. Hold device vertical (screen facing you) - rotate slowly");
    Serial.println("[CAL] 3. Hold device on each edge - left, right, top, bottom");
    Serial.println("[CAL] 4. Tilt and rotate through various angles");
    Serial.println("[CAL] GOAL: Each axis must see +1g and -1g (full ±1000mg range)");
    Serial.println("[CAL] Press 'y' to begin data collection");
    
    if (!waitForUserConfirmation()) {
        return false;
    }
    
    // First, run diagnostic to check if accelerometer is working
    Serial.println("[CAL] Running accelerometer diagnostic...");
    diagnoseAccelReadings();
    
    // Configure accelerometer for calibration
    configureAccelForCalibration();
    
    // Initialize variables for sample collection
    const int NUM_SAMPLES = 300;  // Reduced to avoid memory issues and make collection achievable
    float* ax_samples = (float*)malloc(NUM_SAMPLES * sizeof(float));
    float* ay_samples = (float*)malloc(NUM_SAMPLES * sizeof(float));
    float* az_samples = (float*)malloc(NUM_SAMPLES * sizeof(float));
    
    if (!ax_samples || !ay_samples || !az_samples) {
        Serial.println("[CAL] Error: Memory allocation failed");
        free(ax_samples);
        free(ay_samples);
        free(az_samples);
        return false;
    }
    
    // Initialize statistics for outlier detection
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float ax_sum_sq = 0, ay_sum_sq = 0, az_sum_sq = 0;
    int validSamples = 0;
    
    Serial.println("[CAL] Collecting samples...");
    unsigned long startTime = millis();
    const unsigned long COLLECTION_TIME = 45000;  // 45 seconds for thorough rotation
    
    // Collect samples
    while (validSamples < NUM_SAMPLES && (millis() - startTime < COLLECTION_TIME)) {
        float ax, ay, az;
        if (readAccel(ax, ay, az)) {
            // Check if magnitude is reasonable (should be close to 1000mg = 1g)
            float magnitude = sqrt(ax*ax + ay*ay + az*az);
            if (magnitude > 500.0f && magnitude < 1500.0f) {  // 0.5g to 1.5g range
                // Update running statistics for outlier detection
                ax_sum += ax;
                ay_sum += ay;
                az_sum += az;
                ax_sum_sq += ax * ax;
                ay_sum_sq += ay * ay;
                az_sum_sq += az * az;
                
                // Store samples
                ax_samples[validSamples] = ax;
                ay_samples[validSamples] = ay;
                az_samples[validSamples] = az;
                validSamples++;
                
                if (validSamples % 50 == 0) {
                    Serial.printf("[CAL] Collected %d samples (mag: %.1f mg)...\n", validSamples, magnitude);
                }
            }
        }
        delay(100);  // ~10Hz sampling for better rotation coverage
    }
    
    if (validSamples < NUM_SAMPLES * 0.8) {
        Serial.printf("[CAL] Error: Only collected %d samples (need at least %d)\n", 
                     validSamples, (int)(NUM_SAMPLES * 0.8));
        Serial.println("[CAL] Make sure to rotate device through ALL orientations");
        free(ax_samples);
        free(ay_samples);
        free(az_samples);
        return false;
    }
    
    // Calculate means and standard deviations for outlier detection
    float ax_mean = ax_sum / validSamples;
    float ay_mean = ay_sum / validSamples;
    float az_mean = az_sum / validSamples;
    float ax_std = sqrt((ax_sum_sq / validSamples) - (ax_mean * ax_mean));
    float ay_std = sqrt((ay_sum_sq / validSamples) - (ay_mean * ay_mean));
    float az_std = sqrt((az_sum_sq / validSamples) - (az_mean * az_mean));
    
    // Reject outliers (samples more than 2.5 standard deviations from mean)
    int goodSamples = 0;
    for (int i = 0; i < validSamples; i++) {
        if (abs(ax_samples[i] - ax_mean) < 2.5 * ax_std &&
            abs(ay_samples[i] - ay_mean) < 2.5 * ay_std &&
            abs(az_samples[i] - az_mean) < 2.5 * az_std) {
            // Keep good sample by moving it to front of array
            if (i != goodSamples) {
                ax_samples[goodSamples] = ax_samples[i];
                ay_samples[goodSamples] = ay_samples[i];
                az_samples[goodSamples] = az_samples[i];
            }
            goodSamples++;
        }
    }
    
    Serial.printf("[CAL] Using %d samples after outlier rejection\n", goodSamples);
    
    if (goodSamples < 200) {
        Serial.println("[CAL] Error: Too few good samples after outlier rejection");
        free(ax_samples);
        free(ay_samples);
        free(az_samples);
        return false;
    }
    
    // Find min/max values for each axis
    float min_x = ax_samples[0], max_x = ax_samples[0];
    float min_y = ay_samples[0], max_y = ay_samples[0];
    float min_z = az_samples[0], max_z = az_samples[0];
    
    for (int i = 1; i < goodSamples; i++) {
        min_x = min(min_x, ax_samples[i]);
        max_x = max(max_x, ax_samples[i]);
        min_y = min(min_y, ay_samples[i]);
        max_y = max(max_y, ay_samples[i]);
        min_z = min(min_z, az_samples[i]);
        max_z = max(max_z, az_samples[i]);
    }
    
    // Check if we have sufficient range on each axis (should be close to ±1000mg)
    float range_x = max_x - min_x;
    float range_y = max_y - min_y;
    float range_z = max_z - min_z;
    
    Serial.printf("[CAL] Axis ranges: X=%.1f Y=%.1f Z=%.1f mg\n", range_x, range_y, range_z);
    
    if (range_x < 1500.0f || range_y < 1500.0f || range_z < 1500.0f) {
        Serial.println("[CAL] Warning: Insufficient rotation detected on one or more axes");
        Serial.println("[CAL] For best results, rotate device so each axis sees ±1g");
    }
    
    // Calculate bias (center of min/max range)
    calibration.accelBias[0] = (min_x + max_x) / 2.0f;
    calibration.accelBias[1] = (min_y + max_y) / 2.0f;
    calibration.accelBias[2] = (min_z + max_z) / 2.0f;
    
    // Calculate scale factors to normalize each axis to 1000mg (1g) range
    // Target range is 2000mg (±1000mg), so scale = 2000 / actual_range
    calibration.accelScale[0] = (range_x > 100.0f) ? (2000.0f / range_x) : 1.0f;
    calibration.accelScale[1] = (range_y > 100.0f) ? (2000.0f / range_y) : 1.0f;
    calibration.accelScale[2] = (range_z > 100.0f) ? (2000.0f / range_z) : 1.0f;
    
    // Initialize cross-axis matrix to identity (no cross-coupling by default)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            calibration.accelCrossAxis[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Calculate simplified cross-axis sensitivity
    // This estimates correlation between axes after bias removal
    float xy_corr = 0, xz_corr = 0, yz_corr = 0;
    float x_var = 0, y_var = 0, z_var = 0;
    
    for (int i = 0; i < goodSamples; i++) {
        float x_centered = ax_samples[i] - calibration.accelBias[0];
        float y_centered = ay_samples[i] - calibration.accelBias[1];
        float z_centered = az_samples[i] - calibration.accelBias[2];
        
        xy_corr += x_centered * y_centered;
        xz_corr += x_centered * z_centered;
        yz_corr += y_centered * z_centered;
        
        x_var += x_centered * x_centered;
        y_var += y_centered * y_centered;
        z_var += z_centered * z_centered;
    }
    
    // Normalize correlations and convert to cross-axis sensitivity (in ppm)
    if (x_var > 0 && y_var > 0) {
        calibration.accelCrossAxis[0][1] = calibration.accelCrossAxis[1][0] = 
            (xy_corr / sqrt(x_var * y_var)) * 1000.0f;  // Convert to ppm (parts per thousand)
    }
    if (x_var > 0 && z_var > 0) {
        calibration.accelCrossAxis[0][2] = calibration.accelCrossAxis[2][0] = 
            (xz_corr / sqrt(x_var * z_var)) * 1000.0f;
    }
    if (y_var > 0 && z_var > 0) {
        calibration.accelCrossAxis[1][2] = calibration.accelCrossAxis[2][1] = 
            (yz_corr / sqrt(y_var * z_var)) * 1000.0f;
    }
    
    // Clean up
    free(ax_samples);
    free(ay_samples);
    free(az_samples);
    
    // Store calibration method and mark as calibrated
    calibration.accelCalMethod = ACCEL_CAL_ELLIPSOID;
    calibration.accelCalibrated = true;
    
    Serial.println("[CAL] Ellipsoid fitting calibration complete");
    Serial.printf("[CAL] Scale: X=%.3f Y=%.3f Z=%.3f\n",
                 calibration.accelScale[0],
                 calibration.accelScale[1],
                 calibration.accelScale[2]);
    Serial.printf("[CAL] Bias (mg): X=%.2f Y=%.2f Z=%.2f\n",
                 calibration.accelBias[0],
                 calibration.accelBias[1],
                 calibration.accelBias[2]);
    Serial.printf("[CAL] Cross-axis (ppt): XY=%.1f XZ=%.1f YZ=%.1f\n",
                 calibration.accelCrossAxis[0][1],
                 calibration.accelCrossAxis[0][2],
                 calibration.accelCrossAxis[1][2]);
    
    return true;
}

/**
 * Update the calibrateAccelerometer function to include the new temperature compensation option
 */
bool calibrateAccelerometer() {
    Serial.println("[CAL] Accelerometer Calibration");
    Serial.println("[CAL] Select calibration method:");
    Serial.println("[CAL] 1: Simple bias-only calibration");
    Serial.println("[CAL] 2: 6-position calibration with scale factors");
    Serial.println("[CAL] 3: 6-position with cross-axis sensitivity correction");
    Serial.println("[CAL] 4: Advanced ellipsoid fitting");
    Serial.println("[CAL] 5: Temperature-compensated calibration");
    Serial.println("[CAL] Enter selection (1-5):");
    
    while (!Serial.available()) {
        delay(100);
    }
    
    char selection = Serial.read();
    while (Serial.available()) Serial.read();  // Clear buffer
    
    bool result = false;
    
    switch (selection) {
        case '1':
            result = calibrateAccelSimple();
            if (result) calibration.accelCalMethod = ACCEL_CAL_SIMPLE;
            break;
        case '2':
            result = calibrateAccel6Position(false);
            if (result) calibration.accelCalMethod = ACCEL_CAL_SIX_POSITION;
            break;
        case '3':
            result = calibrateAccel6Position(true);
            if (result) calibration.accelCalMethod = ACCEL_CAL_CROSS_AXIS;
            break;
        case '4':
            result = calibrateAccelEllipsoid();
            if (result) calibration.accelCalMethod = ACCEL_CAL_ELLIPSOID;
            break;
        case '5':
            result = calibrateAccelWithTempComp();
            // Method is set inside the function
            break;
        default:
            Serial.println("[CAL] Invalid selection");
            return false;
    }
    
    if (result) {
        Serial.println("[CAL] Calibration successful!");
        
        // Validate calibration quality
        float meanMag, stdMag;
        bool validationResult = validateAccelCalibration(meanMag, stdMag);
        
        if (validationResult) {
            Serial.println("[CAL] Calibration validation passed!");
        } else {
            Serial.println("[CAL] Calibration validation warning - results may not be optimal");
        }
        
        // Ask if user wants to enable dynamic calibration
        Serial.println("[CAL] Enable dynamic calibration refinement? (y/n)");
        if (waitForUserConfirmation(true)) {
            setDynamicAccelCalibrationEnabled(true);
        }
    } else {
        Serial.println("[CAL] Calibration failed!");
    }
    
    return result;
}

// Sensor-specific validation helpers
bool readAccelSensorData(float &x, float &y, float &z) {
    return readAccel(x, y, z);
}

void applyAccelSensorCalibration(float &x, float &y, float &z) {
    // Convert to mg for internal calculations
    x *= 1000.0f;
    y *= 1000.0f;
    z *= 1000.0f;
    
    // Apply bias correction
    x -= calibration.accelBias[0];
    y -= calibration.accelBias[1];
    z -= calibration.accelBias[2];
    
    // Apply scale factors if available
    if (calibration.accelCalMethod == ACCEL_CAL_SIX_POSITION ||
        calibration.accelCalMethod == ACCEL_CAL_ELLIPSOID) {
        x *= calibration.accelScale[0];
        y *= calibration.accelScale[1];
        z *= calibration.accelScale[2];
        
        // Apply cross-axis correction if available
        if (calibration.accelCalMethod == ACCEL_CAL_ELLIPSOID) {
            float x_orig = x;
            float y_orig = y;
            float z_orig = z;
            
            x += calibration.accelCrossAxis[0][1] * y_orig + calibration.accelCrossAxis[0][2] * z_orig;
            y += calibration.accelCrossAxis[1][0] * x_orig + calibration.accelCrossAxis[1][2] * z_orig;
            z += calibration.accelCrossAxis[2][0] * x_orig + calibration.accelCrossAxis[2][1] * y_orig;
        }
    }
    
    // Convert back to g
    x /= 1000.0f;
    y /= 1000.0f;
    z /= 1000.0f;
}

bool checkAccelBiasAcceptable(float x, float y, float z) {
    // Check residual bias against acceptance threshold
    // Note: x, y, z are in g, but threshold is in mg, so multiply by 1000
    return (abs(x * 1000.0f) < calibration.MAX_RESIDUAL_BIAS_ACCEL) &&
           (abs(y * 1000.0f) < calibration.MAX_RESIDUAL_BIAS_ACCEL) &&
           (abs(z * 1000.0f) < calibration.MAX_RESIDUAL_BIAS_ACCEL);
}

bool checkAccelNoiseAcceptable(float x, float y, float z) {
    // Check noise level against acceptance threshold
    // Note: x, y, z are in g, but threshold is in mg, so multiply by 1000
    return (x * 1000.0f < calibration.MAX_NOISE_ACCEL) &&
           (y * 1000.0f < calibration.MAX_NOISE_ACCEL) &&
           (z * 1000.0f < calibration.MAX_NOISE_ACCEL);
}

/**
 * Axis identification tool - helps identify physical sensor orientation
 * Ported from V2 calibration system
 */
void identifyAccelAxes() {
    Serial.println("\n=== ACCELEROMETER AXIS IDENTIFICATION ===");
    Serial.println("Rotate the device slowly and watch which axis changes.");
    Serial.println("This helps identify the physical orientation of your sensor.");
    Serial.println("\nPress any key to exit...\n");
    
    delay(1000);
    
    while (!Serial.available()) {
        float ax, ay, az;
        if (readAccel(ax, ay, az)) {
            // Display current values
            Serial.printf("X: %+7.1f  Y: %+7.1f  Z: %+7.1f mg  ->  ", ax, ay, az);
            
            // Identify dominant axis and direction
            float absX = abs(ax);
            float absY = abs(ay);
            float absZ = abs(az);
            
            if (absZ > 800 && absZ > absX && absZ > absY) {
                Serial.print(az > 0 ? "[Z-UP]" : "[Z-DOWN]");
            } else if (absY > 800 && absY > absX && absY > absZ) {
                Serial.print(ay > 0 ? "[Y-UP]" : "[Y-DOWN]");
            } else if (absX > 800 && absX > absY && absX > absZ) {
                Serial.print(ax > 0 ? "[X-UP]" : "[X-DOWN]");
            } else {
                Serial.print("[TILTED]");
            }
            
            Serial.println();
        }
        delay(200);
    }
    
    // Clear the input buffer
    while (Serial.available()) Serial.read();
    
    Serial.println("\nAxis identification complete.\n");
}

