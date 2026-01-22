#include "gyro_calibration.h"
#include "imu_validation.h"
#include "temp_sensor.h"

// Forward declarations of sensor read helpers defined in main.cpp
bool readGyro(float& gx, float& gy, float& gz);
bool readAccel(float& ax, float& ay, float& az);

// Forward declaration of ICM-20948 access functions
extern void setBank(uint8_t bank);
extern uint8_t readRegister(uint8_t reg);
extern void writeRegister(uint8_t reg, uint8_t value);

/**
 * Configure the gyroscope registers for optimal calibration
 * 
 * This function configures the ICM-20948 gyroscope with settings optimized for
 * accurate calibration:
 * - ±250 dps full-scale range for maximum sensitivity
 * - DLPF enabled with appropriate bandwidth to reduce noise
 * - Sample rate optimized for calibration
 */
void configureGyroForCalibration() {
    // Wake up the device if needed (Bank 0, Register 0x06 - PWR_MGMT_1)
    setBank(0);
    writeRegister(0x06, 0x01);  // Auto-select clock source
    delay(10);
    
    // Configure gyroscope settings (Bank 2)
    setBank(2);
    
    // Set gyroscope full-scale range to ±250 dps for maximum sensitivity
    // GYRO_CONFIG_1 register (0x01), bits [2:1] = 00 for ±250 dps
    writeRegister(0x01, 0x00);  // 0x00 = ±250 dps
    delay(10);
    
    // Enable Digital Low Pass Filter (DLPF) for gyroscope
    // GYRO_CONFIG_1 register (0x01), bit [0] = 1 to enable DLPF
    uint8_t gyro_config_1 = readRegister(0x01);
    writeRegister(0x01, gyro_config_1 | 0x01);  // Set bit 0 to enable DLPF
    delay(10);
    
    // Configure DLPF bandwidth
    // GYRO_CONFIG_2 register (0x02), bits [2:0] = 000 for 196.6Hz bandwidth
    writeRegister(0x02, 0x00);  // 0x00 = 196.6Hz bandwidth
    delay(10);
    
    // Configure sample rate divider for gyroscope
    // GYRO_SMPLRT_DIV register (0x00), value = 0 for maximum sample rate
    writeRegister(0x00, 0x00);  // No additional division
    delay(10);
    
    // Return to Bank 0
    setBank(0);
    
    Serial.println("[CAL] Gyroscope configured for calibration:");
    Serial.println("[CAL] - Full-scale range: ±250 dps");
    Serial.println("[CAL] - DLPF enabled with 196.6Hz bandwidth");
    Serial.println("[CAL] - Maximum sample rate");
}

/**
 * Apply gyroscope calibration to raw sensor readings
 * 
 * This function applies the calibration parameters to the raw gyroscope readings,
 * including temperature compensation if enabled.
 * 
 * @param gx_dps X-axis gyroscope reading in degrees per second (in/out parameter)
 * @param gy_dps Y-axis gyroscope reading in degrees per second (in/out parameter)
 * @param gz_dps Z-axis gyroscope reading in degrees per second (in/out parameter)
 * @param temperature Current temperature in degrees Celsius (for temperature compensation)
 */
void applyGyroCalibration(float& gx_dps, float& gy_dps, float& gz_dps, float temperature) {
    // Only apply calibration if gyroscope is calibrated
    if (calibration.gyroCalibrated) {
        // Apply bias correction
        gx_dps -= calibration.gyroBias[0];
        gy_dps -= calibration.gyroBias[1];
        gz_dps -= calibration.gyroBias[2];
        
        // Apply temperature compensation if enabled
        if (calibration.gyroTempCompEnabled) {
            // Calculate temperature difference from calibration temperature
            float tempDiff = temperature - calibration.gyroCalTemp;
            
            // Apply temperature coefficients
            gx_dps -= calibration.gyroTempCoeff[0] * tempDiff;
            gy_dps -= calibration.gyroTempCoeff[1] * tempDiff;
            gz_dps -= calibration.gyroTempCoeff[2] * tempDiff;
        }
    }
}

/**
 * Perform simple gyroscope calibration (bias-only)
 * 
 * This basic calibration method collects gyroscope data while the device is
 * stationary and calculates the bias/offset for each axis.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateGyroSimple() {
    Serial.println("[CAL] Starting simple gyroscope calibration...");
    Serial.println("[CAL] Place the device on a flat, stable surface");
    Serial.println("[CAL] When ready, press 'y' to start data collection");
    waitForUserConfirmation();
    
    // Configure gyroscope for calibration
    configureGyroForCalibration();
    
    bool success = false;
    do {
        // Collect gyroscope data
        const int numSamples = 1000;
        float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
        int validSamples = 0;
        
        Serial.println("[CAL] Collecting gyroscope data. Keep the device still...");
        
        unsigned long startTime = millis();
        unsigned long lastFeedbackTime = startTime;
        
        // Collect data for a few seconds
        while (validSamples < numSamples) {
            float gx, gy, gz;
            
            if (readGyro(gx, gy, gz)) {
                // Add to running sum
                gx_sum += gx;
                gy_sum += gy;
                gz_sum += gz;
                validSamples++;
                
                // Provide feedback every second
                if (millis() - lastFeedbackTime > 1000) {
                    Serial.print("[CAL] Collected ");
                    Serial.print(validSamples);
                    Serial.print("/");
                    Serial.print(numSamples);
                    Serial.println(" samples");
                    lastFeedbackTime = millis();
                }
            }
            
            delay(5);
            
            // Timeout after 10 seconds
            if (millis() - startTime > 10000 && validSamples < numSamples * 0.9) {
                Serial.println("[CAL] Calibration timed out. Not enough valid samples.");
                return false;
            }
        }
        
        // Calculate average bias
        calibration.gyroBias[0] = gx_sum / validSamples;
        calibration.gyroBias[1] = gy_sum / validSamples;
        calibration.gyroBias[2] = gz_sum / validSamples;
        
        // Read current temperature
        float currentTemp = 0.0f;
        readTemperature(currentTemp);
        calibration.gyroCalTemp = currentTemp;
        calibration.referenceTemp = currentTemp;
        
        // Set calibration flags
        calibration.gyroTempCompEnabled = false;
        calibration.gyroCalibrated = true;
        
        Serial.println("[CAL] Simple gyroscope calibration complete");
        Serial.print("[CAL] Bias (dps): X=");
        Serial.print(calibration.gyroBias[0], 4);
        Serial.print(" Y=");
        Serial.print(calibration.gyroBias[1], 4);
        Serial.print(" Z=");
        Serial.println(calibration.gyroBias[2], 4);
        Serial.print("[CAL] Temperature: ");
        Serial.print(currentTemp);
        Serial.println("°C");
        
        // Register gyroscope validation callbacks and validate
        registerValidationCallbacks(readGyroSensorData, applyGyroSensorCalibration, 
                                   checkGyroBiasAcceptable, checkGyroNoiseAcceptable);
        
        // Validate calibration quality
        if (!validateWithRetryOption()) {
            return false;
        }
        
        success = true;
    } while (!success);
    
    return true;
}

/**
 * Perform temperature-compensated gyroscope calibration
 * 
 * This method calibrates the gyroscope at the current temperature and calculates
 * temperature coefficients to compensate for drift across different temperatures.
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateGyroWithTempComp() {
    Serial.println("[CAL] Starting temperature-compensated gyroscope calibration...");
    Serial.println("[CAL] This procedure requires calibrating at different temperatures");
    Serial.println("[CAL] Place the device on a flat, stable surface");
    Serial.println("[CAL] When ready, press 'y' to start calibration at room temperature");
    waitForUserConfirmation();
    
    // Configure gyroscope for calibration
    configureGyroForCalibration();
    
    bool success = false;
    do {
        // Step 1: Calibrate at room temperature
        Serial.println("[CAL] Calibrating at room temperature...");
        
        // Collect gyroscope data at room temperature
        const int numSamples = 1000;
        float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
        int validSamples = 0;
        
        unsigned long startTime = millis();
        unsigned long lastFeedbackTime = startTime;
        
        // Collect data for a few seconds
        while (validSamples < numSamples) {
            float gx, gy, gz;
            
            if (readGyro(gx, gy, gz)) {
                // Add to running sum
                gx_sum += gx;
                gy_sum += gy;
                gz_sum += gz;
                validSamples++;
                
                // Provide feedback every second
                if (millis() - lastFeedbackTime > 1000) {
                    Serial.print("[CAL] Collected ");
                    Serial.print(validSamples);
                    Serial.print("/");
                    Serial.print(numSamples);
                    Serial.println(" samples");
                    lastFeedbackTime = millis();
                }
            }
            
            delay(5);
            
            // Timeout after 10 seconds
            if (millis() - startTime > 10000 && validSamples < numSamples * 0.9) {
                Serial.println("[CAL] Calibration timed out. Not enough valid samples.");
                return false;
            }
        }
        
        // Calculate bias at room temperature
        float roomTempBias[3];
        roomTempBias[0] = gx_sum / validSamples;
        roomTempBias[1] = gy_sum / validSamples;
        roomTempBias[2] = gz_sum / validSamples;
        
        // Read current temperature
        float roomTemp = 0.0f;
        readTemperature(roomTemp);
        
        Serial.print("[CAL] Room temperature: ");
        Serial.print(roomTemp);
        Serial.println("°C");
        Serial.print("[CAL] Room temperature bias (dps): X=");
        Serial.print(roomTempBias[0], 4);
        Serial.print(" Y=");
        Serial.print(roomTempBias[1], 4);
        Serial.print(" Z=");
        Serial.println(roomTempBias[2], 4);
        
        // Step 2: Ask user to change the temperature
        Serial.println("[CAL] Now we need to calibrate at a different temperature.");
        Serial.println("[CAL] Please warm up the device (e.g., hold it in your hands for a minute)");
        Serial.println("[CAL] or cool it down (e.g., place it near a cool surface).");
        Serial.println("[CAL] When the temperature has changed by at least 5°C,");
        Serial.println("[CAL] place the device on a flat, stable surface again");
        Serial.println("[CAL] and press 'y' to continue.");
        waitForUserConfirmation();
        
        // Step 3: Calibrate at different temperature
        Serial.println("[CAL] Calibrating at different temperature...");
        
        // Reset variables
        gx_sum = 0.0f;
        gy_sum = 0.0f;
        gz_sum = 0.0f;
        validSamples = 0;
        
        startTime = millis();
        lastFeedbackTime = startTime;
        
        // Collect data for a few seconds
        while (validSamples < numSamples) {
            float gx, gy, gz;
            
            if (readGyro(gx, gy, gz)) {
                // Add to running sum
                gx_sum += gx;
                gy_sum += gy;
                gz_sum += gz;
                validSamples++;
                
                // Provide feedback every second
                if (millis() - lastFeedbackTime > 1000) {
                    Serial.print("[CAL] Collected ");
                    Serial.print(validSamples);
                    Serial.print("/");
                    Serial.print(numSamples);
                    Serial.println(" samples");
                    lastFeedbackTime = millis();
                }
            }
            
            delay(5);
            
            // Timeout after 10 seconds
            if (millis() - startTime > 10000 && validSamples < numSamples * 0.9) {
                Serial.println("[CAL] Calibration timed out. Not enough valid samples.");
                return false;
            }
        }
        
        // Calculate bias at different temperature
        float diffTempBias[3];
        diffTempBias[0] = gx_sum / validSamples;
        diffTempBias[1] = gy_sum / validSamples;
        diffTempBias[2] = gz_sum / validSamples;
        
        // Read current temperature
        float diffTemp = 0.0f;
        readTemperature(diffTemp);
        
        Serial.print("[CAL] Different temperature: ");
        Serial.print(diffTemp);
        Serial.println("°C");
        Serial.print("[CAL] Different temperature bias (dps): X=");
        Serial.print(diffTempBias[0], 4);
        Serial.print(" Y=");
        Serial.print(diffTempBias[1], 4);
        Serial.print(" Z=");
        Serial.println(diffTempBias[2], 4);
        
        // Check if temperature difference is sufficient
        float tempDiff = abs(diffTemp - roomTemp);
        if (tempDiff < 3.0f) {
            Serial.println("[CAL] Temperature difference too small. Need at least 3°C difference.");
            Serial.println("[CAL] Using simple calibration instead.");
            
            // Use room temperature calibration as simple calibration
            calibration.gyroBias[0] = roomTempBias[0];
            calibration.gyroBias[1] = roomTempBias[1];
            calibration.gyroBias[2] = roomTempBias[2];
            calibration.gyroCalTemp = roomTemp;
            calibration.referenceTemp = roomTemp;
            calibration.gyroTempCompEnabled = false;
            calibration.gyroCalibrated = true;
            
            return true;
        }
        
        // Step 4: Calculate temperature coefficients
        calibration.gyroTempCoeff[0] = (diffTempBias[0] - roomTempBias[0]) / tempDiff;
        calibration.gyroTempCoeff[1] = (diffTempBias[1] - roomTempBias[1]) / tempDiff;
        calibration.gyroTempCoeff[2] = (diffTempBias[2] - roomTempBias[2]) / tempDiff;
        
        // Use room temperature as reference
        calibration.gyroBias[0] = roomTempBias[0];
        calibration.gyroBias[1] = roomTempBias[1];
        calibration.gyroBias[2] = roomTempBias[2];
        calibration.gyroCalTemp = roomTemp;
        calibration.referenceTemp = roomTemp;
        calibration.gyroTempCompEnabled = true;
        calibration.gyroCalibrated = true;
        
        Serial.println("[CAL] Temperature-compensated gyroscope calibration complete");
        Serial.print("[CAL] Reference temperature: ");
        Serial.print(roomTemp);
        Serial.println("°C");
        Serial.print("[CAL] Temperature coefficients (dps/°C): ");
        Serial.print(calibration.gyroTempCoeff[0], 6);
        Serial.print(" Y=");
        Serial.print(calibration.gyroTempCoeff[1], 6);
        Serial.print(" Z=");
        Serial.println(calibration.gyroTempCoeff[2], 6);
        
        // Register gyroscope validation callbacks and validate
        registerValidationCallbacks(readGyroSensorData, applyGyroSensorCalibration, 
                                   checkGyroBiasAcceptable, checkGyroNoiseAcceptable);
        
        // Validate calibration quality
        if (!validateWithRetryOption()) {
            return false;
        }
        success = true;
    } while (!success);
    
    return true;
}

// Simple alias for backward compatibility with older code references
bool calibrateGyroWithTemp() {
    return calibrateGyroWithTempComp();
}

/**
 * Perform 6-position gyroscope calibration with auto-detection
 * 
 * This method implements the 6-position (cube) calibration method for gyroscopes.
 * The device is placed in 6 different orientations (each face of a cube), and
 * gyroscope data is collected in each position. This helps account for gravity-induced errors.
 * 
 * @param withTempComp If true, also perform temperature compensation
 * @return true if calibration was successful, false otherwise
 */
bool calibrateGyro6Position(bool withTempComp) {
    Serial.println("[CAL] Starting 6-position gyroscope calibration...");
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
    
    // Configure gyroscope for calibration
    configureGyroForCalibration();
    
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
    
    bool success = false;
    do {
        // Arrays to store data for each position
        float positionBias[6][3] = {0};
        float positionTemp[6] = {0};
        
        // Collect data for each position
        for (int position = 0; position < 6; position++) {
            Serial.printf("\n[CAL] Position %d of 6: %s\n", 
                         position + 1, positions[position].description);
            
            // Wait for stable orientation with auto-detection
            if (!waitForStableOrientation(positions[position].axis, 
                                          positions[position].target, 
                                          200.0f, 30000)) {
                Serial.println("[CAL] Failed to achieve stable orientation.");
                Serial.println("[CAL] Press 'y' to retry this position, or 'n' to abort");
                if (!waitForUserConfirmation()) {
                    return false;
                }
                position--; // Retry this position
                continue;
            }
            
            // Collect gyroscope data
            const int numSamples = 500;
            float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
            int validSamples = 0;
            
            Serial.println("[CAL] Collecting gyroscope data. Keep the device still...");
            
            unsigned long startTime = millis();
            unsigned long lastFeedbackTime = startTime;
            
            // Collect data for a few seconds
            while (validSamples < numSamples) {
                float gx, gy, gz;
                
                if (readGyro(gx, gy, gz)) {
                    // Add to running sum
                    gx_sum += gx;
                    gy_sum += gy;
                    gz_sum += gz;
                    validSamples++;
                    
                    // Provide feedback every second
                    if (millis() - lastFeedbackTime > 1000) {
                        Serial.print("[CAL] Collected ");
                        Serial.print(validSamples);
                        Serial.print("/");
                        Serial.print(numSamples);
                        Serial.println(" samples");
                        lastFeedbackTime = millis();
                    }
                }
                
                delay(5);
                
                // Timeout after 10 seconds
                if (millis() - startTime > 10000 && validSamples < numSamples * 0.9) {
                    Serial.println("[CAL] Calibration timed out. Not enough valid samples.");
                    position--; // Retry this position
                    continue;
                }
            }
            
            // Calculate average bias for this position
            positionBias[position][0] = gx_sum / validSamples;
            positionBias[position][1] = gy_sum / validSamples;
            positionBias[position][2] = gz_sum / validSamples;
            
            // Read current temperature
            readTemperature(positionTemp[position]);
            
            Serial.print("[CAL] Position ");
            Serial.print(position + 1);
            Serial.println(" complete");
            Serial.print("[CAL] Bias (dps): ");
            Serial.print(positionBias[position][0], 4);
            Serial.print(" Y=");
            Serial.print(positionBias[position][1], 4);
            Serial.print(" Z=");
            Serial.println(positionBias[position][2], 4);
            Serial.print("[CAL] Temperature: ");
            Serial.print(positionTemp[position]);
            Serial.println("°C");
            
            // Short delay before next position
            delay(1000);
        }
        
        // Calculate average bias across all positions
        float avgBias[3] = {0};
        float avgTemp = 0;
        
        for (int position = 0; position < 6; position++) {
            avgBias[0] += positionBias[position][0];
            avgBias[1] += positionBias[position][1];
            avgBias[2] += positionBias[position][2];
            avgTemp += positionTemp[position];
        }
        
        avgBias[0] /= 6;
        avgBias[1] /= 6;
        avgBias[2] /= 6;
        avgTemp /= 6;
        
        // Store calibration data
        calibration.gyroBias[0] = avgBias[0];
        calibration.gyroBias[1] = avgBias[1];
        calibration.gyroBias[2] = avgBias[2];
        calibration.gyroCalTemp = avgTemp;
        calibration.referenceTemp = avgTemp;
        
        // Store individual position offsets
        for (int position = 0; position < 6; position++) {
            calibration.gyroOffsets[position][0] = positionBias[position][0];
            calibration.gyroOffsets[position][1] = positionBias[position][1];
            calibration.gyroOffsets[position][2] = positionBias[position][2];
        }
        
        // Set calibration flags
        calibration.sixPositionCalibrated = true;
        calibration.gyroCalibrated = true;
        
        // Temperature compensation
        if (withTempComp) {
            // Check if temperature variation is sufficient
            float minTemp = positionTemp[0];
            float maxTemp = positionTemp[0];
            
            for (int i = 1; i < 6; i++) {
                minTemp = min(minTemp, positionTemp[i]);
                maxTemp = max(maxTemp, positionTemp[i]);
            }
            
            float tempRange = maxTemp - minTemp;
            
            if (tempRange >= 3.0f) {
                // Calculate temperature coefficients
                // This is a simplified approach - a more sophisticated algorithm would
                // use linear regression across all temperature points
                
                // Find positions with min and max temperatures
                int minTempPos = 0;
                int maxTempPos = 0;
                
                for (int i = 1; i < 6; i++) {
                    if (positionTemp[i] < positionTemp[minTempPos]) minTempPos = i;
                    if (positionTemp[i] > positionTemp[maxTempPos]) maxTempPos = i;
                }
                
                // Calculate temperature coefficients
                calibration.gyroTempCoeff[0] = (positionBias[maxTempPos][0] - positionBias[minTempPos][0]) / tempRange;
                calibration.gyroTempCoeff[1] = (positionBias[maxTempPos][1] - positionBias[minTempPos][1]) / tempRange;
                calibration.gyroTempCoeff[2] = (positionBias[maxTempPos][2] - positionBias[minTempPos][2]) / tempRange;
                calibration.gyroTempCompEnabled = true;
                
                Serial.println("[CAL] Temperature compensation enabled");
                Serial.print("[CAL] Temperature range: ");
                Serial.print(minTemp);
                Serial.print("°C to ");
                Serial.print(maxTemp);
                Serial.print("°C (");
                Serial.print(tempRange);
                Serial.println("°C)");
                Serial.print("[CAL] Temperature coefficients (dps/°C): ");
                Serial.print(calibration.gyroTempCoeff[0], 6);
                Serial.print(" Y=");
                Serial.print(calibration.gyroTempCoeff[1], 6);
                Serial.print(" Z=");
                Serial.println(calibration.gyroTempCoeff[2], 6);
            } else {
                Serial.println("[CAL] Temperature variation too small for temperature compensation");
                Serial.print("[CAL] Temperature range: ");
                Serial.print(tempRange);
                Serial.println("°C (need at least 3°C)");
                calibration.gyroTempCompEnabled = false;
            }
        } else {
            calibration.gyroTempCompEnabled = false;
        }
        
        Serial.println("[CAL] 6-position gyroscope calibration complete");
        Serial.print("[CAL] Average bias (dps): ");
        Serial.print(avgBias[0], 4);
        Serial.print(" Y=");
        Serial.print(avgBias[1], 4);
        Serial.print(" Z=");
        Serial.println(avgBias[2], 4);
        Serial.print("[CAL] Average temperature: ");
        Serial.print(avgTemp);
        Serial.println("°C");
        
        // Register gyroscope validation callbacks and validate
        registerValidationCallbacks(readGyroSensorData, applyGyroSensorCalibration, 
                                   checkGyroBiasAcceptable, checkGyroNoiseAcceptable);
        
        // Validate calibration quality
        if (!validateWithRetryOption()) {
            return false;
        }
        
        success = true;
    } while (!success);
    
    return true;
}

/**
 * Validate gyroscope calibration quality
 * 
 * This function performs a comprehensive validation of the gyroscope calibration quality
 * by collecting samples while the device is stationary and checking:
 * 1. Bias stability (how close to zero after calibration)
 * 2. Noise characteristics (standard deviation)
 * 3. Temperature sensitivity (if temperature data available)
 * 
 * @param meanBias Output parameter for mean bias in dps
 * @param noiseSigma Output parameter for noise standard deviation in dps
 * @param tempSensitivity Output parameter for temperature sensitivity in dps/°C
 * @return true if calibration quality meets acceptance criteria, false otherwise
 */
bool validateGyroCalibration(float meanBias[3], float noiseSigma[3], float tempSensitivity[3]) {
    Serial.println("[VAL] Validating gyroscope calibration quality...");
    
    if (!calibration.gyroCalibrated) {
        Serial.println("[VAL] Gyroscope is not calibrated!");
        return false;
    }
    
    // Constants for validation
    const int SAMPLE_COUNT = 200;
    const float MAX_ACCEPTABLE_BIAS = 0.5f;      // Maximum acceptable residual bias in dps
    const float MAX_ACCEPTABLE_NOISE = 0.2f;     // Maximum acceptable noise in dps
    const float MAX_ACCEPTABLE_TEMP_SENS = 0.05f; // Maximum acceptable temp sensitivity in dps/°C
    
    // Arrays to store collected data
    float gx_samples[SAMPLE_COUNT];
    float gy_samples[SAMPLE_COUNT];
    float gz_samples[SAMPLE_COUNT];
    float temperatures[SAMPLE_COUNT];
    
    // Initialize statistics variables
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float sumSquared[3] = {0.0f, 0.0f, 0.0f};
    float sumTemp = 0.0f;
    float sumTempGyro[3] = {0.0f, 0.0f, 0.0f};
    
    Serial.println("[VAL] Collecting gyroscope data. Keep device stationary...");
    
    int validSamples = 0;
    unsigned long startTime = millis();
    unsigned long lastFeedbackTime = startTime;
    
    // Collect samples
    while (validSamples < SAMPLE_COUNT && (millis() - startTime < 10000)) {
        float gx, gy, gz, temp = 0.0f;
        bool tempAvailable = readTemperature(temp);
        
        if (readGyro(gx, gy, gz)) {
            // Apply calibration with temperature compensation if available
            if (tempAvailable) {
                applyGyroCalibration(gx, gy, gz, temp);
                temperatures[validSamples] = temp;
            } else {
                applyGyroCalibration(gx, gy, gz, 0);
                temperatures[validSamples] = 0;
            }
            
            // Store calibrated values
            gx_samples[validSamples] = gx;
            gy_samples[validSamples] = gy;
            gz_samples[validSamples] = gz;
            
            // Update running statistics
            sum[0] += gx;
            sum[1] += gy;
            sum[2] += gz;
            
            if (tempAvailable) {
                sumTemp += temp;
                sumTempGyro[0] += temp * gx;
                sumTempGyro[1] += temp * gy;
                sumTempGyro[2] += temp * gz;
            }
            
            validSamples++;
            
            // Provide feedback every second
            if (millis() - lastFeedbackTime > 1000) {
                Serial.print("[VAL] Collected ");
                Serial.print(validSamples);
                Serial.print("/");
                Serial.print(SAMPLE_COUNT);
                Serial.println(" samples");
                lastFeedbackTime = millis();
            }
        }
        
        delay(10); // 100 Hz sampling
    }
    
    if (validSamples < SAMPLE_COUNT * 0.8) {
        Serial.println("[VAL] Not enough valid samples collected for validation.");
        return false;
    }
    
    // Calculate mean bias
    meanBias[0] = sum[0] / validSamples;
    meanBias[1] = sum[1] / validSamples;
    meanBias[2] = sum[2] / validSamples;
    
    // Calculate standard deviation (noise)
    for (int i = 0; i < validSamples; i++) {
        sumSquared[0] += (gx_samples[i] - meanBias[0]) * (gx_samples[i] - meanBias[0]);
        sumSquared[1] += (gy_samples[i] - meanBias[1]) * (gy_samples[i] - meanBias[1]);
        sumSquared[2] += (gz_samples[i] - meanBias[2]) * (gz_samples[i] - meanBias[2]);
    }
    
    noiseSigma[0] = sqrt(sumSquared[0] / validSamples);
    noiseSigma[1] = sqrt(sumSquared[1] / validSamples);
    noiseSigma[2] = sqrt(sumSquared[2] / validSamples);
    
    // Calculate temperature sensitivity if temperature data available
    bool tempDataAvailable = (temperatures[0] != 0);
    if (tempDataAvailable) {
        float meanTemp = sumTemp / validSamples;
        
        // Calculate temperature covariance
        float tempCovariance[3] = {0.0f, 0.0f, 0.0f};
        float tempVariance = 0.0f;
        
        for (int i = 0; i < validSamples; i++) {
            float tempDiff = temperatures[i] - meanTemp;
            tempCovariance[0] += tempDiff * (gx_samples[i] - meanBias[0]);
            tempCovariance[1] += tempDiff * (gy_samples[i] - meanBias[1]);
            tempCovariance[2] += tempDiff * (gz_samples[i] - meanBias[2]);
            tempVariance += tempDiff * tempDiff;
        }
        
        // Calculate temperature coefficients if we have meaningful temperature variation
        if (tempVariance > 0.01f) {
            tempSensitivity[0] = tempCovariance[0] / tempVariance;
            tempSensitivity[1] = tempCovariance[1] / tempVariance;
            tempSensitivity[2] = tempCovariance[2] / tempVariance;
        } else {
            tempSensitivity[0] = tempSensitivity[1] = tempSensitivity[2] = 0.0f;
        }
    } else {
        tempSensitivity[0] = tempSensitivity[1] = tempSensitivity[2] = 0.0f;
    }
    
    // Evaluate validation criteria
    bool biasOk = (fabs(meanBias[0]) < MAX_ACCEPTABLE_BIAS &&
                   fabs(meanBias[1]) < MAX_ACCEPTABLE_BIAS &&
                   fabs(meanBias[2]) < MAX_ACCEPTABLE_BIAS);
    
    bool noiseOk = (noiseSigma[0] < MAX_ACCEPTABLE_NOISE &&
                    noiseSigma[1] < MAX_ACCEPTABLE_NOISE &&
                    noiseSigma[2] < MAX_ACCEPTABLE_NOISE);
    
    bool tempSensOk = (!tempDataAvailable ||
                      (fabs(tempSensitivity[0]) < MAX_ACCEPTABLE_TEMP_SENS &&
                       fabs(tempSensitivity[1]) < MAX_ACCEPTABLE_TEMP_SENS &&
                       fabs(tempSensitivity[2]) < MAX_ACCEPTABLE_TEMP_SENS));
    
    // Report results
    Serial.println("[VAL] Gyroscope Calibration Validation Results:");
    Serial.printf("[VAL] Residual bias (dps): X=%.4f Y=%.4f Z=%.4f (max: %.2f)\n",
                 meanBias[0], meanBias[1], meanBias[2], MAX_ACCEPTABLE_BIAS);
    Serial.printf("[VAL] Noise sigma (dps): X=%.4f Y=%.4f Z=%.4f (max: %.2f)\n",
                 noiseSigma[0], noiseSigma[1], noiseSigma[2], MAX_ACCEPTABLE_NOISE);
    
    if (tempDataAvailable) {
        Serial.printf("[VAL] Temp sensitivity (dps/°C): X=%.5f Y=%.5f Z=%.5f (max: %.3f)\n",
                     tempSensitivity[0], tempSensitivity[1], tempSensitivity[2], MAX_ACCEPTABLE_TEMP_SENS);
    } else {
        Serial.println("[VAL] Temperature sensitivity: No temperature data available");
    }
    
    // Overall validation result
    bool validationPassed = biasOk && noiseOk && tempSensOk;
    
    if (!biasOk) {
        Serial.println("[VAL] Warning: Residual bias too high");
        Serial.println("[VAL] Recommendation: Re-calibrate gyroscope");
    }
    
    if (!noiseOk) {
        Serial.println("[VAL] Warning: Noise level too high");
        Serial.println("[VAL] Recommendation: Check for vibration or interference");
    }
    
    if (tempDataAvailable && !tempSensOk) {
        Serial.println("[VAL] Warning: Temperature sensitivity too high");
        Serial.println("[VAL] Recommendation: Perform temperature-compensated calibration");
    }
    
    if (validationPassed) {
        Serial.println("[VAL] Gyroscope calibration validation PASSED");
    } else {
        Serial.println("[VAL] Gyroscope calibration validation FAILED");
    }
    
    return validationPassed;
}

/**
 * Perform enhanced gyroscope calibration validation
 * 
 * This function validates the gyroscope calibration by collecting data
 * while the device is stationary and analyzing the stability, noise,
 * and temperature response characteristics.
 * 
 * @param driftRate Output parameter for gyro drift rate in degrees/hour
 * @param noiseFloor Output parameter for gyro noise floor in dps/√Hz
 * @param tempStability Output parameter for temperature stability in dps/°C
 * @return true if calibration meets quality thresholds, false otherwise
 */
bool validateGyroCalibrationEnhanced(float driftRate[3], float noiseFloor[3], float tempStability[3]) {
    Serial.println("[VAL] Performing enhanced gyroscope calibration validation...");
    Serial.println("[VAL] This test measures long-term stability and noise characteristics.");
    Serial.println("[VAL] Keep the device completely stationary for best results.");
    Serial.println("[VAL] When ready, press 'y' to begin");
    waitForUserConfirmation();
    
    if (!calibration.gyroCalibrated) {
        Serial.println("[VAL] Gyroscope is not calibrated!");
        return false;
    }
    
    // Constants for validation
    const int SAMPLE_COUNT = 500;  // More samples for better statistics
    const float SAMPLE_RATE_HZ = 100.0f;  // Target sample rate in Hz
    const float MAX_DRIFT_RATE = 10.0f;   // Maximum acceptable drift in degrees/hour
    const float MAX_NOISE_FLOOR = 0.05f;  // Maximum acceptable noise floor in dps/√Hz
    const float MAX_TEMP_STABILITY = 0.02f; // Maximum acceptable temp stability in dps/°C
    
    // Arrays to store collected data
    float gx_data[SAMPLE_COUNT];
    float gy_data[SAMPLE_COUNT];
    float gz_data[SAMPLE_COUNT];
    float temp_data[SAMPLE_COUNT];
    unsigned long timestamps[SAMPLE_COUNT];
    
    // Initialize Allan variance calculation variables
    float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    float gx_prev = 0.0f, gy_prev = 0.0f, gz_prev = 0.0f;
    float temp_sum = 0.0f;
    
    Serial.println("[VAL] Collecting gyroscope data for enhanced validation...");
    Serial.println("[VAL] This will take approximately 30 seconds.");
    
    int validSamples = 0;
    unsigned long startTime = millis();
    unsigned long lastFeedbackTime = startTime;
    unsigned long lastSampleTime = startTime;
    
    // Collect samples
    while (validSamples < SAMPLE_COUNT && (millis() - startTime < 30000)) {
        // Control sample rate
        if (millis() - lastSampleTime < (1000.0f / SAMPLE_RATE_HZ)) {
            continue;
        }
        
        float gx, gy, gz, temp = 0.0f;
        bool tempAvailable = readTemperature(temp);
        
        if (readGyro(gx, gy, gz)) {
            // Apply calibration with temperature compensation if available
            if (tempAvailable) {
                applyGyroCalibration(gx, gy, gz, temp);
                temp_data[validSamples] = temp;
            } else {
                applyGyroCalibration(gx, gy, gz, 0);
                temp_data[validSamples] = 0;
            }
            
            // Store calibrated values and timestamp
            gx_data[validSamples] = gx;
            gy_data[validSamples] = gy;
            gz_data[validSamples] = gz;
            timestamps[validSamples] = millis();
            
            // Update running sums for mean calculation
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            if (tempAvailable) temp_sum += temp;
            
            // Store previous values for drift calculation
            if (validSamples > 0) {
                gx_prev = gx;
                gy_prev = gy;
                gz_prev = gz;
            }
            
            validSamples++;
            lastSampleTime = millis();
            
            // Provide feedback every second
            if (millis() - lastFeedbackTime > 1000) {
                Serial.print("[VAL] Collected ");
                Serial.print(validSamples);
                Serial.print("/");
                Serial.print(SAMPLE_COUNT);
                Serial.println(" samples");
                lastFeedbackTime = millis();
            }
        }
        
        delay(1); // Small delay to prevent CPU hogging
    }
    
    if (validSamples < SAMPLE_COUNT * 0.8) {
        Serial.println("[VAL] Not enough valid samples collected for validation.");
        return false;
    }
    
    // Calculate mean values
    float gx_mean = gx_sum / validSamples;
    float gy_mean = gy_sum / validSamples;
    float gz_mean = gz_sum / validSamples;
    float temp_mean = (temp_data[0] != 0) ? temp_sum / validSamples : 0;
    
    // Calculate drift rate (degrees/hour)
    // Using linear regression to find rate of change over time
    float sum_x = 0, sum_y[3] = {0}, sum_xx = 0, sum_xy[3] = {0};
    float time_scale = 1.0f / 1000.0f; // Convert ms to seconds
    
    for (int i = 0; i < validSamples; i++) {
        float x = (timestamps[i] - timestamps[0]) * time_scale; // Time in seconds
        sum_x += x;
        sum_xx += x * x;
        
        sum_y[0] += gx_data[i];
        sum_y[1] += gy_data[i];
        sum_y[2] += gz_data[i];
        
        sum_xy[0] += x * gx_data[i];
        sum_xy[1] += x * gy_data[i];
        sum_xy[2] += x * gz_data[i];
    }
    
    // Calculate slope (rate of change) for each axis
    float slope[3];
    for (int i = 0; i < 3; i++) {
        slope[i] = (validSamples * sum_xy[i] - sum_x * sum_y[i]) / 
                  (validSamples * sum_xx - sum_x * sum_x);
    }
    
    // Convert slope from dps/second to degrees/hour
    driftRate[0] = slope[0] * 3600.0f;
    driftRate[1] = slope[1] * 3600.0f;
    driftRate[2] = slope[2] * 3600.0f;
    
    // Calculate noise floor (dps/√Hz)
    // Using power spectral density estimation
    float variance[3] = {0, 0, 0};
    for (int i = 0; i < validSamples; i++) {
        variance[0] += (gx_data[i] - gx_mean) * (gx_data[i] - gx_mean);
        variance[1] += (gy_data[i] - gy_mean) * (gy_data[i] - gy_mean);
        variance[2] += (gz_data[i] - gz_mean) * (gz_data[i] - gz_mean);
    }
    
    for (int i = 0; i < 3; i++) {
        variance[i] /= validSamples;
        // Convert variance to noise floor in dps/√Hz
        // Noise floor = sqrt(variance * sample_rate / 2)
        noiseFloor[i] = sqrt(variance[i] * SAMPLE_RATE_HZ / 2.0f);
    }
    
    // Calculate temperature stability
    bool tempDataAvailable = (temp_data[0] != 0);
    if (tempDataAvailable) {
        // Calculate temperature-gyro covariance
        float temp_variance = 0;
        float temp_gyro_cov[3] = {0, 0, 0};
        
        for (int i = 0; i < validSamples; i++) {
            float temp_diff = temp_data[i] - temp_mean;
            temp_variance += temp_diff * temp_diff;
            
            temp_gyro_cov[0] += temp_diff * (gx_data[i] - gx_mean);
            temp_gyro_cov[1] += temp_diff * (gy_data[i] - gy_mean);
            temp_gyro_cov[2] += temp_diff * (gz_data[i] - gz_mean);
        }
        
        // Calculate temperature coefficients
        if (temp_variance > 0.01f) {
            tempStability[0] = temp_gyro_cov[0] / temp_variance;
            tempStability[1] = temp_gyro_cov[1] / temp_variance;
            tempStability[2] = temp_gyro_cov[2] / temp_variance;
        } else {
            tempStability[0] = tempStability[1] = tempStability[2] = 0.0f;
        }
    } else {
        tempStability[0] = tempStability[1] = tempStability[2] = 0.0f;
    }
    
    // Evaluate validation criteria
    bool driftOk = (fabs(driftRate[0]) < MAX_DRIFT_RATE &&
                    fabs(driftRate[1]) < MAX_DRIFT_RATE &&
                    fabs(driftRate[2]) < MAX_DRIFT_RATE);
    
    bool noiseOk = (noiseFloor[0] < MAX_NOISE_FLOOR &&
                    noiseFloor[1] < MAX_NOISE_FLOOR &&
                    noiseFloor[2] < MAX_NOISE_FLOOR);
    
    bool tempStabilityOk = (!tempDataAvailable ||
                           (fabs(tempStability[0]) < MAX_TEMP_STABILITY &&
                            fabs(tempStability[1]) < MAX_TEMP_STABILITY &&
                            fabs(tempStability[2]) < MAX_TEMP_STABILITY));
    
    // Report results
    Serial.println("[VAL] Enhanced Gyroscope Validation Results:");
    Serial.printf("[VAL] Drift rate (°/hr): X=%.2f Y=%.2f Z=%.2f (max: %.1f)\n",
                 driftRate[0], driftRate[1], driftRate[2], MAX_DRIFT_RATE);
    Serial.printf("[VAL] Noise floor (dps/√Hz): X=%.5f Y=%.5f Z=%.5f (max: %.3f)\n",
                 noiseFloor[0], noiseFloor[1], noiseFloor[2], MAX_NOISE_FLOOR);
    
    if (tempDataAvailable) {
        Serial.printf("[VAL] Temp stability (dps/°C): X=%.5f Y=%.5f Z=%.5f (max: %.3f)\n",
                     tempStability[0], tempStability[1], tempStability[2], MAX_TEMP_STABILITY);
    } else {
        Serial.println("[VAL] Temperature stability: No temperature data available");
    }
    
    // Overall validation result
    bool validationPassed = driftOk && noiseOk && tempStabilityOk;
    
    if (!driftOk) {
        Serial.println("[VAL] Warning: Drift rate too high");
        Serial.println("[VAL] Recommendation: Re-calibrate with temperature compensation");
    }
    
    if (!noiseOk) {
        Serial.println("[VAL] Warning: Noise floor too high");
        Serial.println("[VAL] Recommendation: Check for vibration or interference");
    }
    
    if (tempDataAvailable && !tempStabilityOk) {
        Serial.println("[VAL] Warning: Temperature stability too poor");
        Serial.println("[VAL] Recommendation: Perform temperature-compensated calibration");
    }
    
    if (validationPassed) {
        Serial.println("[VAL] Enhanced gyroscope validation PASSED");
        Serial.println("[VAL] Gyroscope calibration quality is excellent");
    } else {
        Serial.println("[VAL] Enhanced gyroscope validation FAILED");
        Serial.println("[VAL] Gyroscope calibration needs improvement");
    }
    
    return validationPassed;
}

/**
 * Run the gyroscope calibration procedure with user-selectable methods
 * 
 * This function presents a menu to the user to select between different
 * gyroscope calibration methods:
 * 1. Simple bias-only calibration
 * 2. Temperature-compensated calibration
 * 3. 6-position calibration
 * 4. 6-position with temperature compensation
 * 
 * @return true if calibration was successful, false otherwise
 */
bool calibrateGyroscope() {
    isCalibrating = true;
    Serial.println("[CAL] Gyroscope Calibration");
    Serial.println("[CAL] Select calibration method:");
    Serial.println("[CAL] 1. Simple bias-only calibration (quick, basic)");
    Serial.println("[CAL] 2. Temperature-compensated calibration (better, takes longer)");
    Serial.println("[CAL] 3. 6-position calibration (comprehensive, no temperature comp)");
    Serial.println("[CAL] 4. 6-position with temperature compensation (best, most time-consuming)");
    Serial.println("[CAL] Enter selection (1-4):");
    
    // Clear any pending serial data
    while (Serial.available()) {
        Serial.read();
    }
    
    // Wait for user selection
    int selection = 0;
    while (selection < 1 || selection > 4) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c >= '1' && c <= '4') {
                selection = c - '0';
                break;
            }
        }
        delay(10);
    }
    
    Serial.print("[CAL] Selected method: ");
    
    bool result = false;
    
    switch (selection) {
        case 1:
            Serial.println("Simple bias-only calibration");
            result = calibrateGyroSimple();
            break;
        case 2:
            Serial.println("Temperature-compensated calibration");
            result = calibrateGyroWithTempComp();
            break;
        case 3:
            Serial.println("6-position calibration");
            result = calibrateGyro6Position(false);
            break;
        case 4:
            Serial.println("6-position with temperature compensation");
            result = calibrateGyro6Position(true);
            break;
        default:
            Serial.println("Invalid selection");
            return false;
    }
    
    if (result) {
        // Note: EEPROM save is handled by the main calibration workflow after all sensors are calibrated
        Serial.println("[CAL] Gyroscope calibration complete");
    }
    
    isCalibrating = false;
    return result;
}

// Sensor-specific validation helpers
bool readGyroSensorData(float &x, float &y, float &z) {
    return readGyro(x, y, z);
}

void applyGyroSensorCalibration(float &x, float &y, float &z) {
    // Apply current calibration
    x -= calibration.gyroBias[0];
    y -= calibration.gyroBias[1];
    z -= calibration.gyroBias[2];
    
    // If temperature compensation is enabled, apply it
    if (calibration.gyroTempCompEnabled) {
        float currentTemp;
        if (readTemperature(currentTemp)) {
            float tempDiff = currentTemp - calibration.referenceTemp;
            x -= calibration.gyroTempCoeff[0] * tempDiff;
            y -= calibration.gyroTempCoeff[1] * tempDiff;
            z -= calibration.gyroTempCoeff[2] * tempDiff;
        }
    }
}

bool checkGyroBiasAcceptable(float x, float y, float z) {
    // Check residual bias against acceptance threshold
    return (abs(x) < calibration.MAX_RESIDUAL_BIAS_GYRO) &&
           (abs(y) < calibration.MAX_RESIDUAL_BIAS_GYRO) &&
           (abs(z) < calibration.MAX_RESIDUAL_BIAS_GYRO);
}

bool checkGyroNoiseAcceptable(float x, float y, float z) {
    // Check noise level against acceptance threshold
    return (x < calibration.MAX_NOISE_GYRO) &&
           (y < calibration.MAX_NOISE_GYRO) &&
           (z < calibration.MAX_NOISE_GYRO);
}
