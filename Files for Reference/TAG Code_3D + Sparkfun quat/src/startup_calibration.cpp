/**
 * @file startup_calibration.cpp
 * @brief Implementation of the startup calibration workflow and runtime management
 * 
 * This file implements the startup and runtime components of the three-tier calibration system:
 * - Startup workflow with user interaction and decision handling
 * - Runtime calibration monitoring and auto-calibration
 * - Command interface for manual calibration control
 * 
 * The startup workflow integrates with the calibration manager to provide a seamless
 * user experience while ensuring sensor accuracy and reliability.
 */

#include "startup_calibration.h"
#include "calibration_manager.h"
#include "imu_config.h"
#include "imu_validation.h"
#include "accel_calibration.h"   // Added
#include "gyro_calibration.h"    // Added
#include "mag_calibration.h"     // Added

// Sensor type definitions
#define SENSOR_TYPE_ACCEL 0
#define SENSOR_TYPE_GYRO  1
#define SENSOR_TYPE_MAG   2

// Forward declarations for sensor reading functions
bool readAccel(float& ax, float& ay, float& az);
bool readGyro(float& gx, float& gy, float& gz);
bool readMag(float& mx, float& my, float& mz);

// Forward declarations for new menu handlers
bool handleIndividualSensorCalibration();
bool handleAccelerometerCalibrationMenu();
bool handleGyroscopeCalibrationMenu();
bool handleMagnetometerCalibrationMenu();

/**
 * @brief Execute the main startup calibration workflow
 * 
 * This function implements the core startup sequence:
 * 1. Initialize calibration manager and load saved data
 * 2. Perform startup verification of all sensors
 * 3. Handle results based on calibration status:
 *    - GOOD: Proceed with normal operation
 *    - MARGINAL/POOR/CORRUPTED: Offer quick/robust calibration options
 * 
 * The workflow provides multiple paths:
 * - Direct proceed if calibration is good
 * - User-guided calibration choices for marginal/poor/corrupted status
 * - Emergency calibration for critical failures
 * 
 * @return true if system is ready to proceed, false if critical error
 */
bool performStartupCalibrationWorkflow() {
    Serial.println(F("\n" COLOR_BLUE "========================================" COLOR_RESET));
    Serial.println(F(COLOR_BLUE "    IMU CALIBRATION STARTUP WORKFLOW    " COLOR_RESET));
    Serial.println(F(COLOR_BLUE "========================================" COLOR_RESET));
    
    // NOTE: initCalibrationManager() is already called in main.cpp setup()
    // DO NOT call it again here as it reinitializes EEPROM and can cause data loss!
    
    // Step 1: Perform startup verification
    CalibrationStatus status = performStartupVerification();
    
    // Step 3: Handle the result based on status
    switch (status) {
        case CAL_STATUS_GOOD:
            Serial.println(F(COLOR_GREEN "✓ Calibration is valid - proceeding with normal operation" COLOR_RESET));
            return true;
            
        case CAL_STATUS_MARGINAL:
        case CAL_STATUS_POOR:
        case CAL_STATUS_NONE:
        case CAL_STATUS_CORRUPTED: {
            // Get user decision
            int decision = getUserCalibrationDecision(status, false);
            
            switch (decision) {
                case 1: // Quick calibration
                    if (performQuickCalibration()) {
                        Serial.println(F(COLOR_GREEN "✓ Quick calibration completed successfully" COLOR_RESET));
                        return true;
                    } else {
                        Serial.println(F(COLOR_RED "✗ Quick calibration failed" COLOR_RESET));
                        return handleCalibrationFailure();
                    }
                    
                case 2: // Robust calibration
                    if (performRobustCalibration()) {
                        Serial.println(F(COLOR_GREEN "✓ Robust calibration completed successfully" COLOR_RESET));
                        return true;
                    } else {
                        Serial.println(F(COLOR_RED "✗ Robust calibration failed" COLOR_RESET));
                        return handleCalibrationFailure();
                    }
                    
                case 3: // Individual sensor calibration menu
                    return handleIndividualSensorCalibration();
                    
                case 4: // Skip
                    Serial.println(F(COLOR_YELLOW "⚠ Proceeding with current calibration" COLOR_RESET));
                    return true;
                    
                default:
                    Serial.println(F(COLOR_RED "Invalid choice - entering failure recovery" COLOR_RESET));
                    return handleCalibrationFailure();
            }
        }
        
        default:
            Serial.println(F(COLOR_RED "✗ Unknown calibration status" COLOR_RESET));
            return handleCalibrationFailure();
    }
}

/**
 * @brief Handle calibration failure scenarios
 * 
 * This function provides recovery options when calibration fails:
 * 1. Emergency calibration - Basic functionality with minimal accuracy
 * 2. Continue without calibration (high risk)
 * 3. Restart the calibration process
 * 
 * Emergency calibration performs:
 * - Simple accelerometer bias correction
 * - Basic gyroscope zero-rate calibration
 * - Minimal magnetometer offset correction
 * 
 * @return true if recovery successful, false if critical failure
 */
bool handleCalibrationFailure() {
    Serial.println(F("\n" COLOR_RED "=== Calibration Failure Recovery ===" COLOR_RESET));
    Serial.println(F("Calibration has failed. Choose recovery option:"));
    Serial.println(F("1 - Attempt emergency calibration"));
    Serial.println(F("2 - Continue without calibration (NOT RECOMMENDED)"));
    Serial.println(F("3 - Restart calibration process"));
    Serial.print(F("Choice (1-3): "));
    
    while (Serial.available()) Serial.read(); // Clear buffer
    
    while (!Serial.available()) {
        delay(100);
    }
    
    int choice = Serial.read() - '0';
    while (Serial.available()) Serial.read(); // Clear remaining
    
    switch (choice) {
        case 1:
            if (performEmergencyCalibration()) {
                Serial.println(F(COLOR_YELLOW "⚠ Emergency calibration completed" COLOR_RESET));
                Serial.println(F(COLOR_YELLOW "⚠ System functional but recommend full calibration later" COLOR_RESET));
                return true;
            } else {
                Serial.println(F(COLOR_RED "✗ Emergency calibration also failed" COLOR_RESET));
                return false;
            }
            
        case 2:
            Serial.println(F(COLOR_RED "⚠ WARNING: Proceeding without calibration" COLOR_RESET));
            Serial.println(F(COLOR_RED "⚠ Sensor readings will be inaccurate!" COLOR_RESET));
            return true;
            
        case 3:
            Serial.println(F(COLOR_CYAN "→ Restarting calibration process..." COLOR_RESET));
            return performStartupCalibrationWorkflow(); // Recursive call
            
        default:
            Serial.println(F(COLOR_RED "✗ Invalid choice - system cannot proceed safely" COLOR_RESET));
            return false;
    }
}

/**
 * @brief Display startup calibration summary and menu
 * 
 * This function provides a user interface for:
 * - Viewing current calibration status
 * - Accessing manual calibration options
 * - Checking calibration history
 * - Viewing environmental conditions
 * 
 * The summary includes:
 * - Overall calibration status
 * - Individual sensor status
 * - Temperature and drift information
 * - Last calibration timestamp
 */
void displayStartupSummary() {
    Serial.println(F("\n" COLOR_CYAN "=== Startup Calibration Summary ===" COLOR_RESET));
    
    // Display current calibration status
    CalibrationStatus current_status = performStartupVerification();
    
    // Show calibration menu option
    Serial.println(F("\nCalibration menu available:"));
    Serial.println(F("Send 'cal' via serial to access calibration options"));
    Serial.println(F("Send 'status' to view current calibration status"));
    Serial.println(F("Send 'help' to view calibration commands"));
    Serial.println();
}

/**
 * @brief Handle serial commands for calibration management
 * 
 * This function processes runtime calibration commands:
 * - cal status: Show current calibration status
 * - cal quick: Perform quick calibration
 * - cal robust: Perform full calibration
 * - cal verify: Run verification routine
 * - cal auto [on|off]: Toggle auto-calibration
 * - cal bypass: Skip calibration (not recommended)
 * 
 * @param command The command string received via serial
 * @return true if command was handled, false if not recognized
 */
bool handleCalibrationCommand(String command) {
    command.toLowerCase();
    command.trim();
    
    if (command == "cal" || command == "calibrate") {
        showCalibrationMenu();
        return true;
    }
    
    if (command == "status") {
        CalibrationStatus status = performStartupVerification();
        displayCalibrationStatus(status, true);
        return true;
    }
    
    if (command == "quick") {
        Serial.println(F(COLOR_BLUE "→ Starting quick calibration..." COLOR_RESET));
        if (performQuickCalibration()) {
            Serial.println(F(COLOR_GREEN "✓ Quick calibration completed" COLOR_RESET));
        } else {
            Serial.println(F(COLOR_RED "✗ Quick calibration failed" COLOR_RESET));
        }
        return true;
    }
    
    if (command == "robust") {
        Serial.println(F(COLOR_BLUE "→ Starting robust calibration..." COLOR_RESET));
        if (performRobustCalibration()) {
            Serial.println(F(COLOR_GREEN "✓ Robust calibration completed" COLOR_RESET));
        } else {
            Serial.println(F(COLOR_RED "✗ Robust calibration failed" COLOR_RESET));
        }
        return true;
    }
    
    if (command == "emergency") {
        Serial.println(F(COLOR_RED "→ Starting emergency calibration..." COLOR_RESET));
        if (performEmergencyCalibration()) {
            Serial.println(F(COLOR_YELLOW "⚠ Emergency calibration completed" COLOR_RESET));
        } else {
            Serial.println(F(COLOR_RED "✗ Emergency calibration failed" COLOR_RESET));
        }
        return true;
    }
    
    if (command == "precision") {
        Serial.println(F("Set precision level:"));
        Serial.println(F("0 - Relaxed (less strict thresholds)"));
        Serial.println(F("1 - Normal (default)"));
        Serial.println(F("2 - High precision (strict thresholds)"));
        Serial.print(F("Choice (0-2): "));
        
        while (Serial.available()) Serial.read();
        while (!Serial.available()) delay(100);
        
        int level = Serial.read() - '0';
        while (Serial.available()) Serial.read();
        
        if (level >= 0 && level <= 2) {
            setAdaptiveThresholds(level);
            Serial.printf("Precision level set to %d\n", level);
        } else {
            Serial.println(F(COLOR_RED "Invalid precision level" COLOR_RESET));
        }
        return true;
    }
    
    if (command == "help") {
        Serial.println(F("\n" COLOR_CYAN "=== Calibration Commands ===" COLOR_RESET));
        Serial.println(F("cal/calibrate - Show calibration menu"));
        Serial.println(F("status        - Show calibration status"));
        Serial.println(F("quick         - Perform quick calibration"));
        Serial.println(F("robust        - Perform robust calibration"));
        Serial.println(F("emergency     - Perform emergency calibration"));
        Serial.println(F("precision     - Set precision level"));
        Serial.println(F("help          - Show this help"));
        return true;
    }
    
    return false; // Command not recognized
}

/**
 * @brief Check if runtime recalibration is needed
 * 
 * This function implements the third tier (runtime management):
 * 1. Monitors sensor drift and data quality
 * 2. Tracks temperature changes
 * 3. Detects environmental variations
 * 4. Suggests appropriate calibration level
 * 
 * Triggers that may suggest recalibration:
 * - Temperature change > threshold
 * - Sensor readings outside expected ranges
 * - Drift in zero-rate outputs
 * - Magnetic field inconsistencies
 * 
 * @return true if recalibration is recommended
 */
bool checkRuntimeCalibrationNeeded() {
    static unsigned long last_check = 0;
    static int check_interval = 300000; // 5 minutes
    
    if (millis() - last_check < check_interval) {
        return false; // Too soon to check again
    }
    
    last_check = millis();
    
    // Check for environmental changes
    if (checkEnvironmentalChange()) {
        Serial.println(F(COLOR_YELLOW "⚠ Environmental change detected - consider recalibration" COLOR_RESET));
        return true;
    }
    
    // Quick sensor health check
    CalibrationStatus accel_status = checkSensorCalibration(SENSOR_TYPE_ACCEL);
    if (accel_status >= CAL_STATUS_MARGINAL) {
        Serial.println(F(COLOR_YELLOW "⚠ Accelerometer calibration drift detected" COLOR_RESET));
        return true;
    }
    
    return false;
}

/**
 * @brief Perform automatic calibration when conditions are suitable
 * 
 * This function provides unattended calibration capability:
 * 1. Monitors device stability
 * 2. Detects optimal calibration conditions
 * 3. Performs quick calibration automatically
 * 4. Updates calibration history
 * 
 * Auto-calibration triggers:
 * - Device detected as stationary
 * - Marginal calibration status
 * - Temperature changes within limits
 * - No user activity detected
 * 
 * @return true if auto-calibration was performed
 */
bool performAutoCalibration() {
    // Check if device has been still for sufficient time
    static unsigned long still_start = 0;
    static bool was_still = false;
    
    float accel[3], gyro[3];
    if (!readAccel(accel[0], accel[1], accel[2]) || 
        !readGyro(gyro[0], gyro[1], gyro[2])) {
        return false;
    }
    
    bool is_still = isDeviceStill(accel, gyro, 0.1f, 0.1f); // Stricter thresholds
    
    if (is_still && !was_still) {
        still_start = millis();
    } else if (!is_still) {
        still_start = 0;
    }
    
    was_still = is_still;
    
    // If still for more than 30 seconds, consider auto-calibration
    if (is_still && (millis() - still_start > 30000)) {
        Serial.println(F(COLOR_BLUE "→ Device stable - performing auto-calibration..." COLOR_RESET));
        
        if (performQuickCalibration()) {
            Serial.println(F(COLOR_GREEN "✓ Auto-calibration completed" COLOR_RESET));
            still_start = millis(); // Reset timer
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Handle individual sensor calibration menu
 * 
 * This function provides a detailed menu for calibrating each sensor individually
 * with different calibration methods available for each sensor type.
 * 
 * @return true if calibration completed successfully, false otherwise
 */
bool handleIndividualSensorCalibration() {
    Serial.println(F("\n" COLOR_CYAN "=== Individual Sensor Calibration Menu ===" COLOR_RESET));
    
    while (true) {
        Serial.println(F("\nSelect sensor to calibrate:"));
        Serial.println(F("  1. Accelerometer"));
        Serial.println(F("  2. Gyroscope"));
        Serial.println(F("  3. Magnetometer"));
        Serial.println(F("  4. All Sensors (Quick)"));
        Serial.println(F("  5. All Sensors (Robust)"));
        Serial.println(F("  0. Return to main menu"));
        Serial.print(F("Enter choice (0-5): "));
        
        while (!Serial.available()) delay(10);
        char choice = Serial.read();
        Serial.println(choice);
        
        switch (choice) {
            case '1':
                if (handleAccelerometerCalibrationMenu()) {
                    Serial.println(F(COLOR_GREEN "✓ Accelerometer calibration completed" COLOR_RESET));
                } else {
                    Serial.println(F(COLOR_RED "✗ Accelerometer calibration failed" COLOR_RESET));
                }
                break;
                
            case '2':
                if (handleGyroscopeCalibrationMenu()) {
                    Serial.println(F(COLOR_GREEN "✓ Gyroscope calibration completed" COLOR_RESET));
                } else {
                    Serial.println(F(COLOR_RED "✗ Gyroscope calibration failed" COLOR_RESET));
                }
                break;
                
            case '3':
                if (handleMagnetometerCalibrationMenu()) {
                    Serial.println(F(COLOR_GREEN "✓ Magnetometer calibration completed" COLOR_RESET));
                } else {
                    Serial.println(F(COLOR_RED "✗ Magnetometer calibration failed" COLOR_RESET));
                }
                break;
                
            case '4':
                Serial.println(F(COLOR_CYAN "→ Starting quick calibration of all sensors" COLOR_RESET));
                return performQuickCalibration();
                
            case '5':
                Serial.println(F(COLOR_CYAN "→ Starting robust calibration of all sensors" COLOR_RESET));
                return performRobustCalibration();
                
            case '0':
                Serial.println(F(COLOR_YELLOW "→ Returning to main calibration menu" COLOR_RESET));
                return true;
                
            default:
                Serial.println(F(COLOR_RED "Invalid choice. Please try again." COLOR_RESET));
                break;
        }
    }
}

/**
 * @brief Handle accelerometer calibration submenu
 */
bool handleAccelerometerCalibrationMenu() {
    Serial.println(F("\n" COLOR_CYAN "=== Accelerometer Calibration Methods ===" COLOR_RESET));
    Serial.println(F("  1. Simple Bias Calibration (1 position)"));
    Serial.println(F("  2. 6-Position Calibration (recommended)"));
    Serial.println(F("  3. Advanced Ellipsoid Calibration"));
    Serial.print(F("Enter choice (1-3): "));
    
    while (!Serial.available()) delay(10);
    char choice = Serial.read();
    Serial.println(choice);
    
    switch (choice) {
        case '1':
            return calibrateAccelSimple();
        case '2':
            return calibrateAccel6Position(true); // with temperature compensation
        case '3':
            return calibrateAccelEllipsoid();
        default:
            Serial.println(F(COLOR_RED "Invalid choice" COLOR_RESET));
            return false;
    }
}

/**
 * @brief Handle gyroscope calibration submenu
 */
bool handleGyroscopeCalibrationMenu() {
    Serial.println(F("\n" COLOR_CYAN "=== Gyroscope Calibration Methods ===" COLOR_RESET));
    Serial.println(F("  1. Simple Bias Calibration"));
    Serial.println(F("  2. Temperature Compensated Calibration"));
    Serial.println(F("  3. 6-Position Calibration"));
    Serial.print(F("Enter choice (1-3): "));
    
    while (!Serial.available()) delay(10);
    char choice = Serial.read();
    Serial.println(choice);
    
    switch (choice) {
        case '1':
            return calibrateGyroSimple();
        case '2':
            return calibrateGyroWithTemp(); // with temperature compensation
        case '3':
            return calibrateGyro6Position(false);
        default:
            Serial.println(F(COLOR_RED "Invalid choice" COLOR_RESET));
            return false;
    }
}

/**
 * @brief Handle magnetometer calibration submenu
 */
bool handleMagnetometerCalibrationMenu() {
    Serial.println(F("\n" COLOR_CYAN "=== Magnetometer Calibration Methods ===" COLOR_RESET));
    Serial.println(F("  1. Simple Offset Calibration"));
    Serial.println(F("  2. Hard/Soft Iron Calibration"));
    Serial.println(F("  3. Enhanced Calibration with Validation"));
    Serial.print(F("Enter choice (1-3): "));
    
    while (!Serial.available()) delay(10);
    char choice = Serial.read();
    Serial.println(choice);
    
    bool result = false;
    
    switch (choice) {
        case '1':
            result = calibrateMagSimple();
            break;
        case '2':
            result = calibrateMagHardSoftIron();
            break;
        case '3':
            result = calibrateMagEllipsoid();
            break;
        default:
            Serial.println(F(COLOR_RED "Invalid choice" COLOR_RESET));
            return false;
    }
    
    // CRITICAL: Save to EEPROM after successful calibration
    if (result) {
        Serial.println(F("\n✓ Saving magnetometer calibration to EEPROM..."));
        saveCalibrationToEEPROM();
        Serial.println(F("✓ Magnetometer calibration saved!"));
    }
    
    return result;
}
