#include "calibration_menu.h"
#include "accel_calibration.h"
#include "gyro_calibration.h"
#include "mag_calibration.h"
#include "calibration_common.h"
#include "calibration_manager.h"
#include "calibration_quality.h"

// Forward declarations
extern bool calibrateAccelSimple();
extern bool calibrateAccel6Position(bool withTemp);
extern bool calibrateGyroSimple();
extern bool calibrateGyro6Position(bool withTempComp);
extern bool calibrateMagnetometer();
void identifyAccelAxes();  // Will implement in accel_calibration.cpp

/**
 * Display the main calibration menu
 */
void showCalibrationMenu() {
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║    CALIBRATION MENU                ║");
    Serial.println("╠════════════════════════════════════╣");
    Serial.println("║ 1. Quick Calibration (~2 min)      ║");
    Serial.println("║ 2. Robust Calibration (~10 min)    ║");
    Serial.println("║ 3. Individual Sensors              ║");
    Serial.println("║ 4. Axis Identification             ║");
    Serial.println("║ 5. Validation & Status             ║");
    Serial.println("║ 6. Advanced Options                ║");
    Serial.println("║ 0. Exit Menu                       ║");
    Serial.println("╚════════════════════════════════════╝");
    Serial.print("\nEnter choice: ");
}

/**
 * Handle menu input
 */
void handleMenuInput(char input) {
    Serial.println(input);  // Echo the input
    
    switch (input) {
        case '1':
            menuQuickCalibration();
            break;
        case '2':
            menuRobustCalibration();
            break;
        case '3':
            menuIndividualSensors();
            break;
        case '4':
            menuAxisIdentification();
            break;
        case '5':
            menuValidationStatus();
            break;
        case '6':
            menuAdvancedOptions();
            break;
        case '0':
            Serial.println("\nExiting calibration menu...\n");
            return;
        default:
            Serial.println("\nInvalid choice. Please try again.");
            break;
    }
    
    // Show menu again after action completes
    if (input != '0') {
        delay(1000);
        showCalibrationMenu();
    }
}

/**
 * Quick calibration (bias-only for all sensors)
 */
void menuQuickCalibration() {
    Serial.println("\n=== QUICK CALIBRATION ===");
    Serial.println("This will calibrate all sensors with simple bias correction.");
    Serial.println("Estimated time: ~2 minutes\n");
    
    // Accelerometer
    Serial.println("--- Accelerometer ---");
    bool accelOk = calibrateAccelSimple();
    
    // Gyroscope
    Serial.println("\n--- Gyroscope ---");
    bool gyroOk = calibrateGyroSimple();
    
    // Magnetometer
    Serial.println("\n--- Magnetometer ---");
    bool magOk = calibrateMagnetometer();
    
    // Summary
    Serial.println("\n=== QUICK CALIBRATION COMPLETE ===");
    Serial.printf("Accelerometer: %s\n", accelOk ? "✓ PASS" : "✗ FAIL");
    Serial.printf("Gyroscope: %s\n", gyroOk ? "✓ PASS" : "✗ FAIL");
    Serial.printf("Magnetometer: %s\n", magOk ? "✓ PASS" : "✗ FAIL");
    
    if (accelOk && gyroOk && magOk) {
        Serial.println("\nSaving calibration to EEPROM...");
        saveCalibrationToEEPROM();
        Serial.println("✓ Calibration saved!");
    } else {
        Serial.println("\n⚠ Some sensors failed calibration. Data not saved.");
    }
}

/**
 * Robust calibration (6-position for accel, 6-position for gyro)
 */
void menuRobustCalibration() {
    Serial.println("\n=== ROBUST CALIBRATION ===");
    Serial.println("This will perform comprehensive calibration:");
    Serial.println("- Accelerometer: 6-position (bias + scale)");
    Serial.println("- Gyroscope: 6-position");
    Serial.println("- Magnetometer: Ellipsoid fit");
    Serial.println("Estimated time: ~10 minutes\n");
    
    // Accelerometer
    Serial.println("--- Accelerometer (6-position) ---");
    bool accelOk = calibrateAccel6Position(false);
    
    // Gyroscope
    Serial.println("\n--- Gyroscope (simple) ---");
    bool gyroOk = calibrateGyroSimple();
    
    // Magnetometer
    Serial.println("\n--- Magnetometer (Ellipsoid Fit) ---");
    bool magOk = calibrateMagnetometer();
    
    // Get calibration data for quality assessment
    const CalibrationData& cal = getCalibrationData();
    
    // Assess quality
    CalibrationQuality accelQuality = assessAccelQuality((float*)cal.accelBias, (float*)cal.accelScale);
    CalibrationQuality gyroQuality = assessGyroQuality((float*)cal.gyroBias);
    CalibrationQuality magQuality = assessMagQuality((float*)cal.magBias, (float*)cal.magScale);
    
    // Summary with quality feedback
    Serial.println("\n╔════════════════════════════════════════════════════╗");
    Serial.println("║       ROBUST CALIBRATION COMPLETE                 ║");
    Serial.println("╠════════════════════════════════════════════════════╣");
    
    Serial.printf("║ Accelerometer: %-8s ", accelOk ? "PASS" : "FAIL");
    Serial.printf("Quality: %-12s ║\n", qualityToString(accelQuality));
    
    Serial.printf("║ Gyroscope:     %-8s ", gyroOk ? "PASS" : "FAIL");
    Serial.printf("Quality: %-12s ║\n", qualityToString(gyroQuality));
    
    Serial.printf("║ Magnetometer:  %-8s ", magOk ? "PASS" : "FAIL");
    Serial.printf("Quality: %-12s ║\n", qualityToString(magQuality));
    
    Serial.println("╚════════════════════════════════════════════════════╝");
    
    // Detailed quality reports
    printQualityReport("Accelerometer", accelQuality);
    printQualityReport("Gyroscope", gyroQuality);
    printQualityReport("Magnetometer", magQuality);
    
    // Save if all passed
    if (accelOk && gyroOk && magOk) {
        Serial.println("\n✓ Saving calibration to EEPROM...");
        saveCalibrationToEEPROM();
        Serial.println("✓ Calibration saved and will be applied to sensor fusion!");
    } else {
        Serial.println("\n⚠ Some sensors failed calibration. Data not saved.");
        Serial.println("Tip: Try recalibrating failed sensors individually from the menu.");
    }
}

/**
 * Individual sensor calibration menu
 */
void menuIndividualSensors() {
    Serial.println("\n=== INDIVIDUAL SENSOR CALIBRATION ===");
    Serial.println("a. Accelerometer");
    Serial.println("b. Gyroscope");
    Serial.println("c. Magnetometer");
    Serial.println("0. Back to main menu");
    Serial.print("\nEnter choice: ");
    
    while (!Serial.available()) delay(10);
    char choice = Serial.read();
    Serial.println(choice);
    
    switch (choice) {
        case 'a':
        case 'A': {
            Serial.println("\n--- Accelerometer Calibration ---");
            Serial.println("1. Simple (bias only)");
            Serial.println("2. 6-position (bias + scale)");
            Serial.print("Choice: ");
            while (!Serial.available()) delay(10);
            char accelChoice = Serial.read();
            Serial.println(accelChoice);
            bool accelSaved = false;
            if (accelChoice == '1') {
                accelSaved = calibrateAccelSimple();
            } else if (accelChoice == '2') {
                accelSaved = calibrateAccel6Position(false);
            }
            if (accelSaved) {
                Serial.println("\n✓ Saving calibration to EEPROM...");
                saveCalibrationToEEPROM();
                Serial.println("✓ Calibration saved!");
            }
            break;
        }
            
        case 'b':
        case 'B': {
            Serial.println("\n--- Gyroscope Calibration ---");
            Serial.println("1. Simple (bias only)");
            Serial.println("2. 6-position (recommended)");
            Serial.print("Choice: ");
            while (!Serial.available()) delay(10);
            char gyroChoice = Serial.read();
            Serial.println(gyroChoice);
            bool gyroSaved = false;
            if (gyroChoice == '1') {
                gyroSaved = calibrateGyroSimple();
            } else if (gyroChoice == '2') {
                gyroSaved = calibrateGyro6Position(false);
            }
            if (gyroSaved) {
                Serial.println("\n✓ Saving calibration to EEPROM...");
                saveCalibrationToEEPROM();
                Serial.println("✓ Calibration saved!");
            }
            break;
        }
            
        case 'c':
        case 'C':
            Serial.println("\n--- Magnetometer Calibration ---");
            if (calibrateMagnetometer()) {
                Serial.println("\n✓ Saving calibration to EEPROM...");
                saveCalibrationToEEPROM();
                Serial.println("✓ Calibration saved!");
            }
            break;
            
        case '0':
            return;
            
        default:
            Serial.println("Invalid choice.");
            break;
    }
}

/**
 * Axis identification tool
 */
void menuAxisIdentification() {
    Serial.println("\n=== AXIS IDENTIFICATION ===");
    identifyAccelAxes();  // Will implement this function
}

/**
 * Show validation and status
 */
void menuValidationStatus() {
    Serial.println("\n=== CALIBRATION STATUS ===");
    
    // Show current calibration status
    const CalibrationData& cal = getCalibrationData();
    
    Serial.println("\n--- Accelerometer ---");
    Serial.printf("Calibrated: %s\n", cal.accelCalibrated ? "YES" : "NO");
    if (cal.accelCalibrated) {
        Serial.printf("Method: %s\n", 
                     cal.accelCalMethod == ACCEL_CAL_SIMPLE ? "Simple" : "6-Position");
        Serial.printf("Bias (mg): X=%.2f, Y=%.2f, Z=%.2f\n",
                     cal.accelBias[0], cal.accelBias[1], cal.accelBias[2]);
        Serial.printf("Scale: X=%.4f, Y=%.4f, Z=%.4f\n",
                     cal.accelScale[0], cal.accelScale[1], cal.accelScale[2]);
    }
    
    Serial.println("\n--- Gyroscope ---");
    Serial.printf("Calibrated: %s\n", cal.gyroCalibrated ? "YES" : "NO");
    if (cal.gyroCalibrated) {
        Serial.printf("Bias (dps): X=%.4f, Y=%.4f, Z=%.4f\n",
                     cal.gyroBias[0], cal.gyroBias[1], cal.gyroBias[2]);
        Serial.printf("Temp Comp: %s\n", cal.gyroTempCompEnabled ? "YES" : "NO");
        if (cal.gyroTempCompEnabled) {
            Serial.printf("Temp Coeff (dps/°C): X=%.6f, Y=%.6f, Z=%.6f\n",
                         cal.gyroTempCoeff[0], cal.gyroTempCoeff[1], cal.gyroTempCoeff[2]);
        }
    }
    
    Serial.println("\n--- Magnetometer ---");
    Serial.printf("Calibrated: %s\n", cal.magCalibrated ? "YES" : "NO");
    if (cal.magCalibrated) {
        Serial.printf("Bias (µT): X=%.2f, Y=%.2f, Z=%.2f\n",
                     cal.magBias[0], cal.magBias[1], cal.magBias[2]);
        Serial.printf("Scale: X=%.4f, Y=%.4f, Z=%.4f\n",
                     cal.magScale[0], cal.magScale[1], cal.magScale[2]);
        Serial.printf("Declination: %.2f°\n", cal.magDeclination);
    }
    
    Serial.println("\nPress any key to continue...");
    while (!Serial.available()) delay(10);
    Serial.read();
}

/**
 * Advanced options menu
 */
void menuAdvancedOptions() {
    Serial.println("\n=== ADVANCED OPTIONS ===");
    Serial.println("1. Clear all calibration data");
    Serial.println("2. Export calibration data");
    Serial.println("3. Run extended validation");
    Serial.println("0. Back to main menu");
    Serial.print("\nEnter choice: ");
    
    while (!Serial.available()) delay(10);
    char choice = Serial.read();
    Serial.println(choice);
    
    switch (choice) {
        case '1':
            Serial.println("\n⚠ WARNING: This will erase all calibration data!");
            Serial.print("Type 'YES' to confirm: ");
            {
                String confirm = "";
                unsigned long timeout = millis() + 10000;
                while (millis() < timeout && confirm.length() < 3) {
                    if (Serial.available()) {
                        char c = Serial.read();
                        Serial.print(c);
                        confirm += c;
                    }
                }
                Serial.println();
                if (confirm == "YES") {
                    clearCalibrationEEPROM();
                    Serial.println("✓ Calibration data cleared.");
                } else {
                    Serial.println("Cancelled.");
                }
            }
            break;
            
        case '2':
            Serial.println("\n=== CALIBRATION DATA EXPORT ===");
            // TODO: Implement JSON export
            Serial.println("Feature coming soon...");
            break;
            
        case '3':
            Serial.println("\n=== EXTENDED VALIDATION ===");
            // TODO: Implement extended validation
            Serial.println("Feature coming soon...");
            break;
            
        case '0':
            return;
            
        default:
            Serial.println("Invalid choice.");
            break;
    }
}
