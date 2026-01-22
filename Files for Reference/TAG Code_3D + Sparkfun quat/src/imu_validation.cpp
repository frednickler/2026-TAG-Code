#include "imu_validation.h"
#include "imu_config.h"
#include <math.h>

// Modified to be more tolerant of slight tilts
bool isDeviceStill(float accel[3], float gyro[3], float accelThreshold, float gyroThreshold) {
    // Calculate accelerometer magnitude
    float accelMagnitude = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    
    // Normalize to g if in mg (common issue with different scale settings)
    if (accelMagnitude > 100.0f) {
        accelMagnitude /= 1000.0f;
    }
    
    // Calculate gyroscope magnitude (should be close to 0 when still)
    float gyroMagnitude = sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    
    // Static variables to track previous readings for stability check
    static float prevAccelMag = 0;
    static float prevGyroMag = 0;
    static bool firstCall = true;
    
    // On first call, initialize previous values
    if (firstCall) {
        prevAccelMag = accelMagnitude;
        prevGyroMag = gyroMagnitude;
        firstCall = false;
        return false; // Need at least two readings to determine stability
    }
    
    // Check if device is still based on three criteria:
    // 1. Gyro magnitude is below threshold (minimal rotation)
    // 2. Accelerometer magnitude is stable (minimal change between readings)
    // 3. Accelerometer magnitude is within reasonable range of 1g (allowing for tilt)
    bool gyroStill = (gyroMagnitude < gyroThreshold);
    bool accelStable = (fabs(accelMagnitude - prevAccelMag) < 0.05f);
    bool accelReasonable = (fabs(accelMagnitude - 1.0f) < accelThreshold);
    
    // Update previous values for next call
    prevAccelMag = accelMagnitude;
    prevGyroMag = gyroMagnitude;
    
    // Device is considered still if all criteria are met
    return gyroStill && accelStable && accelReasonable;
}

/**
 * Wait for device to reach stable orientation with real-time feedback
 * Ported from V2 calibration system
 */
bool waitForStableOrientation(int targetAxis, float targetValue, float threshold, int timeoutMs) {
    const char* axisNames[] = {"X", "Y", "Z"};
    const char* direction = (targetValue > 0) ? "UP" : "DOWN";
    
    Serial.printf("\nTilt device until you see [%s-%s] appear below:\n", axisNames[targetAxis], direction);
    Serial.println("(Device will auto-detect when in correct position)\n");
    
    unsigned long startTime = millis();
    int stableCount = 0;
    const int requiredStableReadings = 10;  // Must be stable for 10 consecutive readings
    
    while (millis() - startTime < timeoutMs) {
        float accel[3], gyro[3];
        
        if (readAccel(accel[0], accel[1], accel[2]) && readGyro(gyro[0], gyro[1], gyro[2])) {
            // Check if target axis is within threshold of target value
            bool axisCorrect = abs(accel[targetAxis] - targetValue) < threshold;
            
            // Check if device is still (using relaxed thresholds for calibration)
            bool deviceStill = isDeviceStill(accel, gyro, 0.5f, 1.0f);
            
            if (axisCorrect && deviceStill) {
                stableCount++;
                if (stableCount >= requiredStableReadings) {
                    Serial.println("\n✓ Stable orientation detected!\n");
                    return true;
                }
            } else {
                stableCount = 0;
            }
            
            // Real-time feedback - show current orientation
            Serial.printf("\rX: %+7.1f  Y: %+7.1f  Z: %+7.1f mg  ->  ", 
                         accel[0], accel[1], accel[2]);
            
            // Identify dominant axis and direction
            float absX = abs(accel[0]);
            float absY = abs(accel[1]);
            float absZ = abs(accel[2]);
            
            if (absZ > 800 && absZ > absX && absZ > absY) {
                Serial.printf("[Z-%s]", accel[2] > 0 ? "UP" : "DOWN");
                if (targetAxis == 2 && axisCorrect) Serial.print(" ✓");
            } else if (absY > 800 && absY > absX && absY > absZ) {
                Serial.printf("[Y-%s]", accel[1] > 0 ? "UP" : "DOWN");
                if (targetAxis == 1 && axisCorrect) Serial.print(" ✓");
            } else if (absX > 800 && absX > absY && absX > absZ) {
                Serial.printf("[X-%s]", accel[0] > 0 ? "UP" : "DOWN");
                if (targetAxis == 0 && axisCorrect) Serial.print(" ✓");
            } else {
                Serial.print("[TILTED]");
            }
            
            // Show stability indicator
            if (stableCount > 0) {
                Serial.printf(" Stable: %d/%d", stableCount, requiredStableReadings);
            }
        }
        
        delay(100);
    }
    
    Serial.println("\n\n✗ Timeout - orientation not achieved");
    return false;
}

// Only one implementation of isDeviceStill should exist in the project.

// Print raw vs filtered data comparison
void printRawVsFiltered(const IMUData& data) {
    Serial.println("=== RAW vs FILTERED DATA ===");
    
    Serial.print("Raw Accel (m/s²): ");
    Serial.print(data.raw_ax, 4); Serial.print(", ");
    Serial.print(data.raw_ay, 4); Serial.print(", ");
    Serial.println(data.raw_az, 4);
    
    Serial.print("Filtered Accel: ");
    Serial.print(data.filt_ax, 4); Serial.print(", ");
    Serial.print(data.filt_ay, 4); Serial.print(", ");
    Serial.println(data.filt_az, 4);
    
    Serial.print("Raw Gyro (rad/s): ");
    Serial.print(data.raw_gx, 4); Serial.print(", ");
    Serial.print(data.raw_gy, 4); Serial.print(", ");
    Serial.println(data.raw_gz, 4);
    
    Serial.print("Filtered Gyro: ");
    Serial.print(data.filt_gx, 4); Serial.print(", ");
    Serial.print(data.filt_gy, 4); Serial.print(", ");
    Serial.println(data.filt_gz, 4);
    
    if (data.raw_mx != 0 || data.raw_my != 0 || data.raw_mz != 0) {
        Serial.print("Raw Mag (μT): ");
        Serial.print(data.raw_mx, 4); Serial.print(", ");
        Serial.print(data.raw_my, 4); Serial.print(", ");
        Serial.println(data.raw_mz, 4);
        
        Serial.print("Filtered Mag: ");
        Serial.print(data.filt_mx, 4); Serial.print(", ");
        Serial.print(data.filt_my, 4); Serial.print(", ");
        Serial.println(data.filt_mz, 4);
    }
    
    Serial.print("Raw Temp (°C): "); Serial.println(data.raw_temp_c, 2);
    Serial.print("Filtered Temp (°C): "); Serial.println(data.filt_temp_c, 2);
    
    Serial.println();
}

// Helper function to calculate vector magnitude
float calculateMagnitude(float x, float y, float z) {
    return sqrt(x*x + y*y + z*z);
}

// Helper function to calculate quaternion magnitude
float calculateQuaternionMagnitude(float w, float x, float y, float z) {
    return sqrt(w*w + x*x + y*y + z*z);
}

// Validate filter response and attenuation
void validateFilterResponse(const IMUData& data) {
    // Calculate magnitudes
    float rawAccelMag = calculateMagnitude(data.raw_ax, data.raw_ay, data.raw_az);
    float filtAccelMag = calculateMagnitude(data.filt_ax, data.filt_ay, data.filt_az);
    float rawGyroMag = calculateMagnitude(data.raw_gx, data.raw_gy, data.raw_gz);
    float filtGyroMag = calculateMagnitude(data.filt_gx, data.filt_gy, data.filt_gz);
    
    // Calculate attenuation ratios
    float accelRatio = (rawAccelMag > 0.01f) ? filtAccelMag / rawAccelMag : 1.0f;
    float gyroRatio = (rawGyroMag > 0.01f) ? filtGyroMag / rawGyroMag : 1.0f;
    
    Serial.println("=== FILTER RESPONSE ===");
    Serial.print("Accel Filter Ratio: "); Serial.println(accelRatio, 4);
    Serial.print("Gyro Filter Ratio: "); Serial.println(gyroRatio, 4);
    
    // Check for filter issues
    if (accelRatio > 1.1f || gyroRatio > 1.1f) {
        Serial.println("WARNING: Filter may be amplifying signals!");
        Serial.print("DLPF Settings - Accel: "); Serial.print(ACCEL_DLPF_LEVEL);
        Serial.print(", Gyro: "); Serial.println(GYRO_DLPF_LEVEL);
    }
    
    Serial.println();
}

// Validate quaternion properties
void validateQuaternion(const IMUData& data) {
    // Calculate quaternion norm
    float norm = calculateQuaternionMagnitude(data.q0, data.q1, data.q2, data.q3);
    
    Serial.println("=== QUATERNION VALIDATION ===");
    Serial.print("Quaternion [w,x,y,z]: ");
    Serial.print(data.q0, 4); Serial.print(", ");
    Serial.print(data.q1, 4); Serial.print(", ");
    Serial.print(data.q2, 4); Serial.print(", ");
    Serial.println(data.q3, 4);
    
    Serial.print("Quaternion Norm: "); Serial.println(norm, 6);
    
    // Check normalization
    if (abs(norm - 1.0f) > 0.01f) {
        Serial.println("WARNING: Quaternion not normalized!");
        Serial.print("Deviation from unit norm: "); 
        Serial.println(abs(norm - 1.0f), 6);
    }
    
    // Check for NaN/Inf
    if (isnan(data.q0) || isnan(data.q1) || isnan(data.q2) || isnan(data.q3) ||
        isinf(data.q0) || isinf(data.q1) || isinf(data.q2) || isinf(data.q3)) {
        Serial.println("ERROR: Quaternion contains NaN or Inf values!");
    }
    
    Serial.println();
}

// Validate temperature readings
void validateTemperature(const IMUData& data) {
    Serial.println("=== TEMPERATURE VALIDATION ===");
    Serial.print("Temperature: "); Serial.print(data.filt_temp_c, 2); Serial.println(" °C");

    if (data.filt_temp_c < 0.0f || data.filt_temp_c > 85.0f) {
        Serial.println("WARNING: Temperature is outside the expected operating range (0-85°C)!");
    }

    Serial.println();
}

// Convert Euler angles to quaternion for validation
void eulerToQuaternion(float roll, float pitch, float yaw, 
                       float& q0, float& q1, float& q2, float& q3) {
    // Convert angles to radians
    float cr = cos(roll * PI / 360.0f);   // Half-angles
    float cp = cos(pitch * PI / 360.0f);
    float cy = cos(yaw * PI / 360.0f);
    float sr = sin(roll * PI / 360.0f);
    float sp = sin(pitch * PI / 360.0f);
    float sy = sin(yaw * PI / 360.0f);
    
    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
}

// Validate Euler angles
void validateEulerAngles(const IMUData& data) {
    Serial.println("=== EULER ANGLE VALIDATION ===");
    Serial.print("Euler Angles (deg) - Roll: ");
    Serial.print(data.roll, 2);
    Serial.print(", Pitch: ");
    Serial.print(data.pitch, 2);
    Serial.print(", Yaw: ");
    Serial.println(data.yaw, 2);
    
    // Check ranges
    if (abs(data.roll) > 180.0f || abs(data.pitch) > 90.0f) {
        Serial.println("WARNING: Roll/Pitch angles out of expected range!");
        Serial.println("Expected: -180° ≤ roll ≤ 180°, -90° ≤ pitch ≤ 90°");
    }
    
    // Convert back to quaternion and compare
    float q0_calc, q1_calc, q2_calc, q3_calc;
    eulerToQuaternion(data.roll, data.pitch, data.yaw, 
                      q0_calc, q1_calc, q2_calc, q3_calc);
    
    // Calculate quaternion difference, accounting for q vs -q ambiguity
    float error1 = abs(q0_calc - data.q0) + abs(q1_calc - data.q1) + 
                   abs(q2_calc - data.q2) + abs(q3_calc - data.q3);

    float error2 = abs(-q0_calc - data.q0) + abs(-q1_calc - data.q1) + 
                   abs(-q2_calc - data.q2) + abs(-q3_calc - data.q3);

    float error = min(error1, error2);
    
    Serial.print("Quaternion Round-trip Error: ");
    Serial.println(error, 6);
    if (error > 0.1f) {
        Serial.println("WARNING: Large quaternion/Euler conversion discrepancy!");
    }
    
    Serial.println();
}

// Validate static conditions
void staticValidation(const IMUData& data) {
    Serial.println("=== STATIC VALIDATION ===");
    
    // Check accelerometer magnitude (should be ~9.81 m/s² when static)
    float accelMag = calculateMagnitude(data.filt_ax, data.filt_ay, data.filt_az);
    Serial.print("Accel Magnitude: ");
    Serial.print(accelMag, 4);
    Serial.print(" m/s² (Expected: 9.81)");
    
    if (abs(accelMag - 9.81f) > 0.5f) {
        Serial.println(" - WARNING: Large deviation from gravity!");
    } else {
        Serial.println(" - OK");
    }
    
    // Check gyroscope magnitude (should be near 0 when static)
    float gyroMag = calculateMagnitude(data.filt_gx, data.filt_gy, data.filt_gz);
    Serial.print("Gyro Magnitude: ");
    Serial.print(gyroMag, 4);
    Serial.print(" rad/s (Expected: ~0)");
    
    if (gyroMag > 0.1f) {
        Serial.println(" - WARNING: High gyro readings in static condition!");
    } else {
        Serial.println(" - OK");
    }
    
    Serial.println();
}

// Log data in CSV format
void logDataCSV(const IMUData& data) {
    static bool headerPrinted = false;
    
    if (!headerPrinted) {
        Serial.println("timestamp,raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,raw_mx,raw_my,raw_mz,raw_temp_c,"
                      "filt_ax,filt_ay,filt_az,filt_gx,filt_gy,filt_gz,filt_mx,filt_my,filt_mz,filt_temp_c,"
                      "q0,q1,q2,q3,roll,pitch,yaw");
        headerPrinted = true;
    }
    
    Serial.print(millis()); Serial.print(",");
    Serial.print(data.raw_ax, 4); Serial.print(",");
    Serial.print(data.raw_ay, 4); Serial.print(",");
    Serial.print(data.raw_az, 4); Serial.print(",");
    Serial.print(data.raw_gx, 4); Serial.print(",");
    Serial.print(data.raw_gy, 4); Serial.print(",");
    Serial.print(data.raw_gz, 4); Serial.print(",");
    Serial.print(data.raw_mx, 4); Serial.print(",");
    Serial.print(data.raw_my, 4); Serial.print(",");
    Serial.print(data.raw_mz, 4); Serial.print(",");
    Serial.print(data.raw_temp_c, 2); Serial.print(",");
    Serial.print(data.filt_ax, 4); Serial.print(",");
    Serial.print(data.filt_ay, 4); Serial.print(",");
    Serial.print(data.filt_az, 4); Serial.print(",");
    Serial.print(data.filt_gx, 4); Serial.print(",");
    Serial.print(data.filt_gy, 4); Serial.print(",");
    Serial.print(data.filt_gz, 4); Serial.print(",");
    Serial.print(data.filt_mx, 4); Serial.print(",");
    Serial.print(data.filt_my, 4); Serial.print(",");
    Serial.print(data.filt_mz, 4); Serial.print(",");
    Serial.print(data.filt_temp_c, 2); Serial.print(",");
    Serial.print(data.q0, 4); Serial.print(",");
    Serial.print(data.q1, 4); Serial.print(",");
    Serial.print(data.q2, 4); Serial.print(",");
    Serial.print(data.q3, 4); Serial.print(",");
    Serial.print(data.roll, 2); Serial.print(",");
    Serial.print(data.pitch, 2); Serial.print(",");
    Serial.println(data.yaw, 2);
}

// Run all validations
void runValidation(const IMUData& data) {
    Serial.println("\n=== IMU VALIDATION REPORT ===");
    printRawVsFiltered(data);
    validateFilterResponse(data);
    validateQuaternion(data);
    validateEulerAngles(data);
    validateTemperature(data);
    // Add extra newline for spacing to prevent serial print glitches
    Serial.println(); 
    staticValidation(data);
    Serial.println("=== END OF REPORT ===\n");
}
