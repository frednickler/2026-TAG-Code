#ifndef IMU_VALIDATION_H
#define IMU_VALIDATION_H

#include <Arduino.h>
#include "sensor_fusion.h"

// Data structure to hold IMU readings for validation
struct IMUData {
    // Raw readings
    float raw_ax, raw_ay, raw_az;  // Accelerometer (m/s²)
    float raw_gx, raw_gy, raw_gz;  // Gyroscope (rad/s)
    float raw_mx, raw_my, raw_mz;  // Magnetometer (μT)
    float raw_temp_c;              // Raw temperature (Celsius)
    
    // Filtered readings
    float filt_ax, filt_ay, filt_az;
    float filt_gx, filt_gy, filt_gz;
    float filt_mx, filt_my, filt_mz;
    float filt_temp_c;             // Filtered temperature (Celsius)
    
    // Quaternion and Euler angles
    float q0, q1, q2, q3;          // Quaternion components
    float roll, pitch, yaw;        // Euler angles (degrees)
};

// Validation functions
void printRawVsFiltered(const IMUData& data);
void validateFilterResponse(const IMUData& data);
void validateQuaternion(const IMUData& data);
void validateEulerAngles(const IMUData& data);
void staticValidation(const IMUData& data);
void logDataCSV(const IMUData& data);

// Main validation function that runs all checks
void runValidation(const IMUData& data);

// Helper functions
float calculateMagnitude(float x, float y, float z);
float calculateQuaternionMagnitude(float w, float x, float y, float z);
void eulerToQuaternion(float roll, float pitch, float yaw, 
                       float& q0, float& q1, float& q2, float& q3);

#endif // IMU_VALIDATION_H
