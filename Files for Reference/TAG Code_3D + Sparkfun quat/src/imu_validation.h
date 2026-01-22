#ifndef IMU_VALIDATION_H
#define IMU_VALIDATION_H

#include <Arduino.h>

// Structure to hold a complete snapshot of IMU data for validation
struct IMUData {
    // Raw (hardware-filtered) sensor data
    float raw_ax, raw_ay, raw_az;       // m/s^2
    float raw_gx, raw_gy, raw_gz;       // rad/s
    float raw_mx, raw_my, raw_mz;       // uT
    float raw_temp_c;

    // Software-filtered sensor data (if any)
    float filt_ax, filt_ay, filt_az;
    float filt_gx, filt_gy, filt_gz;
    float filt_mx, filt_my, filt_mz;
    float filt_temp_c;

    // Fusion output
    float q0, q1, q2, q3;               // Quaternion
    float roll, pitch, yaw;             // Euler angles (degrees)
};

// In imu_validation.h
#pragma once

/**
 * Check if device is still (not moving)
 * 
 * @param accel Accelerometer readings [x, y, z] in mg
 * @param gyro Gyroscope readings [x, y, z] in dps
 * @param accelThreshold Maximum allowed accelerometer deviation from 1g (default 0.35g)
 * @param gyroThreshold Maximum allowed gyroscope magnitude in dps (default 0.2 dps)
 * @return true if device is still, false otherwise
 */
bool isDeviceStill(float accel[3], float gyro[3], float accelThreshold = 0.35f, float gyroThreshold = 0.2f);

/**
 * Wait for device to reach stable orientation
 * 
 * This function waits for the device to be placed in a specific orientation and held still.
 * It provides real-time feedback showing current sensor readings and target orientation.
 * 
 * @param targetAxis Which axis should point up/down (0=X, 1=Y, 2=Z)
 * @param targetValue Expected value in mg (e.g., +1000 for up, -1000 for down)
 * @param threshold Tolerance in mg (default 200mg)
 * @param timeoutMs Maximum time to wait in milliseconds (default 30000ms = 30s)
 * @return true if stable orientation achieved, false if timeout
 */
bool waitForStableOrientation(int targetAxis, float targetValue, float threshold = 200.0f, int timeoutMs = 30000);

// Function prototype for the validation suite
void runValidation(const IMUData& data);

#endif // IMU_VALIDATION_H
