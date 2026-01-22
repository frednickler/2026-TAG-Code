#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <Arduino.h>
#include "imu_config.h"

// Forward declarations for I2C communication functions
// These will be defined in utilities.h/.cpp
void writeRegister(uint8_t reg, uint8_t val);
uint8_t readRegister(uint8_t reg);

// Calibration offsets
extern float accel_offset[3];
extern float gyro_offset[3];
extern float mag_offset[3];
extern float mag_scale[3];  // For soft iron correction

// Calibration status flags
extern bool accel_gyro_calibrated;
extern bool mag_calibrated;

// Function prototypes
void initCalibration();
bool calibrateAccelGyro(unsigned long sample_time_ms = 5000);
bool calibrateMagnetometer(unsigned long sample_time_ms = 15000);
void applyCalibration(float* accel_data, float* gyro_data, float* mag_data);
void saveCalibrationToEEPROM();
bool loadCalibrationFromEEPROM();
void resetCalibration();

// Temperature compensation
float readIMUTemperature();
void updateTempCompOffsets(float temp);


// Helper functions
void printCalibrationValues();
bool isDeviceStill(float* accel_data, float* gyro_data, float threshold_accel = 0.1f, float threshold_gyro = 1.0f);

#endif // IMU_CALIBRATION_H
