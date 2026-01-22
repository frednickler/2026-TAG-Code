#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>

// Structure to hold calibration data
struct CalibrationData {
    float accel_bias[3] = {0, 0, 0};
    float gyro_bias[3] = {0, 0, 0};
    float mag_bias[3] = {0, 0, 0};
    float mag_scale[3] = {1, 1, 1};
    float mag_declination = 0.0f;
};

// Function Prototypes
void initCalibration();
/**
 * Load calibration data from EEPROM
 * @return true if data loaded and validated successfully
 */
bool loadCalibrationFromEEPROM();
void saveCalibrationToEEPROM();
void resetCalibration();
void calibrateAccelGyro();
void calibrateMag();
bool isDeviceStill(float motion_threshold);
const CalibrationData& getCalibrationData();

#endif // CALIBRATION_H
