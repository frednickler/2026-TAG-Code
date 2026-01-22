#ifndef CALIBRATION_QUALITY_H
#define CALIBRATION_QUALITY_H

#include <Arduino.h>

// Quality assessment levels
enum CalibrationQuality {
    CAL_QUALITY_FAILED = 0,
    CAL_QUALITY_POOR = 1,
    CAL_QUALITY_ACCEPTABLE = 2,
    CAL_QUALITY_GOOD = 3,
    CAL_QUALITY_EXCELLENT = 4
};

/**
 * Assess accelerometer calibration quality
 * 
 * @param bias Bias values in mg [x, y, z]
 * @param scale Scale factors [x, y, z]
 * @return Quality level
 */
CalibrationQuality assessAccelQuality(float bias[3], float scale[3]);

/**
 * Assess gyroscope calibration quality
 * 
 * @param bias Bias values in dps [x, y, z]
 * @return Quality level
 */
CalibrationQuality assessGyroQuality(float bias[3]);

/**
 * Assess magnetometer calibration quality
 * 
 * @param bias Bias values in µT [x, y, z]
 * @param scale Scale factors [x, y, z]
 * @return Quality level
 */
CalibrationQuality assessMagQuality(float bias[3], float scale[3]);

/**
 * Print calibration quality report
 * 
 * @param sensorName Name of the sensor
 * @param quality Quality level
 */
void printQualityReport(const char* sensorName, CalibrationQuality quality);

/**
 * Get quality level as string
 */
const char* qualityToString(CalibrationQuality quality);

#endif
