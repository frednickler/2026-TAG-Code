#include "calibration_quality.h"
#include <math.h>

/**
 * Assess accelerometer calibration quality
 */
CalibrationQuality assessAccelQuality(float bias[3], float scale[3]) {
    // Check for NaN or invalid values
    for (int i = 0; i < 3; i++) {
        if (isnan(bias[i]) || isnan(scale[i]) || isinf(bias[i]) || isinf(scale[i])) {
            return CAL_QUALITY_FAILED;
        }
    }
    
    // Calculate maximum bias magnitude
    float maxBias = 0;
    for (int i = 0; i < 3; i++) {
        float absBias = abs(bias[i]);
        if (absBias > maxBias) maxBias = absBias;
    }
    
    // Calculate maximum scale deviation from 1.0
    float maxScaleError = 0;
    for (int i = 0; i < 3; i++) {
        float scaleError = abs(scale[i] - 1.0f);
        if (scaleError > maxScaleError) maxScaleError = scaleError;
    }
    
    // Quality thresholds
    // Bias: < 10mg = Excellent, < 30mg = Good, < 50mg = Acceptable, < 100mg = Poor
    // Scale: < 0.01 (1%) = Excellent, < 0.03 (3%) = Good, < 0.05 (5%) = Acceptable, < 0.10 (10%) = Poor
    
    if (maxBias < 10.0f && maxScaleError < 0.01f) {
        return CAL_QUALITY_EXCELLENT;
    } else if (maxBias < 30.0f && maxScaleError < 0.03f) {
        return CAL_QUALITY_GOOD;
    } else if (maxBias < 50.0f && maxScaleError < 0.05f) {
        return CAL_QUALITY_ACCEPTABLE;
    } else if (maxBias < 100.0f && maxScaleError < 0.10f) {
        return CAL_QUALITY_POOR;
    } else {
        return CAL_QUALITY_FAILED;
    }
}

/**
 * Assess gyroscope calibration quality
 */
CalibrationQuality assessGyroQuality(float bias[3]) {
    // Check for NaN or invalid values
    for (int i = 0; i < 3; i++) {
        if (isnan(bias[i]) || isinf(bias[i])) {
            return CAL_QUALITY_FAILED;
        }
    }
    
    // Calculate maximum bias magnitude
    float maxBias = 0;
    for (int i = 0; i < 3; i++) {
        float absBias = abs(bias[i]);
        if (absBias > maxBias) maxBias = absBias;
    }
    
    // Quality thresholds (in dps)
    // < 0.1 = Excellent, < 0.5 = Good, < 1.0 = Acceptable, < 2.0 = Poor
    
    if (maxBias < 0.1f) {
        return CAL_QUALITY_EXCELLENT;
    } else if (maxBias < 0.5f) {
        return CAL_QUALITY_GOOD;
    } else if (maxBias < 1.0f) {
        return CAL_QUALITY_ACCEPTABLE;
    } else if (maxBias < 2.0f) {
        return CAL_QUALITY_POOR;
    } else {
        return CAL_QUALITY_FAILED;
    }
}

/**
 * Assess magnetometer calibration quality
 */
CalibrationQuality assessMagQuality(float bias[3], float scale[3]) {
    // Check for NaN or invalid values
    for (int i = 0; i < 3; i++) {
        if (isnan(bias[i]) || isnan(scale[i]) || isinf(bias[i]) || isinf(scale[i])) {
            return CAL_QUALITY_FAILED;
        }
    }
    
    // Calculate bias magnitude
    float biasMag = sqrt(bias[0]*bias[0] + bias[1]*bias[1] + bias[2]*bias[2]);
    
    // Calculate maximum scale deviation from 1.0
    float maxScaleError = 0;
    for (int i = 0; i < 3; i++) {
        float scaleError = abs(scale[i] - 1.0f);
        if (scaleError > maxScaleError) maxScaleError = scaleError;
    }
    
    // Quality thresholds
    // Bias magnitude: < 20µT = Excellent, < 50µT = Good, < 100µT = Acceptable, < 200µT = Poor
    // Scale: < 0.05 (5%) = Excellent, < 0.10 (10%) = Good, < 0.15 (15%) = Acceptable, < 0.25 (25%) = Poor
    
    if (biasMag < 20.0f && maxScaleError < 0.05f) {
        return CAL_QUALITY_EXCELLENT;
    } else if (biasMag < 50.0f && maxScaleError < 0.10f) {
        return CAL_QUALITY_GOOD;
    } else if (biasMag < 100.0f && maxScaleError < 0.15f) {
        return CAL_QUALITY_ACCEPTABLE;
    } else if (biasMag < 200.0f && maxScaleError < 0.25f) {
        return CAL_QUALITY_POOR;
    } else {
        return CAL_QUALITY_FAILED;
    }
}

/**
 * Get quality level as string
 */
const char* qualityToString(CalibrationQuality quality) {
    switch (quality) {
        case CAL_QUALITY_EXCELLENT:  return "EXCELLENT";
        case CAL_QUALITY_GOOD:       return "GOOD";
        case CAL_QUALITY_ACCEPTABLE: return "ACCEPTABLE";
        case CAL_QUALITY_POOR:       return "POOR";
        case CAL_QUALITY_FAILED:     return "FAILED";
        default:                     return "UNKNOWN";
    }
}

/**
 * Print calibration quality report
 */
void printQualityReport(const char* sensorName, CalibrationQuality quality) {
    Serial.printf("\n[QUALITY] %s Calibration: ", sensorName);
    
    switch (quality) {
        case CAL_QUALITY_EXCELLENT:
            Serial.println("✓ EXCELLENT - Professional-grade calibration");
            break;
        case CAL_QUALITY_GOOD:
            Serial.println("✓ GOOD - Suitable for most applications");
            break;
        case CAL_QUALITY_ACCEPTABLE:
            Serial.println("⚠ ACCEPTABLE - Usable but could be improved");
            break;
        case CAL_QUALITY_POOR:
            Serial.println("⚠ POOR - Recalibration recommended");
            break;
        case CAL_QUALITY_FAILED:
            Serial.println("✗ FAILED - Invalid data, must recalibrate");
            break;
    }
}
