#ifndef CALIBRATION_ENGINE_H
#define CALIBRATION_ENGINE_H

#include <Arduino.h>
#include "SensorDriverV2.h"

struct CalStorage {
    uint32_t magic;
    uint32_t version;
    uint32_t calibration_count; // Increments with each calibration
    float accel_bias[3];
    float gyro_bias[3];
    float mag_offset[3];
    float mag_scale[3];
    bool valid;
};

class CalibrationEngine {
public:
    static void begin();
    static void loop();

private:
    static void showMenu();
    static void handleInput();
    static void runQuickCal();
    static void runRobustCal();
    static void runBestCal();
    static void runValidation();
    static void runExtendedValidation();
    static void printStatus();
    static void saveCalibration();
    static void loadCalibration();
    static void performGyroCal();
    static void performMagCal();
    static void identifyAxes();
    static void waitForStableOrientation(int axis, float target, float threshold);
    
    static CalStorage calData;
};

#endif
