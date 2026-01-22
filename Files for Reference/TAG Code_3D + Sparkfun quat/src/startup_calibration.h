#ifndef STARTUP_CALIBRATION_H
#define STARTUP_CALIBRATION_H

#include <Arduino.h>
#include "calibration_manager.h"

/**
 * Main startup calibration workflow
 * This function should be called early in setup() after IMU initialization
 * 
 * @return true if system is ready to proceed, false if critical error
 */
bool performStartupCalibrationWorkflow();

/**
 * Handle calibration failure scenarios
 * Provides fallback options and emergency calibration
 * 
 * @return true if emergency calibration succeeds, false for critical failure
 */
bool handleCalibrationFailure();

/**
 * Display startup calibration summary
 * Shows current status and provides menu for manual calibration
 */
void displayStartupSummary();

/**
 * Handle serial commands for calibration management
 * Call this from your main loop to handle user commands
 * 
 * @param command The command string received via serial
 * @return true if command was handled, false if not recognized
 */
bool handleCalibrationCommand(String command);

/**
 * Check if calibration is needed based on runtime conditions
 * Call this periodically during operation to detect drift
 * 
 * @return true if recalibration is recommended
 */
bool checkRuntimeCalibrationNeeded();

/**
 * Auto-calibration routine for unattended operation
 * Performs quick calibration when device is detected to be stationary
 * 
 * @return true if auto-calibration was performed
 */
bool performAutoCalibration();

#endif // STARTUP_CALIBRATION_H
