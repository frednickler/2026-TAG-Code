#ifndef CALIBRATION_MENU_H
#define CALIBRATION_MENU_H

#include <Arduino.h>

// Interactive calibration menu system
void showCalibrationMenu();
void handleMenuInput(char input);

// Menu options
void menuQuickCalibration();
void menuRobustCalibration();
void menuIndividualSensors();
void menuAxisIdentification();
void menuValidationStatus();
void menuAdvancedOptions();

#endif
