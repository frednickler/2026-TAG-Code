#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>
#include <MadgwickAHRS.h>

// =================== Madgwick Filter Configuration ===================
// Select your filter preset by changing the number below (1-5):
//
// 1 = Ultra Smooth (slow, very stable)
// 2 = Smooth
// 3 = Balanced (default)
// 4 = Responsive
// 5 = Ultra Responsive (fast, noisy)
#define MADGWICK_FILTER_PRESET 3

// ---- Preset Table ----
//   Preset | Beta   | Sample Freq (Hz)
//   -------------------------------
//      1   | 0.04f  |   50
//      2   | 0.08f  |   100
//      3   | 0.12f  |   100
//      4   | 0.18f  |   200
//      5   | 0.25f  |   500
// -------------------------------

#if   MADGWICK_FILTER_PRESET == 1
  #define MADGWICK_BETA_DEFAULT 0.04f
  #define MADGWICK_SAMPLE_FREQ 50.0f
#elif MADGWICK_FILTER_PRESET == 2
  #define MADGWICK_BETA_DEFAULT 0.08f
  #define MADGWICK_SAMPLE_FREQ 100.0f
#elif MADGWICK_FILTER_PRESET == 3
  #define MADGWICK_BETA_DEFAULT 0.12f
  #define MADGWICK_SAMPLE_FREQ 100.0f
#elif MADGWICK_FILTER_PRESET == 4
  #define MADGWICK_BETA_DEFAULT 0.18f
  #define MADGWICK_SAMPLE_FREQ 200.0f
#elif MADGWICK_FILTER_PRESET == 5
  #define MADGWICK_BETA_DEFAULT 0.25f
  #define MADGWICK_SAMPLE_FREQ 500.0f
#else
  #error "Invalid MADGWICK_FILTER_PRESET: must be 1-5"
#endif
// ============================

// Other Madgwick configuration options:
// - Beta: filter gain (responsiveness vs noise)
// - Sample frequency: update rate (Hz)
// The library does not expose other runtime parameters. Quaternion state is internal.
// For advanced users: you may adjust algorithm update rate and beta at runtime via the filter API.

// Uncomment to enable debug prints for fusion output
#define FUSION_DEBUG_PRINT

// =================== Sample Rate Note ===============================
// The filter is initialized for 400Hz (IMU update rate). Adjust if your IMU runs slower.
// Change in sensor_fusion.cpp if needed.
// ====================================================================

// =================== Function Prototypes ============================
void initSensorFusion(float beta = MADGWICK_BETA_DEFAULT);
void updateSensorFusion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void getQuaternion(float &q0, float &q1, float &q2, float &q3);
float getHeading();
void getEulerAngles(float &yaw, float &pitch, float &roll);

#endif // SENSOR_FUSION_H
