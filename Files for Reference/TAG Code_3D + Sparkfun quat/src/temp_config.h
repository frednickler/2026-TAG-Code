#ifndef TEMP_CONFIG_H
#define TEMP_CONFIG_H

/*-----------------------------------------------------------------------------
 * temp_config.h –  Configuration options for the internal temperature sensor
 *
 * The temperature sensor shares registers with the gyro, so we can’t set a
 * standalone ODR.  Instead we expose **post-processing** knobs such as moving
 * average length and median filtering to trade latency for noise reduction.
 *
 * WHAT YOU CAN TUNE
 * -----------------
 * 1. Averaging Window (AVG_SAMPLES) – number of consecutive raw readings to
 *    average.  Noise scales with 1/√N while latency scales linearly with N.
 *
 * 2. Median Filter – optional 3-point median filter which removes occasional
 *    outliers caused by digital interference.
 *
 * CLI RUNTIME SELECTION
 * ---------------------
 * Command "tcfg" followed by a number selects common presets:
 *     1 →  1-sample (raw)          – lowest latency, highest noise
 *     2 →  4-sample avg            – good for control loops
 *     3 →  8-sample avg + median   – default, ~0.04 °C RMS noise
 *     4 → 16-sample avg + median   – best accuracy, ~150 ms latency
 * The preset table is in `temp_config.cpp`; you can add more.
 *---------------------------------------------------------------------------*/
#include <Arduino.h>

namespace TempCfg {
// Number of samples to average when returning `readTemperature()` values.
// Larger N reduces noise (1 LSB ≈ 0.12 °C) at the cost of bandwidth.
constexpr uint8_t USER_AVG_SAMPLES = 8; // 8-sample moving average ≈ 20 Hz @160 Hz raw

// Optional median filter for outlier rejection (costs CPU).
constexpr bool USER_ENABLE_MEDIAN = true;

// API stub – implemented in temp_sensor.cpp
void setTempPostProcess(uint8_t avgSamples = USER_AVG_SAMPLES, bool median = USER_ENABLE_MEDIAN);
bool applyTempPreset(uint8_t presetId);

} // namespace TempCfg

#endif // TEMP_CONFIG_H
