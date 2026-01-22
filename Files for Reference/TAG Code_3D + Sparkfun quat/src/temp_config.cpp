/*-----------------------------------------------------------------------------
 * temp_config.cpp – Post-processing configuration for the internal temperature
 * sensor.  Unlike accel/gyro/mag, there are no dedicated temp registers; we
 * therefore expose software presets that control the moving-average window and
 * optional median filter used in `readTemperature()`.
 *---------------------------------------------------------------------------*/
#include "temp_config.h"
#include <Arduino.h>

using namespace TempCfg;

// Active settings (initialised to compile-time defaults)
static uint8_t g_avgSamples  = USER_AVG_SAMPLES;
static bool    g_useMedian   = USER_ENABLE_MEDIAN;

// Simple circular buffer for averaging
static float sampleBuf[32]; // supports up to 32-sample window
static uint8_t bufIdx = 0;
static uint8_t bufCount = 0;

void TempCfg::setTempPostProcess(uint8_t avgSamples, bool median) {
    if (avgSamples == 0 || avgSamples > 32) avgSamples = 1;
    g_avgSamples = avgSamples;
    g_useMedian  = median;
    bufIdx = bufCount = 0; // reset history
}

bool TempCfg::applyTempPreset(uint8_t id) {
    switch (id) {
        case 1: TempCfg::setTempPostProcess(1, false);  return true;
        case 2: TempCfg::setTempPostProcess(4, false);  return true;
        case 3: TempCfg::setTempPostProcess(8, true);   return true;
        case 4: TempCfg::setTempPostProcess(16, true);  return true;
        default: return false;
    }
}

// Helper called from temp_sensor.cpp to push a new raw reading and get the
// processed value.
float _processTempSample(float newSample) {
    sampleBuf[bufIdx] = newSample;
    bufIdx = (bufIdx + 1) % g_avgSamples;
    if (bufCount < g_avgSamples) bufCount++;

    // Compute mean (simple for small N)
    float sum = 0;
    for (uint8_t i = 0; i < bufCount; ++i) sum += sampleBuf[i];
    float mean = sum / bufCount;

    if (!g_useMedian || bufCount < 3) return mean;

    // Median of last 3 samples around current index (rough, avoids extra buffer)
    int8_t i1 = (bufIdx + g_avgSamples - 1) % g_avgSamples;
    int8_t i2 = (bufIdx + g_avgSamples - 2) % g_avgSamples;
    int8_t i3 = (bufIdx + g_avgSamples - 3) % g_avgSamples;
    float a = sampleBuf[i1], b = sampleBuf[i2], c = sampleBuf[i3];
    float median;
    if ((a<=b && b<=c)||(c<=b && b<=a)) median=b; else if ((b<=a&&a<=c)||(c<=a&&a<=b)) median=a; else median=c;
    return (mean + median) * 0.5f; // simple mean of mean+median for stability
}
