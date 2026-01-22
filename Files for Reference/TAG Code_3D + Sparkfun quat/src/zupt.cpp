#include "zupt.h"
#include "config_store.h"

// Thresholds for detecting stationary state
// ADJUSTED: More permissive thresholds to ensure ZUPT engages on desk
#define ZUPT_ACCEL_THRESHOLD 0.15f  // g (increased from 0.05) - deviation from 1g
#define ZUPT_GYRO_THRESHOLD  1.5f   // dps (increased from 0.5) - vibration tolerance (absolute value)
#define ZUPT_STABLE_TIME_MS   200    // Time required to be stable before engaging ZUPT

static unsigned long stationaryStartTime = 0;
static bool wasStationary = false;

void initZUPT() {
    stationaryStartTime = 0;
    wasStationary = false;
    Serial.println("[ZUPT] Initialized");
}

bool isdeviceStationary(float ax, float ay, float az, float gx, float gy, float gz) {
    // Calculate accelerometer magnitude (assuming input is in g)
    // If input is in mg, divide by 1000.0f
    // If input is in m/s^2, divide by 9.81f
    
    // Note: main.cpp passes calibrated accel in mg (e.g. 1000 = 1g)
    // and gyro in dps.
    
    float accelMag = sqrt(ax*ax + ay*ay + az*az);
    float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
    
    // Check if accelerometer is measuring gravity (~1000 mg)
    float accelDiff = fabs(accelMag - 1000.0f);
    bool accelStatic = (accelDiff < (ZUPT_ACCEL_THRESHOLD * 1000.0f));
    
    // Check if gyro is near zero
    bool gyroStatic = (gyroMag < ZUPT_GYRO_THRESHOLD);
    
    // DEBUG: Print status every 500ms to verify logic
    // static unsigned long lastDebug = 0;
    // if (millis() - lastDebug > 500) {
    //     lastDebug = millis();
    //     Serial.printf("[ZUPT CHK] AMag: %.1f (Diff: %.1f/%.1f) GMag: %.2f (Lim: %.1f) -> %s\n", 
    //                  accelMag, accelDiff, ZUPT_ACCEL_THRESHOLD * 1000.0f, 
    //                  gyroMag, ZUPT_GYRO_THRESHOLD,
    //                  (accelStatic && gyroStatic) ? "STATIC" : "MOVING");
    // }
    
    return accelStatic && gyroStatic;
}

void applyZUPT(float& gx, float& gy, float& gz, float ax, float ay, float az) {
    // Only apply if enabled in config
    if (!g_sysConfig.zuptEnabled) {
        return;
    }

    if (isdeviceStationary(ax, ay, az, gx, gy, gz)) {
        if (!wasStationary) {
            stationaryStartTime = millis();
            wasStationary = true;
            // Serial.println("[ZUPT] Entered Stationary State");
        }
        
        // Only apply ZUPT after being stationary for a short time
        unsigned long stableTime = millis() - stationaryStartTime;
        
        if (stableTime > ZUPT_STABLE_TIME_MS) {
            // Force gyro to zero
            gx = 0.0f;
            gy = 0.0f;
            gz = 0.0f;
            
            // Debug engaged (once per second)
            // static unsigned long lastEngagedPrint = 0;
            // if (millis() - lastEngagedPrint > 1000) {
            //     lastEngagedPrint = millis();
            //     Serial.println("[ZUPT] ENGAGED (Zeroing Gyro)");
            // }
        }
    } else {
        if (wasStationary && (millis() - stationaryStartTime > ZUPT_STABLE_TIME_MS)) {
             // Serial.println("[ZUPT] MOVEMENT DETECTED - Disengaging");
        }
        wasStationary = false;
        stationaryStartTime = 0;
    }
}
