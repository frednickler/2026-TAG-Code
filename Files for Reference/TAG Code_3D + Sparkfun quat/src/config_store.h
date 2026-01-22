#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <Arduino.h>

struct SystemConfig {
    uint16_t magic;         // 0xABCD to verify valid config
    uint8_t accelPreset;    // 1-5
    uint8_t gyroPreset;     // 1-5
    uint8_t magPreset;      // 0,1, 2-5
    uint8_t gpsRate;        // 1=1Hz, 5=5Hz, 10=10Hz, 25=25Hz
    bool radioDebug;        // true/false
    
    // Detailed GPS Config
    uint8_t gnssConstellation; // 0=GPS+GLO+GAL+BDS, 1=GPS+GLO+GAL, 2=GPS+GLO, 3=GPS Only
    uint8_t dynamicModel;      // 0=Portable, 1=Pedestrian, 2=Automotive, 3=Airborne, 4=Sea
    uint8_t i2cClockSpeed;     // 0=Standard(100k), 1=Fast(400k) - Mapped from Baud Rate menu
    bool sbasEnabled;
    bool qzssEnabled;
    uint8_t antiJamming;       // 0=No, 1=Fixed, 2=Adaptive
    uint8_t powerMode;         // 0=Full Power (Hardcoded in menu, but stored here)
    uint8_t unitSystem;        // 0=Metric, 1=Imperial
    uint8_t targetMac[6];      // Destination MAC Address
    bool zuptEnabled;          // Enable/Disable Zero Velocity Update
};

// Global instance
extern SystemConfig g_sysConfig;

// API
void initConfigStore();
void saveConfigStore();
void printConfigStore();

#endif // CONFIG_STORE_H
