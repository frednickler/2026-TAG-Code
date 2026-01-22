#ifndef SYSTEM_SETTINGS_H
#define SYSTEM_SETTINGS_H

#include <Arduino.h>

// Output display modes
enum class OutputMode : uint8_t {
    DEBUG = 0,    // All sensor data + tracking calculations
    COMPACT = 1,  // GPS + Heading + Radio status only
    SILENT = 2    // No periodic output (errors/warnings only)
};

// Radio packet mode (USER DECISION: Option A with 2 modes)
enum class RadioPacketMode : uint8_t {
    GPS_HEADING = 0,      // GPS + Heading only (23 bytes)
    GPS_HEADING_IMU = 1   // GPS + Heading + IMU (47 bytes)
};

struct RuntimeConfig {
    uint32_t magic;         // 0xDEADBEEF to verify valid config
    
    // ==================== ICM-20948 CONFIGURATION ====================
    // Gyroscope Configuration
    uint8_t gyroRange;      // 0=250dps, 1=500dps, 2=1000dps, 3=2000dps (Default: 1=500dps)
    uint8_t gyroDLPF;       // 0=361Hz, 1=197Hz, 2=152Hz, 3=120Hz, 4=51Hz, 5=24Hz, 6=12Hz, 7=6Hz (Default: 5)
    uint8_t gyroPreset;     // 1-4: Preset modes (1=250dps, 2=500dps, 3=1000dps, 4=2000dps)
    
    // Accelerometer Configuration
    uint8_t accelRange;     // 0=2g, 1=4g, 2=8g, 3=16g (Default: 1=4g)
    uint8_t accelDLPF;      // 0=473Hz, 1=246Hz, 2=111Hz, 3=50Hz, 4=24Hz, 5=12Hz, 6=6Hz, 7=? (Default: 4)
    uint8_t accelPreset;    // 1-4: Preset modes (1=2g, 2=4g, 3=8g, 4=16g)
    
    // ==================== BMM350 CONFIGURATION ====================
    // Magnetometer Output Data Rate (exact Hz values from datasheet)
    float magODR;           // Hz: 1.5625, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400 (Default: 12.5)
    
    // Magnetometer Averaging (Power Mode)
    uint8_t magAveraging;   // 0=No avg (1 sample), 1=2 samples, 2=4 samples, 3=8 samples (Default: 1)
    
    // Magnetometer Preset Mode (combines averaging + recommended ODR)
    uint8_t magPreset;      // 1=Low Power (avg=0), 2=Regular (avg=1), 3=Low Noise (avg=2), 4=Ultra Low Noise (avg=3) (Default: 2)
    
    // ==================== GPS CONFIGURATION ====================
    uint8_t gpsRate;        // Hz (Default: 10)
    uint8_t gnssConstellation; // 0=All, 1=GPS+GLO+GAL, 2=GPS+GLO, 3=GPS (Default: 0)
    uint8_t dynamicModel;      // 0=Portable, 2=Auto, 3=Air, 5=Stationary (Default: 0)
    
    // Detailed GPS Config
    bool sbasEnabled;
    bool qzssEnabled;
    uint8_t antiJamming;       // 0=No, 1=Fixed, 2=Adaptive (Default: 0)
    
    // ==================== SYSTEM CONFIGURATION ====================
    uint8_t i2cClockSpeed;  // 0=100kHz, 1=400kHz (Default: 1)
    
    // Main Loop Rate
    uint16_t loopRate;      // Hz: 25, 50, 100, 200 (Default: 100)
    
    // Alignment Settings
    uint8_t gpsAlignmentMode; // 0=Last (Default), 1=Average (60s)
    float savedHeadingOffset; // User tare value (degrees)
    
    // VQF Tuning Params  (Same as BASE Code)
    float vqfTauAcc;         // Accel time constant (default 3.0s)
    float vqfTauMag;         // Mag time constant (default 2.0s)
    bool vqfMagRejection;    // Enable magnetic disturbance rejection
    
    // Mounting
    bool mountUpsideDown;    // Invert Y and Z axes
    
    // Output Mode
    uint8_t outputMode;      // 0=Debug, 1=Compact, 2=Silent
    
    // ==================== RADIO (ESP-NOW) CONFIGURATION ====================
    uint8_t radioRole;           // 0=BASE (receiver), 1=TAG (transmitter)
    uint8_t radioChannel;        // 1-14 (Default: 1)
    uint8_t radioTxPower;        // 0=LOW(11dBm), 1=MED(15dBm), 2=HIGH(20dBm)
    uint8_t radioDataRate;       // 0=1Mbps, 1=2Mbps, 2=5.5Mbps, 3=11Mbps
    bool    radioAckEnabled;     // Send/receive ACK packets (Default: true)
    uint8_t radioTargetMAC[6];   // Target device MAC (Default: FF:FF:FF:FF:FF:FF = broadcast)
    bool    radioEnabled;        // Radio on/off (Default: true)
    uint8_t radioTagTxRate;      // TAG transmit rate Hz: 1, 2, 5, 10 (Default: 1)
    uint8_t radioPacketMode;     // 0=GPS+Heading, 1=GPS+Heading+IMU (Default: 0)
};

class SystemSettings {
public:
    static void init();
    static void save();
    static void loadDefaults();
    
    static RuntimeConfig& getConfig() {
        return config;
    }
    
    // Output mode helpers
    static OutputMode getOutputMode() {
        return static_cast<OutputMode>(config.outputMode);
    }
    
    static void setOutputMode(OutputMode mode) {
        config.outputMode = static_cast<uint8_t>(mode);
        save();
    }
    
    // Radio packet mode helpers
    static RadioPacketMode getRadioPacketMode() {
        return static_cast<RadioPacketMode>(config.radioPacketMode);
    }
    
    static void setRadioPacketMode(RadioPacketMode mode) {
        config.radioPacketMode = static_cast<uint8_t>(mode);
        save();
    }
    
    // Loop Rate helpers
    static uint16_t getLoopRate() {
        return config.loopRate > 0 ? config.loopRate : 100;
    }
    
    static void setLoopRate(uint16_t rate) {
        if (rate == 25 || rate == 50 || rate == 100 || rate == 200) {
            config.loopRate = rate;
            save();
        }
    }

private:
    static RuntimeConfig config;
};

#endif // SYSTEM_SETTINGS_H
