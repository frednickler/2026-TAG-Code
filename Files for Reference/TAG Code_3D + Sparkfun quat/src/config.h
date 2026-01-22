#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Optional EEPROM self-test on startup (compile-time toggle)
// Set to 1 to verify that the existing calibration blob can be read,
// written back, and re-read without corruption. The self-test does NOT
// overwrite calibration values with dummy data.
#ifndef ENABLE_EEPROM_SELFTEST
#define ENABLE_EEPROM_SELFTEST 0  // change to 1 for debug
#endif

// Optional forced EEPROM clear on every boot (for development only)
#ifndef CLEAR_CALIBRATION_EEPROM_ON_BOOT
#define CLEAR_CALIBRATION_EEPROM_ON_BOOT 0  // set to 1 ONLY when you need to wipe calibration data
#endif

// -----------------------------------------------------------------------------
// USER CONFIGURATION: ACCELEROMETER RANGE
// -----------------------------------------------------------------------------
// Set to 2, 4, 8, or 16 for ±2g, ±4g, ±8g, or ±16g range
// NOTE: Hardware appears to be native ±4g (8192 LSB/g). 
//       Setting to 2 will result in 0.5x scaling error on this specific unit.
#define ACCEL_FSR_G  4

// -----------------------------------------------------------------------------
// USER CONFIGURATION: GPS & I2C
// -----------------------------------------------------------------------------
#define GPS_SDA_PIN 8
#define GPS_SCL_PIN 9
#define GPS_I2C_ADDR 0x42
#define IMU_SDA_PIN 4
#define IMU_SCL_PIN 5


// AUTOMATIC DERIVATION (DO NOT EDIT BELOW)
#if ACCEL_FSR_G == 2
    #define ACCEL_FS_SEL_BITS_REG 0x00  // [2:1] = 00
    #define ACCEL_LSB_PER_G       16384.0f
#elif ACCEL_FSR_G == 4
    #define ACCEL_FS_SEL_BITS_REG 0x02  // [2:1] = 01 (shifted left by 1 comes to 2)
    #define ACCEL_LSB_PER_G       8192.0f
#elif ACCEL_FSR_G == 8
    #define ACCEL_FS_SEL_BITS_REG 0x04  // [2:1] = 10
    #define ACCEL_LSB_PER_G       4096.0f
#elif ACCEL_FSR_G == 16
    #define ACCEL_FS_SEL_BITS_REG 0x06  // [2:1] = 11
    #define ACCEL_LSB_PER_G       2048.0f
#else
    #error "Invalid ACCEL_FSR_G value! Must be 2, 4, 8, or 16."
#endif

namespace Config {
    // Automatically scaled factor based on selected FSR
    // EMPIRICAL CORRECTION: Hardware reads ~0.975g with theoretical scale.
    // Multiplying by (1.0/0.975) = 1.02564 to normalize to 1.0g.
    constexpr float ACCEL_SCALE = (1000.0f / ACCEL_LSB_PER_G) * 1.02564f; 
    
    // Legacy fixed constants (will be updated)
    constexpr float GYRO_SCALE = (1.0f / 32.8f) * DEG_TO_RAD; // For ±1000dps
    constexpr float MAG_SCALE = 0.15f; // For AK09916
    constexpr uint32_t STREAM_INTERVAL_MS = 10; // 10Hz
    constexpr uint32_t DEBUG_INTERVAL_MS = 6000;
    constexpr size_t JSON_BUFFER_SIZE = 2048;
    constexpr float SENSOR_CHANGE_THRESH = 0.001f; // deg
    constexpr uint32_t WEBSOCKET_PING_INTERVAL = 30000; // ms
    constexpr uint32_t CALIB_CHECK_INTERVAL = 1000; // ms
    
    // Accelerometer calibration method constants
    constexpr uint8_t ACCEL_CAL_SIMPLE = 0;       // Simple bias-only calibration
    constexpr uint8_t ACCEL_CAL_SIX_POSITION = 1; // 6-position calibration with scale factors
    constexpr uint8_t ACCEL_CAL_CROSS_AXIS = 2;   // 6-position with cross-axis sensitivity
    constexpr uint8_t ACCEL_CAL_ELLIPSOID = 3;    // Advanced ellipsoid fitting

    constexpr bool EEPROM_SELFTEST = ENABLE_EEPROM_SELFTEST;
}

// --- Global Flags ---
extern bool g_debugMode;

#endif // CONFIG_H
