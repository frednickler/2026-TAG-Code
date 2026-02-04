#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <Arduino.h>

// =============================================================================
// PROJECT IDENTIFICATION
// =============================================================================
#define PROJECT_NAME        "2026 TAG Code"
#define PROJECT_VERSION     "1.0.0"
#define HARDWARE_VERSION    "ESP32-S3 + ICM-20948 + BMM350 + NEO-M9N + OLED"

// =============================================================================
// HARDWARE PIN DEFINITIONS
// =============================================================================

// Status LED
#define STATUS_LED_PIN          2

// =============================================================================
// I2C CONFIGURATION - DUAL BUS SETUP
// =============================================================================

// I2C Bus 1: Sensors (IMU, Magnetometer, Display)
// CONFIRMED by I2C scanner: SDA=GPIO4, SCL=GPIO7
// Devices detected: 0x14 (BMM350), 0x3C (OLED), 0x68 (ICM-20948)
#define I2C_SDA_PIN             4   // GPIO 4 (SDA) - Sensor Bus
#define I2C_SCL_PIN             7   // GPIO 7 (SCL) - Sensor Bus

// I2C Bus 2: GPS Module  
// GPIO 8/9 are default ESP32-S3 I2C pins
#define GPS_I2C_SDA_PIN         8   // GPIO 8 (SDA) - GPS Only
#define GPS_I2C_SCL_PIN         9   // GPIO 9 (SCL) - GPS Only

// =============================================================================
// I2C ADDRESSES
// =============================================================================

// ICM-20948 IMU (Accelerometer + Gyroscope)
#define ICM20948_I2C_ADDR       0x68

// BMM350 Magnetometer
// NOTE: Default address 0x14 - VERIFY when hardware is integrated
// The actual address may be 0x14 or 0x15 depending on SDO pin configuration
#define BMM350_I2C_ADDR         0x14

// u-blox NEO-M9N GPS (M5Stack GNSS Module compatible)
#define GPS_I2C_ADDR            0x42

// OLED Display (0.96" 128x64)
#define OLED_I2C_ADDR           0x3C

// =============================================================================
// SERIAL CONFIGURATION
// =============================================================================
#define DEBUG_SERIAL            Serial
#define DEBUG_BAUD_RATE         115200

// =============================================================================
// TIMING CONSTANTS
// =============================================================================
#define DEFAULT_MAIN_LOOP_RATE_HZ 100     // Default main loop frequency (Hz)
#define IMU_UPDATE_RATE_HZ      100     // IMU sensor fusion rate (Hz)
#define GPS_POLL_RATE_MS        100     // GPS polling interval (ms)

// =============================================================================
// WATCHDOG CONFIGURATION
// =============================================================================
#define WATCHDOG_TIMEOUT_SEC    10      // Watchdog timeout (seconds)

// =============================================================================
// RADIO CONFIGURATION (ESP-NOW)
// =============================================================================
#define RADIO_PAIRING_TIMEOUT_MS        30000   // 30 seconds
#define RADIO_BEACON_INTERVAL_MS        1000    // 1 second
#define RADIO_CONNECTION_TIMEOUT_MS     5000    // 5 seconds (consider disconnected)

// =============================================================================
// CALIBRATION CONFIGURATION
// =============================================================================
#define CAL_GYRO_SAMPLES            500     // Gyro calibration: 500 samples @ 100Hz = 5 seconds
#define CAL_GYRO_MAX_NOISE          0.05f   // Max gyro noise (rad/s) during calibration
#define CAL_ACCEL_SAMPLES           500     // Accel calibration: 500 samples per position
#define CAL_ACCEL_GRAVITY_TARGET    9.8f    // Target gravity magnitude (m/s²)
#define CAL_ACCEL_GRAVITY_TOLERANCE 0.5f    // Gravity tolerance (m/s²)
#define CAL_MAG_MIN_SAMPLES         100     // Minimum magnetometer samples for ellipsoid fit
#define CAL_MAG_FIELD_MIN           25.0f   // Minimum expected Earth field (µT)
#define CAL_MAG_FIELD_MAX           65.0f   // Maximum expected Earth field (µT)

// =============================================================================
// NVS (NON-VOLATILE STORAGE) NAMESPACES
// =============================================================================
#define NVS_NAMESPACE_SETTINGS      "settings"
#define NVS_NAMESPACE_CALIBRATION   "calibration"

// =============================================================================
// DEBUG MACROS
// =============================================================================
#define DEBUG_INFO(fmt, ...)    DEBUG_SERIAL.printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#define DEBUG_WARN(fmt, ...)    DEBUG_SERIAL.printf("[WARN] " fmt "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(fmt, ...)   DEBUG_SERIAL.printf("[ERROR] " fmt "\n", ##__VA_ARGS__)

// =============================================================================
// COORDINATE FRAME CONVENTIONS
// =============================================================================
// VQF sensor fusion uses ENU (East-North-Up) coordinate frame:
//   X-axis → East
//   Y-axis → North
//   Z-axis → Up (against gravity)
//
// Heading calculation converts VQF yaw (ENU) to compass bearing:
//   Heading = 90° + yaw
//   0° = North, 90° = East, 180° = South, 270° = West

// =============================================================================
// CRITICAL HARDWARE NOTES
// =============================================================================
// ICM-20948:
//   - I2C Address: 0x68 (AD0 pin low)
//   - Accelerometer: ±2g / ±4g / ±8g / ±16g (configurable)
//   - Gyroscope: ±250dps / ±500dps / ±1000dps / ±2000dps (configurable)
//   - AK09916 auxiliary magnetometer: DISABLED (not used)
//   - Axis mapping: Refer to legacy TAG Code for physical orientation
//
// BMM350:
//   - I2C Address: 0x14 or 0x15 (SDO pin dependent) - DEFAULT 0x14
//   - Range: ±2000µT (fixed, cannot be changed)
//   - ODR: 1.5625 to 400Hz (configurable)
//   - Axis mapping: TBD - must be determined empirically via axis alignment tests
//
// GPS (NEO-M9N):
//   - I2C Address: 0x42 (same as BASE Code - M5Stack GNSS Module)
//   - Same hardware and configuration as BASE Code

#endif // SYSTEM_CONFIG_H
