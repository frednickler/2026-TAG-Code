#ifndef ACCEL_CONFIG_H
#define ACCEL_CONFIG_H

/*-----------------------------------------------------------------------------
 * accel_config.h  –  User-tunable configuration for the ICM-20948 accelerometer
 *
 * COMPREHENSIVE ACCELEROMETER CONFIGURATION REFERENCE
 * Based on ICM-20948 Datasheet Rev.1.3 (DS-000189)
 * https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
 *
 * The goal of this file is to isolate **all** accelerometer-specific register
 * settings (full-scale range, digital-low-pass-filter bandwidth, ODR, etc.) in
 * one place so firmware users can adjust behaviour without digging through the
 * driver code.  Each parameter is documented with datasheet references and
 * recommended values for common use-cases.
 *
 * HOW TO USE
 * ----------
 * 1.  Review the comprehensive reference tables below to understand all available
 *     configurations for FSR, DLPF, and ODR settings.
 * 2.  Edit the "User-editable section" below using the symbolic constants.
 *     Each parameter maps 1-to-1 to register bit-fields.
 * 3.  Re-build and flash the firmware.  During `setup()` the function
 *     `applyAccelConfig()` will write the selected settings to the IMU.
 * 4.  At runtime you can use CLI presets (`acfg 1-5`) to test different
 *     configurations without recompiling.
 * 5.  Call `printAccelConfig()` from CLI/menu to confirm current register contents.
 *
 * If you do NOT need to tweak the accelerometer you can leave this file as-is.
 *---------------------------------------------------------------------------*/

#include <Arduino.h>

namespace AccelCfg {

/*=============================================================================
 * ICM-20948 ACCELEROMETER CONFIGURATION REFERENCE TABLES
 * Source: ICM-20948 Datasheet Rev.1.3, Table 36, 37, 38
 *===========================================================================*/

/*-----------------------------------------------------------------------------
 * TABLE 1: FULL-SCALE RANGE (FSR) CONFIGURATIONS
 * Register: BANK 2, ACCEL_CONFIG[4:3] = ACCEL_FS_SEL[1:0]
 * Datasheet Reference: Table 36 (Page 41)
 *---------------------------------------------------------------------------*/
//
// ┌─────────┬──────────────┬─────────────────┬──────────────────────────────┐
// │ FS_SEL  │ Full Scale   │ Sensitivity     │ Recommended Applications     │
// │ [1:0]   │ Range        │ (LSB/g)         │                              │
// ├─────────┼──────────────┼─────────────────┼──────────────────────────────┤
// │   00    │    ±2 g      │    16,384       │ Human motion, orientation,   │
// │         │              │                 │ gentle robotics, wearables   │
// ├─────────┼──────────────┼─────────────────┼──────────────────────────────┤
// │   01    │    ±4 g      │     8,192       │ General purpose, drones,     │
// │         │              │                 │ small vehicles, default      │
// ├─────────┼──────────────┼─────────────────┼──────────────────────────────┤
// │   10    │    ±8 g      │     4,096       │ Sports equipment, fast       │
// │         │              │                 │ aircraft, moderate shocks    │
// ├─────────┼──────────────┼─────────────────┼──────────────────────────────┤
// │   11    │   ±16 g      │     2,048       │ Extreme shocks, aerobatics,  │
// │         │              │                 │ crash detection, vibration   │
// └─────────┴──────────────┴─────────────────┴──────────────────────────────┘
//
constexpr uint8_t FS_2G  = 0b00; // ±2 g  – Highest resolution (16,384 LSB/g)
constexpr uint8_t FS_4G  = 0b01; // ±4 g  – Good balance (8,192 LSB/g) [DEFAULT]
constexpr uint8_t FS_8G  = 0b10; // ±8 g  – Higher dynamic range (4,096 LSB/g)
constexpr uint8_t FS_16G = 0b11; // ±16 g – Maximum range (2,048 LSB/g)

/*-----------------------------------------------------------------------------
 * TABLE 2: DIGITAL LOW-PASS FILTER (DLPF) CONFIGURATIONS
 * Register: BANK 2, ACCEL_CONFIG[2:0] = ACCEL_DLPFCFG[2:0]
 * Datasheet Reference: Table 37 (Page 41)
 *---------------------------------------------------------------------------*/
//
// ┌─────────┬─────────────┬─────────────┬─────────────┬──────────────────────┐
// │ DLPFCFG │ 3dB BW      │ Noise BW    │ Delay       │ Recommended Use      │
// │ [2:0]   │ (Hz)        │ (Hz)        │ (ms)        │                      │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   000   │ 1209.0      │ 1130.0      │ 0.17        │ Bypass mode, raw     │
// │         │ (Bypass)    │ (Bypass)    │             │ data, high bandwidth │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   001   │  246.0      │  265.0      │ 0.97        │ Fast response,       │
// │         │             │             │             │ minimal filtering    │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   010   │  111.4      │  136.0      │ 1.69        │ Moderate filtering   │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   011   │   50.4      │   68.8      │ 3.9         │ General purpose,     │
// │         │             │             │             │ good balance [DFLT]  │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   100   │   23.9      │   36.5      │ 7.8         │ Smooth motion,       │
// │         │             │             │             │ noise reduction      │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   101   │   11.5      │   18.6      │ 15.5        │ High noise rejection │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   110   │    5.7      │    9.9      │ 27.9        │ Maximum smoothing,   │
// │         │             │             │             │ vibration isolation  │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   111   │  473.0      │  499.0      │ 0.66        │ High bandwidth with  │
// │         │             │             │             │ 4 kHz ODR only       │
// └─────────┴─────────────┴─────────────┴─────────────┴──────────────────────┘
//
constexpr uint8_t DLPF_DISABLED = 0;   // Bypass – 1209 Hz BW, 0.17 ms delay
constexpr uint8_t DLPF_246HZ    = 1;   // 246 Hz BW, 0.97 ms delay
constexpr uint8_t DLPF_111HZ    = 2;   // 111 Hz BW, 1.69 ms delay
constexpr uint8_t DLPF_50HZ     = 3;   // 50 Hz BW, 3.9 ms delay [DEFAULT]
constexpr uint8_t DLPF_24HZ     = 4;   // 24 Hz BW, 7.8 ms delay
constexpr uint8_t DLPF_12HZ     = 5;   // 12 Hz BW, 15.5 ms delay
constexpr uint8_t DLPF_6HZ      = 6;   // 6 Hz BW, 27.9 ms delay
constexpr uint8_t DLPF_473HZ    = 7;   // 473 Hz BW, 0.66 ms delay (4 kHz ODR)

/*-----------------------------------------------------------------------------
 * TABLE 3: OUTPUT DATA RATE (ODR) CONFIGURATIONS
 * Register: BANK 2, ACCEL_SMPLRT_DIV_1[3:0] & ACCEL_SMPLRT_DIV_2[7:0]
 * Datasheet Reference: Table 38 (Page 42)
 *---------------------------------------------------------------------------*/
//
// ODR CALCULATION:
// • When DLPF is ENABLED (DLPFCFG = 1-7):  Base ODR = 1.125 kHz
// • When DLPF is DISABLED (DLPFCFG = 0):   Base ODR = 4.5 kHz
// • Actual ODR = Base ODR / (1 + ACCEL_SMPLRT_DIV)
// • ACCEL_SMPLRT_DIV is a 12-bit value (0-4095)
//
// ┌─────────────┬─────────────────┬─────────────────┬──────────────────────┐
// │ Desired ODR │ DLPF Enabled    │ DLPF Disabled   │ Recommended Use      │
// │ (Hz)        │ Divider Value   │ Divider Value   │                      │
// ├─────────────┼─────────────────┼─────────────────┼──────────────────────┤
// │   1125      │       0         │      N/A        │ Maximum rate, DLPF   │
// │   1000      │       0.125     │      N/A        │ High-speed sampling  │
// │    562      │       1         │      N/A        │ Fast control loops   │
// │    450      │       1.5       │       9         │ Moderate sampling    │
// │    375      │       2         │      11         │ Standard rate        │
// │    225      │       4         │      19         │ General purpose      │
// │    112      │       9         │      39         │ Lower bandwidth      │
// │     56      │      19         │      79         │ Slow applications    │
// │     28      │      39         │     159         │ Very slow sampling   │
// │     14      │      79         │     320         │ Minimal power        │
// │      7      │     159         │     642         │ Ultra-low power      │
// │      1      │    1124         │    4499         │ Logging applications │
// └─────────────┴─────────────────┴─────────────────┴──────────────────────┘
//
// COMMON ODR DIVIDER VALUES (for DLPF enabled, 1.125 kHz base):
constexpr uint16_t ODR_1125HZ_DIV = 0;    // 1125 Hz (maximum rate)
constexpr uint16_t ODR_562HZ_DIV  = 1;    // 562.5 Hz
constexpr uint16_t ODR_375HZ_DIV  = 2;    // 375 Hz
constexpr uint16_t ODR_225HZ_DIV  = 4;    // 225 Hz [DEFAULT]
constexpr uint16_t ODR_112HZ_DIV  = 9;    // 112.5 Hz
constexpr uint16_t ODR_56HZ_DIV   = 19;   // 56.25 Hz
constexpr uint16_t ODR_28HZ_DIV   = 39;   // 28.125 Hz
constexpr uint16_t ODR_14HZ_DIV   = 79;   // 14.06 Hz
constexpr uint16_t ODR_7HZ_DIV    = 159;  // 7.03 Hz
constexpr uint16_t ODR_1HZ_DIV    = 1124; // 1.0 Hz

/*=============================================================================
 * CONFIGURATION PRESET REFERENCE
 * These presets are available via CLI command: acfg <number>
 * Defined in accel_config.cpp PRESETS[] array
 *===========================================================================*/
//
// ┌────────┬─────────┬─────────┬─────────┬─────────────┬─────────────────────┐
// │ Preset │   FSR   │  DLPF   │   ODR   │ Sensitivity │ Application         │
// │ Number │         │         │  (Hz)   │  (LSB/g)    │                     │
// ├────────┼─────────┼─────────┼─────────┼─────────────┼─────────────────────┤
// │   1    │  ±2 g   │  50 Hz  │  225    │   16,384    │ High-res, low-g     │
// │   2    │  ±4 g   │  50 Hz  │  225    │    8,192    │ General purpose     │
// │   3    │  ±8 g   │  50 Hz  │  225    │    4,096    │ Moderate shocks     │
// │   4    │ ±16 g   │  50 Hz  │  225    │    2,048    │ Maximum headroom    │
// │   5    │  ±4 g   │  24 Hz  │  112    │    8,192    │ Extra noise reject  │
// └────────┴─────────┴─────────┴─────────┴─────────────┴─────────────────────┘

/*=============================================================================
 * USER-EDITABLE CONFIGURATION SECTION
 * Edit these values to customize accelerometer behavior
 *===========================================================================*/

// ---------------------------------------------------------------------------
// 1) FULL-SCALE RANGE SELECTION
// Choose from: FS_2G, FS_4G, FS_8G, FS_16G
// ---------------------------------------------------------------------------
constexpr uint8_t USER_FS_SEL = FS_4G;   // ±4 g (8,192 LSB/g) – DEFAULT
//
// SELECTION GUIDE:
// • FS_2G  (±2 g):  Best for human motion, gentle robotics, orientation sensing
// • FS_4G  (±4 g):  Good all-around choice for drones, small vehicles
// • FS_8G  (±8 g):  Sports equipment, fast aircraft, moderate impact detection
// • FS_16G (±16 g): Extreme shocks, aerobatics, crash detection, high vibration
//
// TIP: Test different ranges at runtime using CLI presets:
//      Serial command: acfg <1-5>
//      Each preset combines FSR + DLPF + ODR for specific use cases.

// ---------------------------------------------------------------------------
// 2) DIGITAL LOW-PASS FILTER SELECTION
// Choose from: DLPF_DISABLED, DLPF_246HZ, DLPF_111HZ, DLPF_50HZ, 
//              DLPF_24HZ, DLPF_12HZ, DLPF_6HZ, DLPF_473HZ
// ---------------------------------------------------------------------------
constexpr uint8_t USER_DLPF_CFG = DLPF_50HZ; // 50 Hz BW, 3.9 ms delay – DEFAULT
//
// SELECTION GUIDE:
// • DLPF_DISABLED: Raw data, maximum bandwidth, minimal delay (vibration-prone)
// • DLPF_246HZ:    Fast response, minimal filtering (high-speed applications)
// • DLPF_111HZ:    Moderate filtering, good for dynamic motion
// • DLPF_50HZ:     General purpose, good balance of smoothness and response
// • DLPF_24HZ:     Smooth motion, reduced noise (slower response)
// • DLPF_12HZ:     High noise rejection, stable readings (attitude estimation)
// • DLPF_6HZ:      Maximum smoothing, vibration isolation (slow motion only)
// • DLPF_473HZ:    High bandwidth with 4 kHz ODR (special applications)
//
// TRADE-OFFS:
// Lower bandwidth → Less noise, more delay, smoother output
// Higher bandwidth → More noise, less delay, faster response

// ---------------------------------------------------------------------------
// 3) OUTPUT DATA RATE (ODR) SELECTION
// Choose from predefined ODR_xxxHZ_DIV constants or custom divider value
// ---------------------------------------------------------------------------
constexpr uint16_t USER_SMPLRT_DIV = ODR_225HZ_DIV; // 225 Hz ODR – DEFAULT
//
// SELECTION GUIDE:
// • ODR_1125HZ_DIV (0):    Maximum rate, high CPU load, fast control loops
// • ODR_562HZ_DIV (1):     High-speed sampling, moderate CPU load
// • ODR_375HZ_DIV (2):     Standard high-rate applications
// • ODR_225HZ_DIV (4):     General purpose, good balance [DEFAULT]
// • ODR_112HZ_DIV (9):     Lower bandwidth, reduced power consumption
// • ODR_56HZ_DIV (19):     Slow applications, minimal processing
// • ODR_28HZ_DIV (39):     Very slow sampling, battery-powered devices
// • ODR_14HZ_DIV (79):     Minimal power consumption
// • ODR_7HZ_DIV (159):     Ultra-low power, logging applications
// • ODR_1HZ_DIV (1124):    Data logging, long-term monitoring
//
// CUSTOM ODR CALCULATION:
// For custom ODR: divider = (1125 / desired_ODR) - 1
// Example: For 300 Hz → divider = (1125 / 300) - 1 = 2.75 ≈ 3
//
// POWER CONSUMPTION NOTE:
// Lower ODR significantly reduces power consumption and processing load.
// Choose the lowest rate that meets your application requirements.

/*=============================================================================
 * API FUNCTIONS (do not edit below unless you know what you're doing)
 *===========================================================================*/

// Apply the above user settings by writing to the IMU registers.
// Returns true on success (I²C ACK received) or false on communication error.
bool applyAccelConfig();

// Apply one of the predefined CLI presets (1-5 as shown in table above).
// Returns true on success, false on invalid preset number or I²C error.
bool applyAccelPreset(uint8_t presetId);

// Helper function that prints the current effective register values to the
// specified output stream for debugging and verification purposes.
void printAccelConfig(Stream &out = Serial);

/*=============================================================================
 * USER-FRIENDLY CONFIGURATION FUNCTIONS
 * These provide an intuitive interface using actual Hz and g values
 *===========================================================================*/

// Apply accelerometer configuration using user-friendly parameters.
// This is the EASIEST way to configure the accelerometer!
// 
// Parameters:
//   fsr_g: Full-scale range in g-force (2, 4, 8, or 16)
//   dlpf_bandwidth_hz: DLPF 3dB bandwidth in Hz (0, 6, 12, 24, 50, 111, 246, 473)
//                      Use 0 to disable DLPF (bypass mode)
//   odr_hz: Output data rate in Hz (1-1125)
//
// Examples:
//   applyAccelConfigDirect(4, 50, 225);   // ±4g, 50Hz DLPF, 225Hz ODR (general purpose)
//   applyAccelConfigDirect(2, 12, 112);   // ±2g, 12Hz DLPF, 112Hz ODR (high-res, smooth)
//   applyAccelConfigDirect(16, 0, 1125);  // ±16g, no DLPF, 1125Hz ODR (extreme shocks)
//
// Returns true on success, false on invalid parameters or I²C error.
bool applyAccelConfigDirect(uint8_t fsr_g, uint16_t dlpf_bandwidth_hz, uint16_t odr_hz);

// Print all available configuration options with explanations.
// Call this function to see what values you can use with applyAccelConfigDirect().
void printAccelConfigOptions();

} // namespace AccelCfg

#endif // ACCEL_CONFIG_H
