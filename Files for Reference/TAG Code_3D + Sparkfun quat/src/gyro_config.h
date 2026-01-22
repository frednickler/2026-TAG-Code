#ifndef GYRO_CONFIG_H
#define GYRO_CONFIG_H

/*-----------------------------------------------------------------------------
 * gyro_config.h  –  User-tunable configuration for the ICM-20948 gyroscope
 *
 * COMPREHENSIVE GYROSCOPE CONFIGURATION REFERENCE
 * Based on ICM-20948 Datasheet Rev.1.3 (DS-000189)
 * https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
 *
 * This header centralises **all** gyroscope-specific settings so they can be
 * tweaked without hunting through driver code. Each parameter is documented
 * with datasheet references and recommended values for common use-cases.
 *
 * HOW TO USE
 * ----------
 * 1.  Review the comprehensive reference tables below to understand all available
 *     configurations for FSR, DLPF, and ODR settings.
 * 2.  Edit the "User-editable section" below using the symbolic constants.
 *     Each parameter maps 1-to-1 to register bit-fields.
 * 3.  Re-build and flash the firmware. During `setup()` the function
 *     `applyGyroConfig()` will write the selected settings to the IMU.
 * 4.  At runtime you can use CLI presets (`gcfg 1-5`) to test different
 *     configurations without recompiling.
 * 5.  Call `printGyroConfig()` from CLI/menu to confirm current register contents.
 *
 * If you do NOT need to tweak the gyroscope you can leave this file as-is.
 *---------------------------------------------------------------------------*/

#include <Arduino.h>

namespace GyroCfg {

/*=============================================================================
 * ICM-20948 GYROSCOPE CONFIGURATION REFERENCE TABLES
 * Source: ICM-20948 Datasheet Rev.1.3, Table 34, 35, 36
 *===========================================================================*/

/*-----------------------------------------------------------------------------
 * TABLE 1: FULL-SCALE RANGE (FSR) CONFIGURATIONS
 * Register: BANK 2, GYRO_CONFIG_1[2:1] = GYRO_FS_SEL[1:0]
 * Datasheet Reference: Table 34 (Page 40)
 *---------------------------------------------------------------------------*/
//
// ┌─────────┬──────────────┬─────────────────┬──────────────────────────────┐
// │ FS_SEL  │ Full Scale   │ Sensitivity     │ Recommended Applications     │
// │ [1:0]   │ Range        │ (LSB/°/s)       │                              │
// ├─────────┼──────────────┼─────────────────┼──────────────────────────────┤
// │   00    │   ±250 °/s   │     131.0       │ Precision attitude, slow     │
// │         │              │                 │ rotation, high resolution    │
// ├─────────┼──────────────┼─────────────────┼──────────────────────────────┤
// │   01    │   ±500 °/s   │      65.5       │ General purpose, drones,     │
// │         │              │                 │ vehicles, default setting    │
// ├─────────┼──────────────┼─────────────────┼──────────────────────────────┤
// │   10    │  ±1000 °/s   │      32.8       │ Fast maneuvers, sports,      │
// │         │              │                 │ moderate aerobatics          │
// ├─────────┼──────────────┼─────────────────┼──────────────────────────────┤
// │   11    │  ±2000 °/s   │      16.4       │ Extreme rotation, aggressive │
// │         │              │                 │ aerobatics, spin detection   │
// └─────────┴──────────────┴─────────────────┴──────────────────────────────┘
//
constexpr uint8_t FS_250DPS  = 0b00; // ±250 °/s  – Highest resolution (131.0 LSB/°/s)
constexpr uint8_t FS_500DPS  = 0b01; // ±500 °/s  – Good balance (65.5 LSB/°/s) [DEFAULT]
constexpr uint8_t FS_1000DPS = 0b10; // ±1000 °/s – Higher range (32.8 LSB/°/s)
constexpr uint8_t FS_2000DPS = 0b11; // ±2000 °/s – Maximum range (16.4 LSB/°/s)

/*-----------------------------------------------------------------------------
 * TABLE 2: DIGITAL LOW-PASS FILTER (DLPF) CONFIGURATIONS
 * Register: BANK 2, GYRO_CONFIG_1[5:3] = GYRO_DLPFCFG[2:0]
 * Datasheet Reference: Table 35 (Page 40)
 *---------------------------------------------------------------------------*/
//
// ┌─────────┬─────────────┬─────────────┬─────────────┬──────────────────────┐
// │ DLPFCFG │ 3dB BW      │ Noise BW    │ Delay       │ Recommended Use      │
// │ [2:0]   │ (Hz)        │ (Hz)        │ (ms)        │                      │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   000   │ 8173.0      │ 3281.0      │ 0.11        │ Bypass mode, raw     │
// │         │ (Bypass)    │ (Bypass)    │             │ data, high bandwidth │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   001   │  250.0      │  306.0      │ 0.97        │ Fast response,       │
// │         │             │             │             │ minimal filtering    │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   010   │  176.0      │  177.0      │ 1.9         │ Moderate filtering   │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   011   │   92.0      │  113.0      │ 2.8         │ General purpose,     │
// │         │             │             │             │ sensor fusion [DFLT] │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   100   │   41.0      │   61.0      │ 5.9         │ Smooth motion,       │
// │         │             │             │             │ noise reduction      │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   101   │   20.0      │   30.0      │ 11.6        │ High noise rejection │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   110   │   10.0      │   15.0      │ 23.2        │ Maximum smoothing,   │
// │         │             │             │             │ attitude estimation  │
// ├─────────┼─────────────┼─────────────┼─────────────┼──────────────────────┤
// │   111   │    5.0      │    7.0      │ 45.8        │ Ultra-smooth, slow   │
// │         │             │             │             │ motion only          │
// └─────────┴─────────────┴─────────────┴─────────────┴──────────────────────┘
//
constexpr uint8_t DLPF_DISABLED = 0; // Bypass – 8173 Hz BW, 0.11 ms delay
constexpr uint8_t DLPF_250HZ    = 1; // 250 Hz BW, 0.97 ms delay
constexpr uint8_t DLPF_176HZ    = 2; // 176 Hz BW, 1.9 ms delay
constexpr uint8_t DLPF_92HZ     = 3; // 92 Hz BW, 2.8 ms delay [DEFAULT]
constexpr uint8_t DLPF_41HZ     = 4; // 41 Hz BW, 5.9 ms delay
constexpr uint8_t DLPF_20HZ     = 5; // 20 Hz BW, 11.6 ms delay
constexpr uint8_t DLPF_10HZ     = 6; // 10 Hz BW, 23.2 ms delay
constexpr uint8_t DLPF_5HZ      = 7; // 5 Hz BW, 45.8 ms delay

/*-----------------------------------------------------------------------------
 * TABLE 3: OUTPUT DATA RATE (ODR) CONFIGURATIONS
 * Register: BANK 2, GYRO_SMPLRT_DIV (8-bit value)
 * Datasheet Reference: Table 36 (Page 40)
 *---------------------------------------------------------------------------*/
//
// ODR CALCULATION:
// • When DLPF is ENABLED (DLPFCFG = 1-7):  Base ODR = 1.0 kHz
// • When DLPF is DISABLED (DLPFCFG = 0):   Base ODR = 8.0 kHz
// • Actual ODR = Base ODR / (1 + GYRO_SMPLRT_DIV)
// • GYRO_SMPLRT_DIV is an 8-bit value (0-255)
//
// ┌─────────────┬─────────────────┬─────────────────┬──────────────────────┐
// │ Desired ODR │ DLPF Enabled    │ DLPF Disabled   │ Recommended Use      │
// │ (Hz)        │ Divider Value   │ Divider Value   │                      │
// ├─────────────┼─────────────────┼─────────────────┼──────────────────────┤
// │   1000      │       0         │      N/A        │ Maximum rate, DLPF   │
// │    500      │       1         │      15         │ High-speed sampling  │
// │    333      │       2         │      23         │ Fast control loops   │
// │    250      │       3         │      31         │ Moderate sampling    │
// │    200      │       4         │      39         │ General purpose      │
// │    125      │       7         │      63         │ Standard rate        │
// │    100      │       9         │      79         │ Lower bandwidth      │
// │     50      │      19         │     159         │ Slow applications    │
// │     25      │      39         │     319         │ Very slow sampling   │
// │     10      │      99         │     799         │ Minimal power        │
// │      5      │     199         │    1599         │ Ultra-low power      │
// │      1      │     999         │    7999         │ Logging applications │
// └─────────────┴─────────────────┴─────────────────┴──────────────────────┘
//
// COMMON ODR DIVIDER VALUES (for DLPF enabled, 1.0 kHz base):
constexpr uint16_t ODR_1000HZ_DIV = 0;   // 1000 Hz (maximum rate)
constexpr uint16_t ODR_500HZ_DIV  = 1;   // 500 Hz
constexpr uint16_t ODR_333HZ_DIV  = 2;   // 333 Hz
constexpr uint16_t ODR_250HZ_DIV  = 3;   // 250 Hz
constexpr uint16_t ODR_200HZ_DIV  = 4;   // 200 Hz [DEFAULT]
constexpr uint16_t ODR_125HZ_DIV  = 7;   // 125 Hz
constexpr uint16_t ODR_100HZ_DIV  = 9;   // 100 Hz
constexpr uint16_t ODR_50HZ_DIV   = 19;  // 50 Hz
constexpr uint16_t ODR_25HZ_DIV   = 39;  // 25 Hz
constexpr uint16_t ODR_10HZ_DIV   = 99;  // 10 Hz
constexpr uint16_t ODR_5HZ_DIV    = 199; // 5 Hz
constexpr uint16_t ODR_1HZ_DIV    = 999; // 1 Hz

/*=============================================================================
 * CONFIGURATION PRESET REFERENCE
 * These presets are available via CLI command: gcfg <number>
 * Defined in gyro_config.cpp PRESETS[] array
 *===========================================================================*/
//
// ┌────────┬─────────┬─────────┬─────────┬─────────────┬─────────────────────┐
// │ Preset │   FSR   │  DLPF   │   ODR   │ Sensitivity │ Application         │
// │ Number │         │         │  (Hz)   │ (LSB/°/s)   │                     │
// ├────────┼─────────┼─────────┼─────────┼─────────────┼─────────────────────┤
// │   1    │ ±250°/s │  92 Hz  │  200    │    131.0    │ High-res, slow rot  │
// │   2    │ ±500°/s │  92 Hz  │  200    │     65.5    │ General purpose     │
// │   3    │±1000°/s │  92 Hz  │  200    │     32.8    │ Fast maneuvers      │
// │   4    │±2000°/s │  92 Hz  │  200    │     16.4    │ Extreme rotation    │
// │   5    │ ±500°/s │  41 Hz  │  100    │     65.5    │ Extra noise reject  │
// └────────┴─────────┴─────────┴─────────┴─────────────┴─────────────────────┘

/*=============================================================================
 * USER-EDITABLE CONFIGURATION SECTION
 * Edit these values to customize gyroscope behavior
 *===========================================================================*/

// ---------------------------------------------------------------------------
// 1) FULL-SCALE RANGE SELECTION
// Choose from: FS_250DPS, FS_500DPS, FS_1000DPS, FS_2000DPS
// ---------------------------------------------------------------------------
constexpr uint8_t USER_FS_SEL = FS_500DPS; // ±500 °/s (65.5 LSB/°/s) – DEFAULT
//
// SELECTION GUIDE:
// • FS_250DPS  (±250°/s):  Best for precision attitude, slow rotation, high resolution
// • FS_500DPS  (±500°/s):  Good all-around choice for drones, vehicles
// • FS_1000DPS (±1000°/s): Fast maneuvers, sports equipment, moderate aerobatics
// • FS_2000DPS (±2000°/s): Extreme rotation, aggressive aerobatics, spin detection
//
// TIP: Test different ranges at runtime using CLI presets:
//      Serial command: gcfg <1-5>
//      Each preset combines FSR + DLPF + ODR for specific use cases.

// ---------------------------------------------------------------------------
// 2) DIGITAL LOW-PASS FILTER SELECTION
// Choose from: DLPF_DISABLED, DLPF_250HZ, DLPF_176HZ, DLPF_92HZ,
//              DLPF_41HZ, DLPF_20HZ, DLPF_10HZ, DLPF_5HZ
// ---------------------------------------------------------------------------
constexpr uint8_t USER_DLPF_CFG = DLPF_92HZ; // 92 Hz BW, 2.8 ms delay – DEFAULT
//
// SELECTION GUIDE:
// • DLPF_DISABLED: Raw data, maximum bandwidth, minimal delay (vibration-prone)
// • DLPF_250HZ:    Fast response, minimal filtering (high-speed applications)
// • DLPF_176HZ:    Moderate filtering, good for dynamic motion
// • DLPF_92HZ:     General purpose, ideal for sensor fusion [RECOMMENDED]
// • DLPF_41HZ:     Smooth motion, reduced noise (slower response)
// • DLPF_20HZ:     High noise rejection, stable readings
// • DLPF_10HZ:     Maximum smoothing, attitude estimation only
// • DLPF_5HZ:      Ultra-smooth, very slow motion only
//
// TRADE-OFFS:
// Lower bandwidth → Less noise, more delay, smoother output
// Higher bandwidth → More noise, less delay, faster response

// ---------------------------------------------------------------------------
// 3) OUTPUT DATA RATE (ODR) SELECTION
// Choose from predefined ODR_xxxHZ_DIV constants or custom divider value
// ---------------------------------------------------------------------------
constexpr uint16_t USER_SMPLRT_DIV = ODR_200HZ_DIV; // 200 Hz ODR – DEFAULT
//
// SELECTION GUIDE:
// • ODR_1000HZ_DIV (0):   Maximum rate, high CPU load, fast control loops
// • ODR_500HZ_DIV (1):    High-speed sampling, moderate CPU load
// • ODR_333HZ_DIV (2):    Fast applications, good balance
// • ODR_250HZ_DIV (3):    Standard high-rate applications
// • ODR_200HZ_DIV (4):    General purpose, sensor fusion [DEFAULT]
// • ODR_125HZ_DIV (7):    Standard rate, reduced processing
// • ODR_100HZ_DIV (9):    Lower bandwidth, reduced power consumption
// • ODR_50HZ_DIV (19):    Slow applications, minimal processing
// • ODR_25HZ_DIV (39):    Very slow sampling, battery-powered devices
// • ODR_10HZ_DIV (99):    Minimal power consumption
// • ODR_5HZ_DIV (199):    Ultra-low power, logging applications
// • ODR_1HZ_DIV (999):    Data logging, long-term monitoring
//
// CUSTOM ODR CALCULATION:
// For custom ODR: divider = (1000 / desired_ODR) - 1
// Example: For 400 Hz → divider = (1000 / 400) - 1 = 1.5 ≈ 1
//
// POWER CONSUMPTION NOTE:
// Lower ODR significantly reduces power consumption and processing load.
// Choose the lowest rate that meets your application requirements.

/*=============================================================================
 * API FUNCTIONS (do not edit below unless you know what you're doing)
 *===========================================================================*/

// Apply the above user settings by writing to the IMU registers.
// Returns true on success (I²C ACK received) or false on communication error.
bool applyGyroConfig();

// Apply one of the predefined CLI presets (1-5 as shown in table above).
// Returns true on success, false on invalid preset number or I²C error.
bool applyGyroPreset(uint8_t presetId);

// Helper function that prints the current effective register values to the
// specified output stream for debugging and verification purposes.
void printGyroConfig(Stream &out = Serial);

/*=============================================================================
 * USER-FRIENDLY CONFIGURATION FUNCTIONS
 * These provide an intuitive interface using actual Hz and °/s values
 *===========================================================================*/

// Apply gyroscope configuration using user-friendly parameters.
// This is the EASIEST way to configure the gyroscope!
// 
// Parameters:
//   fsr_dps: Full-scale range in degrees per second (250, 500, 1000, or 2000)
//   dlpf_bandwidth_hz: DLPF 3dB bandwidth in Hz (0, 5, 10, 20, 41, 92, 176, 250)
//                      Use 0 to disable DLPF (bypass mode)
//   odr_hz: Output data rate in Hz (1-1000)
//
// Examples:
//   applyGyroConfigDirect(500, 92, 200);   // ±500°/s, 92Hz DLPF, 200Hz ODR (general purpose)
//   applyGyroConfigDirect(250, 20, 100);   // ±250°/s, 20Hz DLPF, 100Hz ODR (high-res, smooth)
//   applyGyroConfigDirect(2000, 0, 1000);  // ±2000°/s, no DLPF, 1000Hz ODR (extreme rotation)
//
// Returns true on success, false on invalid parameters or I²C error.
bool applyGyroConfigDirect(uint16_t fsr_dps, uint16_t dlpf_bandwidth_hz, uint16_t odr_hz);

// Print all available configuration options with explanations.
// Call this function to see what values you can use with applyGyroConfigDirect().
void printGyroConfigOptions();

} // namespace GyroCfg

#endif // GYRO_CONFIG_H
