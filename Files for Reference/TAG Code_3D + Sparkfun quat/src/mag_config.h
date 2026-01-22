#ifndef MAG_CONFIG_H
#define MAG_CONFIG_H

/*-----------------------------------------------------------------------------
 * mag_config.h  –  User-tunable configuration for the AK09916 magnetometer
 * (co-packaged with the ICM-20948)
 *
 * COMPREHENSIVE MAGNETOMETER CONFIGURATION REFERENCE
 * Based on ICM-20948 Datasheet Rev.1.3 (DS-000189) and AK09916 Datasheet
 * https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
 *
 * The AK09916 magnetometer is a specialized sensor with fixed full-scale range
 * but configurable operating modes and output data rates. Unlike accelerometer
 * and gyroscope, it does not have user-configurable DLPF settings - filtering
 * is handled by the sensor fusion algorithm.
 *
 * HOW TO USE
 * ----------
 * 1.  Review the comprehensive reference tables below to understand all available
 *     magnetometer operating modes and their characteristics.
 * 2.  Edit the "User-editable section" below using the symbolic constants.
 *     Each parameter maps 1-to-1 to register bit-fields.
 * 3.  Re-build and flash the firmware. During `setup()` the function
 *     `applyMagConfig()` will write the selected settings to the magnetometer.
 * 4.  At runtime you can use CLI presets (`mcfg 0-5`) to test different
 *     configurations without recompiling.
 * 5.  Call `printMagConfig()` from CLI/menu to confirm current register contents.
 *
 * If you do NOT need to tweak the magnetometer you can leave this file as-is.
 *---------------------------------------------------------------------------*/

#include <Arduino.h>

namespace MagCfg {

/*=============================================================================
 * AK09916 MAGNETOMETER CONFIGURATION REFERENCE TABLES
 * Source: ICM-20948 Datasheet Rev.1.3, AK09916 Datasheet Section 9.3
 *===========================================================================*/

/*-----------------------------------------------------------------------------
 * TABLE 1: FULL-SCALE RANGE (FSR) - FIXED FOR AK09916
 * The AK09916 magnetometer has a FIXED full-scale range - no user configuration
 * Datasheet Reference: AK09916 Datasheet Section 8.1
 *---------------------------------------------------------------------------*/
//
// ┌─────────────────┬─────────────────┬─────────────────┬──────────────────────┐
// │ Parameter       │ Value           │ Unit            │ Notes                │
// ├─────────────────┼─────────────────┼─────────────────┼──────────────────────┤
// │ Measurement     │ ±4900           │ µT              │ Fixed range, cannot  │
// │ Range           │                 │                 │ be changed by user   │
// ├─────────────────┼─────────────────┼─────────────────┼──────────────────────┤
// │ Sensitivity     │ 0.15            │ µT/LSB          │ 16-bit resolution    │
// │                 │                 │                 │ (0.15 µT per count)  │
// ├─────────────────┼─────────────────┼─────────────────┼──────────────────────┤
// │ Resolution      │ 16              │ bits            │ Signed integer       │
// │                 │                 │                 │ output format        │
// └─────────────────┴─────────────────┴─────────────────┴──────────────────────┘
//
// NOTE: Unlike accelerometer and gyroscope, the magnetometer FSR is FIXED at ±4900 µT.
//       This range covers typical Earth magnetic field strength (25-65 µT) with plenty
//       of headroom for magnetic interference and calibration.

/*-----------------------------------------------------------------------------
 * TABLE 2: OPERATING MODES AND OUTPUT DATA RATES (ODR)
 * Register: AK09916 CNTL2[4:0] = Operating Mode
 * Datasheet Reference: AK09916 Datasheet Section 9.3
 *---------------------------------------------------------------------------*/
//
// ┌─────────┬─────────────────┬─────────┬─────────────┬──────────────────────────────┐
// │ Mode    │ Operating Mode  │   ODR   │ Current     │ Recommended Applications     │
// │ Code    │ Description     │  (Hz)   │ Consumption │                              │
// ├─────────┼─────────────────┼─────────┼─────────────┼──────────────────────────────┤
// │  0x00   │ Power-down      │    0    │   0.1 µA    │ Sleep mode, battery saving,  │
// │         │                 │         │             │ when magnetometer not needed │
// ├─────────┼─────────────────┼─────────┼─────────────┼──────────────────────────────┤
// │  0x01   │ Single          │  On-    │   ~10 µA    │ Triggered measurements,      │
// │         │ measurement     │ demand  │  per shot   │ ultra-low power applications │
// ├─────────┼─────────────────┼─────────┼─────────────┼──────────────────────────────┤
// │  0x02   │ Continuous      │   10    │   ~100 µA   │ Low-power compass, basic     │
// │         │ 10 Hz           │         │             │ orientation, battery devices │
// ├─────────┼─────────────────┼─────────┼─────────────┼──────────────────────────────┤
// │  0x04   │ Continuous      │   20    │   ~200 µA   │ Standard compass, slow       │
// │         │ 20 Hz           │         │             │ motion tracking              │
// ├─────────┼─────────────────┼─────────┼─────────────┼──────────────────────────────┤
// │  0x06   │ Continuous      │   50    │   ~500 µA   │ Moderate-speed applications, │
// │         │ 50 Hz           │         │             │ walking, vehicle navigation  │
// ├─────────┼─────────────────┼─────────┼─────────────┼──────────────────────────────┤
// │  0x08   │ Continuous      │  100    │   ~1000 µA  │ High-rate sensor fusion,     │
// │         │ 100 Hz          │         │             │ drones, fast motion [DFLT]  │
// ├─────────┼─────────────────┼─────────┼─────────────┼──────────────────────────────┤
// │  0x0A   │ Self-test       │   N/A   │   Variable  │ Factory test mode - not      │
// │         │                 │         │             │ recommended for normal use   │
// ├─────────┼─────────────────┼─────────┼─────────────┼──────────────────────────────┤
// │  0x0C   │ Fuse ROM        │   N/A   │   Variable  │ Factory calibration access  │
// │         │ access          │         │             │ - not for normal operation   │
// └─────────┴─────────────────┴─────────┴─────────────┴──────────────────────────────┘
//
constexpr uint8_t MODE_POWER_DOWN   = 0x00; // Power-down mode (0.1 µA)
constexpr uint8_t MODE_SINGLE       = 0x01; // Single measurement mode (~10 µA per shot)
constexpr uint8_t MODE_CONT_10HZ    = 0x02; // Continuous 10 Hz (~100 µA)
constexpr uint8_t MODE_CONT_20HZ    = 0x04; // Continuous 20 Hz (~200 µA)
constexpr uint8_t MODE_CONT_50HZ    = 0x06; // Continuous 50 Hz (~500 µA)
constexpr uint8_t MODE_CONT_100HZ   = 0x08; // Continuous 100 Hz (~1000 µA) [DEFAULT]
constexpr uint8_t MODE_SELF_TEST    = 0x0A; // Self-test mode (factory use)
constexpr uint8_t MODE_FUSE_ROM     = 0x0C; // Fuse ROM access (factory use)

/*-----------------------------------------------------------------------------
 * TABLE 3: DIGITAL LOW-PASS FILTER (DLPF) - NOT AVAILABLE
 * The AK09916 magnetometer does NOT have user-configurable DLPF settings
 *---------------------------------------------------------------------------*/
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ IMPORTANT: AK09916 DLPF INFORMATION                                     │
// ├─────────────────────────────────────────────────────────────────────────┤
// │ The AK09916 magnetometer does NOT provide user-configurable digital    │
// │ low-pass filtering like the accelerometer and gyroscope. The sensor    │
// │ has internal filtering that is fixed and optimized for each operating  │
// │ mode.                                                                   │
// │                                                                         │
// │ Filtering Strategy:                                                     │
// │ • Hardware filtering is built into each operating mode                 │
// │ • Additional filtering should be applied in software                   │
// │ • The Madgwick sensor fusion algorithm provides filtering              │
// │ • Custom filtering can be added in post-processing                     │
// │                                                                         │
// │ If you need additional magnetometer filtering:                         │
// │ 1. Use lower ODR (10Hz or 20Hz instead of 100Hz)                      │
// │ 2. Implement software filtering in your application                    │
// │ 3. Adjust Madgwick filter beta parameter                              │
// │ 4. Use calibration to remove hard/soft iron distortions               │
// └─────────────────────────────────────────────────────────────────────────┘

/*-----------------------------------------------------------------------------
 * TABLE 4: MEASUREMENT TIMING AND PERFORMANCE
 * Datasheet Reference: AK09916 Datasheet Section 8
 *---------------------------------------------------------------------------*/
//
// ┌─────────────┬─────────────┬─────────────┬─────────────┬──────────────────┐
// │ ODR Mode    │ Measurement │ Conversion  │ Data Ready  │ Recommended Use  │
// │             │ Period (ms) │ Time (ms)   │ Frequency   │ Case             │
// ├─────────────┼─────────────┼─────────────┼─────────────┼──────────────────┤
// │ 10 Hz       │    100      │    ~7.2     │   Every     │ Battery-powered  │
// │             │             │             │   100 ms    │ applications     │
// ├─────────────┼─────────────┼─────────────┼─────────────┼──────────────────┤
// │ 20 Hz       │     50      │    ~7.2     │   Every     │ Standard compass │
// │             │             │             │    50 ms    │ applications     │
// ├─────────────┼─────────────┼─────────────┼─────────────┼──────────────────┤
// │ 50 Hz       │     20      │    ~7.2     │   Every     │ Moderate-speed   │
// │             │             │             │    20 ms    │ motion tracking  │
// ├─────────────┼─────────────┼─────────────┼─────────────┼──────────────────┤
// │ 100 Hz      │     10      │    ~7.2     │   Every     │ High-rate sensor │
// │             │             │             │    10 ms    │ fusion [DEFAULT] │
// └─────────────┴─────────────┴─────────────┴─────────────┴──────────────────┘

/*=============================================================================
 * CONFIGURATION PRESET REFERENCE
 * These presets are available via CLI command: mcfg <number>
 * Defined in mag_config.cpp PRESETS[] array
 *===========================================================================*/
//
// ┌────────┬─────────────────┬─────────┬─────────────┬─────────────────────────┐
// │ Preset │ Operating Mode  │   ODR   │ Current     │ Application             │
// │ Number │                 │  (Hz)   │ Consumption │                         │
// ├────────┼─────────────────┼─────────┼─────────────┼─────────────────────────┤
// │   0    │ Power-down      │    0    │   0.1 µA    │ Sleep/standby mode      │
// │   1    │ Single shot     │ On-     │  ~10 µA     │ Ultra-low power         │
// │        │                 │ demand  │  per shot   │                         │
// │   2    │ Continuous 10Hz │   10    │  ~100 µA    │ Battery-powered compass │
// │   3    │ Continuous 20Hz │   20    │  ~200 µA    │ Standard compass        │
// │   4    │ Continuous 50Hz │   50    │  ~500 µA    │ Moderate-speed tracking │
// │   5    │ Continuous 100Hz│  100    │ ~1000 µA    │ High-rate fusion [DFLT] │
// └────────┴─────────────────┴─────────┴─────────────┴─────────────────────────┘

/*=============================================================================
 * USER-EDITABLE CONFIGURATION SECTION
 * Edit these values to customize magnetometer behavior
 *===========================================================================*/

// ---------------------------------------------------------------------------
// 1) OPERATING MODE SELECTION
// Choose from: MODE_POWER_DOWN, MODE_SINGLE, MODE_CONT_10HZ, MODE_CONT_20HZ,
//              MODE_CONT_50HZ, MODE_CONT_100HZ
// ---------------------------------------------------------------------------
constexpr uint8_t USER_MODE = MODE_CONT_100HZ; // 100 Hz continuous – DEFAULT
//
// SELECTION GUIDE:
// • MODE_POWER_DOWN:   Sleep mode, minimal power (0.1 µA)
// • MODE_SINGLE:       On-demand measurements, ultra-low power (~10 µA/shot)
// • MODE_CONT_10HZ:    Battery-powered applications, basic compass (~100 µA)
// • MODE_CONT_20HZ:    Standard compass applications (~200 µA)
// • MODE_CONT_50HZ:    Moderate-speed motion tracking (~500 µA)
// • MODE_CONT_100HZ:   High-rate sensor fusion, drones, fast motion (~1000 µA)
//
// POWER CONSUMPTION CONSIDERATIONS:
// • 100Hz mode consumes ~10x more power than 10Hz mode
// • For battery applications, use the lowest ODR that meets your needs
// • Single-shot mode is most efficient for infrequent measurements
//
// TIP: Test different modes at runtime using CLI presets:
//      Serial command: mcfg <0-5>
//      Each preset corresponds to a different operating mode.

// ---------------------------------------------------------------------------
// 2) SOFT RESET CONFIGURATION
// Enable/disable soft reset before applying operating mode
// ---------------------------------------------------------------------------
constexpr bool USER_DO_SOFT_RESET = true; // Perform soft reset – RECOMMENDED
//
// SELECTION GUIDE:
// • true:  Perform soft reset before setting operating mode [RECOMMENDED]
//          - Ensures clean state after power-up or I2C errors
//          - Clears any previous configuration or error states
//          - Recommended for reliable operation
// • false: Skip soft reset (advanced users only)
//          - Faster initialization
//          - Use only if you're certain the sensor is in a known good state
//          - May cause issues if sensor was in an error state

/*=============================================================================
 * API FUNCTIONS (do not edit below unless you know what you're doing)
 *===========================================================================*/

// Apply the above user settings by writing to the magnetometer registers.
// Returns true on success (I²C ACK received and register readback confirms)
// or false on communication error or verification failure.
bool applyMagConfig();

// Apply one of the predefined CLI presets (0-5 as shown in table above).
// Returns true on success, false on invalid preset number or I²C error.
bool applyMagPreset(uint8_t presetId);

// Helper function that prints the current effective register values to the
// specified output stream for debugging and verification purposes.
void printMagConfig(Stream &out = Serial);

/*=============================================================================
 * USER-FRIENDLY CONFIGURATION FUNCTIONS
 * These provide an intuitive interface using actual Hz values
 *===========================================================================*/

// Apply magnetometer configuration using user-friendly parameters.
// This is the EASIEST way to configure the magnetometer!
// 
// Parameters:
//   odr_hz: Output data rate in Hz (0, 1, 10, 20, 50, or 100)
//           Use 0 for power-down mode, 1 for single-shot mode
//   do_reset: Whether to perform soft reset before applying mode (recommended: true)
//
// Examples:
//   applyMagConfigDirect(100, true);  // 100Hz continuous, with reset (general purpose)
//   applyMagConfigDirect(20, true);   // 20Hz continuous, with reset (standard compass)
//   applyMagConfigDirect(10, true);   // 10Hz continuous, with reset (battery-powered)
//   applyMagConfigDirect(1, true);    // Single-shot mode, with reset (ultra-low power)
//   applyMagConfigDirect(0, true);    // Power-down mode, with reset (sleep)
//
// Returns true on success, false on invalid parameters or I²C error.
bool applyMagConfigDirect(uint16_t odr_hz, bool do_reset = true);

// Print all available configuration options with explanations.
// Call this function to see what values you can use with applyMagConfigDirect().
void printMagConfigOptions();

} // namespace MagCfg

#endif // MAG_CONFIG_H
