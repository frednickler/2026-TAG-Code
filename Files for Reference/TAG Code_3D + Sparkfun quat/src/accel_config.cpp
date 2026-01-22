/*-----------------------------------------------------------------------------
 * accel_config.cpp – Register write helpers for accelerometer configuration
 *---------------------------------------------------------------------------*/
#include "accel_config.h"
#include "imu_config.h"  // setBank / writeRegister helpers

// Forward declarations (imu_config.cpp provides definitions)
extern void setBank(uint8_t bank);
extern void writeRegister(uint8_t reg, uint8_t value);
extern uint8_t readRegister(uint8_t reg);

using namespace AccelCfg;

// Register addresses (User Bank 2)
static constexpr uint8_t REG_ACCEL_CONFIG      = 0x14; // ACCEL_CONFIG (FSR + DLPF)
static constexpr uint8_t REG_ACCEL_SMPLRT_DIV1 = 0x10; // [11:8] divider MSB
static constexpr uint8_t REG_ACCEL_SMPLRT_DIV2 = 0x11; // [7:0]  divider LSB

//---------------------------------------------------------------------------
// Convenience type to hold a full preset
struct AccelPreset {
    uint8_t fs_sel;
    uint8_t dlpf_cfg;
    uint16_t smplrt_div;
};

// Table matching the CLI numbers documented in the header
static constexpr AccelPreset PRESETS[] = {
    // NOTE: Index 0 is **not** reachable from the CLI; it simply mirrors the
    // compile-time USER_* constants so the firmware starts with predictable
    // settings before any runtime command is issued.
    {FS_4G,  DLPF_50HZ, 4},  // 0  → build-time default (±4 g, 50 Hz BW, 225 Hz ODR)

    // The indices below map 1-to-1 to the number the user types after `acfg`:
    //     acfg 1   → 2 g range, 50 Hz BW, 225 Hz ODR (high-resolution, low-g)
    //     acfg 2   → 4 g range, 50 Hz BW, 225 Hz ODR (general-purpose)
    //     acfg 3   → 8 g range, 50 Hz BW, 225 Hz ODR (moderate shocks)
    //     acfg 4   → 16 g range, 50 Hz BW, 225 Hz ODR (maximum headroom)
    //     acfg 5   → 4 g range, 24 Hz BW, 112 Hz ODR (extra noise rejection)
    // Feel free to edit or append presets — the CLI will accept any index
    // within the bounds of this table.

    {FS_2G,  DLPF_50HZ, 4},  // 1  – high-resolution ±2 g
    {FS_4G,  DLPF_50HZ, 4},  // 2  – default general-purpose
    {FS_8G,  DLPF_50HZ, 4},  // 3  – higher dynamic range
    {FS_16G, DLPF_50HZ, 4},  // 4  – extreme shocks / aerobatics
    {FS_4G,  DLPF_24HZ, 9}   // 5  – low-noise (24 Hz BW, 112 Hz ODR)
};

//---------------------------------------------------------------------------
static void writeAccelRegs(uint8_t fs_sel, uint8_t dlpf_cfg, uint16_t smplrt) {
    setBank(2);
    // Combine FSR & DLPF bits (ACCEL_CONFIG layout: [3:2] FS_SEL, [2:0] DLPFCFG)
    uint8_t cfg = (fs_sel << 1) | (dlpf_cfg & 0x07);
    writeRegister(REG_ACCEL_CONFIG, cfg);
    writeRegister(REG_ACCEL_SMPLRT_DIV1, (smplrt >> 8) & 0x0F);
    writeRegister(REG_ACCEL_SMPLRT_DIV2, smplrt & 0xFF);
    setBank(0);
}

bool AccelCfg::applyAccelConfig() {
    writeAccelRegs(USER_FS_SEL, USER_DLPF_CFG, USER_SMPLRT_DIV);
    return true; // TODO: add read-back verification if desired
}

bool AccelCfg::applyAccelPreset(uint8_t id) {
    if (id == 0 || id >= sizeof(PRESETS) / sizeof(PRESETS[0])) return false;
    const auto &p = PRESETS[id];
    writeAccelRegs(p.fs_sel, p.dlpf_cfg, p.smplrt_div);
    return true;
}

void AccelCfg::printAccelConfig(Stream &out) {
    setBank(2);
    uint8_t cfg  = readRegister(REG_ACCEL_CONFIG);
    uint16_t div = (readRegister(REG_ACCEL_SMPLRT_DIV1) << 8) | readRegister(REG_ACCEL_SMPLRT_DIV2);
    setBank(0);
    out.printf("[ACCEL CFG] FS_SEL=%u  DLPF=%u  DIV=%u\n", (cfg >> 1) & 0x03, cfg & 0x07, div);
}

/*=============================================================================
 * USER-FRIENDLY CONFIGURATION FUNCTIONS
 * These functions allow direct specification using intuitive numbers
 *===========================================================================*/

/**
 * Convert FSR in g-force to register bits
 * @param fsr_g Full-scale range in g (2, 4, 8, or 16)
 * @return Register bits or 0xFF if invalid
 */
static uint8_t fsrToRegisterBits(uint8_t fsr_g) {
    switch (fsr_g) {
        case 2:  return FS_2G;
        case 4:  return FS_4G;
        case 8:  return FS_8G;
        case 16: return FS_16G;
        default: return 0xFF; // Invalid
    }
}

/**
 * Convert DLPF bandwidth in Hz to register bits
 * @param bandwidth_hz DLPF 3dB bandwidth in Hz (6, 12, 24, 50, 111, 246, 473, or 0 for bypass)
 * @return Register bits or 0xFF if invalid
 */
static uint8_t dlpfToRegisterBits(uint16_t bandwidth_hz) {
    switch (bandwidth_hz) {
        case 0:   return DLPF_DISABLED; // Bypass mode
        case 6:   return DLPF_6HZ;
        case 12:  return DLPF_12HZ;
        case 24:  return DLPF_24HZ;
        case 50:  return DLPF_50HZ;
        case 111: return DLPF_111HZ;
        case 246: return DLPF_246HZ;
        case 473: return DLPF_473HZ;
        default:  return 0xFF; // Invalid
    }
}

/**
 * Convert ODR frequency in Hz to sample rate divider
 * @param odr_hz Desired output data rate in Hz (1-1125)
 * @return Sample rate divider value
 */
static uint16_t odrToSampleRateDivider(uint16_t odr_hz) {
    if (odr_hz == 0 || odr_hz > 1125) return 0xFFFF; // Invalid
    
    // Base ODR is 1125 Hz when DLPF is enabled
    // Actual ODR = 1125 / (1 + divider)
    // Therefore: divider = (1125 / desired_odr) - 1
    uint16_t divider = (1125 / odr_hz) - 1;
    
    // Clamp to valid range (0-4095)
    if (divider > 4095) divider = 4095;
    
    return divider;
}

/**
 * Apply accelerometer configuration using user-friendly parameters
 * @param fsr_g Full-scale range in g (2, 4, 8, or 16)
 * @param dlpf_bandwidth_hz DLPF 3dB bandwidth in Hz (0, 6, 12, 24, 50, 111, 246, 473)
 * @param odr_hz Output data rate in Hz (1-1125)
 * @return true on success, false on invalid parameters or I2C error
 */
bool applyAccelConfigDirect(uint8_t fsr_g, uint16_t dlpf_bandwidth_hz, uint16_t odr_hz) {
    // Convert user-friendly values to register bits
    uint8_t fs_sel = fsrToRegisterBits(fsr_g);
    uint8_t dlpf_cfg = dlpfToRegisterBits(dlpf_bandwidth_hz);
    uint16_t smplrt_div = odrToSampleRateDivider(odr_hz);
    
    // Validate all parameters
    if (fs_sel == 0xFF || dlpf_cfg == 0xFF || smplrt_div == 0xFFFF) {
        Serial.println("ERROR: Invalid accelerometer configuration parameters");
        Serial.print("  FSR must be: 2, 4, 8, or 16 g (given: "); Serial.print(fsr_g); Serial.println(" g)");
        Serial.print("  DLPF must be: 0, 6, 12, 24, 50, 111, 246, or 473 Hz (given: "); Serial.print(dlpf_bandwidth_hz); Serial.println(" Hz)");
        Serial.print("  ODR must be: 1-1125 Hz (given: "); Serial.print(odr_hz); Serial.println(" Hz)");
        return false;
    }
    
    // Apply the configuration
    writeAccelRegs(fs_sel, dlpf_cfg, smplrt_div);
    
    // Print confirmation
    Serial.println("Accelerometer configuration applied:");
    Serial.print("  Full-Scale Range: ±"); Serial.print(fsr_g); Serial.println(" g");
    Serial.print("  DLPF Bandwidth: "); 
    if (dlpf_bandwidth_hz == 0) {
        Serial.println("DISABLED (bypass mode)");
    } else {
        Serial.print(dlpf_bandwidth_hz); Serial.println(" Hz");
    }
    Serial.print("  Output Data Rate: "); Serial.print(odr_hz); Serial.println(" Hz");
    
    return true;
}

/**
 * Print available configuration options for user reference
 */
void printAccelConfigOptions() {
    Serial.println("\n=== ACCELEROMETER CONFIGURATION OPTIONS ===");
    
    Serial.println("\n1. FULL-SCALE RANGE (FSR):");
    Serial.println("   2   →  ±2 g  (highest resolution: 16,384 LSB/g)");
    Serial.println("   4   →  ±4 g  (good balance: 8,192 LSB/g)");
    Serial.println("   8   →  ±8 g  (higher range: 4,096 LSB/g)");
    Serial.println("   16  →  ±16 g (maximum range: 2,048 LSB/g)");
    
    Serial.println("\n2. DIGITAL LOW-PASS FILTER BANDWIDTH:");
    Serial.println("   0   →  DISABLED (bypass mode, ~1209 Hz BW)");
    Serial.println("   6   →  6 Hz    (maximum smoothing, 27.9 ms delay)");
    Serial.println("   12  →  12 Hz   (high noise rejection, 15.5 ms delay)");
    Serial.println("   24  →  24 Hz   (smooth motion, 7.8 ms delay)");
    Serial.println("   50  →  50 Hz   (general purpose, 3.9 ms delay)");
    Serial.println("   111 →  111 Hz  (moderate filtering, 1.69 ms delay)");
    Serial.println("   246 →  246 Hz  (fast response, 0.97 ms delay)");
    Serial.println("   473 →  473 Hz  (high bandwidth, 0.66 ms delay)");
    
    Serial.println("\n3. OUTPUT DATA RATE (ODR):");
    Serial.println("   1    →  1 Hz     (ultra-low power)");
    Serial.println("   7    →  7 Hz     (low power logging)");
    Serial.println("   14   →  14 Hz    (minimal power)");
    Serial.println("   28   →  28 Hz    (very slow sampling)");
    Serial.println("   56   →  56 Hz    (slow applications)");
    Serial.println("   112  →  112 Hz   (lower bandwidth)");
    Serial.println("   225  →  225 Hz   (general purpose)");
    Serial.println("   375  →  375 Hz   (standard rate)");
    Serial.println("   562  →  562 Hz   (fast control loops)");
    Serial.println("   1125 →  1125 Hz  (maximum rate)");
    
    Serial.println("\nUSAGE EXAMPLES:");
    Serial.println("  acfg_direct 4 50 225    → ±4g, 50Hz DLPF, 225Hz ODR (general purpose)");
    Serial.println("  acfg_direct 2 12 112    → ±2g, 12Hz DLPF, 112Hz ODR (high resolution, smooth)");
    Serial.println("  acfg_direct 16 0 1125   → ±16g, no DLPF, 1125Hz ODR (extreme shocks, fast)");
    Serial.println("  acfg_direct 8 246 562   → ±8g, 246Hz DLPF, 562Hz ODR (sports/aerobatics)");
}
