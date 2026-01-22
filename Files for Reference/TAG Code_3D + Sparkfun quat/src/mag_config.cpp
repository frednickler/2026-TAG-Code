/*-----------------------------------------------------------------------------
 * mag_config.cpp – Helpers for configuring AK09916 magnetometer
 *---------------------------------------------------------------------------*/
#include "mag_config.h"
#include "imu_config.h" // I2C_IMU reference + AK09916_ADDR constant + write helpers

using namespace MagCfg;

// AK09916 register map excerpts
static constexpr uint8_t REG_CNTL2 = 0x31;
static constexpr uint8_t REG_CNTL3 = 0x32; // Soft-reset when bit0=1

struct MagPreset { uint8_t mode; bool softReset; };

static constexpr MagPreset PRESETS[] = {
    // Index 0 = build-time default so the IMU powers-up in a known state.
    // Not addressable from the CLI.
    {USER_MODE, USER_DO_SOFT_RESET},         // 0  → build-time default

    // Runtime presets (`mcfg <N>`):
    //   mcfg 1 → Power-down (lowest power)
    //   mcfg 2 → Single-shot (one reading then power-down)
    //   mcfg 3 → Continuous 10 Hz  (low update, low power)
    //   mcfg 4 → Continuous 20 Hz  (balance)
    //   mcfg 5 → Continuous 50 Hz  (default for fusion)
    //   mcfg 6 → Continuous 100 Hz (max throughput)
    // The bool flag triggers a soft-reset before applying the mode which is
    // recommended after errors or when changing ODR.  Tweak as needed.

    {MODE_POWER_DOWN,   true},  // 1  – power-down
    {MODE_SINGLE,       true},  // 2  – single measurement
    {MODE_CONT_10HZ,    true},  // 3  – 10 Hz continuous
    {MODE_CONT_20HZ,    true},  // 4  – 20 Hz continuous
    {MODE_CONT_50HZ,    true},  // 5  – 50 Hz continuous (default)
    {MODE_CONT_100HZ,   true}   // 6  – 100 Hz continuous
};

static void writeMagReg(uint8_t reg, uint8_t val) {
    I2C_IMU.beginTransmission(AK09916_ADDR);
    I2C_IMU.write(reg);
    I2C_IMU.write(val);
    I2C_IMU.endTransmission();
    delay(2);
}

static uint8_t readMagReg(uint8_t reg) {
    I2C_IMU.beginTransmission(AK09916_ADDR);
    I2C_IMU.write(reg);
    I2C_IMU.endTransmission(false);
    I2C_IMU.requestFrom((uint8_t)AK09916_ADDR, (uint8_t)1);
    return I2C_IMU.read();
}

static bool apply(uint8_t mode, bool doReset) {
    if (doReset) {
        writeMagReg(REG_CNTL3, 0x01); // Soft reset
        delay(100);
    }
    writeMagReg(REG_CNTL2, mode);
    delay(10);
    return readMagReg(REG_CNTL2) == mode;
}

bool MagCfg::applyMagConfig() {
    return apply(USER_MODE, USER_DO_SOFT_RESET);
}

bool MagCfg::applyMagPreset(uint8_t id) {
    if (id >= sizeof(PRESETS) / sizeof(PRESETS[0])) return false;
    const auto &p = PRESETS[id];
    return apply(p.mode, p.softReset);
}

void MagCfg::printMagConfig(Stream &out) {
    uint8_t mode = readMagReg(REG_CNTL2);
    out.printf("[MAG CFG] Mode: 0x%02X ", mode);
    if (mode == MODE_POWER_DOWN) out.print("(Power-down)");
    else if (mode == MODE_SINGLE) out.print("(Single-shot)");
    else if (mode == MODE_CONT_10HZ) out.print("(10 Hz)");
    else if (mode == MODE_CONT_20HZ) out.print("(20 Hz)");
    else if (mode == MODE_CONT_50HZ) out.print("(50 Hz)");
    else if (mode == MODE_CONT_100HZ) out.print("(100 Hz)");
    out.println();
}
