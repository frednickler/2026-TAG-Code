/*-----------------------------------------------------------------------------
 * gyro_config.cpp – Register write helpers for gyroscope configuration
 *---------------------------------------------------------------------------*/
#include "gyro_config.h"
#include "imu_config.h" // setBank / writeRegister / readRegister

extern void setBank(uint8_t bank);
extern void writeRegister(uint8_t reg, uint8_t value);
extern uint8_t readRegister(uint8_t reg);

using namespace GyroCfg;

// Register addresses (User Bank 2)
static constexpr uint8_t REG_GYRO_SMPLRT_DIV = 0x00; // 8-bit divider
static constexpr uint8_t REG_GYRO_CONFIG_1  = 0x01; // FSR + DLPF bits

struct GyroPreset { uint8_t fs_sel; uint8_t dlpf_cfg; uint8_t smplrt_div; };

static constexpr GyroPreset PRESETS[] = {
    // Index 0 mirrors the compile-time defaults so the board powers-up with a
    // known configuration *before* any CLI command is received.  It is NOT
    // selectable from the menu.
    {USER_FS_SEL, USER_DLPF_CFG, USER_SMPLRT_DIV}, // 0  → build-time default

    // Run-time selectable presets (type `gcfg <N>`):
    //   gcfg 1 → ±250 dps, 92 Hz BW, 225 Hz ODR  – highest resolution
    //   gcfg 2 → ±500 dps, 92 Hz BW, 225 Hz ODR  – general purpose
    //   gcfg 3 → ±1000 dps, 92 Hz BW, 225 Hz ODR – fast rotations
    //   gcfg 4 → ±2000 dps, 92 Hz BW, 225 Hz ODR – extreme manoeuvres
    //   gcfg 5 → ±500 dps, 41 Hz BW, 112 Hz ODR  – low-noise attitude hold
    // Feel free to tweak or append rows; the CLI will accept any valid index.

    {FS_250DPS,  DLPF_92HZ,  4}, // 1  – highest resolution ±250 dps
    {FS_500DPS,  DLPF_92HZ,  4}, // 2  – default ±500 dps
    {FS_1000DPS, DLPF_92HZ,  4}, // 3  – faster turns
    {FS_2000DPS, DLPF_92HZ,  4}, // 4  – maximum range
    {FS_500DPS,  DLPF_41HZ,  9}  // 5  – low-noise hold mode
};

static void writeGyroRegs(uint8_t fs_sel, uint8_t dlpf_cfg, uint8_t div) {
    setBank(2);
    uint8_t cfg1 = (fs_sel << 1) | (dlpf_cfg & 0x07);
    writeRegister(REG_GYRO_CONFIG_1, cfg1);
    writeRegister(REG_GYRO_SMPLRT_DIV, div);
    setBank(0);
}

bool GyroCfg::applyGyroConfig() {
    writeGyroRegs(USER_FS_SEL, USER_DLPF_CFG, USER_SMPLRT_DIV);
    return true;
}

bool GyroCfg::applyGyroPreset(uint8_t id) {
    if (id == 0 || id >= sizeof(PRESETS)/sizeof(PRESETS[0])) return false;
    const auto &p = PRESETS[id];
    writeGyroRegs(p.fs_sel, p.dlpf_cfg, p.smplrt_div);
    return true;
}

void GyroCfg::printGyroConfig(Stream &out) {
    setBank(2);
    uint8_t cfg = readRegister(REG_GYRO_CONFIG_1);
    uint8_t div = readRegister(REG_GYRO_SMPLRT_DIV);
    setBank(0);
    out.printf("[GYRO CFG] FS_SEL=%u  DLPF=%u  DIV=%u\n", (cfg >> 1) & 0x03, cfg & 0x07, div);
}
