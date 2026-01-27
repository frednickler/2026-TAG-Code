#include "SystemSettings.h"
#include "SystemConfig.h"
#include <Preferences.h>

#define CONFIG_MAGIC 0xDEADBEEF
#define PREF_NAMESPACE "sys_cfg"

RuntimeConfig SystemSettings::config;
static Preferences prefs;

void SystemSettings::init() {
    prefs.begin(PREF_NAMESPACE, false); // Read-Write
    
    if (prefs.getBytesLength("config") != sizeof(RuntimeConfig)) {
        Serial.println("[CFG] No valid config found or size mismatch. Loading defaults...");
        loadDefaults();
    } else {
        prefs.getBytes("config", &config, sizeof(RuntimeConfig));
        if (config.magic != CONFIG_MAGIC) {
            Serial.println("[CFG] Invalid magic. Loading defaults...");
            loadDefaults();
        } else {
            Serial.println("[CFG] Configuration loaded.");
        }
    }
}

void SystemSettings::save() {
    config.magic = CONFIG_MAGIC;
    prefs.putBytes("config", &config, sizeof(RuntimeConfig));
    Serial.println("[CFG] Configuration saved.");
}

void SystemSettings::loadDefaults() {
    config.magic = CONFIG_MAGIC;
    
    // ==================== ICM-20948 DEFAULTS ====================
    // Gyroscope: 500dps (Default preset 2)
    config.gyroRange = 1;       // 1 = 500dps
    config.gyroDLPF = 5;        // 5 = 23.9 Hz (datasheet default)
    config.gyroPreset = 2;      // Default (500dps)
    
    // Accelerometer: 4g (Default preset 2)
    config.accelRange = 1;      // 1 = 4g
    config.accelDLPF = 4;       // 4 = 23.9 Hz (datasheet default)
    config.accelPreset = 2;     // Default (4g)
    
    // ==================== BMM350 DEFAULTS ====================
    // Output Data Rate: 12.5Hz
    config.magODR = 12.5f;      // 12.5 Hz (good balance)
    
    // Averaging: 2 samples
    config.magAveraging = 1;    // 1 = 2 samples
    
    // Preset Mode: Regular
    config.magPreset = 2;       // 2 = Regular (avg=1, good balance)
    
    // ==================== GPS DEFAULTS ====================
    config.gpsRate = 10;           // 10Hz
    config.gnssConstellation = 0;  // All (4 Sats)
    config.dynamicModel = 0;       // Portable
    config.sbasEnabled = true;
    config.qzssEnabled = true;
    config.antiJamming = 0;        // No Anti-Jamming
    
    // ==================== I2C BUS DEFAULTS ====================
    config.sensorI2CClockKHz = 400;  // 400 kHz (recommended for sensors)
    config.gpsI2CClockKHz = 400;     // 400 kHz (recommended for 6-10 Hz)
    config.gpsProtocolMode = 0;      // 0 = UBX (default)
    
    // ==================== SYSTEM DEFAULTS ====================
    config.loopRate = DEFAULT_MAIN_LOOP_RATE_HZ;
    config.mountUpsideDown = false;// Standard mounting by default

    // Alignment Defaults
    config.gpsAlignmentMode = 0;   // Last Position (Default)
    config.savedHeadingOffset = 0.0f;
    
    // VQF Tuning Defaults
    config.vqfTauAcc = 3.0f;
    config.vqfTauMag = 2.0f;
    config.vqfMagRejection = true;
    
    // Output Mode Default
    config.outputMode = static_cast<uint8_t>(OutputMode::COMPACT);
    
    // ==================== RADIO DEFAULTS ====================
    config.radioEnabled = true;
    config.radioRole = 1;        // 1 = TAG (transmitter)
    config.radioChannel = 1;
    config.radioTxPower = 2;     // 2 = HIGH (20dBm)
    config.radioDataRate = 0;    // 0 = 1Mbps
    config.radioAckEnabled = true;
    config.radioTagTxRate = 1;   // 1 Hz for TAG
    config.radioPacketMode = 0;  // 0 = GPS+Heading only
    
    // Broadcast MAC (FF:FF:FF:FF:FF:FF) - will be paired later
    for (int i = 0; i < 6; i++) {
        config.radioTargetMAC[i] = 0xFF;
    }
    
    // Save defaults to NVS
    save();
}
