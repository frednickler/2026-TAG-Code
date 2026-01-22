#include "config_store.h"
#include <EEPROM.h>

#define CONFIG_MAGIC 0xABCE
#define CONFIG_ADDR  0x00

SystemConfig g_sysConfig;

void initConfigStore() {
    // EEPROM should be started by calibration_manager or main, but we check size
    // We assume EEPROM.begin(1024) has been called.
    
    EEPROM.get(CONFIG_ADDR, g_sysConfig);
    
    if (g_sysConfig.magic != CONFIG_MAGIC) {
        Serial.println("[CFG] No valid config found. Using Defaults.");
        g_sysConfig.magic = CONFIG_MAGIC;
        g_sysConfig.accelPreset = 2; // +/- 4g
        g_sysConfig.gyroPreset = 2;  // +/- 500dps
        g_sysConfig.magPreset = 5;   // 100Hz
        g_sysConfig.gpsRate = 25;       // 25Hz (as requested)
        g_sysConfig.radioDebug = false;
        
        // GNSS Defaults
        g_sysConfig.gnssConstellation = 0; // GPS+GLO+GAL+BDS
        g_sysConfig.dynamicModel = 0;      // Portable (Default) - user can change to 2 (Auto)
        g_sysConfig.i2cClockSpeed = 1;     // Fast (400kHz) -> "115200 Equivalent"
        g_sysConfig.sbasEnabled = true;    // Enabled
        g_sysConfig.qzssEnabled = false;   // Disabled
        g_sysConfig.antiJamming = 2;       // Adaptive
        g_sysConfig.powerMode = 0;         // Full Power
        g_sysConfig.unitSystem = 0;        // Metric
        g_sysConfig.zuptEnabled = false;   // Disabled by default on startup (Safety)
        
        // Default MAC: Broadcast
        memset(g_sysConfig.targetMac, 0xFF, 6);
        
        saveConfigStore();
    } else {
        Serial.println("[CFG] Configuration loaded from EEPROM.");
        // FORCE ZUPT OFF ON BOOT (Requested by User)
        // Even if saved as ON, we start with OFF to ensure calibration/safety.
        g_sysConfig.zuptEnabled = false; 
    }
}

void saveConfigStore() {
    EEPROM.put(CONFIG_ADDR, g_sysConfig);
    if (EEPROM.commit()) {
        Serial.println("[CFG] Settings Saved.");
    } else {
        Serial.println("[CFG] Error Saving Settings!");
    }
}

void printConfigStore() {
    Serial.printf("[CFG] Accel:%d Gyro:%d Mag:%d GPS:%dHz Dbg:%d ZUPT:%d\n", 
        g_sysConfig.accelPreset, g_sysConfig.gyroPreset, 
        g_sysConfig.magPreset, g_sysConfig.gpsRate, g_sysConfig.radioDebug, g_sysConfig.zuptEnabled);
}
