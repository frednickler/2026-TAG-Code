#include "GPSManager.h"
#include "../system/Watchdog.h"
#include "../config/SystemSettings.h"

// Initialize static members
SFE_UBLOX_GNSS GPSManager::myGNSS;
GPSData GPSManager::data = {};
bool GPSManager::initialized = false;
unsigned long GPSManager::lastUpdate = 0;

// Debug statistics
unsigned long GPSManager::totalUpdates = 0;
unsigned long GPSManager::validFixes = 0;
unsigned long GPSManager::i2cErrors = 0;

// Configuration tracking
uint32_t GPSManager::currentClockSpeed = 400000;  // 400 kHz default in Hz
uint8_t GPSManager::currentProtocolMode = 0;       // 0 = UBX default

bool GPSManager::init() {
    DEBUG_INFO("Initializing GPSManager (I2C Mode)...");
    
    // Initialize GPS I2C bus on Wire1 (separate from sensor I2C bus on Wire)
    // Sensors use Wire on GPIO 4/7, GPS uses Wire1 on GPIO 8/9
    Wire1.begin(GPS_I2C_SDA_PIN, GPS_I2C_SCL_PIN);
    
    // Get clock speed from config (no hardcoded values!)
    RuntimeConfig& cfg = SystemSettings::getConfig();
    Wire1.setClock(cfg.gpsI2CClockKHz * 1000);  // Convert kHz to Hz
    
    DEBUG_INFO("GPS I2C Bus: SDA=GPIO%d, SCL=GPIO%d, Clock=%d kHz", 
               GPS_I2C_SDA_PIN, GPS_I2C_SCL_PIN, cfg.gpsI2CClockKHz);
    
    // Attempt to connect to GPS on I2C
    if (!myGNSS.begin(Wire1, GPS_I2C_ADDR)) {
        DEBUG_ERROR("GPS not detected on I2C at 0x%02X", GPS_I2C_ADDR);
        return false;
    }
    
    DEBUG_INFO("GPS detected on I2C at 0x%02X", GPS_I2C_ADDR);
    
    // Configure GPS for optimal performance
    myGNSS.setI2COutput(COM_TYPE_UBX); // Use UBX binary protocol (more efficient than NMEA)
    myGNSS.setNavigationFrequency(1);  // 1 Hz default (can be changed via setRefreshRate)
    
    // Enable automatic PVT (Position/Velocity/Time) updates
    myGNSS.setAutoPVT(true);
    
    // Save configuration to GPS flash memory (persists across power cycles)
    myGNSS.saveConfiguration();
    
    initialized = true;
    DEBUG_INFO("GPS initialized successfully");
    
    return true;
}

void GPSManager::update() {
    if (!initialized) return;
    
    totalUpdates++;
    
    // Request fresh PVT data from GPS
    if (myGNSS.getPVT()) {
        // Successfully retrieved position data
        
        // Extract latitude/longitude (units: degrees * 1e-7)
        data.latitude = myGNSS.getLatitude() / 10000000.0;
        data.longitude = myGNSS.getLongitude() / 10000000.0;
        
        // Extract altitude MSL (units: mm)
        data.altitude = myGNSS.getAltitudeMSL() / 1000.0; // Convert mm to meters
        
        // Extract speed (units: mm/s)
        data.speed = myGNSS.getGroundSpeed() / 1000.0; // Convert mm/s to m/s
        
        // Extract heading (units: degrees * 1e-5)
        data.heading = myGNSS.getHeading() / 100000.0;
        
        // Get satellite count
        data.satellites = myGNSS.getSIV(); // Satellites In View
        
        // Check fix validity (fixType: 0=no fix, 2=2D, 3=3D, 4=GNSS+DR)
        uint8_t fixType = myGNSS.getFixType();
        data.fixValid = (fixType >= 2); // 2D or 3D fix
        
        if (data.fixValid) {
            validFixes++;
        }
        
        // Update timestamp
        data.lastUpdate = millis();
        lastUpdate = millis();
        
    } else {
        // Failed to get PVT data (I2C error or GPS not ready)
        i2cErrors++;
    }
}

bool GPSManager::hasFix() {
    if (!initialized) return false;
    // Check if data is stale (older than 10 seconds)
    if (millis() - lastUpdate > 10000) return false;
    return data.fixValid;
}

bool GPSManager::isAvailable() {
    return initialized;
}

// Data Accessors
double GPSManager::getLatitude() { return data.latitude; }
double GPSManager::getLongitude() { return data.longitude; }
float GPSManager::getAltitudeMSL() { return data.altitude; }
float GPSManager::getSpeed() { return data.speed; }
float GPSManager::getHeading() { return data.heading; }
uint8_t GPSManager::getSatelliteCount() { return data.satellites; }

bool GPSManager::checkHealth() {
    // Return true if we have received ANY valid update in the last 5 seconds
    return (millis() - lastUpdate < 5000);
}

void GPSManager::printDebug() {
    DEBUG_INFO("========================================");
    DEBUG_INFO("GPS I2C DEBUG DIAGNOSTICS");
    DEBUG_INFO("========================================");
    DEBUG_INFO("I2C Configuration:");
    DEBUG_INFO("  SDA Pin: GPIO%d", GPS_I2C_SDA_PIN);
    DEBUG_INFO("  SCL Pin: GPIO%d", GPS_I2C_SCL_PIN);
    DEBUG_INFO("  GPS Address: 0x%02X", GPS_I2C_ADDR);
    DEBUG_INFO("");
    
    DEBUG_INFO("Communication Statistics:");
    DEBUG_INFO("  Total Updates: %lu", totalUpdates);
    DEBUG_INFO("  Valid Fixes: %lu", validFixes);
    DEBUG_INFO("  I2C Errors: %lu", i2cErrors);
    DEBUG_INFO("");
    
    DEBUG_INFO("GPS Data Status:");
    DEBUG_INFO("  Fix Valid: %s", data.fixValid ? "YES" : "NO");
    DEBUG_INFO("  Fix Type: %d", myGNSS.getFixType());
    DEBUG_INFO("  Satellites: %d", data.satellites);
    DEBUG_INFO("  Last Update: %lu ms ago", millis() - lastUpdate);
    DEBUG_INFO("  Latitude: %.9f°", data.latitude);
    DEBUG_INFO("  Longitude: %.9f°", data.longitude);
    DEBUG_INFO("  Altitude: %.2f m", data.altitude);
    DEBUG_INFO("  Speed: %.2f m/s", data.speed);
    DEBUG_INFO("  Heading: %.2f°", data.heading);
    DEBUG_INFO("");
    
    // Diagnosis
    if (!initialized) {
        DEBUG_ERROR("❌ GPS NOT INITIALIZED!");
    } else if (totalUpdates == 0) {
        DEBUG_WARN("⚠ No update() calls yet");
    } else if (i2cErrors > totalUpdates / 2) {
        DEBUG_WARN("⚠ High I2C error rate: %lu/%lu", i2cErrors, totalUpdates);
        DEBUG_WARN("  Check I2C wiring and pull-up resistors");
    } else if (data.satellites == 0) {
        DEBUG_WARN("⚠ No satellites visible");
        DEBUG_WARN("  GPS module may need clear sky view");
    } else if (!data.fixValid) {
        DEBUG_INFO("⏳ GPS acquiring fix... (%d sats visible)", data.satellites);
    } else {
        DEBUG_INFO("✓ GPS working correctly!");
    }
    DEBUG_INFO("========================================");
}


// Configuration Methods (Now fully functional via UBX protocol)

void GPSManager::setRefreshRate(uint8_t hz) {
    if (!initialized) {
        DEBUG_WARN("GPS not initialized, cannot set refresh rate");
        return;
    }
    
    if (hz < 1 || hz > 25) {
        DEBUG_WARN("GPS refresh rate must be 1-25 Hz, got %d", hz);
        return;
    }
    
    myGNSS.setNavigationFrequency(hz);
    myGNSS.saveConfiguration();
    DEBUG_INFO("GPS refresh rate set to %d Hz", hz);
}

void GPSManager::setConstellation(uint8_t preset) {
    if (!initialized) {
        DEBUG_WARN("GPS not initialized, cannot set constellation");
        return;
    }
    
    // 0=All, 1=GPS+GLO+GAL, 2=GPS+GLO
    switch (preset) {
        case 0: // All constellations
            myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
            myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
            myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
            myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
            DEBUG_INFO("GPS constellation: All (GPS+GLO+GAL+BDS)");
            break;
        case 1: // GPS + GLONASS + Galileo
            myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
            myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
            myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
            myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
            DEBUG_INFO("GPS constellation: GPS+GLO+GAL");
            break;
        case 2: // GPS + GLONASS only
            myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
            myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
            myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO);
            myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
            DEBUG_INFO("GPS constellation: GPS+GLO");
            break;
        default:
            DEBUG_WARN("Invalid constellation preset: %d", preset);
            return;
    }
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_NAVCONF);
}

void GPSManager::setDynamicModel(uint8_t model) {
    if (!initialized) {
        DEBUG_WARN("GPS not initialized, cannot set dynamic model");
        return;
    }
    
    // Map to u-blox dynamic models
    dynModel ubxModel;
    const char* modelName;
    
    switch (model) {
        case 0: ubxModel = DYN_MODEL_PORTABLE; modelName = "Portable"; break;
        case 2: ubxModel = DYN_MODEL_AUTOMOTIVE; modelName = "Automotive"; break;
        case 3: ubxModel = DYN_MODEL_AIRBORNE1g; modelName = "Airborne <1g"; break;
        case 5: ubxModel = DYN_MODEL_STATIONARY; modelName = "Stationary"; break;
        default:
            DEBUG_WARN("Invalid dynamic model: %d", model);
            return;
    }
    
    myGNSS.setDynamicModel(ubxModel);
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_NAVCONF);
    DEBUG_INFO("GPS dynamic model: %s", modelName);
}

void GPSManager::setSBAS(bool enable) {
    if (!initialized) {
        DEBUG_WARN("GPS not initialized, cannot set SBAS");
        return;
    }
    
    myGNSS.enableGNSS(enable, SFE_UBLOX_GNSS_ID_SBAS);
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_NAVCONF);
    DEBUG_INFO("GPS SBAS: %s", enable ? "Enabled" : "Disabled");
}

void GPSManager::setQZSS(bool enable) {
    if (!initialized) {
        DEBUG_WARN("GPS not initialized, cannot set QZSS");
        return;
    }
    
    myGNSS.enableGNSS(enable, SFE_UBLOX_GNSS_ID_QZSS);
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_NAVCONF);
    DEBUG_INFO("GPS QZSS: %s", enable ? "Enabled" : "Disabled");
}

void GPSManager::setAntiJamming(uint8_t mode) {
    if (!initialized) {
        DEBUG_WARN("GPS not initialized, cannot set anti-jamming");
        return;
    }
    
    // Note: Anti-jamming configuration is module-specific
    // NEO-M9N supports jamming/interference monitoring
    // This is a placeholder for future implementation
}

// ==============================================================================
// I2C Clock Speed Management
// ==============================================================================

void GPSManager::setI2CClockSpeed(uint16_t khz) {
    // Validate input
    if (khz != 100 && khz != 200 && khz != 400 && khz != 600 && khz != 800 && khz != 1000) {
        DEBUG_WARN("Invalid I2C clock speed: %d kHz. Must be 100, 200, 400, 600, 800, or 1000", khz);
        return;
    }
    
    currentClockSpeed = khz * 1000; // Convert to Hz
    Wire1.setClock(currentClockSpeed);
    DEBUG_INFO("GPS I2C clock set to %d kHz", khz);
}

uint16_t GPSManager::getI2CClockSpeed() {
    return currentClockSpeed / 1000; // Return in kHz
}

const char* GPSManager::getClockSpeedRecommendation(uint8_t updateHz) {
    if (updateHz <= 5) return "200 kHz";
    if (updateHz <= 10) return "400 kHz";
    if (updateHz <= 15) return "600 kHz";
    if (updateHz <= 20) return "800 kHz";
    return "1000 kHz";
}

// ==============================================================================
// Protocol Management
// ==============================================================================

void GPSManager::setProtocolMode(uint8_t mode) {
    if (!initialized) {
        DEBUG_WARN("GPS not initialized, cannot set protocol mode");
        return;
    }
    
    if (mode > 1) {
        DEBUG_WARN("Invalid protocol mode: %d. Must be 0 (UBX) or 1 (NMEA Debug)", mode);
        return;
    }
    
    // Warn user about restart requirement
    Serial.println("\n⚠ WARNING: Protocol change requires GPS restart");
    if (!promptYesNo("Restart GPS now?")) {
        Serial.println("Protocol change cancelled");
        return;
    }
    
    if (mode == 0) {
        // UBX mode (Binary only - production mode)
        myGNSS.setI2COutput(COM_TYPE_UBX);
        currentProtocolMode = 0;
        DEBUG_INFO("Switched to UBX protocol (binary only)");
    } else {
        // NMEA Debug mode (Both UBX + NMEA for debugging)
        myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);
        currentProtocolMode = 1;
        DEBUG_INFO("Switched to NMEA debug mode (UBX + NMEA)");
    }
    
    myGNSS.saveConfiguration();
    
    // Restart GPS
    if (restartGPS()) {
        Serial.println("✓ GPS restarted successfully");
    } else {
        Serial.println("❌ GPS restart failed - manual power cycle may be required");
    }
}

uint8_t GPSManager::getProtocolMode() {
    return currentProtocolMode;
}

bool GPSManager::restartGPS() {
    DEBUG_INFO("Restarting GPS...");
    
    // Software reset (GNSS only, preserves configuration)
    myGNSS.softwareResetGNSSOnly();
    delay(2000); // Wait for restart
    
    // Reconnect
    if (!myGNSS.begin(Wire1, GPS_I2C_ADDR)) {
        DEBUG_ERROR("Failed to reconnect to GPS after restart");
        return false;
    }
    
    DEBUG_INFO("GPS restarted and reconnected");
    return true;
}

// ==============================================================================
// Factory Reset
// ==============================================================================

bool GPSManager::factoryReset() {
    if (!initialized) {
        DEBUG_ERROR("GPS not initialized, cannot perform factory reset");
        return false;
    }
    
    // Display comprehensive warning
    Serial.println("\n╔══════════════════════════════════════════════════╗");
    Serial.println("║         ⚠  GPS FACTORY RESET WARNING  ⚠         ║");
    Serial.println("╚══════════════════════════════════════════════════╝");
    Serial.println("\nThis will PERMANENTLY ERASE all GPS settings:");
    Serial.println("  • Update rate (Hz)");
    Serial.println("  • Constellation configuration");
    Serial.println("  • Dynamic model");
    Serial.println("  • SBAS/QZSS settings");
    Serial.println("  • I2C protocol mode");
    Serial.println("  • All saved configurations");
    Serial.println("\nThe GPS will be restored to I2C UBX mode at 1 Hz.");
    Serial.println("You will need to reconfigure all settings via menu.\n");
    
    if (!promptYesNo("Continue with factory reset?")) {
        Serial.println("✓ Factory reset cancelled");
        return false;
    }
    
    DEBUG_INFO("Performing factory reset...");
    Serial.println("\nResetting GPS to factory defaults...");
    
    // Factory reset
    myGNSS.factoryDefault();
    delay(3000); // Wait for reset to complete
    
    // Critical: Restore I2C communication (GPS defaults to UART!)
    Serial.println("Restoring I2C communication...");
    if (!myGNSS.begin(Wire1, GPS_I2C_ADDR)) {
        Serial.println("\n❌ CRITICAL ERROR: Failed to restore I2C after factory reset!");
        Serial.println("The GPS may have reverted to UART mode.");
        Serial.println("Power cycle the device and check wiring.");
        DEBUG_ERROR("Failed to restore I2C after factory reset!");
        return false;
    }
    
    // Apply minimal config to restore I2C operation
    Serial.println("Applying minimal I2C configuration...");
    myGNSS.setI2COutput(COM_TYPE_UBX);
    myGNSS.setNavigationFrequency(1);  // 1 Hz
    myGNSS.setAutoPVT(true);
    myGNSS.saveConfiguration();
    
    // Update tracking variables
    currentProtocolMode = 0;  // UBX
    
    Serial.println("\n✓ Factory reset complete!");
    Serial.println("GPS restored to minimal I2C configuration:");
    Serial.println("  • Protocol: UBX");
    Serial.println("  • Update rate: 1 Hz");
    Serial.println("\nPlease reconfigure GPS via menu.\n");
    
    DEBUG_INFO("Factory reset complete");
    return true;
}

// ==============================================================================
// User Interaction Helpers
// ==============================================================================

bool GPSManager::promptYesNo(const char* question) {
    Serial.printf("%s (Y/N): ", question);
    Serial.flush();
    
    unsigned long timeout = millis() + 30000; // 30 second timeout
    
    while (millis() < timeout) {
        if (Serial.available()) {
            char c = Serial.read();
            // Clear any remaining input
            while (Serial.available()) Serial.read();
            
            Serial.println(c); // Echo
            
            if (c == 'Y' || c == 'y') return true;
            if (c == 'N' || c == 'n') return false;
            
            Serial.print("Invalid input. Please enter Y or N: ");
            Serial.flush();
        }
        delay(10);
        Watchdog::feed(); // Keep watchdog happy during user input
    }
    
    Serial.println("\nTimeout - assuming 'No'");
    return false;
}

