#include "gps_module.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include "config.h" 
#include "imu_config.h"
#include "geomag.h"  // Local declination computation

#define GPS_INIT_MAX_ATTEMPTS 3

SFE_UBLOX_GNSS myGNSS;
static GPSData latestData = {};
static GPSConfigStatus configStatus = {};

// Cache last computed declination to avoid frequent calculations
static struct {
    double lat = 999.0, lon = 999.0, alt = 0;
    time_t timestamp = 0;
    float declination = 0.0f;
    bool valid = false;
} declinationCache;

// Convert preset to actual clock speed in Hz
uint32_t getClockSpeedFromPreset(uint8_t preset) {
    switch (preset) {
        case GPS_CLOCK_PRESET_LOW:      return 100000;  // 100 kHz
        case GPS_CLOCK_PRESET_STANDARD: return 200000;  // 200 kHz
        case GPS_CLOCK_PRESET_MEDIUM:   return 300000;  // 300 kHz
        case GPS_CLOCK_PRESET_FAST:     return 400000;  // 400 kHz
        // case GPS_CLOCK_PRESET_ULTRA:    return 1000000; // REMOVED FOR STABILITY
        default:                        return 400000;  // Default to 400 kHz
    }
}

// Verify if a configuration setting was applied successfully
bool verifyConfig(const char* settingName, bool success, uint32_t expected, uint32_t actual) {
    if (!success) {
        Serial.printf("[GPS] Failed to set %s\n", settingName);
        return false;
    }
    
    if (expected != actual) {
        Serial.printf("[GPS] %s verification failed: expected %lu, got %lu\n", 
                     settingName, expected, actual);
        return false;
    }
    
    Serial.printf("[GPS] %s verified: %lu\n", settingName, actual);
    return true;
}

// Convert preset to refresh rate in Hz
uint8_t getRefreshRateFromPreset(uint8_t preset) {
    switch (preset) {
        case GPS_REFRESH_PRESET_LOW:      return 1;  // 1 Hz
        case GPS_REFRESH_PRESET_STANDARD: return 5;  // 5 Hz
        case GPS_REFRESH_PRESET_MEDIUM:   return 10; // 10 Hz
        case GPS_REFRESH_PRESET_FAST:     return 20; // 20 Hz
        // case GPS_REFRESH_PRESET_ULTRA:    return 40; // REMOVED FOR STABILITY
        default:                          return 10; // Default to 10 Hz
    }
}

bool setupGPS() {
    // Reset config status
    configStatus = {false, 0, 0, false, 0};
    
    // Wire1 is initialized in main.cpp, we just set the clock speed here
    
    // Set I2C clock speed based on preset
    uint32_t clockSpeed = getClockSpeedFromPreset(GPS_CLOCK_PRESET);
    Wire1.setClock(clockSpeed);
    
    Serial.println("[GPS] Initializing...");
    
    // Initialize GPS with retries
    bool gpsFound = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        // begin() returns true if the module responds to I2C ping
        if (myGNSS.begin(Wire1, GPS_I2C_ADDR)) { 
            gpsFound = true;
            break;
        }
        Serial.printf("[GPS] Initialization attempt %d/3 failed\n", attempt);
        delay(500);
    }
    
    if (!gpsFound) {
        Serial.println("[GPS] Failed to communicate (Check wiring or address!)");
        return false;
    }
    
    Serial.println("[GPS] Module detected! Checking current state...");

    // SMART INIT: Check if GPS is already running and valid (e.g. after soft reset)
    // We try multiple times because the GPS might be between updates.
    // If we receive ANY valid PVT packet, we assume the GPS is happy and skip config.
    bool existingDataFound = false;
    for (int i = 0; i < 20; i++) {
        myGNSS.checkUblox(); // Process incoming bytes
        if (myGNSS.getPVT()) {
            existingDataFound = true;
            break;
        }
        delay(50); // Scan for 1 second total
    }

    if (existingDataFound) {
        uint8_t fixType = myGNSS.getFixType();
        uint8_t siv = myGNSS.getSIV();
        Serial.printf("[GPS] SMART INIT: Module already active! (Fix: %d, SIV: %d)\n", fixType, siv);
        Serial.println("[GPS] Skipping configuration to preserve satellite lock.");
        
        // Ensure strictly necessary settings are re-asserted just in case without full reset
        // myGNSS.setNavigationFrequency(getRefreshRateFromPreset(GPS_REFRESH_PRESET)); 

        configStatus.isConfigured = true;
        configStatus.actualClockSpeed = GPS_CLOCK_PRESET;
        return true;
    }
    
    Serial.println("[GPS] No active data stream detected after 1s. Performing full configuration...");
    
    // --- FULL CONFIGURATION (Only runs if "Cold Start") ---

    // Set update rate with verification
    uint8_t refreshRate = getRefreshRateFromPreset(GPS_REFRESH_PRESET);
    bool refreshRateSet = myGNSS.setNavigationFrequency(refreshRate); // Returns true if command accepted
    
    // Verify navigation frequency was set correctly
    uint8_t actualRate = myGNSS.getNavigationFrequency();
    configStatus.actualRefreshRate = actualRate;
    // Consider refresh rate set only if both command accepted and readback matches
    refreshRateSet = refreshRateSet && (actualRate == refreshRate);
    
    // Disable all NMEA messages on I2C port to reduce bandwidth
    myGNSS.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C);
    myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_I2C);
    myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
    myGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);
    myGNSS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_I2C);
    myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_I2C);
    myGNSS.disableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_I2C);
    delay(100);

    // Enable only UBX-NAV-PVT message for efficient position data
    bool pvtEnabled = myGNSS.setAutoPVT(true, true);
    delay(100);

    // Set dynamic model to Pedestrian (better for low speed/walking)
    // DYN_MODEL_PEDESTRIAN = 3 (Low speed, low acceleration)
    myGNSS.setDynamicModel(DYN_MODEL_PEDESTRIAN);

    // Confirm configuration by trying to read a PVT message
    myGNSS.checkUblox();
    myGNSS.checkCallbacks();
    
    // Try to get a PVT message to confirm configuration
    bool pvtAvailable = myGNSS.getPVT();
    
    // Update configuration status
    // isConfigured = refresh rate set, PVT enabled, and PVT message available
    configStatus.isConfigured = refreshRateSet && pvtEnabled && pvtAvailable;
    configStatus.actualClockSpeed = GPS_CLOCK_PRESET;
    
    if (configStatus.isConfigured) {
        Serial.println("[GPS] Configuration successful: Pedestrian Mode");
    } else {
        Serial.println("[GPS] Configuration WARNING: PVT check failed (Wait for lock?)");
        // We return true anyway if we established comms, as fix might just take time
        return true; 
    }

    return configStatus.isConfigured;
}

bool updateGPS() {
    static uint32_t lastValidUpdate = 0;
    
    myGNSS.checkUblox();
    myGNSS.checkCallbacks();
    
    if (myGNSS.getPVT()) {
        lastValidUpdate = millis();
        latestData.latitude = myGNSS.getLatitude() / 1e7;
        latestData.longitude = myGNSS.getLongitude() / 1e7;
        latestData.altitude = myGNSS.getAltitude() / 1000.0;
        latestData.speed_m_s = myGNSS.getGroundSpeed() / 1000.0;
        latestData.heading_deg = myGNSS.getHeading() / 100000.0; // Convert to degrees

        // Select declination source based on user flag
        // USE_WMM_DECLINATION == 1: always use WMM/NOAA model (default)
        // USE_WMM_DECLINATION == 0: use GPS-provided value if valid, else fallback to WMM
        time_t currentTime = 0;
        struct tm timeinfo;
        if (myGNSS.getDateValid() && myGNSS.getTimeValid()) {
            timeinfo.tm_year = myGNSS.getYear() - 1900;
            timeinfo.tm_mon = myGNSS.getMonth() - 1;
            timeinfo.tm_mday = myGNSS.getDay();
            timeinfo.tm_hour = myGNSS.getHour();
            timeinfo.tm_min = myGNSS.getMinute();
            timeinfo.tm_sec = myGNSS.getSecond();
            currentTime = mktime(&timeinfo);
        } else {
            currentTime = time(nullptr);
        }
        #if USE_WMM_DECLINATION
        // --- WMM/NOAA model (default) ---
        bool updateCache = false;
        if (!declinationCache.valid) {
            updateCache = true;
        } else {
            double dLat = fabs(latestData.latitude - declinationCache.lat);
            double dLon = fabs(latestData.longitude - declinationCache.lon);
            double distKm = sqrt(dLat * dLat + dLon * dLon) * 111.0;
            if (distKm > 20.0 || (currentTime - declinationCache.timestamp) > 3600) {
                updateCache = true;
            }
        }
        if (updateCache) {
            declinationCache.lat = latestData.latitude;
            declinationCache.lon = latestData.longitude;
            declinationCache.alt = latestData.altitude;
            declinationCache.timestamp = currentTime;
            declinationCache.declination = computeDeclination(
                latestData.latitude, 
                latestData.longitude, 
                latestData.altitude,
                currentTime
            );
            declinationCache.valid = true;
        }
        latestData.magDec_deg = declinationCache.declination;
        latestData.magDecFromGPS = false;
        #else
        // --- GPS-provided value if valid, else fallback to WMM ---
        int32_t rawMagDec = myGNSS.getMagDec();
        if (rawMagDec != 2147483647 && rawMagDec != 0) {
            latestData.magDec_deg = rawMagDec / 100.0f;
            latestData.magDecFromGPS = true;
        } else {
            // Fallback to WMM
            bool updateCache = false;
            if (!declinationCache.valid) {
                updateCache = true;
            } else {
                double dLat = fabs(latestData.latitude - declinationCache.lat);
                double dLon = fabs(latestData.longitude - declinationCache.lon);
                double distKm = sqrt(dLat * dLat + dLon * dLon) * 111.0;
                if (distKm > 20.0 || (currentTime - declinationCache.timestamp) > 3600) {
                    updateCache = true;
                }
            }
            if (updateCache) {
                declinationCache.lat = latestData.latitude;
                declinationCache.lon = latestData.longitude;
                declinationCache.alt = latestData.altitude;
                declinationCache.timestamp = currentTime;
                declinationCache.declination = computeDeclination(
                    latestData.latitude, 
                    latestData.longitude, 
                    latestData.altitude,
                    currentTime
                );
                declinationCache.valid = true;
            }
            latestData.magDec_deg = declinationCache.declination;
            latestData.magDecFromGPS = false;
        }
        #endif
        latestData.fixType = myGNSS.getFixType();
        latestData.numSV = myGNSS.getSIV();
        latestData.horizontalAccuracy = myGNSS.getHorizontalAccEst();
        latestData.verticalAccuracy = myGNSS.getVerticalAccEst();
        // latestData.itow = 0; // No direct ITOW accessor in new library
        latestData.valid = (latestData.fixType >= 3); // 3D fix or better
        return true;
    }
    
    // Watchdog: If no data for > 2 seconds, attempt recovery
    static uint32_t lastRecoveryAttempt = 0;
    if (millis() - lastValidUpdate > 2000 && millis() - lastRecoveryAttempt > 5000) {
        Serial.println("[GPS] Watchdog: No data for 2s. Attempting recovery...");
        // Re-initialize I2C and GPS
        setupGPS(); 
        lastRecoveryAttempt = millis();
        lastValidUpdate = millis(); // Reset timer to give it a chance
    }
    
    return false;
}

const GPSData& getLatestGPSData() {
    return latestData;
}

const GPSConfigStatus& getGPSConfigStatus() {
    return configStatus;
}

void setGPSRefreshRate(uint8_t hz) {
    if (myGNSS.setNavigationFrequency(hz)) {
        Serial.printf("[GPS] Refresh Rate set to %d Hz\n", hz);
        configStatus.actualRefreshRate = hz;
    } else {
        Serial.println("[GPS] Failed to set Refresh Rate");
    }
}
void setGPSConstellation(uint8_t preset) {
    // 0=All, 1=GPS+GLO+GAL, 2=GPS+GLO, 3=GPS Only
    bool gps = true;
    bool glo = (preset <= 2);
    bool gal = (preset <= 1);
    bool bds = (preset == 0);
    
    // U-Blox Gen 9 uses detailed configuration keys, but SparkFun lib provides `enableGNSS` helper
    // Valid IDs: SFE_UBLOX_GNSS_ID_GPS, _SBAS, _GALILEO, _BEIDOU, _IMES, _QZSS, _GLONASS
    
    myGNSS.enableGNSS(gps, SFE_UBLOX_GNSS_ID_GPS);
    myGNSS.enableGNSS(glo, SFE_UBLOX_GNSS_ID_GLONASS);
    myGNSS.enableGNSS(gal, SFE_UBLOX_GNSS_ID_GALILEO);
    myGNSS.enableGNSS(bds, SFE_UBLOX_GNSS_ID_BEIDOU);
    
    // Note: We don't touch QZSS/SBAS here as they have their own toggles
    Serial.printf("[GPS] Constellation preset %d applied.\n", preset);
}

void setGPSDynamicModel(uint8_t model) {
    dynModel modelEnum;
    switch(model) {
        case 0: modelEnum = DYN_MODEL_PORTABLE; break;
        case 1: modelEnum = DYN_MODEL_PEDESTRIAN; break;
        case 2: modelEnum = DYN_MODEL_AUTOMOTIVE; break;
        case 3: modelEnum = DYN_MODEL_AIRBORNE1g; break; // <1g
        case 4: modelEnum = DYN_MODEL_SEA; break;
        default: modelEnum = DYN_MODEL_PORTABLE;
    }
    if (myGNSS.setDynamicModel(modelEnum)) {
        Serial.printf("[GPS] Dynamic Model set to %d.\n", model);
    } else {
        Serial.println("[GPS] Failed to set Dynamic Model.");
    }
}

void setGPSI2CClock(uint8_t mode) {
    uint32_t speed = 100000;
    switch(mode) {
        case 0: speed = 50000; break;  // Low
        case 1: speed = 100000; break; // Standard
        case 2: speed = 200000; break; // High
        case 3: speed = 400000; break; // Fast
        default: speed = 100000;
    }
    Wire1.setClock(speed);
    Serial.printf("[GPS] I2C Clock set to %d Hz.\n", speed);
}

void setGPSSBAS(bool enable) {
    if (myGNSS.enableGNSS(enable, SFE_UBLOX_GNSS_ID_SBAS)) {
        Serial.printf("[GPS] SBAS %s.\n", enable ? "Enabled" : "Disabled");
    } else {
        Serial.println("[GPS] Failed to set SBAS.");
    }
}

void setGPSQZSS(bool enable) {
    if (myGNSS.enableGNSS(enable, SFE_UBLOX_GNSS_ID_QZSS)) {
        Serial.printf("[GPS] QZSS %s.\n", enable ? "Enabled" : "Disabled");
    } else {
        Serial.println("[GPS] Failed to set QZSS.");
    }
}

void setGPSAntiJamming(uint8_t mode) {
    // 0=No, 1=Fixed, 2=Adaptive
    // Not all M9N modules expose simple API for this, might need raw UBX.
    Serial.println("[GPS] Anti-Jamming configuration requires library support (Not Implemented).");
}
