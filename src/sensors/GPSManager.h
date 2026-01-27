#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_u-blox_GNSS_v3.h"
#include "../config/SystemConfig.h"

// Separate I2C bus for GPS (GPIO 8/9) - different from sensor bus (GPIO 4/7)
extern TwoWire Wire1;

// Struct to hold processed GPS data
struct GPSData {
    double latitude;
    double longitude;
    float altitude; // MSL
    float speed;
    float heading;
    uint8_t satellites;
    bool fixValid;
    unsigned long lastUpdate;
};

/**
 * @brief GPS Manager (NEO-M9N) - I2C MODE
 * 
 * Uses SparkFun u-blox GNSS library for I2C communication.
 * GPS communicates on I2C bus at address 0x42.
 * Supports full bidirectional configuration via UBX protocol.
 */
class GPSManager {
public:
    static bool init();
    static void update();
    static bool hasFix();
    static bool isAvailable();
    
    // Data Access
    static double getLatitude();
    static double getLongitude();
    static float getAltitudeMSL();
    static float getSpeed();
    static float getHeading();
    static uint8_t getSatelliteCount();
    static bool checkHealth();
    
    // Debug / Diagnostics
    static void printDebug();

    // Configuration Methods (Now fully functional via UBX)
    static void setRefreshRate(uint8_t hz); 
    static void setConstellation(uint8_t preset); // 0=All, 1=GPS+GLO+GAL, 2=GPS+GLO
    static void setDynamicModel(uint8_t model);   // 0=Portable, 2=Auto, 3=Air, 5=Stationary
    static void setSBAS(bool enable);
    static void setQZSS(bool enable);
    static void setAntiJamming(uint8_t mode);     // 0=No, 1=Fixed, 2=Adaptive
    
    // I2C Clock Speed Management
    static void setI2CClockSpeed(uint16_t khz);   // 100, 200, 400, 600, 800, 1000
    static uint16_t getI2CClockSpeed();
    static const char* getClockSpeedRecommendation(uint8_t updateHz);
    
    // Protocol Management  
    static void setProtocolMode(uint8_t mode);    // 0=UBX, 1=NMEA Debug
    static uint8_t getProtocolMode();
    static bool restartGPS();                     // For protocol switching
    
    // Factory Reset
    static bool factoryReset();                   // Returns true if reset succeeded
    
    // User Interaction Helpers
    static bool promptYesNo(const char* question);

private:
    static SFE_UBLOX_GNSS myGNSS;
    static GPSData data;
    static bool initialized;
    static unsigned long lastUpdate;
    
    // Debug statistics
    static unsigned long totalUpdates;
    static unsigned long validFixes;
    static unsigned long i2cErrors;
    
    // Configuration tracking
    static uint32_t currentClockSpeed;  // Current I2C clock in Hz (was uint16_t, caused overflow)
    static uint8_t currentProtocolMode; // 0=UBX, 1=NMEA
};

#endif // GPS_MANAGER_H
