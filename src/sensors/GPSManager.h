#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include "../config/SystemConfig.h"
#include "NMEAParser.h" // Switched from SparkFun library to custom NMEA parser

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
 * @brief GPS Manager (NEO-M9N) - NMEA MODE
 * 
 * Reverted to simple NMEA parsing because the RX line appears broken/unreliable,
 * preventing bidirectional communication required by the SparkFun UBX library.
 * This ensures we can still receive data even if valid control packets cannot be sent.
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

    // Configuration Methods (Stubs / NMEA commands)
    static void setRefreshRate(uint8_t hz); 
    static void setConstellation(uint8_t preset); // 0=All, 1=GPS+GLO+GAL, 2=GPS+GLO
    static void setDynamicModel(uint8_t model);   // 0=Portable, 2=Auto, 3=Air, 5=Stationary
    static void setSBAS(bool enable);
    static void setQZSS(bool enable);
    static void setAntiJamming(uint8_t mode);     // 0=No, 1=Fixed, 2=Adaptive

private:
    static GPSData data;
    static bool initialized;
    static unsigned long lastUpdate;
    
    // Buffer for NMEA sentences
    static String nmeaBuffer;
    
    // Internal parser data
    static NMEAData tempParserData;
};

#endif // GPS_MANAGER_H
