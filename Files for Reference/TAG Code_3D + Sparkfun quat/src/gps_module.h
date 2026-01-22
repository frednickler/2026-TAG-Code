#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#define GPS_REFRESH_PRESET GPS_REFRESH_PRESET_FAST
#define GPS_CLOCK_PRESET   GPS_CLOCK_PRESET_FAST

#define GPS_CLOCK_PRESET_LOW      0
#define GPS_CLOCK_PRESET_STANDARD 1
#define GPS_CLOCK_PRESET_MEDIUM   2
#define GPS_CLOCK_PRESET_FAST     3
#define GPS_CLOCK_PRESET_ULTRA    4

#define GPS_REFRESH_PRESET_LOW      0
#define GPS_REFRESH_PRESET_STANDARD 1
#define GPS_REFRESH_PRESET_MEDIUM   2
#define GPS_REFRESH_PRESET_FAST     3
#define GPS_REFRESH_PRESET_ULTRA    4

struct GPSData {
    double latitude;
    double longitude;
    double altitude;
    double speed_m_s;
    double heading_deg;
    uint8_t fixType;
    uint8_t numSV;
    double horizontalAccuracy;
    double verticalAccuracy;
    bool valid;
    double magDec_deg;
    bool magDecFromGPS;
};

struct GPSConfigStatus {
    bool success;
    uint32_t clockFreq;
    uint8_t refreshRate;
    bool gnssConfigSuccess;
    uint16_t gnssMessages;
    bool isConfigured;
    uint32_t actualClockSpeed;
    uint8_t actualRefreshRate;
};

bool setupGPS();
bool updateGPS();
const GPSData& getLatestGPSData();
const GPSConfigStatus& getGPSConfigStatus();
void setGPSRefreshRate(uint8_t hz);
void setGPSConstellation(uint8_t preset);
void setGPSDynamicModel(uint8_t model);
void setGPSI2CClock(uint8_t mode);
void setGPSSBAS(bool enable);
void setGPSQZSS(bool enable);
void setGPSAntiJamming(uint8_t mode);

#endif // GPS_MODULE_H