#pragma once
#include <stdint.h>

/*
 * GPS I2C Clock Speed Presets:
 * 1 - Low Speed (100kHz): Most stable, good for longer cables or noisy environments
 * 2 - Standard Speed (200kHz): Balanced performance and stability
 * 3 - Medium Speed (300kHz): Better performance with good stability
 * 4 - Fast Speed (400kHz): High performance, standard for most I2C devices
 * 5 - Ultra Fast (1MHz): Maximum throughput, may be unstable with some hardware
 */
#define GPS_CLOCK_PRESET_LOW      1  // 100 kHz
#define GPS_CLOCK_PRESET_STANDARD 2  // 200 kHz
#define GPS_CLOCK_PRESET_MEDIUM   3  // 300 kHz
#define GPS_CLOCK_PRESET_FAST     4  // 400 kHz (default)
#define GPS_CLOCK_PRESET_ULTRA    5  // 1000 kHz (1 MHz)

// Select the desired GPS I2C clock speed preset (1-5)
#define GPS_CLOCK_PRESET GPS_CLOCK_PRESET_FAST

/*
 * GPS Refresh Rate Presets:
 * 1 - Low Rate (1Hz): Lowest power consumption, sufficient for slow-moving applications
 * 2 - Standard Rate (5Hz): Good balance of update frequency and power usage
 * 3 - Medium Rate (10Hz): Better tracking of movement, higher power usage
 * 4 - Fast Rate (20Hz): High-frequency updates for fast-moving objects
 * 5 - Ultra Fast (40Hz): Maximum update rate, highest power consumption
 *     Note: Not all GPS modules support rates above 10Hz
 */
#define GPS_REFRESH_PRESET_LOW      1  // 1 Hz
#define GPS_REFRESH_PRESET_STANDARD 2  // 5 Hz
#define GPS_REFRESH_PRESET_MEDIUM   3  // 10 Hz (default)
#define GPS_REFRESH_PRESET_FAST     4  // 20 Hz
#define GPS_REFRESH_PRESET_ULTRA    5  // 40 Hz

// Select the desired GPS refresh rate preset (1-5)
#define GPS_REFRESH_PRESET GPS_REFRESH_PRESET_MEDIUM

struct GPSData {
    double latitude;
    double longitude;
    double altitude;
    float speed_m_s;
    float heading_deg;
    float magDec_deg; // Magnetic declination
    bool magDecFromGPS; // true = value from UBX message, false = locally computed (WMM approx)
    uint8_t fixType;
    uint8_t numSV;
    uint32_t horizontalAccuracy;
    uint32_t verticalAccuracy;
    uint32_t itow;
    bool valid;
};

// Call once in setup
bool setupGPS();

// Call regularly in loop
bool updateGPS();

// Get latest parsed GPS data
const GPSData& getLatestGPSData();

// Get the current GPS configuration status
struct GPSConfigStatus {
    bool isConnected;
    uint8_t actualClockSpeed; // 1-5 preset
    uint8_t actualRefreshRate; // 1-5 preset
    bool isConfigured;
    uint8_t initAttempts;
};

const GPSConfigStatus& getGPSConfigStatus();
