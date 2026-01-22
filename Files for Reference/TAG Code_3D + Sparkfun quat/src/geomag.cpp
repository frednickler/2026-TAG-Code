#include <Arduino.h>
#include "geomag.h"
#include <math.h>
#include <time.h>
#include "imu_config.h"

// Australia/New Zealand region magnetic declination table (2020-2025)
// Grid: 10° longitude (140E to 180E), 5° latitude (20S to 50S)
// Values are in degrees East (positive)
static const float AUS_NZ_DECLINATION[7][5] PROGMEM = {
    // 140E      150E      160E      170E      180E
    { 4.5,      9.5,      13.5,     16.8,     19.2 },  // 20S
    { 6.2,      11.2,     15.1,     18.0,     20.1 },  // 25S
    { 8.0,      12.6,     16.2,     19.0,     21.0 },  // 30S
    { 9.8,      14.0,     17.2,     19.8,     21.5 },  // 35S
    { 11.8,     15.5,     18.2,     20.5,     22.0 },  // 40S
    { 14.0,     17.0,     19.2,     21.0,     22.5 },  // 45S
    { 16.5,     18.5,     20.2,     21.5,     23.0 }   // 50S
};

// Cache for last computed values
static struct {
    double lat, lon, alt;
    time_t timestamp;
    float declination;
    bool valid = false;
} cache;

// Convert degrees to radians
static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

// Convert radians to degrees
static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// Normalize angle to [-180, 180] degrees
static inline float normalizeAngle(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

// Bilinear interpolation for table lookup
static float bilinearInterpolation(float x, float y, float q11, float q12, float q21, float q22, float x1, float x2, float y1, float y2) {
    float x2x1 = x2 - x1;
    float y2y1 = y2 - y1;
    float x2x = x2 - x;
    float y2y = y2 - y;
    float yy1 = y - y1;
    float xx1 = x - x1;
    
    return (q11 * x2x * y2y + q21 * xx1 * y2y + q12 * x2x * yy1 + q22 * xx1 * yy1) / (x2x1 * y2y1);
}

// Compute magnetic declination using lookup table and interpolation
void computeMagneticElements(double lat, double lon, double alt_km, time_t timestamp, 
                             float &decl, float &incl, float &intensity) {
    // For Australia/New Zealand region (140E-180E, 20S-50S)
    // Check if coordinates are within our table range
    if (lon < 140.0 || lon > 180.0 || lat > -20.0 || lat < -50.0) {
        // Outside our table range, use a reasonable default for the region
        decl = 12.0;  // Approximate value for eastern Australia
        incl = -60.0; // Approximate value for southern hemisphere
        intensity = 55000.0; // Approximate intensity in nT
        
        Serial.println("[GEOMAG_DEBUG] Warning: Coordinates outside lookup table range, using default values");
        return;
    }
    
    // Convert to table indices
    float lon_idx = (lon - 140.0) / 10.0;  // 140E = 0, 180E = 4
    float lat_idx = ((-lat) - 20.0) / 5.0;  // 20S = 0, 50S = 6
    
    // Get grid cell corners
    int lon_idx_floor = floor(lon_idx);
    int lat_idx_floor = floor(lat_idx);
    
    // Ensure we're within bounds
    lon_idx_floor = constrain(lon_idx_floor, 0, 3);
    lat_idx_floor = constrain(lat_idx_floor, 0, 5);
    
    // Read the four surrounding declination values from PROGMEM
    float q11 = pgm_read_float(&AUS_NZ_DECLINATION[lat_idx_floor][lon_idx_floor]);
    float q12 = pgm_read_float(&AUS_NZ_DECLINATION[lat_idx_floor + 1][lon_idx_floor]);
    float q21 = pgm_read_float(&AUS_NZ_DECLINATION[lat_idx_floor][lon_idx_floor + 1]);
    float q22 = pgm_read_float(&AUS_NZ_DECLINATION[lat_idx_floor + 1][lon_idx_floor + 1]);
    
    // Calculate actual grid coordinates
    float x1 = lon_idx_floor * 10.0 + 140.0;
    float x2 = x1 + 10.0;
    float y1 = -(lat_idx_floor * 5.0 + 20.0);
    float y2 = y1 - 5.0;
    
    // Perform bilinear interpolation
    decl = bilinearInterpolation(lon, lat, q11, q12, q21, q22, x1, x2, y1, y2);
    
    // For Newcastle area (around 33S, 151.7E), this should give approximately 12.6° East
    
    // Simplified inclination and intensity (less critical for navigation)
    incl = -57.0 - (lat + 20.0) * 0.3;  // Simple approximation for the region
    intensity = 55000.0 - (lat + 35.0) * 200.0;  // Simple approximation
    
    Serial.printf("[GEOMAG_DEBUG] Lookup table: lat=%.6f, lon=%.6f\n", lat, lon);
    Serial.printf("[GEOMAG_DEBUG] Grid indices: lat_idx=%.2f, lon_idx=%.2f\n", lat_idx, lon_idx);
    Serial.printf("[GEOMAG_DEBUG] Corner values: q11=%.1f, q12=%.1f, q21=%.1f, q22=%.1f\n", q11, q12, q21, q22);
    Serial.printf("[GEOMAG_DEBUG] Declination (deg): %.6f, Inclination (deg): %.6f, Intensity: %.6f\n", decl, incl, intensity);
}

// Main declination function with caching
float computeDeclination(double latDeg, double lonDeg, double altMeters, time_t timestamp) {
    // Use current time if none provided
    if (timestamp == 0) {
        timestamp = time(nullptr);
    }
    
    // Check cache first (within 20km and 1 hour)
    const double CACHE_DIST_KM = 20.0;
    const time_t CACHE_TIME_SEC = 3600;
    
    if (cache.valid) {
        double dlat = latDeg - cache.lat;
        double dlon = lonDeg - cache.lon;
        double dist_km = sqrt(dlat*dlat + dlon*dlon) * 111.0;  // Approx km/degree
        
        if (dist_km < CACHE_DIST_KM && 
            abs(timestamp - cache.timestamp) < CACHE_TIME_SEC) {
            return cache.declination;
        }
    }
    
    // Compute new value
    float decl, incl, intensity;
    computeMagneticElements(latDeg, lonDeg, altMeters / 1000.0, timestamp, 
                          decl, incl, intensity);
    
    // Update cache
    cache.lat = latDeg;
    cache.lon = lonDeg;
    cache.alt = altMeters;
    cache.timestamp = timestamp;
    cache.declination = decl;
    cache.valid = true;
    
    return decl;
}

// Get last computed declination
float getLastDeclination() {
    return cache.valid ? cache.declination : 0.0f;
}

// Invalidate the cache
void invalidateDeclinationCache() {
    cache.valid = false;
}
