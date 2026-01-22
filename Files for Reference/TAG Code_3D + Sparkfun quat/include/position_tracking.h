#ifndef POSITION_TRACKING_H
#define POSITION_TRACKING_H

#include <Arduino.h>
#include "gps_module.h"

// Structure to hold position and orientation data
struct PositionData {
    // Position (from GPS)
    double latitude;
    double longitude;
    double altitude;
    
    // Orientation (from IMU)
    float yaw;    // heading in degrees
    float pitch;  // pitch in degrees
    float roll;   // roll in degrees
    
    // Quaternion (from IMU)
    float q0;
    float q1;
    float q2;
    float q3;
    
    // Velocity (from GPS)
    float speed_m_s;
    
    // Accuracy and status
    uint8_t fixType;
    uint8_t numSV;
    uint32_t horizontalAccuracy;
    uint32_t verticalAccuracy;
    bool valid;
    bool using9DOF;  // Whether using full 9DOF fusion or 6DOF fallback
    
    // Timestamp
    uint32_t timestamp;

    // --- CesiumJS visualization fields ---
    float raw_heading;       // Magnetic/IMU heading, uncorrected for tilt/declination
    float true_heading;      // Yaw + mag declination
    float gps_heading;       // GPS course over ground
    float mag_declination;   // Magnetic declination (degrees)
    float hdop;              // Horizontal dilution of precision (if available)
};

// Initialize position tracking
void initPositionTracking();

// Update position data with latest GPS and IMU information
void updatePositionData(const GPSData& gps, float yaw, float pitch, float roll, 
                       float q0, float q1, float q2, float q3, bool using9DOF, bool headingValid);

// Get the latest position data
const PositionData& getLatestPositionData();

// Format position data as JSON for visualization
String getPositionDataJson();

#endif // POSITION_TRACKING_H
