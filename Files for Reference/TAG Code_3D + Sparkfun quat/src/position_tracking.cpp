#include "position_tracking.h"
#include <math.h>

// Convert NaN/Inf to 0 so JSON always contains valid numbers
static inline float safeFloat(float v) { return (isnan(v) || isinf(v)) ? 0.0f : v; }

// Global position data instance
static PositionData currentPosition;
static bool currentHeadingValid = false; // Tracks heading validity for JSON

// Initialize position tracking
void initPositionTracking() {
    // Initialize position data with default values
    currentPosition.latitude = 0.0;
    currentPosition.longitude = 0.0;
    currentPosition.altitude = 0.0;
    
    currentPosition.yaw = 0.0f;
    currentPosition.pitch = 0.0f;
    currentPosition.roll = 0.0f;
    
    currentPosition.q0 = 1.0f;
    currentPosition.q1 = 0.0f;
    currentPosition.q2 = 0.0f;
    currentPosition.q3 = 0.0f;
    
    currentPosition.speed_m_s = 0.0f;
    
    currentPosition.fixType = 0;
    currentPosition.numSV = 0;
    currentPosition.horizontalAccuracy = 0;
    currentPosition.verticalAccuracy = 0;
    currentPosition.valid = false;
    currentPosition.using9DOF = false;
    
    currentPosition.timestamp = 0;

    // CesiumJS fields
    currentPosition.raw_heading = 0.0f;
    currentPosition.true_heading = 0.0f;
    currentPosition.gps_heading = 0.0f;
    currentPosition.mag_declination = 0.0f;
    currentPosition.hdop = 0.0f;
}

// Update position data with latest GPS and IMU information
void updatePositionData(const GPSData& gps, float fusionYaw, float fusionPitch, float fusionRoll, float q0, float q1, float q2, float q3, bool using9DOF, bool headingValid) {
    // Update position from GPS
    currentPosition.latitude = gps.latitude;
    currentPosition.longitude = gps.longitude;
    currentPosition.altitude = gps.altitude;
    
    // Update orientation from IMU
    currentPosition.yaw = headingValid ? fusionYaw : NAN; // This is the true heading
    currentPosition.pitch = fusionPitch;
    currentPosition.roll = fusionRoll;
    currentHeadingValid = headingValid;
    
    // Update quaternion from IMU
    currentPosition.q0 = q0;
    currentPosition.q1 = q1;
    currentPosition.q2 = q2;
    currentPosition.q3 = q3;
    
    // Update velocity from GPS
    currentPosition.speed_m_s = gps.speed_m_s;
    
    // Update accuracy and status
    currentPosition.fixType = gps.fixType;
    currentPosition.numSV = gps.numSV;
    currentPosition.horizontalAccuracy = gps.horizontalAccuracy;
    currentPosition.verticalAccuracy = gps.verticalAccuracy;
    currentPosition.valid = gps.valid;
    currentPosition.using9DOF = using9DOF;
    
    // CesiumJS fields
    // 'raw_heading' is the uncompensated yaw (before declination correction). If you have a separate raw heading, set it here.
    currentPosition.raw_heading = headingValid ? fusionYaw : NAN; // Use fusionYaw as raw_heading for now
    currentPosition.true_heading = headingValid ? fusionYaw : NAN;      // Or fusionYaw + declination
    currentPosition.gps_heading = gps.heading_deg; // From GPS
    currentPosition.mag_declination = gps.magDec_deg; // From GPS or geomag
    currentPosition.hdop = gps.horizontalAccuracy / 1000.0f; // Convert mm to meters. Note: This is hAcc, not HDOP.

    // Update timestamp
    currentPosition.timestamp = millis();
    // Optionally, could add a timestamp for last valid heading
}

// Get the latest position data
const PositionData& getLatestPositionData() {
    return currentPosition;
}

// Format position data as JSON for visualization
String getPositionDataJson() {
    String json = "{";
    
    // Position
    json += "\"lat\":" + String(safeFloat(currentPosition.latitude), 7) + ",";
    json += "\"lon\":" + String(safeFloat(currentPosition.longitude), 7) + ",";
    json += "\"alt\":" + String(safeFloat(currentPosition.altitude), 2) + ",";
    
    // Orientation
    json += "\"yaw\":" + String(safeFloat(currentPosition.yaw), 2) + ",";
    json += "\"pitch\":" + String(safeFloat(currentPosition.pitch), 2) + ",";
    json += "\"roll\":" + String(safeFloat(currentPosition.roll), 2) + ",";
    
    // Quaternion
    json += "\"q0\":" + String(safeFloat(currentPosition.q0), 4) + ",";
    json += "\"q1\":" + String(safeFloat(currentPosition.q1), 4) + ",";
    json += "\"q2\":" + String(safeFloat(currentPosition.q2), 4) + ",";
    json += "\"q3\":" + String(safeFloat(currentPosition.q3), 4) + ",";
    
    // Velocity
    json += "\"speed\":" + String(safeFloat(currentPosition.speed_m_s), 2) + ",";
    // Headings for CesiumJS visualization
    json += "\"raw_heading\":" + String(safeFloat(currentPosition.raw_heading), 2) + ",";
    json += "\"true_heading\":" + String(safeFloat(currentPosition.true_heading), 2) + ",";
    json += "\"gps_heading\":" + String(safeFloat(currentPosition.gps_heading), 2) + ",";
    json += "\"mag_declination\":" + String(safeFloat(currentPosition.mag_declination), 2) + ",";
    // Optionally keep legacy heading field if needed
    // json += "\"heading\":" + String(currentPosition.yaw, 2) + ",";
    
    // Status
    json += "\"sats\":" + String(currentPosition.numSV) + ",";
    json += "\"fix\":" + String(currentPosition.fixType) + ",";
    json += "\"hdop\":" + String(safeFloat(currentPosition.hdop), 2) + ","; // Note: This is hAcc (m), not true HDOP
    json += "\"valid\":" + String(currentPosition.valid ? "true" : "false") + ",";
    json += "\"using9dof\":" + String(currentPosition.using9DOF ? "true" : "false") + ",";
    json += "\"heading_valid\":" + String(currentHeadingValid ? "true" : "false");
    // Timestamp
    json += ",\"ts\":" + String(currentPosition.timestamp);
    json += "}";
    return json;
}


