#ifndef POSITION_TRACKING_H
#define POSITION_TRACKING_H

#include <Arduino.h>
#include "gps_module.h"

struct PositionData {
    float latitude;
    float longitude;
    float altitude;
    float speed_m_s;
    float gps_heading;
    float fusion_heading;
    float mag_declination;
    float hdop;
    uint8_t fixType;
    uint8_t numSV;
    long horizontalAccuracy;
    long verticalAccuracy;
    bool valid;
    float yaw;
    float pitch;
    float roll;
    float q0;
    float q1;
    float q2;
    float q3;
    bool using9DOF;
    unsigned long timestamp;
    float raw_heading;
    float true_heading;
};

void initPositionTracking();
void updatePositionData(const GPSData& gps, float fusionYaw, float fusionPitch, float fusionRoll, float q0, float q1, float q2, float q3, bool using9DOF, bool headingValid);
const PositionData& getLatestPositionData();
String getPositionDataJson();

#endif // POSITION_TRACKING_H