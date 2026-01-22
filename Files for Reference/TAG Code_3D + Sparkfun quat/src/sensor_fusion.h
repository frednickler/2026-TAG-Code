#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>
#include "Madgwick.h"

// Default beta value for the Madgwick filter
#define MADGWICK_BETA_DEFAULT 0.1f

// Define the sample frequency for the Madgwick filter
#define MADGWICK_SAMPLE_FREQ 100.0f

// Function prototypes for functions in sensor_fusion.cpp
void initSensorFusion(float sample_freq);

// Renamed for clarity: 9-DOF (Accel + Gyro + Mag)
void updateSensorFusion9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void getQuaternion9DOF(float &w, float &x, float &y, float &z);

// New functions for 6-DOF (Accel + Gyro only)
void updateSensorFusion6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void getQuaternion6DOF(float &w, float &x, float &y, float &z);

// Compute an instantaneous quaternion from gravity (accel) and magnetic field
// (no filtering). Returns a normalised quaternion (w,x,y,z).
void computeAccelMagQuaternion(float ax, float ay, float az,
                               float mx, float my, float mz,
                               float &w, float &x, float &y, float &z);

// Directly overwrite the 6-DOF quaternion (used by main when skipping filter)
void setQuaternion6DOF(float w, float x, float y, float z);

void getEulerAngles(float &roll, float &pitch, float &yaw); // This will use the 9-DOF result

// Functions to get Euler angles and heading (from 9-DOF filter)
float getRoll();
float getPitch();
float getYaw();
float getHeading();

#endif // SENSOR_FUSION_H
