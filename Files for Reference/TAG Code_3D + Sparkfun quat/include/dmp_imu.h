#ifndef DMP_IMU_H
#define DMP_IMU_H

#include <Arduino.h>
#include "Arduino-ICM20948.h" // isouriadakis library

// Initialize the DMP
bool initDMP();

// Poll the DMP for new data (call in loop)
bool updateDMP();

// Get the latest quaternion (w, x, y, z)
// Returns true if new data was available since last read
bool readDMPQuaternion(float &qw, float &qx, float &qy, float &qz);

// Get calibrated sensor data (accel in g, gyro in dps, mag in uT)
bool readDMPCalibratedSensors(float &ax, float &ay, float &az, 
                              float &gx, float &gy, float &gz, 
                              float &mx, float &my, float &mz);

// Get the latest heading (0-360)
float getDMPHeading();

// Get hybrid heading using DMP Pitch/Roll + corrected Magnetometer (0-360)
// This is the recommended heading source for upside-down sensor mounting
float getHybridHeading();

// Check if DMP is initialized
bool isDMPInitialized();

#endif
