#ifndef ZUPT_H
#define ZUPT_H

#include <Arduino.h>

/**
 * @brief Initialize ZUPT system
 */
void initZUPT();

/**
 * @brief Check if device is stationary and detect zero velocity
 * 
 * @param ax Accelerometer X (m/s^2 or g)
 * @param ay Accelerometer Y (m/s^2 or g)
 * @param az Accelerometer Z (m/s^2 or g)
 * @param gx Gyroscope X (rad/s or deg/s)
 * @param gy Gyroscope Y (rad/s or deg/s)
 * @param gz Gyroscope Z (rad/s or deg/s)
 * @return true if device is detected as stationary
 */
bool isdeviceStationary(float ax, float ay, float az, float gx, float gy, float gz);

/**
 * @brief Apply Zero Velocity Update correction
 * 
 * If the device is detected as stationary, this function forces the 
 * gyroscope readings to zero to prevent integration drift.
 * 
 * @param gx Gyroscope X (reference, will be modified)
 * @param gy Gyroscope Y (reference, will be modified)
 * @param gz Gyroscope Z (reference, will be modified)
 * @param ax Accelerometer X
 * @param ay Accelerometer Y
 * @param az Accelerometer Z
 */
void applyZUPT(float& gx, float& gy, float& gz, float ax, float ay, float az);

#endif // ZUPT_H
