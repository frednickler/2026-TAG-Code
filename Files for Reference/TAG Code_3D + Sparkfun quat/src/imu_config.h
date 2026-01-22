/**
 * @file imu_config.h
 * @brief Configuration and interface for the ICM-20948 IMU with AK09916 magnetometer
 * 
 * This module provides a high-level interface for configuring and reading from the
 * ICM-20948 9-axis IMU and its integrated AK09916 magnetometer. It handles all the
 * low-level I2C communication, sensor configuration, and axis mapping to provide
 * consistent sensor readings in a right-handed coordinate system.
 * 
 * Key Features:
 * - Unified axis mapping across all sensors
 * - Configurable digital low-pass filters (DLPF)
 * - Multiple output data rate (ODR) options
 * - Temperature-compensated sensor readings
 * - Automatic error recovery
 * 
 * @version 1.3.0
 * @date 2025-07-20
 */

#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#include <Arduino.h>
#include <Wire.h>

/**
 * @defgroup imu_config IMU Configuration
 * @brief Configuration and interface for the ICM-20948 IMU
 * @{
 */

// =============================================
// User-Configurable Settings
// =============================================

/**
 * @brief Digital Low-Pass Filter (DLPF) configuration for accelerometer
 * 
 * Available settings (0-7):
 * - 0: No filter (bandwidth = 1046Hz)
 * - 1: 5Hz
 * - 2: 10Hz
 * - 3: 21Hz
 * - 4: 45Hz
 * - 5: 99Hz
 * - 6: 218Hz
 * - 7: 420Hz
 */
#define ACCEL_DLPF_LEVEL 2

/**
 * @brief Digital Low-Pass Filter (DLPF) configuration for gyroscope
 * 
 * Available settings (0-7):
 * - 0: No filter (bandwidth = 8173Hz)
 * - 1: 5Hz
 * - 2: 10Hz
 * - 3: 20Hz
 * - 4: 51Hz
 * - 5: 104Hz
 * - 6: 218Hz
 * - 7: 420Hz
 */
#define GYRO_DLPF_LEVEL  2

/**
 * @brief Digital Low-Pass Filter (DLPF) configuration for temperature sensor
 * 
 * Available settings (0-7):
 * - 0: No filter
 * - 1-7: Various filter levels (see datasheet)
 */
#define TEMP_DLPF_LEVEL  2

/**
 * @brief Full Scale Range (FSR) configuration for accelerometer
 * 
 * Available presets (0-3):
 * - 0: ±2g
 * - 1: ±4g
 * - 2: ±8g
 * - 3: ±16g
 */
#define ACCEL_FSS_PRESET 0

/**
 * @brief Full Scale Range (FSR) configuration for gyroscope
 * 
 * Available presets (0-3):
 * - 0: ±250dps
 * - 1: ±500dps
 * - 2: ±1000dps
 * - 3: ±2000dps
 */
#define GYRO_FSS_PRESET  0

/**
 * @brief Output Data Rate (ODR) configuration for magnetometer
 * 
 * Available presets (0-5):
 * - 0: 10Hz
 * - 1: 20Hz
 * - 2: 50Hz
 * - 3: 100Hz
 * - 4: 200Hz (not recommended for continuous mode)
 * - 5: 400Hz (not recommended for continuous mode)
 */
#define MAG_ODR_PRESET   4

// =============================================
// Hardware Configuration
// =============================================

/// I2C address of the ICM-20948
#define ICM_20948_ADDR 0x68

/// I2C address of the AK09916 magnetometer
#define AK09916_ADDR   0x0C

/// Magnetometer control register 2 (CNTL2)
#define AK09916_CNTL2  0x31

/// Magnetometer control register 3 (CNTL3)
#define AK09916_CNTL3  0x32

// =============================================
// Function Declarations
// =============================================

/**
 * @brief Initialize the IMU with default settings
 * 
 * This function:
 * 1. Initializes I2C communication
 * 2. Configures the ICM-20948 and AK09916
 * 3. Sets up DLPF and ODR according to the presets
 * 4. Enables necessary sensors
 * 
 * @return true if initialization was successful, false otherwise
 */
bool initIMU();

// --- Sensor Configuration Functions ---

/**
 * @brief Set the accelerometer digital low-pass filter
 * 
 * @param mode Filter mode (0-7)
 * @see ACCEL_DLPF_LEVEL for mode descriptions
 */
void setAccelDLPF(uint8_t mode);

/**
 * @brief Set the gyroscope digital low-pass filter
 * 
 * @param mode Filter mode (0-7)
 * @see GYRO_DLPF_LEVEL for mode descriptions
 */
void setGyroDLPF(uint8_t mode);

/**
 * @brief Set the temperature sensor digital low-pass filter
 * 
 * @param mode Filter mode (0-7)
 * @see TEMP_DLPF_LEVEL for mode descriptions
 */
void setTempDLPF(uint8_t mode);

/**
 * @brief Set the magnetometer output data rate
 * 
 * @param rate_config Rate configuration (0-5)
 * @see MAG_ODR_PRESET for rate descriptions
 */
void setMagOutputDataRate(uint8_t rate_config);

// --- Low-level I2C Functions ---

/**
 * @brief Switch between register banks on the ICM-20948
 * 
 * @param bank The bank number to switch to (0-3)
 */
void setBank(uint8_t bank);

/**
 * @brief Write a value to an IMU register
 * 
 * @param reg Register address to write to
 * @param val Value to write
 */
void writeRegister(uint8_t reg, uint8_t val);

/**
 * @brief Read a value from an IMU register
 * 
 * @param reg Register address to read from
 * @return uint8_t The value read from the register
 */
uint8_t readRegister(uint8_t reg);

/**
 * @brief Read multiple bytes from IMU registers
 * 
 * @param reg Starting register address
 * @param data Buffer to store the read data
 * @param len Number of bytes to read
 * @return uint8_t Number of bytes actually read
 */
uint8_t readRegisters(uint8_t reg, uint8_t* data, uint8_t len);

// --- Sensor Reading Functions ---

/**
 * @brief Read raw accelerometer data
 * 
 * @param ax Reference to store X-axis acceleration (in g)
 * @param ay Reference to store Y-axis acceleration (in g)
 * @param az Reference to store Z-axis acceleration (in g)
 * @return true if read was successful, false otherwise
 */
bool readAccel(float& ax, float& ay, float& az);

/**
 * @brief Read raw gyroscope data
 * 
 * @param gx Reference to store X-axis angular rate (in dps)
 * @param gy Reference to store Y-axis angular rate (in dps)
 * @param gz Reference to store Z-axis angular rate (in dps)
 * @return true if read was successful, false otherwise
 */
bool readGyro(float& gx, float& gy, float& gz);

/**
 * @brief Read raw magnetometer data
 * 
 * @param mx Reference to store X-axis magnetic field (in uT)
 * @param my Reference to store Y-axis magnetic field (in uT)
 * @param mz Reference to store Z-axis magnetic field (in uT)
 * @return true if read was successful, false otherwise
 */
bool readMag(float& mx, float& my, float& mz);

/**
 * @brief Read raw magnetometer data (alias for readMag)
 * 
 * This is provided for backward compatibility.
 */
bool readMagnetometer(float& mx, float& my, float& mz);

/**
 * @brief Perform magnetometer recovery procedure
 * 
 * This function attempts to recover from magnetometer communication errors
 * by resetting the I2C bus and reinitializing the magnetometer.
 * 
 * @param consecutive_read_failures Reference to the failure counter
 */
void performMagnetometerRecovery(uint8_t& consecutive_read_failures);

// --- Axis Mapping Functions ---

/**
 * @brief Apply axis mapping to raw sensor data
 * 
 * This function transforms the raw sensor data into a consistent right-handed
 * coordinate system used throughout the application.
 * 
 * Transformations applied:
 * - X' = -Y
 * - Y' = X
 * - Z' = -Z
 * 
 * @param raw_x Raw X-axis value
 * @param raw_y Raw Y-axis value
 * @param raw_z Raw Z-axis value
 * @param mapped_x Reference to store mapped X-axis value
 * @param mapped_y Reference to store mapped Y-axis value
 * @param mapped_z Reference to store mapped Z-axis value
 */
void applyAxisMapping(float raw_x, float raw_y, float raw_z,
                     float& mapped_x, float& mapped_y, float& mapped_z);

/**
 * @brief Read accelerometer data with axis mapping applied
 * 
 * This is the recommended function to use for reading accelerometer data
 * as it ensures consistent axis orientation with other sensors.
 * 
 * @param ax Reference to store X-axis acceleration (in g)
 * @param ay Reference to store Y-axis acceleration (in g)
 * @param az Reference to store Z-axis acceleration (in g)
 * @return true if read was successful, false otherwise
 */
bool readAccelMapped(float& ax, float& ay, float& az);

/**
 * @brief Read gyroscope data with axis mapping applied
 * 
 * This is the recommended function to use for reading gyroscope data
 * as it ensures consistent axis orientation with other sensors.
 * 
 * @param gx Reference to store X-axis angular rate (in dps)
 * @param gy Reference to store Y-axis angular rate (in dps)
 * @param gz Reference to store Z-axis angular rate (in dps)
 * @return true if read was successful, false otherwise
 */
bool readGyroMapped(float& gx, float& gy, float& gz);

/**
 * @brief Read magnetometer data with axis mapping applied
 * 
 * This is the recommended function to use for reading magnetometer data
 * as it ensures consistent axis orientation with other sensors.
 * 
 * @param mx Reference to store X-axis magnetic field (in uT)
 * @param my Reference to store Y-axis magnetic field (in uT)
 * @param mz Reference to store Z-axis magnetic field (in uT)
 * @return true if read was successful, false otherwise
 */
bool readMagMapped(float& mx, float& my, float& mz);

/** @} */ // end of imu_config group

// External references to configuration tables
extern const uint8_t accel_dlpf_table[8];
extern const uint8_t gyro_dlpf_table[8];
extern const uint8_t temp_dlpf_table[8];
extern const uint8_t mag_odr_table[6];
extern const uint8_t accel_fss_table[4];
extern const uint8_t gyro_fss_table[4];

// External I2C interface
extern TwoWire &I2C_IMU;

#endif // IMU_CONFIG_H
