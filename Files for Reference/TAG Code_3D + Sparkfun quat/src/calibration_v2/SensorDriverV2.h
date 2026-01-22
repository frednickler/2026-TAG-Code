#ifndef SENSOR_DRIVER_V2_H
#define SENSOR_DRIVER_V2_H

#include <Arduino.h>
#include <Wire.h>

// Calibration Configuration
#define CAL_IMU_ADDR 0x68
#define CAL_MAG_ADDR 0x0C

// Register Map
#define REG_WHO_AM_I       0x00
#define REG_USER_CTRL      0x03
#define REG_PWR_MGMT_1     0x06
#define REG_PWR_MGMT_2     0x07
#define REG_INT_PIN_CFG    0x0F

#define REG_ACCEL_XOUT_H   0x2D
#define REG_GYRO_XOUT_H    0x33

// Bank 2
#define REG_GYRO_SMPLRT_DIV 0x00
#define REG_GYRO_CONFIG_1   0x01
#define REG_GYRO_CONFIG_2   0x02
#define REG_ACCEL_SMPLRT_DIV_1 0x10
#define REG_ACCEL_SMPLRT_DIV_2 0x11
#define REG_ACCEL_CONFIG    0x14 // [0] = DLPF_CFG, [2:1] = FS_SEL, [5:3] = DLPF_CFG
#define REG_ACCEL_CONFIG_2  0x15

// Magnetometer
#define MAG_ST1            0x10
#define MAG_HXL            0x11
#define MAG_CNTL2          0x31
#define MAG_CNTL3          0x32
#define MAG_ST2            0x18

struct RawSensorData {
    float ax, ay, az; // g
    float gx, gy, gz; // dps
    float mx, my, mz; // uT
    bool valid;
};

class SensorDriverV2 {
public:
    static bool begin();
    static void configureForCalibration();
    static RawSensorData readAll();
    static bool readMag(float &x, float &y, float &z);
    static void writeGyroHardwareOffsets(float x_dps, float y_dps, float z_dps);
    
private:
    static void writeRegister(uint8_t bank, uint8_t reg, uint8_t val);
    static uint8_t readRegister(uint8_t bank, uint8_t reg);
    static void setBank(uint8_t bank);
    static bool initMagnetometer();
};

#endif
