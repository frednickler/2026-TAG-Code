#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#include <Wire.h>

#define I2C_IMU Wire

// Set to 1 to enable, set 0 to disable verbose geomagnetic model debug prints
#define GEOMAG_DEBUG 1

// Select declination source: 1 = WMM/NOAA model (default), 0 = GPS-provided
#define USE_WMM_DECLINATION 1

// IMU and Magnetometer Register Addresses
#define ICM_20948_ADDR 0x68
#define WHO_AM_I_REG    0x00
#define PWR_MGMT_1      0x06
#define PWR_MGMT_2      0x07
#define ACCEL_XOUT_H    0x2D
#define GYRO_XOUT_H     0x33
#define TEMP_OUT_H      0x39
#define USER_CTRL       0x03
#define INT_PIN_CFG     0x0F
#define REG_BANK_SEL    0x7F
#define I2C_MST_CTRL    0x01
#define I2C_SLV0_ADDR   0x03
#define I2C_SLV0_REG    0x04
#define I2C_SLV0_CTRL   0x05
#define I2C_SLV0_DO     0x06
#define EXT_SENS_DATA_00 0x3B

#define AK09916_ADDR    0x0C
#define AK09916_WIA2    0x01
#define AK09916_ST1     0x10
#define AK09916_HXL     0x11
#define AK09916_ST2     0x18
#define AK09916_CNTL2   0x31
#define AK09916_CNTL3   0x32

// Shared I2C register access
void writeRegister(uint8_t reg, uint8_t val);
uint8_t readRegister(uint8_t reg);

// === User-selectable digital-filter levels (0-7) ===
// 0 = bypass (fastest, no filtering); 7 = heaviest filtering, most lag
// Simply change these three numbers and re-upload.
#define GYRO_DLPF_LEVEL 3    // datasheet §4.7.1 (see table below)
#define ACCEL_DLPF_LEVEL 3   // datasheet §4.7.3
#define TEMP_DLPF_LEVEL 3    // datasheet §4.7.2 (shares same reg)

#define MAG_ODR_PRESET 4     // unchanged (0x02-0x08 choices)

extern const uint8_t mag_odr_table[6];
extern const uint8_t accel_dlpf_table[8];
extern const uint8_t gyro_dlpf_table[8];
extern const uint8_t temp_dlpf_table[8];

// =================== Magnetometer ODR Table (AK09916 CNTL2) ===================
// Preset | CNTL2 value | Output Data Rate (Hz)
//   1    | 0x02        | 10
//   2    | 0x04        | 20
//   3    | 0x06        | 50
//   4    | 0x08        | 100 (default)
//   5    | 0x08        | 100 (alias for preset 4)

// =================== Gyro / Temp DLPF Table (bits 2:0 GYRO_CONFIG_1) ===================
// Level | Gyro BW (Hz) | Delay(ms) | Temp BW (Hz) | Notes
//   0   | 3281         | 0.17      | 7932         | bypass (no filter)
//   1   | 250          | 0.97      | 217.9        | light filter
//   2   | 176          | 1.97      | 123.5
//   3   | 92           | 2.9       | 65.9         | balanced (default)
//   4   | 41           | 4.9       | 34.1
//   5   | 20           | 8.5       | 17.3
//   6   | 10           | 13.8      | 8.8
//   7   | 5            | 19.0      | 7932*        | max filtering

// =================== Accel DLPF Table (bits 2:0 ACCEL_CONFIG) ===================
// Level | BW(Hz) | Delay(ms)
//   0   | 1046   | 0.503   (bypass)
//   1   | 420    | 1.34
//   2   | 218    | 1.88
//   3   | 99     | 2.9     (default)
//   4   | 44.8   | 4.9
//   5   | 21.2   | 8.5
//   6   | 10.2   | 13.8
//   7   | 5.05   | 19.0

void setMagOutputDataRate(uint8_t mode);
void setAccelDLPF(uint8_t mode);
void setGyroDLPF(uint8_t mode);
void setTempDLPF(uint8_t mode);

#endif // IMU_CONFIG_H
