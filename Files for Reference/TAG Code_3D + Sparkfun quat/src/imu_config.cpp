#include "imu_config.h"
#include "config.h"
#include <Wire.h>

// Create a reference alias so imu_config can use I2C_IMU transparently
TwoWire &I2C_IMU = Wire; // Reuses default Wire instance initialised in setup()

// ICM-20948 I2C address
#define ICM20948_ADDRESS 0x68

// AK09916 I2C address
#define AK09916_ADDR 0x0C

// User Bank 0
#define REG_BANK_SEL 0x7F

//   Mag ODR: index 1..5 (0 unused) 10/20/50/100/100 Hz
const uint8_t mag_odr_table[6]    = {0, 0x02, 0x04, 0x06, 0x08, 0x08};
//   Accel DLPF: 1=5Hz, 2=10Hz, 3=111Hz, 4=246Hz, 5=No filter
const uint8_t accel_dlpf_table[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
//   Gyro DLPF:  1=5Hz, 2=10Hz, 3=111Hz, 4=246Hz, 5=No filter
const uint8_t gyro_dlpf_table[8]  = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
const uint8_t temp_dlpf_table[8]  = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

/**
 * Switch between register banks on the ICM-20948
 * 
 * @param bank The bank number to switch to (0-3)
 */
void setBank(uint8_t bank) {
    // Only the lower 4 bits are used for bank selection
    uint8_t bankValue = bank & 0x07;
    
    // REG_BANK_SEL is always at address 0x7F regardless of current bank
    // The bank value goes in bits 4-5 (0b00110000 mask)
    writeRegister(0x7F, bankValue << 4);
    
    // Small delay to ensure the bank switch completes
    delayMicroseconds(10);
}

/**
 * @brief Write a byte to a register on the ICM-20948
 * @param reg The register to write to
 * @param value The value to write
 */
void writeRegister(uint8_t reg, uint8_t value) {
    I2C_IMU.beginTransmission(ICM20948_ADDRESS);
    I2C_IMU.write(reg);
    I2C_IMU.write(value);
    I2C_IMU.endTransmission();
    delay(2);
}

/**
 * @brief Read a byte from a register on the ICM-20948
 * @param reg The register to read from
 * @return The value read from the register
 */
uint8_t readRegister(uint8_t reg) {
    I2C_IMU.beginTransmission(ICM20948_ADDRESS);
    I2C_IMU.write(reg);
    I2C_IMU.endTransmission(false); // Send repeated start
    I2C_IMU.requestFrom(ICM20948_ADDRESS, 1);
    return I2C_IMU.read();
}

/**
 * @brief Read multiple bytes from a register on the ICM-20948
 * @param reg The register to start reading from
 * @param data A pointer to the buffer to store the data
 * @param len The number of bytes to read
 * @return The number of bytes read
 */
uint8_t readRegisters(uint8_t reg, uint8_t* data, uint8_t len) {
    I2C_IMU.beginTransmission(ICM20948_ADDRESS);
    I2C_IMU.write(reg);
    I2C_IMU.endTransmission(false); // Send repeated start
    // Explicitly cast parameters to resolve ambiguity
    uint8_t bytesRead = I2C_IMU.requestFrom((uint8_t)ICM20948_ADDRESS, (uint8_t)len, (uint8_t)true);
    for (uint8_t i = 0; i < bytesRead; i++) {
        data[i] = I2C_IMU.read();
    }
    return bytesRead;
}

void setMagOutputDataRate(uint8_t mode) {
    I2C_IMU.beginTransmission(AK09916_ADDR);
    I2C_IMU.write(AK09916_CNTL2);
    I2C_IMU.write(mode);
    I2C_IMU.endTransmission();
    delay(2);
}

void setAccelDLPF(uint8_t mode) {
    // Accelerometer DLPF is in Bank 2, Register 0x14 (ACCEL_CONFIG)
    // Bits [5:3] are ACCEL_DLPFCFG, Bit 0 is ACCEL_FCHOICE (1=Enable DLPF)
    setBank(2);
    
    // Read current value to preserve FSR (bits [2:1])
    uint8_t current_val = readRegister(0x14); // ACCEL_CONFIG
    
    // Clear DLPF bits [5:3] and FCHOICE bit [0]
    // 0x39 = 0011 1001. We want to clear bits 5,4,3 and 0. 
    // Wait, simpler: Mask is ~0x39? 
    // Bits to clear: 0011 1000 (0x38) | 0000 0001 (0x01) = 0x39
    // So mask is ~0x39 = 0xC6 (1100 0110).Preserve bits 7,6,2,1. 
    uint8_t mask = 0xC6;
    
    // If mode is 0 (Disabled/Bypass), we set FCHOICE=0.
    // If mode > 0, we set FCHOICE=1 and set DLPF bits.
    
    uint8_t new_val;
    if (mode == 0) {
        // Bypass mode: FCHOICE=0, DLPF=don't care (but we'll zero it)
        new_val = (current_val & mask); 
    } else {
        // Enable DLPF: FCHOICE=1, DLPF = mode
        // Note: mode is 1-7 in our API, mapping to hardware 0-7? 
        // accel_dlpf_table maps API 1->1, 2->2 etc.
        // Let's assume input 'mode' from accel_dlpf_table is the raw bit value.
        // Bit 0 must be 1 to enable.
        new_val = (current_val & mask) | ((mode & 0x07) << 3) | 0x01;
    }

    writeRegister(0x14, new_val);
    setBank(0); // Return to Bank 0
}

void setGyroDLPF(uint8_t mode) {
    // Gyroscope DLPF is in Bank 2, Register 0x01 (GYRO_CONFIG_1)
    // Bits [5:3] are GYRO_DLPFCFG, Bit 0 is GYRO_FCHOICE (1=Enable DLPF)
    setBank(2);
    
    uint8_t current_val = readRegister(0x01); // GYRO_CONFIG_1
    
    // Clear DLPF bits [5:3] and FCHOICE bit [0]
    uint8_t mask = 0xC6;
    
    uint8_t new_val;
    if (mode == 0) {
        // Bypass
        new_val = (current_val & mask);
    } else {
        // Enable
        new_val = (current_val & mask) | ((mode & 0x07) << 3) | 0x01;
    }
    
    writeRegister(0x01, new_val);
    setBank(0); // Return to Bank 0
}

void setTempDLPF(uint8_t mode) {
    // Temperature sensor shares same DLPF bits as gyro or has no specific config in ICM-20948?
    // Datasheet says Temp config is usually tied to Gyro config or separate TEMP_CONFIG (Bank 2, 0x53)
    // Checking DS: Bank 2, Register 0x53 (TEMP_CONFIG). Bits [2:0] = TEMP_DLPFCFG.
    setBank(2);
    writeRegister(0x53, mode & 0x07);
    setBank(0);
}

bool readAccel(float& ax, float& ay, float& az) {
    // Set bank 0 to access accelerometer data registers
    setBank(0);
    
    // Read 6 bytes starting from ACCEL_XOUT_H (0x2D)
    uint8_t data[6];
    if (readRegisters(0x2D, data, 6) != 6) {
        ax = ay = az = 0.0f;
        return false;
    }
    
    // Convert to signed 16-bit values and apply scale factor
    // Scale factor is automatically derived from ACCEL_FSR_G in config.h
    const float accel_scale = Config::ACCEL_SCALE; // Convert to mg
    
    int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);
    
    ax = raw_ax * accel_scale; // Returns mg
    ay = raw_ay * accel_scale;
    az = raw_az * accel_scale;
    
    // DEBUG: Only print if magnitude is wildly off (detection of hardware failure)
    /*
    static int debug_counter = 0;
    if (++debug_counter >= 100) {
        debug_counter = 0;
        Serial.printf("[ACCEL DEBUG] Raw: %d, %d, %d | Scaled: %.2f, %.2f, %.2f mg\n",
                     raw_ax, raw_ay, raw_az, ax, ay, az);
    }
    */
    
    return true;
}

bool readGyro(float& gx, float& gy, float& gz) {
    // Set bank 0 to access gyroscope data registers
    setBank(0);
    
    // Read 6 bytes starting from GYRO_XOUT_H (0x33)
    uint8_t data[6];
    if (readRegisters(0x33, data, 6) != 6) {
        gx = gy = gz = 0.0f;
        return false;
    }
    
    // Convert to signed 16-bit values and apply scale factor
    // ICM-20948 gyroscope scale: ±250 dps = 131 LSB/dps
    const float gyro_scale = 250.0f / 32768.0f; // Convert to dps
    
    int16_t raw_gx = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_gy = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_gz = (int16_t)((data[4] << 8) | data[5]);
    
    gx = raw_gx * gyro_scale;
    gy = raw_gy * gyro_scale;
    gz = raw_gz * gyro_scale;
    
    return true;
}

bool readMag(float& mx, float& my, float& mz) {
    // Use the robust magnetometer reading function
    return readMagnetometer(mx, my, mz);
}

/**
 * Read magnetometer data using I2C bypass mode exclusively
 * Based on the working reference implementation
 */
bool readMagnetometer(float& mx, float& my, float& mz) {
    static uint32_t last_read = 0;
    static uint32_t last_success = 0;
    static uint32_t last_continuous_mode_assert = 0;
    static uint8_t consecutive_read_failures = 0;
    const uint8_t MAG_MAX_FAILS = 3;
    
    // Force recovery if no successful reads in 500ms
    if (millis() - last_success > 500) {
        consecutive_read_failures = MAG_MAX_FAILS;
    }
    
    // Enforce minimum 5ms between reads (200Hz max)
    uint32_t now = micros();
    if (now - last_read < 5000) {
        return false;
    }
    last_read = now;
    
    // Ensure I2C bypass mode is enabled
    setBank(0);
    writeRegister(0x03, 0x00); // USER_CTRL: Disable I2C master
    writeRegister(0x0F, 0x02); // INT_PIN_CFG: Enable I2C bypass
    delay(1);
    
    // Read status register 1 (ST1) directly from AK09916
    Wire.beginTransmission(0x0C); // AK09916 address
    Wire.write(0x10); // ST1 register
    if (Wire.endTransmission(false) != 0) {
        consecutive_read_failures++;
        // Check for recovery
        if (consecutive_read_failures >= MAG_MAX_FAILS) {
            performMagnetometerRecovery(consecutive_read_failures);
        }
        mx = my = mz = 0.0f;
        return false;
    }
    
    if (Wire.requestFrom(0x0C, 1) != 1) {
        consecutive_read_failures++;
        // Check for recovery
        if (consecutive_read_failures >= MAG_MAX_FAILS) {
            performMagnetometerRecovery(consecutive_read_failures);
        }
        mx = my = mz = 0.0f;
        return false;
    }
    
    uint8_t st1 = Wire.read();
    
    // Check if data is ready
    if (!(st1 & 0x01)) {
        // Handle data overrun (bit 1 set) by reading ST2 to clear it
        if (st1 & 0x02) {
            Wire.beginTransmission(0x0C);
            Wire.write(0x18); // ST2 register
            Wire.endTransmission(false);
            if (Wire.requestFrom(0x0C, 1) == 1) {
                Wire.read(); // Clear ST2
            }
        }
        
        // If we're getting repeated not-ready, force recovery
        if (consecutive_read_failures > 3) {
            if (consecutive_read_failures >= MAG_MAX_FAILS) {
                performMagnetometerRecovery(consecutive_read_failures);
            }
        }
        
        return false;
    }
    
    // Read the 6 data registers (HXL to HZH)
    Wire.beginTransmission(0x0C);
    Wire.write(0x11); // HXL register (start of data)
    if (Wire.endTransmission(false) != 0) {
        consecutive_read_failures++;
        // Check for recovery
        if (consecutive_read_failures >= MAG_MAX_FAILS) {
            performMagnetometerRecovery(consecutive_read_failures);
        }
        mx = my = mz = 0.0f;
        return false;
    }
    
    if (Wire.requestFrom(0x0C, 6) != 6) {
        consecutive_read_failures++;
        // Check for recovery
        if (consecutive_read_failures >= MAG_MAX_FAILS) {
            performMagnetometerRecovery(consecutive_read_failures);
        }
        mx = my = mz = 0.0f;
        return false;
    }
    
    uint8_t data[6];
    for (int i = 0; i < 6; i++) {
        data[i] = Wire.read();
    }
    
    // Read ST2 to check for overflow and clear DRDY
    Wire.beginTransmission(0x0C);
    Wire.write(0x18); // ST2 register
    if (Wire.endTransmission(false) != 0) {
        consecutive_read_failures++;
        // Check for recovery
        if (consecutive_read_failures >= MAG_MAX_FAILS) {
            performMagnetometerRecovery(consecutive_read_failures);
        }
        mx = my = mz = 0.0f;
        return false;
    }
    
    if (Wire.requestFrom(0x0C, 1) != 1) {
        consecutive_read_failures++;
        // Check for recovery
        if (consecutive_read_failures >= MAG_MAX_FAILS) {
            performMagnetometerRecovery(consecutive_read_failures);
        }
        mx = my = mz = 0.0f;
        return false;
    }
    
    uint8_t st2 = Wire.read();
    
    // Check for sensor overflow
    if (st2 & 0x08) {
        consecutive_read_failures++;
        // Check for recovery
        if (consecutive_read_failures >= MAG_MAX_FAILS) {
            performMagnetometerRecovery(consecutive_read_failures);
        }
        mx = my = mz = 0.0f;
        return false;
    }
    
    // Data is valid, process it
    int16_t raw_mx = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_my = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_mz = (int16_t)((data[5] << 8) | data[4]);
    
    // Convert to μT (AK09916 scale: 0.15 μT/LSB)
    const float mag_scale = 0.15f;
    mx = raw_mx * mag_scale;
    my = raw_my * mag_scale;
    mz = raw_mz * mag_scale;
    
    last_success = millis();
    consecutive_read_failures = 0;
    
    // Periodically reassert continuous mode (every 5s) to prevent stalls
    if (millis() - last_continuous_mode_assert > 5000) {
        Wire.beginTransmission(0x0C);
        Wire.write(0x31); // CNTL2 register
        Wire.write(0x08); // Continuous measurement mode 4 (100Hz)
        Wire.endTransmission(true);
        last_continuous_mode_assert = millis();
    }
    
    return true;
}

/**
 * Helper function to perform magnetometer recovery
 */
void performMagnetometerRecovery(uint8_t& consecutive_read_failures) {
    Serial.println("[MAG DEBUG] Recovery triggered, reinitializing magnetometer");
    
    // Reset I2C bus
    Wire.begin(4, 5); // SDA=4, SCL=5
    Wire.setClock(400000);
    delay(10);
    
    // Ensure bypass mode
    setBank(0);
    writeRegister(0x03, 0x00); // USER_CTRL: Disable I2C master
    writeRegister(0x0F, 0x02); // INT_PIN_CFG: Enable I2C bypass
    delay(10);
    
    // Reset magnetometer
    Wire.beginTransmission(0x0C);
    Wire.write(0x32); // CNTL3
    Wire.write(0x01); // Soft reset
    if (Wire.endTransmission(true) == 0) {
        delay(10);
        
        // Set continuous measurement mode
        Wire.beginTransmission(0x0C);
        Wire.write(0x31); // CNTL2
        Wire.write(0x08); // Mode 4 (100Hz)
        if (Wire.endTransmission(true) == 0) {
            consecutive_read_failures = 0;
            Serial.println("[MAG DEBUG] Magnetometer recovery successful");
        }
    }
}

bool initIMU() {
    Serial.println("[INIT] Initializing ICM-20948...");
    
    // Reset the device and wait for reset to complete
    setBank(0);
    writeRegister(0x06, 0x80); // PWR_MGMT_1: Reset device
    delay(100); // Wait for all registers to reset
    
    // Check WHO_AM_I register
    setBank(0);
    uint8_t whoami = readRegister(0x00);
    if (whoami != 0xEA) {
        Serial.printf("[INIT] Unexpected WHO_AM_I: 0x%02X (expected 0xEA)\n", whoami);
        return false;
    }
    Serial.println("[INIT] ICM-20948 WHO_AM_I verified");
    
    // Wake up device and select best available clock source
    setBank(0);
    writeRegister(0x06, 0x01); // PWR_MGMT_1: Clock Source = Auto select best available
    delay(20);
    
    // Verify device is awake
    uint8_t pwr1 = readRegister(0x06);
    if (pwr1 & 0x40) { // Make sure sleep bit is not set
        Serial.println("[INIT] Device not waking up");
        return false;
    }
    
    // Enable all axes of accelerometer and gyroscope
    writeRegister(0x07, 0x00); // PWR_MGMT_2: Enable all axes
    delay(10);
    
    // Configure bank 2 registers - Gyroscope and Accelerometer
    setBank(2);
    
    // Configure gyroscope: ±250dps, DLPF enabled
    // DLPF Config 3 (~51Hz) to strictly match 100Hz loop (Nyquist) and eliminate aliasing
    writeRegister(0x01, 0x19); // GYRO_CONFIG_1: ±250dps, DLPF enabled, BW=51Hz
    writeRegister(0x02, 0x00); // GYRO_CONFIG_2: DLPF bandwidth
    writeRegister(0x00, 0x00); // GYRO_SMPLRT_DIV: No decimation
    
    // Configure accelerometer using centralized setting from config.h
    // ACCEL_CONFIG (0x14) Configuration:
    // [5:3] DLPF Config = 3 (50.4Hz) -> 0x19 (0001 1001)
    // [2:1] FSR         = From Config         -> ACCEL_FS_SEL_BITS_REG
    // [0]   FCHOICE     = 1 (Enable DLPF)     -> 0x01
    // (ACCEL_FS_SEL_BITS_REG is already shifted to bits 2:1)
    writeRegister(0x14, ACCEL_FS_SEL_BITS_REG | 0x19); 
    
    // DEBUG: Verify register was written correctly
    delay(10);
    uint8_t accel_cfg_verify = readRegister(0x14);
    Serial.printf("[IMU INIT] ACCEL_CONFIG: 0x%02X (Expected 0x%02X for ±%dg)\n", 
                 accel_cfg_verify, ACCEL_FS_SEL_BITS_REG | 0x01, ACCEL_FSR_G);
    
    // Set Sample Rate Divider (ACCEL_SMPLRT_DIV)
    // Bank 2, Reg 0x10 MSB, 0x11 LSB. 
    // Setting to 0 for Max Rate.
    writeRegister(0x10, 0x00); 
    writeRegister(0x11, 0x00);
    delay(10);
    
    // Return to bank 0
    setBank(0);
    
    // Disable I2C Master and enable I2C Bypass for magnetometer
    writeRegister(0x03, 0x00); // USER_CTRL: Disable I2C master
    delay(20);
    writeRegister(0x0F, 0x02); // INT_PIN_CFG: Enable I2C bypass
    delay(10);
    
    // Initialize magnetometer directly via I2C bypass
    Serial.println("[INIT] Initializing magnetometer...");
    
    // Reset magnetometer
    Wire.beginTransmission(0x0C);
    Wire.write(0x32); // CNTL3
    Wire.write(0x01); // Soft reset
    if (Wire.endTransmission(true) != 0) {
        Serial.println("[INIT] Failed to reset magnetometer");
        return false;
    }
    delay(10);
    
    // Check magnetometer WHO_AM_I
    Wire.beginTransmission(0x0C);
    Wire.write(0x01); // WIA2
    if (Wire.endTransmission(false) != 0) {
        Serial.println("[INIT] Failed to access magnetometer");
        return false;
    }
    
    Wire.requestFrom(0x0C, 1);
    if (!Wire.available()) {
        Serial.println("[INIT] No response from magnetometer");
        return false;
    }
    
    uint8_t mag_whoami = Wire.read();
    if (mag_whoami != 0x09) {
        Serial.printf("[INIT] Unexpected magnetometer WHO_AM_I: 0x%02X (expected 0x09)\n", mag_whoami);
        return false;
    }
    Serial.println("[INIT] AK09916 WHO_AM_I verified");
    
    // Set magnetometer to continuous measurement mode 4 (100Hz)
    Wire.beginTransmission(0x0C);
    Wire.write(0x31); // CNTL2
    Wire.write(0x08); // Mode 4 (100Hz)
    if (Wire.endTransmission(true) != 0) {
        Serial.println("[INIT] Failed to set magnetometer mode");
        return false;
    }
    delay(10);
    
    // Verify mode was set
    Wire.beginTransmission(0x0C);
    Wire.write(0x31); // CNTL2
    if (Wire.endTransmission(false) != 0) {
        Serial.println("[INIT] Failed to verify magnetometer mode");
        return false;
    }
    
    Wire.requestFrom(0x0C, 1);
    if (!Wire.available()) {
        Serial.println("[INIT] No response when verifying magnetometer mode");
        return false;
    }
    
    uint8_t mode = Wire.read();
    if (mode != 0x08) {
        Serial.printf("[INIT] Magnetometer mode not set correctly: 0x%02X (expected 0x08)\n", mode);
        return false;
    }
    
    Serial.println("[INIT] IMU initialization complete");
    return true;
}

/**
 * Apply axis mapping to ensure consistent coordinate system across ALL sensors
 * 
 * PURPOSE: Map physical sensor axes to standard visualization coordinate system
 * SCOPE: This mapping affects ALL sensor data before quaternion calculation
 * 
 * USER VERIFIED CONFIGURATION (2025-12-19):
 * - mapX=1, signX=+1  → Roll  = +SensorY
 * - mapY=2, signY=-1  → Pitch = -SensorZ
 * - mapZ=0, signZ=-1  → Yaw   = -SensorX
 * 
 * This mapping is applied BEFORE the Madgwick filter to ensure correct quaternion calculation.
 */
void applyAxisMapping(float raw_x, float raw_y, float raw_z,
                     float& mapped_x, float& mapped_y, float& mapped_z) {
    // Store raw values in array for indexed access
    float raw[3] = {raw_x, raw_y, raw_z};
    
    // STANDARD MAPPING (Identity with Z correction)
    // Z is inverted because Sensor Z is Up (+1G), but NED Body Z is Down (+1G).
    // This fixes Roll being 180 (Upside Down) and likely fixes Yaw direction.
    
    mapped_x = raw[0] * 1.0f;   // Map X = Raw X
    mapped_y = raw[1] * 1.0f;   // Map Y = Raw Y
    mapped_z = raw[2] * -1.0f;  // Map Z = -Raw Z (Fix Upside Down)
}

/**
 * Read accelerometer with consistent axis mapping applied
 */
bool readAccelMapped(float& ax, float& ay, float& az) {
    float raw_ax, raw_ay, raw_az;
    if (!readAccel(raw_ax, raw_ay, raw_az)) {
        return false;
    }
    
    applyAxisMapping(raw_ax, raw_ay, raw_az, ax, ay, az);
    return true;
}

/**
 * Read gyroscope with consistent axis mapping applied
 */
bool readGyroMapped(float& gx, float& gy, float& gz) {
    float raw_gx, raw_gy, raw_gz;
    if (!readGyro(raw_gx, raw_gy, raw_gz)) {
        return false;
    }
    
    applyAxisMapping(raw_gx, raw_gy, raw_gz, gx, gy, gz);
    return true;
}

/**
 * Read magnetometer with consistent axis mapping applied
 */
bool readMagMapped(float& mx, float& my, float& mz) {
    float raw_mx, raw_my, raw_mz;
    if (!readMag(raw_mx, raw_my, raw_mz)) {
        return false;
    }
    
    applyAxisMapping(raw_mx, raw_my, raw_mz, mx, my, mz);
    return true;
}
