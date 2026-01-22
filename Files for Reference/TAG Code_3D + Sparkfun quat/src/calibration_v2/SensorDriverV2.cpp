#include "SensorDriverV2.h"

// Hardware Defaults
const uint8_t ACCEL_FS_SEL = 0x01; // ±4g (Bit 1=0, Bit 2=1 ? Check datasheet. Usually 00=2g, 01=4g, 10=8g, 11=16g) 
// Actually ICM-20948: FS_SEL [2:1]
// 00 = 2g
// 01 = 4g
// 10 = 8g
// 11 = 16g
// We will use ±4g for generic use, but maybe ±2g is better for calibration precision if we don't shake it?
// Let's stick to user's ±4g preference or use ±2g for gravity cal. Gravity is 1g. ±2g is best for resolution.

// Using ±2g for maximum calibration resolution
const uint8_t CAL_ACCEL_FS = 0x00; 

// Gyro ±250dps for best resolution
const uint8_t CAL_GYRO_FS = 0x00; 

void SensorDriverV2::setBank(uint8_t bank) {
    Wire.beginTransmission(CAL_IMU_ADDR);
    Wire.write(0x7F);
    Wire.write(bank << 4);
    Wire.endTransmission();
}

void SensorDriverV2::writeRegister(uint8_t bank, uint8_t reg, uint8_t val) {
    setBank(bank);
    Wire.beginTransmission(CAL_IMU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t SensorDriverV2::readRegister(uint8_t bank, uint8_t reg) {
    setBank(bank);
    Wire.beginTransmission(CAL_IMU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(CAL_IMU_ADDR, 1);
    return Wire.read();
}

bool SensorDriverV2::initMagnetometer() {
    // 1. Enable Bypass Mode on ICM-20948 (Bank 0)
    setBank(0);
    writeRegister(0, 0x0F, 0x02); // INT_PIN_CFG -> BYPASS_EN = 1
    delay(50);
    
    // 2. Soft Reset Mag (0x32 -> 0x01)
    Wire.beginTransmission(CAL_MAG_ADDR);
    Wire.write(MAG_CNTL3);
    Wire.write(0x01);
    Wire.endTransmission();
    delay(100);
    
    // 3. Check Who Am I
    Wire.beginTransmission(CAL_MAG_ADDR);
    Wire.write(0x01); // WIA2
    Wire.endTransmission(); // Stop
    Wire.requestFrom(CAL_MAG_ADDR, 1);
    uint8_t who = Wire.read();
    
    if (who != 0x09) {
        Serial.printf("[DRIVER] Mag WhoAmI failed: 0x%02X\n", who);
        return false;
    }

    // 4. Set to Power Down Mode first (Required by datasheet)
    Wire.beginTransmission(CAL_MAG_ADDR);
    Wire.write(MAG_CNTL2);
    Wire.write(0x00); // 0x00 = Power Down
    Wire.endTransmission();
    delay(20);

    // 5. Set to Continuous Measurement Mode 4 (100Hz)
    Wire.beginTransmission(CAL_MAG_ADDR);
    Wire.write(MAG_CNTL2);
    Wire.write(0x08); // 0x08 = Continuous 4
    Wire.endTransmission();
    delay(20);
    
    // VERIFY Mode
    Wire.beginTransmission(CAL_MAG_ADDR);
    Wire.write(MAG_CNTL2);
    Wire.endTransmission();
    Wire.requestFrom(CAL_MAG_ADDR, 1);
    
    if (Wire.available()) {
        uint8_t mode = Wire.read();
        Serial.printf("[DRIVER] Mag Mode: 0x%02X\n", mode);
        if (mode != 0x08) {
             Serial.println("[DRIVER] Retrying Mode Set...");
             Wire.beginTransmission(CAL_MAG_ADDR);
             Wire.write(MAG_CNTL2);
             Wire.write(0x08);
             Wire.endTransmission();
        }
    }
    
    return true;
}

bool SensorDriverV2::begin() {
    Wire.begin(4, 5); // SDA, SCL - Explicitly set pins
    Wire.setClock(400000);
    delay(100);

    Serial.println("[DRIVER] Resetting IMU...");
    
    // 1. Full Reset
    writeRegister(0, REG_PWR_MGMT_1, 0x80);
    delay(100);
    
    // 2. Wake Up
    writeRegister(0, REG_PWR_MGMT_1, 0x01); // Auto-select clock
    delay(100);

    // 3. ID Check
    uint8_t who = readRegister(0, REG_WHO_AM_I);
    if (who != 0xEA) {
        Serial.printf("[DRIVER] IMU ID Mismatch: 0x%02X (Expected 0xEA)\n", who);
        return false;
    }

    configureForCalibration();

    if (!initMagnetometer()) {
        Serial.println("[DRIVER] Mag Init Failed");
        return false;
    }

    Serial.println("[DRIVER] Init Success");
    return true;
}

void SensorDriverV2::configureForCalibration() {
    // Bank 2 Config
    // Accel: ±2g, DLPF enabled 5Hz (for super stable calibration readings)
    // Gyro: ±250dps, DLPF enabled 5Hz
    
    // GYRO_CONFIG_1: [2:1] FS_SEL, [0] DLPF_EN
    writeRegister(2, REG_GYRO_CONFIG_1, (CAL_GYRO_FS << 1) | 0x01);
    
    // GYRO_CONFIG_2: [2:0] DLPF_CFG (5Hz = 6?? Check datasheet. Usually higher # is lower freq)
    // 0=196Hz, 7=361Hz... Wait datasheet check.
    // Let's use generic low noise settings. 
    // DLPF_CFG=0 is usually safe "standard". Let's use 0 for now.
    writeRegister(2, REG_GYRO_CONFIG_2, 0x00); 

    // ACCEL_CONFIG: [2:1] FS_SEL, [0] DLPF_EN?? 
    // ICM20948 Accel Config (0x14):
    // [0] = DLPF_CFG LSB? No.
    // [0] = FCHOICE (0=Bypass DLPF, 1=Enable DLPF)
    // [2:1] = FS_SEL
    // [5:3] = DLPF_CFG
    
    // We want ±2g (00), DLPF Enabled (1)
    writeRegister(2, REG_ACCEL_CONFIG, (CAL_ACCEL_FS << 1) | 0x01);
    
    // ACCEL_CONFIG_2: DLPF Bandwidth
    // 0x15? Wait, 0x14 holds DLPF_CFG in bits 5:3? 
    // Let's re-verify datasheet. 
    // REG_ACCEL_CONFIG (0x14): | ... | DLPF_CFG[2:0] | FS_SEL[1:0] | FCHOICE |
    // Wait, typical Inventsense is:
    // [5:3] DLPF_CFG
    // [2:1] FS_SEL
    // [0]   FCHOICE (1=Enable)
    // So 0x01 = DLPF=0 (246Hz), FS=0 (2g), FCHOICE=1 (Enable). Good.
}

RawSensorData SensorDriverV2::readAll() {
    RawSensorData d;
    d.valid = false;
    
    uint8_t buff[12];
    setBank(0);
    Wire.beginTransmission(CAL_IMU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(CAL_IMU_ADDR, 12);
    
    if (Wire.available() == 12) {
        for(int i=0; i<12; i++) buff[i] = Wire.read();
        
        int16_t ax_raw = (buff[0] << 8) | buff[1];
        int16_t ay_raw = (buff[2] << 8) | buff[3];
        int16_t az_raw = (buff[4] << 8) | buff[5];
        
        int16_t gx_raw = (buff[6] << 8) | buff[7];
        int16_t gy_raw = (buff[8] << 8) | buff[9];
        int16_t gz_raw = (buff[10] << 8) | buff[11];
        
        // Convert
        d.ax = ax_raw / 16384.0f; // 2g scale
        d.ay = ay_raw / 16384.0f;
        d.az = az_raw / 16384.0f;
        
        d.gx = gx_raw / 131.0f; // 250dps scale
        d.gy = gy_raw / 131.0f;
        d.gz = gz_raw / 131.0f;
        
        d.valid = true;
    }
    
    readMag(d.mx, d.my, d.mz);
    
    return d;
}

bool SensorDriverV2::readMag(float &x, float &y, float &z) {
    // 1. Check ST1 for Data Ready
    Wire.beginTransmission(CAL_MAG_ADDR);
    Wire.write(MAG_ST1);
    Wire.endTransmission(); // STOP
    Wire.requestFrom(CAL_MAG_ADDR, 1);
    
    if (Wire.available()) {
        if (!(Wire.read() & 0x01)) return false; // Data not ready
    } else {
        return false;
    }

    // 2. Read Data (HXL-HZH) + Reserved + ST2
    // AK09916 Register Map:
    // 0x11-0x16: HXL, HXH, HYL, HYH, HZL, HZH (6 bytes)
    // 0x17: Reserved (1 byte)
    // 0x18: ST2 (1 byte)
    // Total: 8 bytes
    Wire.beginTransmission(CAL_MAG_ADDR);
    Wire.write(MAG_HXL); // 0x11
    Wire.endTransmission(); // STOP
    Wire.requestFrom(CAL_MAG_ADDR, 8); // Read through ST2
    
    if (Wire.available() == 8) {
        uint8_t magData[8];
        for(int i=0; i<8; i++) magData[i] = Wire.read();
        
        // Check ST2 (index 7) for overflow (Bit 3)
        if (magData[7] & 0x08) return false; 
        
        // Data is Little Endian (LSB first)
        int16_t mx_raw = (magData[1] << 8) | magData[0];
        int16_t my_raw = (magData[3] << 8) | magData[2];
        int16_t mz_raw = (magData[5] << 8) | magData[4];
        
        // Convert to uT (0.15 uT/LSB for AK09916)
        x = mx_raw * 0.15f;
        y = my_raw * 0.15f;
        z = mz_raw * 0.15f;
        
        return true;
    }
    return false;
}


void SensorDriverV2::writeGyroHardwareOffsets(float x_dps, float y_dps, float z_dps) {
    // Convert dps to raw LSB (±250dps scale = 131 LSB/dps)
    int16_t x_offset = (int16_t)(-x_dps * 131.0f);
    int16_t y_offset = (int16_t)(-y_dps * 131.0f);
    int16_t z_offset = (int16_t)(-z_dps * 131.0f);
    
    // Write to Bank 2, 0x03-0x08
    setBank(2);
    writeRegister(2, 0x03, (x_offset >> 8) & 0xFF);
    writeRegister(2, 0x04, x_offset & 0xFF);
    writeRegister(2, 0x05, (y_offset >> 8) & 0xFF);
    writeRegister(2, 0x06, y_offset & 0xFF);
    writeRegister(2, 0x07, (z_offset >> 8) & 0xFF);
    writeRegister(2, 0x08, z_offset & 0xFF);
    
    Serial.printf("[DRIVER] Gyro hardware offsets: X=%.2f Y=%.2f Z=%.2f dps\n", x_dps, y_dps, z_dps);
}
