#include "IMUManager.h"
#include "../config/SystemConfig.h"
#include "../config/SystemSettings.h"
#include "../calibration/CalibrationManager.h"
#include "../system/Watchdog.h"
#include "../algorithms/vqf.h"
#include <Wire.h>

// ICM-20948 library (SparkFun - proven and simple)
#include <ICM_20948.h>

// BMM350 core driver (extracted from DFRobot library)
extern "C" {
    #include "bmm350.h"
    #include "bmm350_defs.h"
}

// =============================================================================
// STATIC MEMBER INITIALIZATION
// =============================================================================
bool IMUManager::initialized = false;

// Sensor objects (static to persist)
static ICM_20948_I2C icm;  // SparkFun ICM-20948 object
static struct bmm350_dev bmm_dev;       // BMM350 device structure

// VQF algorithm
static VQF *vqf = nullptr;

// Calibration data
float IMUManager::gyroBias[3] = {0.0f, 0.0f, 0.0f};
float IMUManager::accelBias[3] = {0.0f, 0.0f, 0.0f};
float IMUManager::accelScale[3] = {1.0f, 1.0f, 1.0f};
float IMUManager::magHardIron[3] = {0.0f, 0.0f, 0.0f};
float IMUManager::magSoftIron[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// Latest sensor readings (calibrated)
float IMUManager::accel[3] = {0.0f, 0.0f, 0.0f};
float IMUManager::gyro[3] = {0.0f, 0.0f, 0.0f};
float IMUManager::mag[3] = {0.0f, 0.0f, 0.0f};

// Latest sensor readings (raw)
float IMUManager::accelRaw[3] = {0.0f, 0.0f, 0.0f};
float IMUManager::gyroRaw[3] = {0.0f, 0.0f, 0.0f};
float IMUManager::magRaw[3] = {0.0f, 0.0f, 0.0f};

// VQF output
float IMUManager::quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float IMUManager::heading = 0.0f;
float IMUManager::roll = 0.0f;
float IMUManager::pitch = 0.0f;

// Statistics
uint32_t IMUManager::lastUpdateTime = 0;
uint32_t IMUManager::updateCount = 0;

// =============================================================================
// BMM350 I2C WRAPPER FUNCTIONS
// =============================================================================
// These functions are required by bmm350.c driver

// I2C read function for BMM350
int8_t bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission() != 0) {
        return BMM350_E_COM_FAIL;
    }
    
    Wire.requestFrom(dev_addr, (uint8_t)len);
    for (uint32_t i = 0; i < len; i++) {
        if (Wire.available()) {
            reg_data[i] = Wire.read();
        } else {
            return BMM350_E_COM_FAIL;
        }
    }
    
    return BMM350_OK;
}

// I2C write function for BMM350
int8_t bmm350_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) {
        Wire.write(reg_data[i]);
    }
    
    if (Wire.endTransmission() != 0) {
        return BMM350_E_COM_FAIL;
    }
    
    return BMM350_OK;
}

// Delay function for BMM350
void bmm350_delay_us(uint32_t period, void *intf_ptr) {
    delayMicroseconds(period);
}

// =============================================================================
// INITIALIZATION
// =============================================================================

bool IMUManager::init() {
    DEBUG_INFO("Initializing IMUManager (ICM-20948 + BMM350)...");
    
    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    RuntimeConfig& cfg = SystemSettings::getConfig();
    uint32_t i2cFreq = (cfg.i2cClockSpeed == 0) ? 100000 : 400000;
    Wire.setClock(i2cFreq);
    Wire.setTimeOut(50);  // 50ms timeout
    delay(100);
    
    DEBUG_INFO("  I2C initialized: %d kHz", i2cFreq / 1000);
    
    // Scan I2C bus to detect connected devices
    DEBUG_INFO("  Scanning I2C bus...");
    bool icmFound = false;
    bool bmmFound = false;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            DEBUG_INFO("    Found device at 0x%02X", addr);
            if (addr == ICM20948_I2C_ADDR || addr == 0x69) icmFound = true;
            if (addr == BMM350_I2C_ADDR) bmmFound = true;
        }
        Watchdog::feed();  // Feed watchdog during scan
    }
    
    if (!icmFound) {
        DEBUG_WARN("  ICM-20948 NOT FOUND on I2C bus!");
        DEBUG_WARN("  Expected at 0x68 or 0x69");
    }
    if (!bmmFound) {
        DEBUG_WARN("  BMM350 NOT FOUND on I2C bus!");
        DEBUG_WARN("  Expected at 0x%02X", BMM350_I2C_ADDR);
    }
    
    // Initialize ICM-20948 (Accel + Gyro)
    if (!initICM20948()) {
        DEBUG_WARN("ICM-20948 initialization failed - continuing with limited functionality");
        // Don't return false - allow system to continue with BMM350 and GPS
    }
    
    // Initialize BMM350 (Magnetometer)
    if (!initBMM350()) {
        DEBUG_ERROR("Failed to initialize BMM350");
        return false;
    }
    
    // Initialize VQF sensor fusion
    if (!initVQF()) {
        DEBUG_ERROR("Failed to initialize VQF");
        return false;
    }
    
    initialized = true;
    DEBUG_INFO("IMUManager initialized successfully");
    
    return true;
}

bool IMUManager::initICM20948() {
    DEBUG_INFO("Initializing ICM-20948...");
    
    // Initialize SparkFun ICM-20948 library
    icm.begin(Wire, 1);  // AD0_VAL = 1 for address 0x69, or 0 for 0x68
    
    if (!icm.status == ICM_20948_Stat_Ok) {
        // Try alternate address
        icm.begin(Wire, 0);
        if (!icm.status == ICM_20948_Stat_Ok) {
            DEBUG_ERROR("  ICM-20948 initialization failed");
            DEBUG_WARN("  Continuing without ICM-20948");
            return false;
        }
    }
    
    DEBUG_INFO("  ICM-20948 initialized successfully");
    
    // Apply saved configuration from SystemSettings
    RuntimeConfig& cfg = SystemSettings::getConfig();
    
    // Configure Gyroscope Range
    ICM_20948_fss_t fss;
    fss.g = cfg.gyroRange;  // 0=250dps, 1=500dps, 2=1000dps, 3=2000dps
    fss.a = cfg.accelRange; // 0=2g, 1=4g, 2=8g, 3=16g
    
    ICM_20948_Status_e status = icm.setFullScale(
        (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), 
        fss
    );
    
    if (status != ICM_20948_Stat_Ok) {
        DEBUG_WARN("  Failed to set gyro/accel range");
    } else {
        DEBUG_INFO("  Gyro range: %d dps, Accel range: %d g", 
                   250 << cfg.gyroRange, 2 << cfg.accelRange);
    }
    
    // Configure DLPF
    ICM_20948_dlpcfg_t dlpf;
    dlpf.g = cfg.gyroDLPF;   // Gyro DLPF config
    dlpf.a = cfg.accelDLPF;  // Accel DLPF config
    
    status = icm.setDLPFcfg(
        (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
        dlpf
    );
    
    if (status != ICM_20948_Stat_Ok) {
        DEBUG_WARN("  Failed to set DLPF");
    } else {
        DEBUG_INFO("  Gyro DLPF: %d, Accel DLPF: %d", cfg.gyroDLPF, cfg.accelDLPF);
    }
    
    // Enable DLPF
    status = icm.enableDLPF(
        (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
        true
    );
    
    if (status != ICM_20948_Stat_Ok) {
        DEBUG_WARN("  Failed to enable DLPF");
    }
    
    DEBUG_INFO("  Accel + Gyro: ENABLED");
    DEBUG_INFO("  AK09916 Mag: DISABLED (using BMM350)");
    
    return true;
}

bool IMUManager::initBMM350() {
    DEBUG_INFO("Initializing BMM350...");
    
    // Use file-scope bmm_dev
    static uint8_t bmm_addr = BMM350_I2C_ADDR;
    
    // Configure BMM350 device structure
    bmm_dev.intfPtr = &bmm_addr;         // Interface pointer (I2C address)
    bmm_dev.read = bmm350_i2c_read;       // Read function
    bmm_dev.write = bmm350_i2c_write;     // Write function
    bmm_dev.delayUs = bmm350_delay_us;    // Delay function
    // Note: BMM350 API doesn't use 'intf' enum like BMI270
    
    // Initialize BMM350
    int8_t rslt = bmm350Init(&bmm_dev);
    if (rslt != BMM350_OK) {
        DEBUG_ERROR("  BMM350 init failed: %d", rslt);
        DEBUG_WARN("  Check I2C address (expected: 0x%02X)", BMM350_I2C_ADDR);
        DEBUG_WARN("  Verify hardware is connected and powered");
        return false;
    }
    
    DEBUG_INFO("  BMM350 initialized at I2C address 0x%02X", BMM350_I2C_ADDR);

    // Enable all axes
    if (bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &bmm_dev) != BMM350_OK) {
        DEBUG_ERROR("  Failed to enable BMM350 axes!");
        return false;
    }
    
    // Enable Data Ready Interrupt (needed for status register update)
    if (bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &bmm_dev) != BMM350_OK) {
        DEBUG_ERROR("  Failed to enable BMM350 interrupt!");
        return false;
    }
    
    // Configure BMM350 ODR, averaging, preset from settings
    RuntimeConfig& cfg = SystemSettings::getConfig();
    
    // Convert float ODR to enum
    enum eBmm350DataRates_t odr_enum;
    if (cfg.magODR >= 400.0f) odr_enum = BMM350_DATA_RATE_400HZ;
    else if (cfg.magODR >= 200.0f) odr_enum = BMM350_DATA_RATE_200HZ;
    else if (cfg.magODR >= 100.0f) odr_enum = BMM350_DATA_RATE_100HZ;
    else if (cfg.magODR >= 50.0f) odr_enum = BMM350_DATA_RATE_50HZ;
    else if (cfg.magODR >= 25.0f) odr_enum = BMM350_DATA_RATE_25HZ;
    else if (cfg.magODR >= 12.5f) odr_enum = BMM350_DATA_RATE_12_5HZ;
    else if (cfg.magODR >= 6.25f) odr_enum = BMM350_DATA_RATE_6_25HZ;
    else if (cfg.magODR >= 3.125f) odr_enum = BMM350_DATA_RATE_3_125HZ;
    else odr_enum = BMM350_DATA_RATE_1_5625HZ;
    
    // Convert averaging to enum
    enum bmm350_performance_parameters avg_enum;
    switch(cfg.magAveraging) {
        case 0: avg_enum = BMM350_NO_AVERAGING; break;
        case 1: avg_enum = BMM350_AVERAGING_2; break;
        case 2: avg_enum = BMM350_AVERAGING_4; break;
        case 3: avg_enum = BMM350_AVERAGING_8; break;
        default: avg_enum = BMM350_AVERAGING_2; break;
    }
    
    // Apply ODR and averaging
    rslt = bmm350SetOdrPerformance(odr_enum, avg_enum, &bmm_dev);
    if (rslt != BMM350_OK) {
        DEBUG_WARN("  Failed to set BMM350 ODR/averaging: %d", rslt);
    } else {
        DEBUG_INFO("  BMM350 configured: ODR=%.2f Hz, Avg=%d samples", 
                   cfg.magODR, (1 << cfg.magAveraging));
    }
    
    // Set BMM350 to normal mode
    rslt = bmm350SetPowerMode(eBmm350NormalMode, &bmm_dev);
    if (rslt != BMM350_OK) {
        DEBUG_WARN("  Failed to set BMM350 power mode: %d", rslt);
    } else {
        DEBUG_INFO("  BMM350 power mode: NORMAL");
    }
    
    return true;
}

bool IMUManager::initVQF() {
    DEBUG_INFO("Initializing VQF sensor fusion...");
    
    RuntimeConfig& cfg = SystemSettings::getConfig();
    
    // Calculate sensor timesteps from configuration
    // Use configured Loop Rate
    uint16_t rate = SystemSettings::getLoopRate();
    float ts = 1.0f / (float)rate;
    
    // VQF timesteps must match the rate at which we call vqf->update()
    // Since we call it every loop cycle, all timesteps = loop period.
    float gyrTs = ts;
    float accTs = ts;
    float magTs = ts;
    
    // Create VQF instance
    vqf = new VQF(gyrTs, accTs, magTs);
    
    if (!vqf) {
        DEBUG_ERROR("  Failed to create VQF instance");
        return false;
    }
    
    // Apply VQF tuning parameters from settings
    vqf->setTauAcc(cfg.vqfTauAcc);
    vqf->setTauMag(cfg.vqfTauMag);
    vqf->setMagDistRejectionEnabled(cfg.vqfMagRejection);
    
    DEBUG_INFO("  VQF initialized:");
    DEBUG_INFO("    Timesteps: Gyr=%.4fs, Acc=%.4fs, Mag=%.4fs", gyrTs, accTs, magTs);
    DEBUG_INFO("    Params: TauAcc=%.1f, TauMag=%.1f, MagReject=%s", 
               cfg.vqfTauAcc, cfg.vqfTauMag, cfg.vqfMagRejection ? "ON" : "OFF");
    
    return true;
}

// =============================================================================
// UPDATE
// =============================================================================

void IMUManager::update() {
    if (!initialized) return;
    
    // Feed watchdog
    Watchdog::feed();
    
    // Read ICM-20948 data using SparkFun library
    if (icm.dataReady()) {
        icm.getAGMT();  // Updates internal data
        
        // Read accelerometer (convert from mg to m/s²)
        // Using standard gravity: 9.80665 m/s² (same as calibration)
        accelRaw[0] = icm.accX() * 9.80665f / 1000.0f;
        accelRaw[1] = icm.accY() * 9.80665f / 1000.0f;
        accelRaw[2] = icm.accZ() * 9.80665f / 1000.0f;
        
        // Read gyroscope (convert from dps to rad/s)
        gyroRaw[0] = icm.gyrX() * (PI / 180.0f);
        gyroRaw[1] = icm.gyrY() * (PI / 180.0f);
        gyroRaw[2] = icm.gyrZ() * (PI / 180.0f);
    }
    
    
    // Read BMM350 magnetometer (µT)
    // Check if it's time to read Mag
    static uint32_t lastMagUpdate = 0;
    const RuntimeConfig& cfg = SystemSettings::getConfig();
    float magInterval = 1000.0f / (cfg.magODR > 0 ? cfg.magODR : 12.5f);
    
    // Structure to hold data
    struct sBmm350MagTempData_t mag_temp_data;
    bool newDataAvailable = false;
    
    // Logic: ALWAYS POLL DATA READY (User Request - Normal Mode)
    // We expect the sensor to be in Normal Mode running at ODR.
    // We check DRDY bit to know when data is valid.
    
    uint8_t int_status = 0;
    // BMM350_REG_INT_STATUS = 0x30, DRDY Bit = Bit 2 (0x04)
    int8_t rslt = bmm350GetRegs(0x30, &int_status, 1, &bmm_dev); 
    
    if (rslt == BMM350_OK && (int_status & 0x04)) {
         // Data Ready! Read it.
         if (bmm350GetCompensatedMagXYZTempData(&mag_temp_data, &bmm_dev) == BMM350_OK) {
            newDataAvailable = true;
         }
    }
    
    if (newDataAvailable) {
        // Data is already in µT (microtesla)
        // ALIGNMENT FIX: BMM350 is rotated 180 deg relative to ICM-20948
        // ICM Frame: X-Fwd, Y-Left, Z-Up
        // BMM Frame: X-Fwd, Y-Right, Z-Down
        // Transformation: X'=X, Y'=-Y, Z'=-Z
        magRaw[0] = mag_temp_data.x;
        magRaw[1] = -mag_temp_data.y;
        magRaw[2] = -mag_temp_data.z;
    }
    
    
    // Copy raw values to calibrated arrays
    accel[0] = accelRaw[0];
    accel[1] = accelRaw[1];
    accel[2] = accelRaw[2];
    
    gyro[0] = gyroRaw[0];
    gyro[1] = gyroRaw[1];
    gyro[2] = gyroRaw[2];
    
    mag[0] = magRaw[0];
    mag[1] = magRaw[1];
    mag[2] = magRaw[2];
    
    // Apply calibration to the copied values
    // CRITICAL: CalibrationManager expects specific units!
    // - Accel calibration is in G, but IMUManager uses m/s²
    // - Gyro calibration is in deg/s, but IMUManager uses rad/s
    
    // Convert accel from m/s² to g for calibration
    accel[0] /= 9.80665f;
    accel[1] /= 9.80665f;
    accel[2] /= 9.80665f;
    
    // Convert gyro from rad/s to deg/s for calibration
    gyro[0] *= RAD_TO_DEG;
    gyro[1] *= RAD_TO_DEG;
    gyro[2] *= RAD_TO_DEG;
    
    // Apply calibration (in g and deg/s)
    CalibrationManager::applyAccel(accel[0], accel[1], accel[2]);
    CalibrationManager::applyGyro(gyro[0], gyro[1], gyro[2]);
    CalibrationManager::applyMag(mag[0], mag[1], mag[2]);
    
    // Convert back to IMUManager units (m/s² and rad/s)
    accel[0] *= 9.80665f;
    accel[1] *= 9.80665f;
    accel[2] *= 9.80665f;
    
    gyro[0] *= DEG_TO_RAD;
    gyro[1] *= DEG_TO_RAD;
    gyro[2] *= DEG_TO_RAD;
    
    // Update VQF
    if (vqf) {
        vqf_real_t gyr[3] = {gyro[0], gyro[1], gyro[2]};
        vqf_real_t acc[3] = {accel[0], accel[1], accel[2]};
        vqf_real_t mg[3] = {mag[0], mag[1], mag[2]};
        
        vqf->update(gyr, acc, mg);
        
        // Get quaternion
        vqf_real_t q[4];
        vqf->getQuat9D(q);
        quat[0] = q[0];
        quat[1] = q[1];
        quat[2] = q[2];
        quat[3] = q[3];
        
        // Calculate orientation
        calculateOrientation();
    }
    
    lastUpdateTime = millis();
    updateCount++;
}

// =============================================================================
// CALIBRATION APPLICATION
// =============================================================================

void IMUManager::applyAccelCalibration(float& ax, float& ay, float& az) {
    // Apply bias
    ax -= accelBias[0];
    ay -= accelBias[1];
    az -= accelBias[2];
    
    // Apply scale
    ax *= accelScale[0];
    ay *= accelScale[1];
    az *= accelScale[2];
}

void IMUManager::applyGyroCalibration(float& gx, float& gy, float& gz) {
    // Apply bias
    gx -= gyroBias[0];
    gy -= gyroBias[1];
    gz -= gyroBias[2];
}

void IMUManager::applyMagCalibration(float& mx, float& my, float& mz) {
    // Apply hard iron offset
    float x = mx - magHardIron[0];
    float y = my - magHardIron[1];
    float z = mz - magHardIron[2];
    
    // Apply soft iron matrix correction
    mx = magSoftIron[0][0] * x + magSoftIron[0][1] * y + magSoftIron[0][2] * z;
    my = magSoftIron[1][0] * x + magSoftIron[1][1] * y + magSoftIron[1][2] * z;
    mz = magSoftIron[2][0] * x + magSoftIron[2][1] * y + magSoftIron[2][2] * z;
}

// =============================================================================
// ORIENTATION CALCULATION
// =============================================================================

void IMUManager::calculateOrientation() {
    // Convert quaternion to Euler angles
    // q[0]=w, q[1]=x, q[2]=y, q[3]=z
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (quat[0] * quat[1] + quat[2] * quat[3]);
    float cosr_cosp = 1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]);
    roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (quat[0] * quat[2] - quat[3] * quat[1]);
    if (fabs(sinp) >= 1.0f)
        pitch = copysign(90.0f, sinp);
    else
        pitch = asin(sinp) * 180.0f / M_PI;
    
    // Yaw (z-axis rotation) - VQF uses ENU frame
    float siny_cosp = 2.0f * (quat[0] * quat[3] + quat[1] * quat[2]);
    float cosy_cosp = 1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]);
    float yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / M_PI;
    
    // Convert ENU yaw to compass heading (0=North, 90=East, 180=South, 270=West)
    heading = 90.0f - yaw;
    if (heading < 0.0f) heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;
}

// =============================================================================
// PUBLIC ACCESSORS
// =============================================================================

bool IMUManager::getAccel(float& ax, float& ay, float& az) {
    if (!initialized) return false;
    ax = accel[0];
    ay = accel[1];
    az = accel[2];
    return true;
}

bool IMUManager::getGyro(float& gx, float& gy, float& gz) {
    if (!initialized) return false;
    gx = gyro[0];
    gy = gyro[1];
    gz = gyro[2];
    return true;
}

bool IMUManager::getMag(float& mx, float& my, float& mz) {
    if (!initialized) return false;
    mx = mag[0];
    my = mag[1];
    mz = mag[2];
    return true;
}

bool IMUManager::getAccelRaw(float& ax, float& ay, float& az) {
    if (!initialized) return false;
    ax = accelRaw[0];
    ay = accelRaw[1];
    az = accelRaw[2];
    return true;
}

bool IMUManager::getGyroRaw(float& gx, float& gy, float& gz) {
    if (!initialized) return false;
    gx = gyroRaw[0];
    gy = gyroRaw[1];
    gz = gyroRaw[2];
    return true;
}

bool IMUManager::getMagRaw(float& mx, float& my, float& mz) {
    if (!initialized) return false;
    mx = magRaw[0];
    my = magRaw[1];
    mz = magRaw[2];
    return true;
}

bool IMUManager::getQuaternion(float& w, float& x, float& y, float& z) {
    if (!initialized) return false;
    w = quat[0];
    x = quat[1];
    y = quat[2];
    z = quat[3];
    return true;
}

float IMUManager::getCompassHeading() {
    return heading;
}

float IMUManager::getRoll() {
    return roll;
}

float IMUManager::getPitch() {
    return pitch;
}

bool IMUManager::isInitialized() {
    return initialized;
}

uint32_t IMUManager::getLastUpdateTime() {
    return lastUpdateTime;
}

uint32_t IMUManager::getUpdateCount() {
    return updateCount;
}

// =============================================================================
// CONFIGURATION SETTERS
// =============================================================================

void IMUManager::setGyroBias(float bx, float by, float bz) {
    gyroBias[0] = bx;
    gyroBias[1] = by;
    gyroBias[2] = bz;
}

void IMUManager::setAccelBias(float bx, float by, float bz) {
    accelBias[0] = bx;
    accelBias[1] = by;
    accelBias[2] = bz;
}

void IMUManager::setAccelScale(float sx, float sy, float sz) {
    accelScale[0] = sx;
    accelScale[1] = sy;
    accelScale[2] = sz;
}

void IMUManager::setMagCalibration(float hx, float hy, float hz, float soft[3][3]) {
    magHardIron[0] = hx;
    magHardIron[1] = hy;
    magHardIron[2] = hz;
    
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            magSoftIron[i][j] = soft[i][j];
        }
    }
}

// =============================================================================
// SENSOR CONFIGURATION (STUBS)
// =============================================================================

bool IMUManager::setGyroRange(uint8_t range) {
    // TODO: Implement ICM-20948 gyro range configuration
    DEBUG_WARN("setGyroRange() STUB - not yet implemented");
    return false;
}

bool IMUManager::setGyroDLPF(uint8_t dlpf) {
    if (dlpf > 7) dlpf = 7;
    
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.gyroDLPF = dlpf;
    
    if (initialized) {
        ICM_20948_dlpcfg_t dlpf_cfg;
        dlpf_cfg.g = dlpf;
        dlpf_cfg.a = cfg.accelDLPF;
        
        ICM_20948_Status_e status = icm.setDLPFcfg(
            (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
            dlpf_cfg
        );
        
        if (status == ICM_20948_Stat_Ok) {
            DEBUG_INFO("Gyro DLPF set to %d", dlpf);
            return true;
        } else {
            DEBUG_WARN("Failed to set Gyro DLPF");
            return false;
        }
    }
    return true;
}

bool IMUManager::setAccelRange(uint8_t range) {
    if (range > 3) range = 3;
    
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.accelRange = range;
    
    if (initialized) {
        ICM_20948_fss_t fss;
        fss.a = range;
        fss.g = cfg.gyroRange;
        
        ICM_20948_Status_e status = icm.setFullScale(
            (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
            fss
        );
        
        if (status == ICM_20948_Stat_Ok) {
            DEBUG_INFO("Accel Range set to %d (%dg)", range, (2 << range));
            return true;
        } else {
            DEBUG_WARN("Failed to set Accel Range");
            return false;
        }
    }
    return true;
}

bool IMUManager::setAccelDLPF(uint8_t dlpf) {
    if (dlpf > 7) dlpf = 7;
    
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.accelDLPF = dlpf;
    
    if (initialized) {
        ICM_20948_dlpcfg_t dlpf_cfg;
        dlpf_cfg.a = dlpf;
        dlpf_cfg.g = cfg.gyroDLPF;
        
        ICM_20948_Status_e status = icm.setDLPFcfg(
            (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
            dlpf_cfg
        );
        
        if (status == ICM_20948_Stat_Ok) {
            DEBUG_INFO("Accel DLPF set to %d", dlpf);
            return true;
        } else {
            DEBUG_WARN("Failed to set Accel DLPF");
            return false;
        }
    }
    return true;
}

bool IMUManager::setMagODR(uint8_t odr) {
    // Map index to float ODR
    float odr_val = 12.5f;
    switch(odr) {
        case 0: odr_val = 1.5625f; break;
        case 1: odr_val = 3.125f; break;
        case 2: odr_val = 6.25f; break;
        case 3: odr_val = 12.5f; break;
        case 4: odr_val = 25.0f; break;
        case 5: odr_val = 50.0f; break;
        case 6: odr_val = 100.0f; break;
        case 7: odr_val = 200.0f; break;
        case 8: odr_val = 400.0f; break;
        default: odr_val = 12.5f; break;
    }
    
    applyMagODR(odr_val);
    return true;
}

bool IMUManager::setMagAveraging(uint8_t avg) {
    if (avg > 3) avg = 3;
    
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.magAveraging = avg;
    
    // Apply (reuses applyMagODR logic which sets both)
    applyMagODR(cfg.magODR);
    
    return true;
}

static uint8_t currentMagPowerMode = 1; // 1=Normal Mode (default)

uint8_t IMUManager::getMagPowerMode() {
    return currentMagPowerMode;
}

bool IMUManager::setMagPowerMode(uint8_t mode) {
    if (mode != 1 && mode != 4) {
        DEBUG_WARN("Invalid power mode %d (use 1=Normal or 4=ForcedFast)", mode);
        return false;
    }
    
    currentMagPowerMode = mode;
    
    // Changing power mode manually invalidates the preset
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.magPreset = 0; // 0 = Custom/Advanced
    
    if (initialized) {
        enum eBmm350PowerModes_t powerMode;
        if (mode == 1) powerMode = eBmm350NormalMode;
        else powerMode = eBmm350ForcedModeFast;
        
        int8_t rslt = bmm350SetPowerMode(powerMode, &bmm_dev);
        if (rslt != BMM350_OK) {
            DEBUG_WARN("Failed to set BMM350 power mode: %d", rslt);
            return false;
        }
        
        DEBUG_INFO("BMM350 power mode set to: %s", (mode == 1) ? "Normal" : "Forced Fast");
    }
    
    return true;
}

bool IMUManager::setMagPreset(uint8_t preset) {
    // TODO: Implement BMM350 preset mode configuration
    DEBUG_WARN("setMagPreset() STUB - not yet implemented");
    return false;
}

bool IMUManager::setLoopRate(uint16_t rate) {
    if (rate != 25 && rate != 50 && rate != 100 && rate != 200) {
        DEBUG_WARN("Invalid loop rate %d Hz (allowed: 25, 50, 100, 200)", rate);
        return false;
    }
    
    RuntimeConfig& cfg = SystemSettings::getConfig();
    // Save new rate via SystemSettings
    SystemSettings::setLoopRate(rate);
    
    if (initialized) {
        DEBUG_INFO("Loop rate changed to %d Hz - Re-initializing VQF...", rate);
        
        // Calculate new timesteps (dt = 1/rate)
        float ts = 1.0f / (float)rate;
        
        // Mag timestep logic:
        // Since we call vqf->update() every loop cycle regardless of sensor ODR,
        // we must effectively tell VQF that the timestep is the loop period.
        // It will "hold" the last mag value and integrate it over small steps.
        float magTs = ts;
        
        // Destroy old instance
        if (vqf) {
            delete vqf;
            vqf = nullptr;
        }
        
        // Create new instance
        vqf = new VQF(ts, ts, magTs);
        
        if (vqf) {
            // Re-apply params
            vqf->setTauAcc(cfg.vqfTauAcc);
            vqf->setTauMag(cfg.vqfTauMag);
            vqf->setMagDistRejectionEnabled(cfg.vqfMagRejection);
            DEBUG_INFO("VQF re-initialized (Gyr/Acc dt=%.4fs, Mag dt=%.4fs)", ts, magTs);
        } else {
            DEBUG_ERROR("Failed to recreate VQF!");
            return false;
        }
    }
    
    return true;
}

// =============================================================================
// DIAGNOSTICS
// =============================================================================

void IMUManager::printAudit() {
    DEBUG_INFO("===============================================");
    DEBUG_INFO("IMU MANAGER ADVERSARIAL AUDIT");
    DEBUG_INFO("===============================================");
    DEBUG_INFO("");
    
    DEBUG_INFO("PHASE 1: ICM-20948 Verification");
    DEBUG_INFO("  TODO: Implement ICM-20948 datasheet verification");
    DEBUG_INFO("  - WHO_AM_I check (expected: 0xEA)");
    DEBUG_INFO("  - Register configuration readback");
    DEBUG_INFO("  - Raw sensor data validation");
    DEBUG_INFO("");
    
    DEBUG_INFO("PHASE 2: BMM350 Verification");
    DEBUG_INFO("  TODO: Implement BMM350 datasheet verification");
    DEBUG_INFO("  - Chip ID check (expected: 0x33)");
    DEBUG_INFO("  - ODR/averaging configuration check");
    DEBUG_INFO("  - Field magnitude validation (25-65 µT)");
    DEBUG_INFO("");
    
    DEBUG_INFO("PHASE 3: Library Verification");
    DEBUG_INFO("  ICM-20948 Library: isouriadakis/Arduino_ICM20948_DMP_Full-Function");
    DEBUG_INFO("  BMM350 Driver: Bosch core driver (extracted from DFRobot_BMM350)");
    DEBUG_INFO("");
    
    DEBUG_INFO("PHASE 4: IMUManager Transformations");
    DEBUG_INFO("  TODO: Verify axis mapping");
    DEBUG_INFO("  TODO: Verify unit conversions");
    DEBUG_INFO("  TODO: Verify calibration application");
    DEBUG_INFO("");
    
    DEBUG_INFO("PHASE 5: VQF Integration");
    DEBUG_INFO("  VQF State: %s", vqf ? "INITIALIZED" : "NOT INITIALIZED");
    if (vqf) {
        DEBUG_INFO("  Coordinate Frame: ENU (East-North-Up)");
        DEBUG_INFO("  Heading Formula: heading = 90° - yaw");
    }
    DEBUG_INFO("");
    
    DEBUG_INFO("PHASE 6: Final Verdict");
    DEBUG_WARN("  ⚠ HARDWARE INTEGRATION INCOMPLETE");
    DEBUG_WARN("  ⚠ ICM-20948 driver not yet integrated");
    DEBUG_WARN("  ⚠ BMM350 axis mapping not yet determined");
    DEBUG_WARN("  ⚠ Full verification requires physical hardware");
    DEBUG_INFO("");
    DEBUG_INFO("===============================================");
}

void IMUManager::printRawSensors() {
    DEBUG_INFO("Raw Sensor Data:");
    DEBUG_INFO("  Accel: [%.3f, %.3f, %.3f] m/s²", accelRaw[0], accelRaw[1], accelRaw[2]);
    DEBUG_INFO("  Gyro:  [%.3f, %.3f, %.3f] rad/s", gyroRaw[0], gyroRaw[1], gyroRaw[2]);
    DEBUG_INFO("  Mag:   [%.1f, %.1f, %.1f] µT", magRaw[0], magRaw[1], magRaw[2]);
}

void IMUManager::printCalibratedSensors() {
    DEBUG_INFO("Calibrated Sensor Data:");
    DEBUG_INFO("  Accel: [%.3f, %.3f, %.3f] m/s²", accel[0], accel[1], accel[2]);
    DEBUG_INFO("  Gyro:  [%.3f, %.3f, %.3f] rad/s", gyro[0], gyro[1], gyro[2]);
    DEBUG_INFO("  Mag:   [%.1f, %.1f, %.1f] µT", mag[0], mag[1], mag[2]);
}

void IMUManager::printVQFStatus() {
    DEBUG_INFO("VQF Status:");
    DEBUG_INFO("  Quaternion: [%.4f, %.4f, %.4f, %.4f]", quat[0], quat[1], quat[2], quat[3]);
    DEBUG_INFO("  Heading: %.1f°", heading);
    DEBUG_INFO("  Roll: %.1f°", roll);
    DEBUG_INFO("  Pitch: %.1f°", pitch);
    DEBUG_INFO("  Update Count: %u", updateCount);
}

void IMUManager::deinit() {
    if (vqf) {
        delete vqf;
        vqf = nullptr;
    }
    
    // TODO: Deinitialize ICM-20948
    // TODO: Deinitialize BMM350
    
    initialized = false;
    DEBUG_INFO("IMUManager deinitialized");
}

// ============================================================================
// PRESET APPLICATION FUNCTIONS (for BASE Code menu compatibility)
// ============================================================================

void IMUManager::applyAccelPreset(uint8_t preset) {
    // Preset mapping for ICM-20948:
    // 1 = Precise (2g), 2 = Default (4g), 3 = Sports (8g), 4+ = Extreme (16g)
    uint8_t range = (preset <= 1) ? 0 : (preset - 1);
    if (range > 3) range = 3;
    
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.accelRange = range;
    cfg.accelPreset = preset;
    
    // Apply to hardware
    if (initialized) {
        ICM_20948_fss_t fss;
        fss.a = range;
        fss.g = cfg.gyroRange;  // Keep current gyro range
        
        ICM_20948_Status_e status = icm.setFullScale(
            (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
            fss
        );
        
        if (status != ICM_20948_Stat_Ok) {
            DEBUG_WARN("Failed to apply accel range to hardware");
        }
    }
    
    DEBUG_INFO("Accel preset %d set (range=%dg)", preset, (1 << (range+1)));
}

void IMUManager::applyGyroPreset(uint8_t preset) {
    // Preset mapping: 1=250dps, 2=500dps, 3=1000dps, 4=2000dps
    uint8_t range = (preset <= 1) ? 0 : (preset - 1);
    if (range > 3) range = 3;
    
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.gyroRange = range;
    cfg.gyroPreset = preset;
    
    // Apply to hardware
    if (initialized) {
        ICM_20948_fss_t fss;
        fss.g = range;
        fss.a = cfg.accelRange;  // Keep current accel range
        
        ICM_20948_Status_e status = icm.setFullScale(
            (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
            fss
        );
        
        if (status != ICM_20948_Stat_Ok) {
            DEBUG_WARN("Failed to apply gyro range to hardware");
        }
    }
    
    DEBUG_INFO("Gyro preset %d set (range=%ddps)", preset, 250 << range);
}

void IMUManager::applyMagPreset(uint8_t preset) {
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.magPreset = preset;
    
    // Map presets to averaging levels
    switch(preset) {
        case 1: cfg.magAveraging = 0; break; // LOW POWER
        case 2: cfg.magAveraging = 1; break; // REGULAR
        case 3: cfg.magAveraging = 2; break; // LOW NOISE
        case 4: cfg.magAveraging = 3; break; // ULTRA LOW NOISE
        default: cfg.magAveraging = 1; break;
    }
    
    // Apply to hardware
    if (initialized) {
        enum bmm350_performance_parameters avg_enum;
        switch(cfg.magAveraging) {
            case 0: avg_enum = BMM350_NO_AVERAGING; break;
            case 1: avg_enum = BMM350_AVERAGING_2; break;
            case 2: avg_enum = BMM350_AVERAGING_4; break;
            case 3: avg_enum = BMM350_AVERAGING_8; break;
            default: avg_enum = BMM350_AVERAGING_2; break;
        }
        
        // Convert current ODR to enum
        enum eBmm350DataRates_t odr_enum;
        if (cfg.magODR >= 400.0f) odr_enum = BMM350_DATA_RATE_400HZ;
        else if (cfg.magODR >= 200.0f) odr_enum = BMM350_DATA_RATE_200HZ;
        else if (cfg.magODR >= 100.0f) odr_enum = BMM350_DATA_RATE_100HZ;
        else if (cfg.magODR >= 50.0f) odr_enum = BMM350_DATA_RATE_50HZ;
        else if (cfg.magODR >= 25.0f) odr_enum = BMM350_DATA_RATE_25HZ;
        else if (cfg.magODR >= 12.5f) odr_enum = BMM350_DATA_RATE_12_5HZ;
        else if (cfg.magODR >= 6.25f) odr_enum = BMM350_DATA_RATE_6_25HZ;
        else if (cfg.magODR >= 3.125f) odr_enum = BMM350_DATA_RATE_3_125HZ;
        else odr_enum = BMM350_DATA_RATE_1_5625HZ;
        
        int8_t rslt = bmm350SetOdrPerformance(odr_enum, avg_enum, &bmm_dev);
        if (rslt != BMM350_OK) {
            DEBUG_WARN("Failed to apply mag averaging to hardware");
        }
    }
    
    DEBUG_INFO("Mag preset %d set (averaging=%d)", preset, cfg.magAveraging);
}

void IMUManager::applyMagODR(float odr) {
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.magODR = odr;
    cfg.magPreset = 0; // Manual config invalidates preset (0 = Custom)
    
    // Apply to hardware
    if (initialized) {
        // Convert ODR to enum
        enum eBmm350DataRates_t odr_enum;
        if (odr >= 400.0f) odr_enum = BMM350_DATA_RATE_400HZ;
        else if (odr >= 200.0f) odr_enum = BMM350_DATA_RATE_200HZ;
        else if (odr >= 100.0f) odr_enum = BMM350_DATA_RATE_100HZ;
        else if (odr >= 50.0f) odr_enum = BMM350_DATA_RATE_50HZ;
        else if (odr >= 25.0f) odr_enum = BMM350_DATA_RATE_25HZ;
        else if (odr >= 12.5f) odr_enum = BMM350_DATA_RATE_12_5HZ;
        else if (odr >= 6.25f) odr_enum = BMM350_DATA_RATE_6_25HZ;
        else if (odr >= 3.125f) odr_enum = BMM350_DATA_RATE_3_125HZ;
        else odr_enum = BMM350_DATA_RATE_1_5625HZ;
        
        // Convert current averaging to enum
        enum bmm350_performance_parameters avg_enum;
        switch(cfg.magAveraging) {
            case 0: avg_enum = BMM350_NO_AVERAGING; break;
            case 1: avg_enum = BMM350_AVERAGING_2; break;
            case 2: avg_enum = BMM350_AVERAGING_4; break;
            case 3: avg_enum = BMM350_AVERAGING_8; break;
            default: avg_enum = BMM350_AVERAGING_2; break;
        }
        
        int8_t rslt = bmm350SetOdrPerformance(odr_enum, avg_enum, &bmm_dev);
        if (rslt != BMM350_OK) {
            DEBUG_WARN("Failed to apply mag ODR to hardware");
        } else {
            // Read back the actual ODR register to verify
            uint8_t actual_odr_reg = 0;
            rslt = bmm350GetRegs(BMM350_REG_PMU_CMD_AGGR_SET, &actual_odr_reg, 1, &bmm_dev);
            if (rslt == BMM350_OK) {
                uint8_t actual_odr_bits = actual_odr_reg & 0x0F; // Lower 4 bits are ODR
                DEBUG_INFO("BMM350 ODR Register readback: 0x%02X (ODR bits: 0x%X, requested: 0x%X)",
                          actual_odr_reg, actual_odr_bits, (uint8_t)odr_enum);
            }
            
            // SMART MODE REMOVED (User Request)
            // Always run in Normal Mode.
            // High ODRs (50Hz+) rely on correct Averaging settings to work.
            DEBUG_INFO("Setting Power Mode: NORMAL");
            setMagPowerMode(1); // 1 = Normal Mode
            bmm350SetPowerMode(eBmm350NormalMode, &bmm_dev);
        }
    }
    
    DEBUG_INFO("Mag ODR set to %.4f Hz", odr);
}

// BMM350 Configuration Validation
bool IMUManager::isValidBMM350Config(float odr, uint8_t averaging) {
    // Rule 1: 400 Hz only works with averaging 0-1 (not enough time for 4 or 8 samples)
    if (odr >= 400.0f && averaging > 1) {
        return false;
    }
    
    // Rule 2: 200 Hz only works with averaging 0-2 (not enough time for 8 samples)
    if (odr >= 200.0f && averaging > 2) {
        return false;
    }
    
    // All other combinations are valid
    return true;
}

const char* IMUManager::getBMM350ValidationError(float odr, uint8_t averaging) {
    if (odr >= 400.0f && averaging > 1) {
        return "400 Hz requires averaging 0 or 1 (not enough time for 4/8 samples)";
    }
    
    if (odr >= 200.0f && averaging > 2) {
        return "200 Hz requires averaging 0, 1, or 2 (not enough time for 8 samples)";
    }
    
    return nullptr; // Valid configuration
}

void IMUManager::setVQFParams(float tauAcc, float tauMag, bool magReject) {
    if (vqf) {
        vqf->setTauAcc(tauAcc);
        vqf->setTauMag(tauMag);
        vqf->setMagDistRejectionEnabled(magReject);
        DEBUG_INFO("VQF params: TauAcc=%.1f TauMag=%.1f MagReject=%d", 
                   tauAcc, tauMag, magReject);
    }
}

// ============================================================================
// DIAGNOSTIC TOOLS
// ============================================================================

void IMUManager::runAxisAlignmentDiagnostic() {
    DEBUG_INFO("========================================");
    DEBUG_INFO("  AXIS ALIGNMENT DIAGNOSTIC");
    DEBUG_INFO("========================================");
    DEBUG_INFO("Move device to see live sensor data");
    DEBUG_INFO("Press any key to exit...");
    DEBUG_INFO("");
    
    while (!Serial.available()) {
        update();
        Watchdog::feed();
        
        Serial.printf("Accel: X=%7.2f Y=%7.2f Z=%7.2f m/s²\n", accel[0], accel[1], accel[2]);
        Serial.printf("Gyro:  X=%7.2f Y=%7.2f Z=%7.2f rad/s\n", gyro[0], gyro[1], gyro[2]);
        Serial.printf("Mag:   X=%7.1f Y=%7.1f Z=%7.1f µT\n", mag[0], mag[1], mag[2]);
        Serial.printf("Heading: %.1f°\n", heading);
        Serial.println();
        
        delay(500);
    }
    while (Serial.available()) Serial.read();
}

void IMUManager::printHeadingDebug() {
    DEBUG_INFO("========================================");
    DEBUG_INFO("  VQF HEADING DEBUG");
    DEBUG_INFO("========================================");
    DEBUG_INFO("Quaternion: w=%.4f x=%.4f y=%.4f z=%.4f", 
               quat[0], quat[1], quat[2], quat[3]);
    DEBUG_INFO("Euler: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°", 
               roll, pitch, heading);
    DEBUG_INFO("Mag: X=%.1f Y=%.1f Z=%.1f µT", mag[0], mag[1], mag[2]);
    DEBUG_INFO("Updates: %u", updateCount);
    DEBUG_INFO("========================================");
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

uint8_t IMUManager::getCurrentAccelRange() {
    return SystemSettings::getConfig().accelRange;
}

uint8_t IMUManager::getCurrentGyroRange() {
    return SystemSettings::getConfig().gyroRange;
}

bool IMUManager::isAvailable() {
    return initialized;
}

bool IMUManager::checkHealth() {
    return (initialized && updateCount > 0);
}

// ============================================================================
// SENSOR DATA ACCESSORS (for BASE Code compatibility)
// ============================================================================

float IMUManager::getAccelX() { return accel[0]; }
float IMUManager::getAccelY() { return accel[1]; }
float IMUManager::getAccelZ() { return accel[2]; }
float IMUManager::getGyroX() { return gyro[0]; }
float IMUManager::getGyroY() { return gyro[1]; }
float IMUManager::getGyroZ() { return gyro[2]; }
float IMUManager::getMagX() { return mag[0]; }
float IMUManager::getMagY() { return mag[1]; }
float IMUManager::getMagZ() { return mag[2]; }

// I2C error tracking (stub for calibration compatibility)
static uint32_t i2cErrorCount = 0;
uint32_t IMUManager::getI2CErrorCount() {
    return i2cErrorCount;
}

float IMUManager::getHeading() { return heading; }
