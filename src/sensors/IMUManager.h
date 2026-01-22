#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>

// IMUManager integrates ICM-20948 (Accel+Gyro) and BMM350 (Mag) sensors
// with VQF sensor fusion algorithm
//
// HARDWARE:
//   - ICM-20948: Accelerometer ±2/4/8/16g, Gryroscope ±250/500/1000/2000dps
//   - BMM350: Magnetometer ±2000µT (fixed range)
//   - VQF: Quaternion-based sensor fusion (ENU frame)
//
// COORDINATE FRAMES:
//   - Raw sensor data: device-specific (requires axis mapping)
//   - VQF input: ENU (East-North-Up)
//   - VQF output: Quaternion (w, x, y, z)
//   - Heading: Compass degrees (0=N, 90=E, 180=S, 270=W)

class IMUManager {
public:
    // Lifecycle
    static bool init();
    static void update();
    static void deinit();
    
    // Sensor data access (calibrated, in VQF input units)
    static bool getAccel(float& ax, float& ay, float& az);      // m/s²
    static bool getGyro(float& gx, float& gy, float& gz);       // rad/s
    static bool getMag(float& mx, float& my, float& mz);        // µT
    
    // Sensor data access (raw, uncalibrated)
    static bool getAccelRaw(float& ax, float& ay, float& az);   // m/s²
    static bool getGyroRaw(float& gx, float& gy, float& gz);    // rad/s
    static bool getMagRaw(float& mx, float& my, float& mz);     // µT
    
    // VQF quaternion output
    static bool getQuaternion(float& w, float& x, float& y, float& z);
    
    // Heading (0-360°, 0=North)
    static float getCompassHeading();
    
    // Orientation (VQF output)
    static float getHeading();            // Compass heading (0-360°)
    static float getRoll();
    static float getPitch();
    
    // Sensor configuration (ICM-20948)
    static bool setGyroRange(uint8_t range);    // 0=250dps, 1=500dps, 2=1000dps, 3=2000dps
    static bool setGyroDLPF(uint8_t dlpf);      // 0-7 (0=bypass, 3=92Hz, 7=5Hz)
    static bool setAccelRange(uint8_t range);   // 0=2g, 1=4g, 2=8g, 3=16g
    static bool setAccelDLPF(uint8_t dlpf);     // 0-7 (0=bypass, 3=99Hz, 7=5Hz)
    
    // Sensor configuration (BMM350)
    static bool setMagODR(uint8_t odr);         // 0=1.5625Hz ... 8=400Hz
    static bool setMagAveraging(uint8_t avg);   // 0=No, 1=2x, 2=4x, 3=8x
    static uint8_t getMagPowerMode();            // Get current power mode
    static bool setMagPowerMode(uint8_t mode);   // 1=Normal, 4=Forced Fast
    static bool setMagPreset(uint8_t preset);   // 1-4
    
    // Configurable Loop Rate
    static bool setLoopRate(uint16_t rate);     // Re-initializes VQF
    
    // Calibration offsets (applied before VQF)
    static void setGyroBias(float bx, float by, float bz);         // rad/s
    static void setAccelBias(float bx, float by, float bz);        // m/s²
    static void setAccelScale(float sx, float sy, float sz);       // scale factors
    static void setMagCalibration(float hx, float hy, float hz,    // hard iron (µT)
                                  float soft[3][3]);
    
    // Status
    static bool isInitialized();
    static uint32_t getLastUpdateTime();      // millis() of last sensor read
    static uint32_t getUpdateCount();         // Total VQF updates
    
    // Diagnostics
    static void printAudit();                 // Adversarial audit report (6 phases)
    static void printRawSensors();            // Raw sensor data snapshot
    static void printCalibratedSensors();     // Calibrated sensor data snapshot
    static void printVQFStatus();             // VQF state and quaternion
    static void runAxisAlignmentDiagnostic(); // Interactive live diagnostic
    static void printHeadingDebug();          // VQF heading breakdown
    
    // Preset application (for menu compatibility with BASE Code)
    static void applyAccelPreset(uint8_t preset);  // 1-5: Precise to Extreme
    static void applyGyroPreset(uint8_t preset);   // 1-4: Precise to Extreme
    static void applyMagPreset(uint8_t preset);    // 1-4: Low Power to High Accuracy
    static void applyMagODR(float odr);          // Hz
    
    // BMM350 Configuration Validation
    static bool isValidBMM350Config(float odr, uint8_t averaging);
    static const char* getBMM350ValidationError(float odr, uint8_t averaging);
    
    // VQF parameter tuning
    static void setVQFParams(float tauAcc, float tauMag, bool magReject);
    
    // Helper functions (for calibration compatibility)
    static uint8_t getCurrentAccelRange();
    static uint8_t getCurrentGyroRange();
    static bool isAvailable();
    static bool checkHealth();
    
    // Sensor data accessors (for BASE Code compatibility)
    static float getAccelX();
    static float getAccelY();
    static float getAccelZ();
    static float getGyroX();
    static float getGyroY();
    static float getGyroZ();
    static float getMagX();
    static float getMagY();
    static float getMagZ();
    
    // I2C error tracking (for calibration)
    static uint32_t getI2CErrorCount();
    
private:
    // Initialization state
    static bool initialized;
    
    // Sensor handles/state
    // (Will be populated with ICM-20948 and BMM350 objects)
    
    // VQF algorithm state
    // (Will be populated with VQF object)
    
    // Calibration data
    static float gyroBias[3];          // rad/s
    static float accelBias[3];         // m/s²
    static float accelScale[3];        // scale factors
    static float magHardIron[3];       // µT
    static float magSoftIron[3][3];    // correction matrix
    
    // Latest sensor readings (calibrated)
    static float accel[3];             // m/s²
    static float gyro[3];              // rad/s
    static float mag[3];               // µT
    
    // Latest sensor readings (raw)
    static float accelRaw[3];          // m/s²
    static float gyroRaw[3];           // rad/s
    static float magRaw[3];            // µT
    
    // VQF output
    static float quat[4];              // w, x, y, z
    static float heading;              // degrees (0-360)
    static float roll;                 // degrees
    static float pitch;                // degrees
    
    // Statistics
    static uint32_t lastUpdateTime;
    static uint32_t updateCount;
    
    // Internal helpers
    static bool initICM20948();
    static bool initBMM350();
    static bool initVQF();
    
    static bool readICM20948Accel(float& ax, float& ay, float& az);
    static bool readICM20948Gyro(float& gx, float& gy, float& gz);
    static bool readBMM350Mag(float& mx, float& my, float& mz);
    
    static void applyAccelCalibration(float& ax, float& ay, float& az);
    static void applyGyroCalibration(float& gx, float& gy, float& gz);
    static void applyMagCalibration(float& mx, float& my, float& mz);
    
    static void applyAxisMapping();       // Transform sensor frames to ENU
    static void calculateOrientation();   // Extract roll/pitch/heading from quaternion
};

#endif // IMU_MANAGER_H
