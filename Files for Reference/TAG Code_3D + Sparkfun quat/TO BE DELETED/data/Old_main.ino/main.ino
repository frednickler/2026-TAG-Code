//from chat gpt
// Fully Revised ICM-20948 + AK09916 Reader
// Includes: Safe I2C access, continuous magnetometer reading, heading calculation, reboot-proof logic

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "imu_calibration.h" // Add calibration header
#include "gps_module.h" // GPS module header
#include "sensor_fusion.h" // Madgwick fusion module
#include "position_tracking.h" // Position tracking module
#include "imu_config.h"
#include "imu_validation.h" // IMU validation suite
// NOTE: Ensure you have the SparkFun Ublox Arduino Library added to platformio.ini:
// lib_deps = sparkfun/SparkFun Ublox Arduino Library@^2.0.22


#define I2C_SDA_IMU 4
#define I2C_SCL_IMU 5
#define ICM20948_INT_PIN 6 // ESP32 S3 DevKit, INT pin

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

// Removed: Only use Wire for IMU (I2C_IMU is now Wire)
bool magnetometerReady = false;
bool magDataValid = false;
bool magUpdated = false;        // True if fresh mag data was read this loop
bool headingValid = false;      // True if heading is trustworthy (fresh mag or valid fallback)
float cachedHeading = 0.0f;
float lastPitch = 0.0, lastRoll = 0.0, lastYaw = 0.0;
float g_magDeclination = 0.0; // Stores the magnetic declination
float lastValidHeading = NAN;  // Stores last valid heading (true heading)
bool everHadValidHeading = false;

// Last raw magnetometer readings for logging
float lastMx, lastMy, lastMz;
// Global IMU sensor values for fusion
float ax_mg = 0, ay_mg = 0, az_mg = 0;
float gx_dps = 0, gy_dps = 0, gz_dps = 0;
float temp_c = 0;

// IMU data structure for validation
IMUData imuData;
unsigned long lastValidation = 0; // Timer for validation reports

bool readAccel() {
  uint8_t data[6];
  I2C_IMU.beginTransmission(ICM_20948_ADDR);
  I2C_IMU.write(ACCEL_XOUT_H);
  if (I2C_IMU.endTransmission(false) != 0) {
    Serial.println("[ERROR] Accel I2C endTransmission failed");
    return false;
  }
  if (I2C_IMU.requestFrom(ICM_20948_ADDR, 6) != 6) {
    Serial.println("[ERROR] Accel I2C requestFrom failed");
    return false;
  }
  for (int i = 0; i < 6; i++) data[i] = I2C_IMU.read();

  int16_t ax = (data[0] << 8) | data[1];
  int16_t ay = (data[2] << 8) | data[3];
  int16_t az = (data[4] << 8) | data[5];

  ax_mg = ax / 16.384f;
  ay_mg = ay / 16.384f;
  az_mg = az / 16.384f;

  Serial.printf("🏃 Accel (mg): X=%.2f Y=%.2f Z=%.2f\n", ax_mg, ay_mg, az_mg);
  return true;
}

bool readGyro() {
  uint8_t data[6];
  I2C_IMU.beginTransmission(ICM_20948_ADDR);
  I2C_IMU.write(GYRO_XOUT_H);
  if (I2C_IMU.endTransmission(false) != 0) {
    Serial.println("[ERROR] Gyro I2C endTransmission failed");
    return false;
  }
  if (I2C_IMU.requestFrom(ICM_20948_ADDR, 6) != 6) {
    Serial.println("[ERROR] Gyro I2C requestFrom failed");
    return false;
  }
  for (int i = 0; i < 6; i++) data[i] = I2C_IMU.read();

  int16_t gx = (data[0] << 8) | data[1];
  int16_t gy = (data[2] << 8) | data[3];
  int16_t gz = (data[4] << 8) | data[5];

  gx_dps = gx / 131.0f;
  gy_dps = gy / 131.0f;
  gz_dps = gz / 131.0f;

  Serial.printf("🌀 Gyro (dps): X=%.2f Y=%.2f Z=%.2f\n", gx_dps, gy_dps, gz_dps);
  return true;
}

bool readTemp() {
  uint8_t data[2];
  I2C_IMU.beginTransmission(ICM_20948_ADDR);
  I2C_IMU.write(TEMP_OUT_H);
  if (I2C_IMU.endTransmission(false) != 0) {
    Serial.println("[ERROR] Temp I2C endTransmission failed");
    return false;
  }
  if (I2C_IMU.requestFrom(ICM_20948_ADDR, 2) != 2) {
    Serial.println("[ERROR] Temp I2C requestFrom failed");
    return false;
  }
  data[0] = I2C_IMU.read();
  data[1] = I2C_IMU.read();

  int16_t raw_temp = (data[0] << 8) | data[1];
  // Formula from datasheet: Temp_in_°C = (TEMP_OUT / 333.87) + 21.0
  temp_c = (raw_temp / 333.87f) + 21.0f;

  Serial.printf("🌡️ Temp: %.2f °C\n", temp_c);
  return true;
}

void setup() {
  // =================== I2C BUS INITIALIZATION =====================
  // =================== I2C BUS INITIALIZATION =====================
  // Initialize both I2C buses BEFORE any IMU or GPS code
  Wire.begin(4, 5);    // IMU: SDA=4, SCL=5
  Wire.setClock(400000);
  Wire1.begin(8, 9);   // GPS: SDA=8, SCL=9
  Wire1.setClock(400000);
  // ================================================================
  // =================== DLPF & ODR CONFIGURATION =====================
// Use user-selectable presets for all filters (default: balanced)
setAccelDLPF(accel_dlpf_table[ACCEL_DLPF_LEVEL]);
setGyroDLPF(gyro_dlpf_table[GYRO_DLPF_LEVEL]);
  setTempDLPF(temp_dlpf_table[TEMP_DLPF_LEVEL]);
setMagOutputDataRate(mag_odr_table[MAG_ODR_PRESET]);
// ================================================================

  // =================== GPS MODULE INITIALIZATION =========================
  if (!setupGPS()) {
    Serial.println("[ERROR] GPS initialization failed!");
  } else {
    Serial.println("[INFO] GPS initialized.");
  }
  // ======================================================================
  Serial.begin(115200);
  // I2C_IMU is an alias for Wire, already initialized above
  // I2C_IMU.begin(I2C_SDA_IMU, I2C_SCL_IMU); // (removed: already done)
  // I2C_IMU.setClock(400000); // (removed: already done)


  delay(100);
  Serial.println("\n\n🔧 Booting up ICM-20948 + AK09916 test...");

  writeRegister(PWR_MGMT_1, 0x80); // Reset
  delay(100);
  writeRegister(PWR_MGMT_1, 0x01); // Clock source
  delay(10);
  writeRegister(PWR_MGMT_2, 0x00); // Enable sensors
  writeRegister(INT_PIN_CFG, 0x02); // Bypass I2C master
  writeRegister(USER_CTRL, 0x00);
  delay(10);

  // Magnetometer init
  I2C_IMU.beginTransmission(AK09916_ADDR);
  if (I2C_IMU.endTransmission() != 0) {
    Serial.println("❌ Magnetometer not detected");
    return;
  }

  // Reset magnetometer
  I2C_IMU.beginTransmission(AK09916_ADDR);
  I2C_IMU.write(AK09916_CNTL3);
  I2C_IMU.write(0x01);
  I2C_IMU.endTransmission();
  delay(100);

  // Set Continuous Mode 2 (100 Hz)
  I2C_IMU.beginTransmission(AK09916_ADDR);
  I2C_IMU.write(AK09916_CNTL2);
  I2C_IMU.write(0x08);
  I2C_IMU.endTransmission();

  magnetometerReady = true;
  Serial.println("✅ Magnetometer ready!");

  // --- Calibration Integration ---
  // (EEPROM removed; use RAM or ESP32 NVS for persistent storage if needed)
  initCalibration();

  // Accelerometer/Gyro Calibration
  if (!calibrateAccelGyro()) {
    Serial.println("Accel/Gyro calibration failed. Please keep the device still and try again.");
  } else {
    Serial.println("Accel/Gyro calibration successful.");
  }

  // Magnetometer calibration starts automatically after accel/gyro calibration
  if (!calibrateMagnetometer()) {
    Serial.println("Magnetometer calibration failed. Try again if needed.");
  } else {
    Serial.println("Magnetometer calibration successful.");
  }
  printCalibrationValues();

  // Wait for GPS to get a fix and a valid magnetic declination
  Serial.println("Waiting for GPS 3D Fix and valid magnetic declination...");
  while (true) {
    updateGPS(); // Reads from the GPS module
    const GPSData& gpsData = getLatestGPSData();
    if (gpsData.fixType >= 3 && gpsData.magDec_deg != 360.0) {
      g_magDeclination = gpsData.magDec_deg;
      Serial.printf("GPS Fix acquired. Magnetic Declination: %.2f degrees (%s)\n",
                    g_magDeclination,
                    gpsData.magDecFromGPS ? "GPS" : "WMM");
      break;
    }
    Serial.print(".");
    delay(500);
  }

  Serial.println("Setup complete. Starting main loop.");
  delay(1000); // Brief pause before main loop

  // =================== Madgwick Sensor Fusion ======================
  // Initialize Madgwick filter with selected beta (see sensor_fusion.h)
  initSensorFusion(MADGWICK_BETA_DEFAULT);
  // ================================================================
  
  // =================== Position Tracking ===========================
  // Initialize position tracking module
  initPositionTracking();
  // ================================================================
}

void loop() {
  // --- Mag read throttling and retry logic ---
  static unsigned long lastMagRead = 0;
  bool magOK = false;
  magUpdated = false;
  if (magnetometerReady && millis() - lastMagRead >= (MAG_ODR_PRESET == 1 ? 100 : MAG_ODR_PRESET == 2 ? 50 : MAG_ODR_PRESET == 3 ? 20 : 10)) {
    for (int attempt = 0; attempt < 3 && !magOK; ++attempt) {
      magOK = readMagnetometer();
      if (!magOK) delay(3);
    }
    if (magOK) {
      lastMagRead = millis();
      magUpdated = true;
    }
  }
  // magOK: true if mag read succeeded this loop; magUpdated: true if fresh data this cycle

  // Update and print GPS data
  updateGPS();
  const GPSData& gps = getLatestGPSData();
  if (gps.valid) {
    Serial.printf("[GPS] Lat: %.7f, Lon: %.7f, Alt: %.2f m, Spd: %.2f m/s, Head: %.2f°, Sats: %d\n",
      gps.latitude, gps.longitude, gps.altitude, gps.speed_m_s, gps.heading_deg, gps.numSV);
  } else {
    Serial.println("[GPS] No valid fix");
  }

  // Heading/mag fallback logic
  static unsigned long lastValidMagTime = 0;
  const unsigned long headingTimeoutMs = 2000; // 2s: after this, fallback to previous heading
  if (magUpdated) {
    lastValidMagTime = millis();
    headingValid = true;
  } else if (millis() - lastValidMagTime > headingTimeoutMs) {
    // Fallback: use last valid heading if available
    if (!isnan(lastValidHeading)) {
      headingValid = true;
    } else {
      headingValid = false;
    }
  }

  // =================== IMU SENSOR READS ============================
  bool accelOK = readAccel(); // updates ax_mg, ay_mg, az_mg (in mg)
  bool gyroOK = readGyro();  // updates gx_dps, gy_dps, gz_dps (in dps)
  bool tempOK = readTemp(); // updates temp_c

  // No extra mag read here; magOK/magUpdated already set above
  if (magOK) {
      cachedHeading = atan2(lastMy, lastMx) * 180.0 / PI;
      if (cachedHeading < 0) cachedHeading += 360.0;
  }
  // Debug print to show which sensor failed
  Serial.printf("[DEBUG] accelOK: %d, gyroOK: %d, magOK: %d, headingValid: %d\n", accelOK, gyroOK, magOK, headingValid);
  if (!accelOK || !gyroOK) {
      Serial.println("[WARN] Skipping fusion update due to IMU read error");
  }
  // ================================================================

  // =================== MADGWICK FUSION UPDATE =====================
  // Convert accel from mg to g, gyro from dps to rad/s
  float ax_g = ax_mg / 1000.0f;
  float ay_g = ay_mg / 1000.0f;
  float az_g = az_mg / 1000.0f;
  float gx_rad = gx_dps * DEG_TO_RAD;
  float gy_rad = gy_dps * DEG_TO_RAD;
  float gz_rad = gz_dps * DEG_TO_RAD;

  // DEBUG: Print all fusion input values
  Serial.printf("[Fusion Input] Accel (g): X=%.4f Y=%.4f Z=%.4f\n", ax_g, ay_g, az_g);
  Serial.printf("[Fusion Input] Gyro (rad/s): X=%.4f Y=%.4f Z=%.4f\n", gx_rad, gy_rad, gz_rad);
  
  // Always update fusion with accel/gyro if they're valid
  if (accelOK && gyroOK) {
    if (magOK) {
      // 9DOF update (accel + gyro + mag)
      Serial.printf("[Fusion Input] Mag (uT): X=%.4f Y=%.4f Z=%.4f\n", lastMx, lastMy, lastMz);
      updateSensorFusion(gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, lastMx, lastMy, lastMz, 0.01f);
      Serial.println("[INFO] 9DOF fusion update (accel+gyro+mag)");
    } else {
      // 6DOF fallback (accel + gyro only)
      Serial.println("[INFO] Mag data not ready, using 6DOF fusion (accel+gyro only)");
      updateSensorFusion(gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, 0, 0, 0, 0.01f);
    }
  } else {
    Serial.println("[WARN] Fusion update skipped due to accel/gyro error");
  }
  // Get quaternion and Euler angles for debug and position tracking
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
  
  // Only get orientation data if fusion was updated
  if (accelOK && gyroOK) {
    getQuaternion(q0, q1, q2, q3);
    getEulerAngles(yaw, pitch, roll);

    // Sanity check for NaN or Inf values to prevent crashes in the visualizer
    if (isnan(yaw) || isinf(yaw)) yaw = 0.0f;
    if (isnan(pitch) || isinf(pitch)) pitch = 0.0f;
    if (isnan(roll) || isinf(roll)) roll = 0.0f;
  }

  // =================== IMU VALIDATION ==========================
  // Populate the IMU data structure for the validation suite
  if (accelOK && gyroOK) {
      // Raw data (from hardware DLPF)
      imuData.raw_ax = ax_g * 9.80665f; // Convert g to m/s²
      imuData.raw_ay = ay_g * 9.80665f; // Convert g to m/s²
      imuData.raw_az = az_g * 9.80665f; // Convert g to m/s²
      imuData.raw_gx = gx_rad; // in rad/s
      imuData.raw_gy = gy_rad; // in rad/s
      imuData.raw_gz = gz_rad; // in rad/s
      imuData.raw_temp_c = temp_c; // in Celsius
      
      // For now, filtered data is the same as raw (DLPF-filtered) data
      imuData.filt_ax = imuData.raw_ax;
      imuData.filt_ay = imuData.raw_ay;
      imuData.filt_az = imuData.raw_az;
      imuData.filt_gx = imuData.raw_gx;
      imuData.filt_gy = imuData.raw_gy;
      imuData.filt_gz = imuData.raw_gz;
      imuData.filt_temp_c = imuData.raw_temp_c;

      if (magOK) {
          imuData.raw_mx = lastMx; // in uT
          imuData.filt_mx = lastMx;
          imuData.filt_my = lastMy;
          imuData.filt_mz = lastMz;
      }

      // Quaternion and Euler angles from fusion
      imuData.q0 = q0;
      imuData.q1 = q1;
      imuData.q2 = q2;
      imuData.q3 = q3;
      imuData.roll = roll;
      imuData.pitch = pitch;
      imuData.yaw = yaw;

      // Run validation suite every second
      if (millis() - lastValidation >= 1000) {
          runValidation(imuData);
          lastValidation = millis();
      }
  }
  // =============================================================

  // Get latest GPS data for declination and source
  const GPSData& gpsData = getLatestGPSData();
  
  // Apply magnetic declination to get true heading
  // Use the declination from GPS if available, otherwise use the WMM model
  float trueHeading = yaw + gpsData.magDec_deg;
  if (trueHeading < 0) trueHeading += 360.0;
  if (trueHeading >= 360.0) trueHeading -= 360.0;

  // Robust fallback: if heading is valid (magUpdated), update lastValidHeading
  if (magUpdated) {
    lastValidHeading = trueHeading;
    everHadValidHeading = true;
  }

  // If heading is not valid, use last valid heading if available
  if (!headingValid) {
    if (!isnan(lastValidHeading)) {
      trueHeading = lastValidHeading;
      yaw = lastValidHeading - gpsData.magDec_deg; // Try to reconstruct mag yaw
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;
      headingValid = true;
    } else {
      trueHeading = NAN;
      yaw = NAN;
      headingValid = false;
    }
  }

  // Log the heading information for debugging
  Serial.printf("[HEADING] MagYaw: %.2f°, Declination: %.2f° (%s), TrueHeading: %.2f°, headingValid: %d\n",
                yaw, gpsData.magDec_deg,
                gpsData.magDecFromGPS ? "GPS" : "WMM",
                trueHeading, headingValid);
  
#ifdef FUSION_DEBUG_PRINT
  Serial.print("[Fusion] Q: ");
  Serial.print(q0); Serial.print(", ");
  Serial.print(q1); Serial.print(", ");
  Serial.print(q2); Serial.print(", ");
  Serial.println(q3);

  // Print Euler angles (yaw, pitch, roll) in degrees.
  Serial.printf("[Fusion] Magnetic Yaw: %.2f°, Pitch: %.2f°, Roll: %.2f°\n", yaw, pitch, roll);
  Serial.printf("         True Heading (Yaw + Declination): %.2f°\n", trueHeading);
#endif
  // ================================================================

  // =================== Position Tracking Update ====================
  // Update position tracking with combined GPS and IMU data
  // Note: 'gps' variable is already declared and updated earlier in the loop
  bool using9DOF = magOK && accelOK && gyroOK;
  // Pass the TRUE heading to the position tracking module, along with headingValid
  updatePositionData(gps, trueHeading, pitch, roll, q0, q1, q2, q3, using9DOF, headingValid);

  // Get position data as JSON for visualization
  String positionJson = getPositionDataJson();

  // Print position data for visualization
  Serial.println("[POSITION_JSON]" + positionJson + "[/POSITION_JSON]");
  if (!headingValid && !everHadValidHeading) {
    Serial.println("[WARN] Heading unavailable: magnetometer data is stale or invalid and no fallback is available.");
  }
  // ================================================================

  Serial.printf("🧭 Mag (uT): X=%.2f Y=%.2f Z=%.2f\n", lastMx, lastMy, lastMz);
  Serial.printf("🧭 Heading: %.2f°\n\n", cachedHeading);

  // --- Loop rate limiter: match sample rate (100Hz for preset 3) ---
  delay(10); // 10ms = 100Hz

  // --- Loop rate monitoring ---
  static int loopCounter = 0;
  static unsigned long lastPrint = 0;
  loopCounter++;
  if (loopCounter >= 100) {
    unsigned long now = millis();
    Serial.printf("[LoopRate] 100 loops in %lu ms (~%.1f Hz)\n", now - lastPrint, 1000.0f * 100 / (now - lastPrint));
    lastPrint = now;
    loopCounter = 0;
  }

}

bool readMagnetometer() {
  // Static variable to track consecutive failures
  static uint8_t failCount = 0;
  static uint32_t lastSuccessTime = 0;
  
  // If we've had multiple consecutive failures, reassert continuous mode
  if (failCount >= 3) {
    reassertMagContinuousMode();
    failCount = 0;
    delay(5); // Give it a moment to stabilize
  }
  
  // Periodically reassert continuous mode (every 1 second) to ensure it stays active
  uint32_t currentTime = millis();
  if (currentTime - lastSuccessTime > 1000) {
    reassertMagContinuousMode();
    delay(2); // Small delay after setting mode
  }

  // Check if data is ready
  I2C_IMU.beginTransmission(AK09916_ADDR);
  I2C_IMU.write(AK09916_ST1);
  if (I2C_IMU.endTransmission(false) != 0) {
    Serial.println("[ERROR] Mag I2C endTransmission (ST1) failed");
    failCount++;
    return false;
  }
  
  if (I2C_IMU.requestFrom(AK09916_ADDR, 1) != 1) {
    Serial.println("[ERROR] Mag I2C requestFrom (ST1) failed");
    failCount++;
    return false;
  }
  
  uint8_t st1 = I2C_IMU.read();
  if (!(st1 & 0x01)) {
    // Data not ready - don't print error as this is expected at high read rates
    failCount++;
    return false;
  }

  // Read magnetometer data
  I2C_IMU.beginTransmission(AK09916_ADDR);
  I2C_IMU.write(AK09916_HXL);
  if (I2C_IMU.endTransmission(false) != 0) {
    Serial.println("[ERROR] Mag I2C endTransmission (HXL) failed");
    failCount++;
    return false;
  }
  
  if (I2C_IMU.requestFrom(AK09916_ADDR, 7) != 7) {
    Serial.println("[ERROR] Mag I2C requestFrom (HXL) failed");
    failCount++;
    return false;
  }

  uint8_t buffer[7];
  for (int i = 0; i < 7; i++) buffer[i] = I2C_IMU.read();

  // Check for overflow
  if (buffer[6] & 0x08) {
    Serial.println("[ERROR] Mag overflow");
    failCount++;
    return false;
  }

  // Convert to signed 16-bit integers
  int16_t mx = (buffer[1] << 8) | buffer[0];
  int16_t my = (buffer[3] << 8) | buffer[2];
  int16_t mz = (buffer[5] << 8) | buffer[4];

  // Convert to microtesla (uT)
  float mx_ut = mx * 0.15f;
  float my_ut = my * 0.15f;
  float mz_ut = mz * 0.15f;
  
  // Apply calibration (offsets and scale)
  lastMx = (mx_ut - mag_offset[0]) * mag_scale[0];
  lastMy = (my_ut - mag_offset[1]) * mag_scale[1];
  lastMz = (mz_ut - mag_offset[2]) * mag_scale[2];
  
  // Reset failure counter and update success time
  failCount = 0;
  lastSuccessTime = currentTime;
  
  return true;
}

void reassertMagContinuousMode() {
  // Reassert continuous mode 2 (100Hz)
  I2C_IMU.beginTransmission(AK09916_ADDR);
  I2C_IMU.write(AK09916_CNTL2);
  I2C_IMU.write(0x08);
  I2C_IMU.endTransmission();
  Serial.println("[INFO] Reasserted mag continuous mode");
}
