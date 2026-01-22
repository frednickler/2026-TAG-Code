/*
 * ICM-20948 Magnetometer Calibration
 * 
 * This sketch collects raw magnetometer data for calibration
 * and can apply calibration parameters.
 * 
 * Commands:
 * - 's'    // Start/stop data collection
 * - 'c' - Apply calibration parameters
 * - 'r' - Reset calibration to default
 * - 'x' - Explicitly stop data collection
 * - 'p' - Print current calibration parameters
 * - 'd' - Toggle debug mode
 */

#include <Wire.h>

// IMU Configuration
#define I2C_SDA_IMU 4
#define I2C_SCL_IMU 5
#define ICM_20948_ADDR 0x68
#define AK09916_ADDR 0x0C

// I2C Configuration
#define I2C_MAX_RETRIES 3
#define I2C_RETRY_DELAY 10  // ms
#define MAG_RECOVERY_INTERVAL 1000  // ms between recovery attempts

// AK09916 Registers
#define AK09916_WIA1    0x00
#define AK09916_WIA2    0x01
#define AK09916_ST1     0x10
#define AK09916_HXL     0x11
#define AK09916_ST2     0x18
#define AK09916_CNTL2   0x31
#define AK09916_CNTL3   0x32
#define AK09916_CNTL2_MODE_CONT_100HZ 0x08

// Global variables
bool collecting = false;
bool debug_mode = false;
bool apply_calibration = false;
int16_t mag_raw[3];
float mag_bias[3] = {0, 0, 0};
float mag_scale[3] = {1.0, 1.0, 1.0};

// Increase buffer size to handle 30 seconds of data at ~100Hz
const size_t MAX_SAMPLES = 3000;  // 30 seconds * 100 Hz
int16_t data_buffer[MAX_SAMPLES][3];
size_t sample_count = 0;

// Magnetometer status and recovery
unsigned long last_mag_recovery = 0;
uint8_t mag_fail_count = 0;
const uint8_t MAG_MAX_FAILS = 3; // Reduced from 5 to recover faster
unsigned long last_print = 0;

 
 // Function prototypes
bool readMagnetometer();
bool initMagnetometer();

void setup() {
   Serial.begin(115200);
   while (!Serial) { delay(10); }
   
   // Initialize I2C at maximum speed
   Wire.begin(I2C_SDA_IMU, I2C_SCL_IMU);
   Wire.setClock(400000);  // Increased to 400kHz for faster I2C communication
   
   // Initialize IMU and magnetometer
   initIMU();
   
   Serial.println(F("ICM-20948 Magnetometer Calibration"));
   Serial.println(F("--------------------------------"));
   Serial.println(F("Commands:"));
   Serial.println(F("  's' - Start/stop data collection"));
   Serial.println(F("  'x' - Explicitly stop data collection"));
   Serial.println(F("  'c' - Apply calibration parameters"));
   Serial.println(F("  'r' - Reset calibration to default"));
   Serial.println(F("  'p' - Print current calibration parameters"));
   Serial.println(F("  'd' - Toggle debug mode"));
   Serial.println(F("--------------------------------"));
   Serial.println(F("Send 's' to start collecting data..."));
 }

void loop() {
  static uint32_t last_status_print = 0;
  static uint32_t last_sample_time = 0;
  static uint32_t collection_start_time = 0;
  static bool calibration_done = false;
  
  uint32_t now = millis();
  
  // Handle serial commands
  if (Serial.available()) {
    char c = Serial.read();

    // Clear any other characters in the buffer to handle only the first command
    while(Serial.available()) {
      Serial.read();
    }

    if (c == 's') {
      collecting = !collecting;
      if (collecting) {
        Serial.println(F("Starting data collection..."));
        // Only reset counters if we are starting a fresh calibration run
        if (!calibration_done) {
          sample_count = 0;
          collection_start_time = now;
          Serial.println(F("Rotate the sensor in all directions for 30 seconds."));
          Serial.println(F("Auto-calibration will run automatically after 30 seconds."));
        } else {
          // For subsequent collections, reset both sample count and start time
          sample_count = 0;
          collection_start_time = now;
          Serial.println(F("Collecting additional data. Send 's' again to stop."));
        }
      } else {
        if (collection_start_time != 0 && sample_count > 0) {
          float total_time = (now - collection_start_time) / 1000.0; // seconds
          float avg_rate = (total_time > 0) ? (sample_count / total_time) : 0;
          Serial.print(F("Collection complete. "));
          Serial.print(sample_count);
          Serial.print(F(" samples in "));
          Serial.print(total_time, 1);
          Serial.print(F("s ("));
          Serial.print(avg_rate, 1);
          Serial.println(F(" Hz avg)"));
          
          // Auto-calibrate if we have enough samples and calibration hasn't been done yet
          if (sample_count >= 100 && !calibration_done) {
            Serial.println(F("\nRunning calibration on collected data..."));
            autoCalibrate();
            calibration_done = true;
            Serial.println(F("Calibration complete!"));
            Serial.println(F("Send 'c' to toggle calibration, 's' to collect more data."));
          }
        }
        // Do not reset start time here to allow for continuous data sessions
      }
    } else if (c == 'x') {
      // Explicit stop command - always stops collection without toggling
      if (collecting) {
        collecting = false;
        if (collection_start_time != 0 && sample_count > 0) {
          float total_time = (now - collection_start_time) / 1000.0; // seconds
          float avg_rate = (total_time > 0) ? (sample_count / total_time) : 0;
          Serial.print(F("Collection stopped. "));
          Serial.print(sample_count);
          Serial.print(F(" samples in "));
          Serial.print(total_time, 1);
          Serial.print(F("s ("));
          Serial.print(avg_rate, 1);
          Serial.println(F(" Hz avg)"));
        }
      } else {
        Serial.println(F("Collection already stopped."));
      }
    } else if (c == 'c') {
      // Toggle calibration application
      apply_calibration = !apply_calibration;
      Serial.print(F("Calibration "));
      Serial.println(apply_calibration ? F("applied") : F("not applied"));
    } else if (c == 'r') {
      // Reset calibration
      mag_bias[0] = mag_bias[1] = mag_bias[2] = 0;
      mag_scale[0] = mag_scale[1] = mag_scale[2] = 1.0;
      Serial.println(F("Calibration reset to default."));
    } else if (c == 'p') {
      printCalibration();
    } else if (c == 'd') {
      // Toggle debug output
      debug_mode = !debug_mode;
      Serial.print(F("Debug mode "));
      Serial.println(debug_mode ? F("enabled") : F("disabled"));
    }
  }
  
  // If we're collecting data, read and output magnetometer values
  if (collecting) {
    if (readMagnetometer()) {
      // Calculate sample rate
      uint32_t delta = now - last_sample_time;
      float sample_rate = (delta > 0) ? 1000.0 / delta : 0;
      last_sample_time = now;
      
      // Store raw data in buffer for calibration
      if (sample_count < MAX_SAMPLES) {
        data_buffer[sample_count][0] = mag_raw[0];
        data_buffer[sample_count][1] = mag_raw[1];
        data_buffer[sample_count][2] = mag_raw[2];
        sample_count++;
      }
      
      // Apply calibration if enabled
      float calibrated[3];
      if (apply_calibration) {
        applyCalibration(mag_raw, calibrated);
      }
      
      // Output data (raw or calibrated)
      if (apply_calibration) {
        Serial.print(F("CAL:"));
        Serial.print(calibrated[0], 2);
        Serial.print(',');
        Serial.print(calibrated[1], 2);
        Serial.print(',');
        Serial.print(calibrated[2], 2);
      } else {
        Serial.print(F("RAW:"));
        Serial.print(mag_raw[0]);
        Serial.print(',');
        Serial.print(mag_raw[1]);
        Serial.print(',');
        Serial.print(mag_raw[2]);
      }
      
      // Output sample rate and status
      if (now - last_status_print >= 1000) { // Every second
        uint32_t elapsed = (now - collection_start_time) / 1000; // seconds
        Serial.print(F(",rate="));
        Serial.print(sample_rate, 1);
        Serial.print(F("Hz,samples="));
        Serial.print(sample_count);
        Serial.print(F(",time="));
        Serial.print(elapsed);
        Serial.print(F("s"));
        last_status_print = now;
      }
      Serial.println();
      
      // In the main loop, update the auto-calibration check to wait for full 30s
      if (!calibration_done && (now - collection_start_time) >= 30000 && sample_count >= 100) {
        collecting = false; // Stop collecting
        Serial.println(F("\nAuto-calibrating..."));
        autoCalibrate();
        calibration_done = true;
        Serial.println(F("Calibration complete!"));
        Serial.println(F("Send 'c' to toggle calibration, 's' to collect more data."));
      }
    }
  }
  
  // Add a small delay to prevent 100% CPU usage
  delay(1);
}

bool initMagnetometer() {
  uint8_t whoami = 0;
  uint8_t status = 0;
  
  // Try up to 3 times to initialize
  for (int attempt = 0; attempt < 3; attempt++) {
    // 1. Reset the magnetometer
    if (!writeAK09916(AK09916_CNTL3, 0x01)) { // CNTL3: Software reset
      delay(10);
      continue;
    }
    
    // Wait for reset to complete (datasheet says 1ms max)
    delay(10);
    
    // 2. Verify we can read the WHO_AM_I register
    if (!readAK09916(AK09916_WIA2, &whoami, 1) || whoami != 0x09) {
      delay(10);
      continue;
    }
    
    // 3. Set to continuous measurement mode 4 (100Hz)
    if (!writeAK09916(AK09916_CNTL2, AK09916_CNTL2_MODE_CONT_100HZ)) {
      delay(10);
      continue;
    }
    
    // 4. Verify mode was set correctly
    if (!readAK09916(AK09916_CNTL2, &status, 1) || status != AK09916_CNTL2_MODE_CONT_100HZ) {
      delay(10);
      continue;
    }
    
    // 5. Wait for first data ready
    uint32_t start = millis();
    while (millis() - start < 100) { // Wait up to 100ms
      if (readAK09916(AK09916_ST1, &status, 1) && (status & 0x01)) {
        // Read data to clear the DRDY bit
        uint8_t dummy[6];
        readAK09916(AK09916_HXL, dummy, 6);
        return true;
      }
      delay(1);
    }
  }
  
  Serial.print(F("Magnetometer init failed. WHO_AM_I=0x"));
  Serial.println(whoami, HEX);
  return false;
}

// Robust magnetometer read with retry and recovery
bool readMagnetometer() {
  static uint32_t last_read = 0;
  static uint32_t last_success = 0;
  static uint32_t last_continuous_mode_assert = 0;
  static uint8_t consecutive_read_failures = 0;
  uint8_t buf[8];
  
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
  
  // Read status register 1 (ST1)
  if (!readAK09916(AK09916_ST1, buf, 1)) {
    consecutive_read_failures++;
    if (debug_mode) Serial.print(F("ST1 read failed, "));
    goto recovery_check;
  }
  
  // Check if data is ready
  if (!(buf[0] & 0x01)) {
    if (debug_mode && millis() % 1000 < 100) {
      Serial.print(F("Data not ready, ST1=0x"));
      Serial.println(buf[0], HEX);
    }
    
    // Handle data overrun (bit 1 set) by reading ST2 to clear it
    if (buf[0] & 0x02) {
      uint8_t st2;
      readAK09916(AK09916_ST2, &st2, 1);
    }
    
    // If we're getting repeated not-ready, force recovery
    if (consecutive_read_failures > 3) {
      goto recovery_check;
    }
    
    delay(2); // Slightly longer delay before retry
    return false;
  }
  
  // Read the 6 data registers (HXL to HZH)
  if (!readAK09916(AK09916_HXL, buf, 6)) {
    consecutive_read_failures++;
    if (debug_mode) Serial.print(F("Data read failed, "));
    goto recovery_check;
  }
  
  // Read ST2 to check for overflow and clear DRDY
  uint8_t st2;
  if (!readAK09916(AK09916_ST2, &st2, 1)) {
    consecutive_read_failures++;
    if (debug_mode) Serial.print(F("ST2 read failed, "));
    goto recovery_check;
  }
  
  // Check for sensor overflow
  if (st2 & 0x08) {
    if (debug_mode) Serial.println(F("Magnetometer overflow detected"));
    consecutive_read_failures++;
    goto recovery_check;
  }
  
  // Data is valid, process it
  mag_raw[0] = (int16_t)(buf[1] << 8) | buf[0];
  mag_raw[1] = (int16_t)(buf[3] << 8) | buf[2];
  mag_raw[2] = (int16_t)(buf[5] << 8) | buf[4];
  
  last_success = millis();
  consecutive_read_failures = 0;
  
  // Periodically reassert continuous mode (every 5s) to prevent stalls
  if (millis() - last_continuous_mode_assert > 5000) {
    writeAK09916(AK09916_CNTL2, AK09916_CNTL2_MODE_CONT_100HZ);
    last_continuous_mode_assert = millis();
  }
  
  return true;
  
recovery_check:
  // If we get here, something went wrong
  if (consecutive_read_failures >= MAG_MAX_FAILS) {
    if (debug_mode) {
      Serial.print(F("Recovery triggered ("));
      Serial.print(consecutive_read_failures);
      Serial.println(F(" failures)"));
      delay(10);
      Wire.begin(I2C_SDA_IMU, I2C_SCL_IMU);
      Wire.setClock(400000);
      
      // Reinitialize magnetometer
      if (initMagnetometer()) {
        mag_fail_count = 0;
        last_success = millis();
      }
    }
  }
  
  return false;
}

 // Initialize IMU and magnetometer
void initIMU() {
  // 1. Reset ICM-20948
  writeICMRegister(0, 0x06, 0x80); // PWR_MGMT_1: Device Reset
  delay(100);

  // 2. Wake up the ICM-20948 and select the best clock source
  writeICMRegister(0, 0x06, 0x01); // PWR_MGMT_1: Clock auto-select
  delay(20);

  // 3. Disable the ICM-20948's I2C Master module
  // This is necessary to enable the I2C Bypass Mode
  writeICMRegister(0, 0x03, 0x00); // USER_CTRL: I2C_MST_EN = 0
  delay(20);

  // 4. Enable I2C Bypass Mode
  // This connects the Arduino's I2C pins directly to the magnetometer
  writeICMRegister(0, 0x0F, 0x02); // INT_PIN_CFG: BYPASS_EN = 1
  delay(5); // Reduced delay after bypass enable

  // 5. Now that bypass is active, initialize the magnetometer directly
  Serial.println(F("Initializing Magnetometer..."));
  if (!initMagnetometer()) {
    Serial.println(F("Failed to initialize magnetometer!"));
  } else {
    Serial.println(F("Magnetometer Initialized Successfully."));
  }
}

 bool writeICMRegister(uint8_t bank, uint8_t reg, uint8_t value) {
    for (int attempt = 0; attempt < I2C_MAX_RETRIES; attempt++) {
        // Select the bank
        Wire.beginTransmission(ICM_20948_ADDR);
        Wire.write(0x7F);  // BANK_SEL
        Wire.write(bank << 4);  // Bank selection
        if (Wire.endTransmission(true) != 0) {
            delay(I2C_RETRY_DELAY);
            continue;
        }
 
        // Write to the register
        Wire.beginTransmission(ICM_20948_ADDR);
        Wire.write(reg);
        Wire.write(value);
        if (Wire.endTransmission(true) == 0) {
            return true;
        }
        delay(I2C_RETRY_DELAY);
    }
    Serial.print(F("I2C Error: Failed to write to register 0x"));
    Serial.print(reg, HEX);
    Serial.print(F(" in bank "));
    Serial.println(bank);
    return false;
 }

 bool writeAK09916(uint8_t reg, uint8_t value) {
    for (int attempt = 0; attempt < I2C_MAX_RETRIES; attempt++) {
        Wire.beginTransmission(AK09916_ADDR);
        Wire.write(reg);
        Wire.write(value);
        if (Wire.endTransmission(true) == 0) {
            return true;
        }
        delay(I2C_RETRY_DELAY);
    }
    Serial.print(F("I2C Error: Failed to write to AK09916 register 0x"));
    Serial.println(reg, HEX);
    return false;
 }

 bool readAK09916(uint8_t reg, uint8_t *buf, uint8_t len) {
    for (int attempt = 0; attempt < I2C_MAX_RETRIES; attempt++) {
        // Reset the I2C bus
        Wire.beginTransmission(AK09916_ADDR);
        if (Wire.endTransmission(true) != 0) {
            delayMicroseconds(100);
            continue;
        }
        
        // Write register address
        Wire.beginTransmission(AK09916_ADDR);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) {
            delayMicroseconds(100);
            continue;
        }
        
        // Read data
        uint8_t received = Wire.requestFrom(AK09916_ADDR, len, (uint8_t)true); // Send stop condition
        if (received == len) {
            for (uint8_t i = 0; i < len; i++) {
                if (Wire.available()) {
                    buf[i] = Wire.read();
                } else {
                    // Incomplete data
                    return false;
                }
            }
            return true;
        }
        
        // Small delay before retry
        delayMicroseconds(100);
    }
    
    // If we get here, all retries failed
    return false;
 }

void printCalibration() {
  Serial.println(F("Current calibration parameters:"));
  Serial.print(F("mag_bias = {"));
  for (int i = 0; i < 3; i++) {
    Serial.print(mag_bias[i]);
    if (i < 2) Serial.print(", ");
  }
  Serial.println("}");
  
  Serial.print(F("mag_scale = {"));
  for (int i = 0; i < 3; i++) {
    Serial.print(mag_scale[i], 6);
    if (i < 2) Serial.print(", ");
  }
  Serial.println("}");
}

void applyCalibration(int16_t raw[3], float calibrated[3]) {
  for (int i = 0; i < 3; i++) {
    // Apply hard iron correction (subtract bias)
    float corrected = raw[i] - mag_bias[i];
    // Apply soft iron correction (divide by scale)
    calibrated[i] = corrected * mag_scale[i];
  }
}

void calibrateHardIron(int16_t *min_vals, int16_t *max_vals) {
  // Calculate bias as the average of min and max values for each axis
  for (int i = 0; i < 3; i++) {
    mag_bias[i] = (min_vals[i] + max_vals[i]) / 2;
  }
  
  if (debug_mode) {
    Serial.println(F("Hard iron calibration complete. New bias values:"));
    Serial.print(F("mag_bias = {"));
    for (int i = 0; i < 3; i++) {
      Serial.print(mag_bias[i]);
      if (i < 2) Serial.print(", ");
    }
    Serial.println("}");
  }
}

void calibrateSoftIron(int16_t *min_vals, int16_t *max_vals) {
  // Calculate average range for each axis
  float avg_range[3];
  for (int i = 0; i < 3; i++) {
    avg_range[i] = (max_vals[i] - min_vals[i]) / 2.0f;
  }
  
  // Find the average range across all axes
  float avg_radius = (avg_range[0] + avg_range[1] + avg_range[2]) / 3.0f;
  
  // Calculate scale factor for each axis (normalize to average radius)
  for (int i = 0; i < 3; i++) {
    mag_scale[i] = avg_radius / avg_range[i];
  }
  
  if (debug_mode) {
    Serial.println(F("Soft iron calibration complete. New scale values:"));
    Serial.print(F("mag_scale = {"));
    for (int i = 0; i < 3; i++) {
      Serial.print(mag_scale[i], 6);
      if (i < 2) Serial.print(", ");
    }
    Serial.println("}");
  }
}

void autoCalibrate() {
  if (sample_count < 100) {
    Serial.println(F("Need at least 100 samples for calibration"));
    return;
  }
  
  // Find min/max values for each axis
  int16_t min_vals[3] = {32767, 32767, 32767};
  int16_t max_vals[3] = {-32768, -32768, -32768};
  
  for (size_t i = 0; i < sample_count; i++) {
    for (int axis = 0; axis < 3; axis++) {
      if (data_buffer[i][axis] < min_vals[axis]) min_vals[axis] = data_buffer[i][axis];
      if (data_buffer[i][axis] > max_vals[axis]) max_vals[axis] = data_buffer[i][axis];
    }
  }
  
  // Perform calibration
  calibrateHardIron(min_vals, max_vals);
  calibrateSoftIron(min_vals, max_vals);
  
  Serial.println(F("Auto-calibration complete!"));
  printCalibration();
}