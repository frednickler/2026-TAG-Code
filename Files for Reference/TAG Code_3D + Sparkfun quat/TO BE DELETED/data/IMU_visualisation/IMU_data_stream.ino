/*
 * ICM-20948 IMU Data Visualization
 *
 * This sketch reads accelerometer and gyroscope data from the ICM-20948 IMU,
 * applies configurable DLPF settings, and streams the data over serial
 * for visualization in Python.
 *
 * Commands:
 * - 's'    // Start/stop data streaming
 * - 'd'    // Toggle debug mode
 * - 'c'    // Toggle calibration mode
 * - 'f'    // Cycle through DLPF settings
 * - 'x'    // Stop data streaming (explicit stop)
 * - 'b'    // Receive calibration bias values from Python
 */

 #include <Wire.h>

 // IMU Configuration
 #define I2C_SDA_IMU 4
 #define I2C_SCL_IMU 5
 #define ICM_20948_ADDR 0x68
 
 // I2C Configuration
 #define I2C_MAX_RETRIES 3
 #define I2C_RETRY_DELAY 10  // ms
 
 // ICM-20948 Registers
 #define REG_BANK_SEL    0x7F
 #define PWR_MGMT_1      0x06  // Bank 0
 #define USER_CTRL       0x03  // Bank 0
 #define INT_PIN_CFG     0x0F  // Bank 0
 #define ACCEL_XOUT_H    0x2D  // Bank 0
 #define ACCEL_XOUT_L    0x2E  // Bank 0
 #define ACCEL_YOUT_H    0x2F  // Bank 0
 #define ACCEL_YOUT_L    0x30  // Bank 0
 #define ACCEL_ZOUT_H    0x31  // Bank 0
 #define ACCEL_ZOUT_L    0x32  // Bank 0
 #define GYRO_XOUT_H     0x33  // Bank 0
 #define GYRO_XOUT_L     0x34  // Bank 0
 #define GYRO_YOUT_H     0x35  // Bank 0
 #define GYRO_YOUT_L     0x36  // Bank 0
 #define GYRO_ZOUT_H     0x37  // Bank 0
 #define GYRO_ZOUT_L     0x38  // Bank 0
 #define ACCEL_CONFIG    0x14  // Bank 2
 #define ACCEL_CONFIG_2  0x15  // Bank 2
 #define GYRO_CONFIG_1   0x01  // Bank 2
 #define GYRO_CONFIG_2   0x02  // Bank 2
 
 // DLPF Settings
 #define DLPF_CFG_0      0x00  // Bandwidth: 246.0/196.6 Hz (Accel/Gyro)
 #define DLPF_CFG_1      0x01  // Bandwidth: 246.0/151.8 Hz
 #define DLPF_CFG_2      0x02  // Bandwidth: 111.4/119.5 Hz
 #define DLPF_CFG_3      0x03  // Bandwidth: 50.4/51.2 Hz
 #define DLPF_CFG_4      0x04  // Bandwidth: 23.9/23.9 Hz
 #define DLPF_CFG_5      0x05  // Bandwidth: 11.5/11.6 Hz
 #define DLPF_CFG_6      0x06  // Bandwidth: 5.7/5.7 Hz
 #define DLPF_CFG_7      0x07  // Bandwidth: 473/361.4 Hz
 
 // Global variables
 bool streaming = false;
 bool debug_mode = false;
 bool calibration_mode = false;
 uint8_t current_dlpf = DLPF_CFG_3;  // Default to medium filtering
 
 // Sensor data
 int16_t accel_raw[3];  // x, y, z
 int16_t gyro_raw[3];   // x, y, z
 float accel_bias[3] = {0, 0, 0};
 float gyro_bias[3] = {0, 0, 0};
 float accel_scale[3] = {1.0, 1.0, 1.0};
 float gyro_scale[3] = {1.0, 1.0, 1.0};
 
 // Calibration buffer
 const size_t MAX_CAL_SAMPLES = 500;
 int16_t accel_cal_buffer[MAX_CAL_SAMPLES][3];
 int16_t gyro_cal_buffer[MAX_CAL_SAMPLES][3];
 size_t cal_sample_count = 0;
 
 // Function prototypes
 bool readAccelerometer();
 bool readGyroscope();
 bool initIMU();
 bool setDLPF(uint8_t setting);
 void calibrateSensors();
 void applyCalibration(int16_t raw[3], float calibrated[3], float bias[3], float scale[3]);
 void parseCalibrationValues();
 bool writeICMRegister(uint8_t bank, uint8_t reg, uint8_t value); // Added prototype for completeness
 
 void setup() {
   Serial.begin(115200);
   while (!Serial) { delay(10); }
 
   // Initialize I2C
   Wire.begin(I2C_SDA_IMU, I2C_SCL_IMU);
   Wire.setClock(400000);  // 400kHz for faster I2C communication
 
   // Initialize IMU
   if (initIMU()) {
     Serial.println(F("ICM-20948 initialized successfully"));
   } else {
     Serial.println(F("Failed to initialize ICM-20948"));
   }
 
   Serial.println(F("ICM-20948 IMU Data Visualization"));
   Serial.println(F("--------------------------------"));
   Serial.println(F("Commands:"));
   Serial.println(F("  's' - Start/stop data streaming"));
   Serial.println(F("  'd' - Toggle debug mode"));
   Serial.println(F("  'c' - Toggle calibration mode"));
   Serial.println(F("  'f' - Cycle through DLPF settings"));
   Serial.println(F("  'x' - Stop data streaming (explicit stop)"));
   Serial.println(F("  'b' - Receive calibration bias values from Python"));
   Serial.println(F("--------------------------------"));
   Serial.println(F("Send 's' to start streaming data..."));
 }
 
 void loop() {
   // Handle serial commands
   if (Serial.available()) {
     char c = Serial.read();
 
     // Only clear the buffer for non-'b' commands
     if (c != 'b') {
       while(Serial.available()) {
         Serial.read();
       }
     }
 
     switch (c) {
       case 's':
         streaming = !streaming;
         if (streaming) {
           Serial.println(F("Streaming started"));
         } else {
           Serial.println(F("Streaming stopped"));
         }
         break;
 
       case 'd':
         debug_mode = !debug_mode;
         Serial.print(F("Debug mode: "));
         Serial.println(debug_mode ? F("ON") : F("OFF"));
         break;
 
       case 'c':
         calibration_mode = !calibration_mode;
         if (calibration_mode) {
           Serial.println(F("Calibration mode ON - collecting data..."));
           cal_sample_count = 0;
         } else {
           if (cal_sample_count > 0) {
             Serial.println(F("Calibration mode OFF - processing calibration..."));
             calibrateSensors();
           } else {
             Serial.println(F("Calibration mode OFF - no data collected"));
           }
         }
         break;
 
       case 'x':
         // Explicit stop command (doesn't toggle)
         streaming = false;
         Serial.println(F("Streaming stopped"));
         break;
 
       case 'f':
         // Cycle through DLPF settings
         switch(current_dlpf) {
           case DLPF_CFG_0: current_dlpf = DLPF_CFG_1; break;
           case DLPF_CFG_1: current_dlpf = DLPF_CFG_2; break;
           case DLPF_CFG_2: current_dlpf = DLPF_CFG_3; break;
           case DLPF_CFG_3: current_dlpf = DLPF_CFG_4; break;
           case DLPF_CFG_4: current_dlpf = DLPF_CFG_5; break;
           case DLPF_CFG_5: current_dlpf = DLPF_CFG_6; break;
           case DLPF_CFG_6: current_dlpf = DLPF_CFG_7; break;
           case DLPF_CFG_7: current_dlpf = DLPF_CFG_0; break;
           default: current_dlpf = DLPF_CFG_3; break;
         }
 
         if (setDLPF(current_dlpf)) {
           Serial.print(F("DLPF set to "));
           Serial.println(current_dlpf);
         } else {
           Serial.println(F("Failed to set DLPF"));
         }
         break;
 
         case 'b': {
          // First, explicitly stop streaming to ensure clean serial communication
          streaming = false;
          Serial.println(F("ARDUINO_READY_FOR_BIAS_DATA")); // Signal Python that Arduino is ready
          Serial.flush(); // Ensure the message is sent immediately
  
          // Clear any potentially old/stale data in the input buffer *before* waiting for new data
          // This ensures we don't accidentally read previous partial messages.
          while(Serial.available() > 0) {
            Serial.read();
          }
  
          // Wait for the start marker 'B' with a timeout.
          // This is crucial to ensure Python has sent the command and data.
          unsigned long start_wait_b_marker = millis();
          bool b_marker_received = false;
          while (millis() - start_wait_b_marker < 3000) { // 3-second timeout for 'B'
            if (Serial.available()) {
              char incoming_char = Serial.peek(); // Peek to see if it's 'B'
              if (incoming_char == 'B') {
                Serial.read(); // Consume the 'B' character
                b_marker_received = true;
                break;
              } else {
                // If it's not 'B', it might be leftover junk, consume it.
                Serial.read();
              }
            }
            delay(5); // Small delay to prevent busy-waiting
          }
  
          if (!b_marker_received) {
            Serial.println(F("ERROR: Timeout or 'B' marker not received from Python."));
            break; // Exit case 'b'
          }
  
          // Now that 'B' is confirmed and consumed, call parseCalibrationValues
          // to read the rest of the line (e.g., ",ax,ay,az,gx,gy,gz")
          parseCalibrationValues();
          break;
        }
     }
   }
 
   // Read sensor data
   bool accel_valid = readAccelerometer();
   bool gyro_valid = readGyroscope();
 
   // If in calibration mode, collect samples
   if (calibration_mode && accel_valid && gyro_valid && cal_sample_count < MAX_CAL_SAMPLES) {
     for (int i = 0; i < 3; i++) {
       accel_cal_buffer[cal_sample_count][i] = accel_raw[i];
       gyro_cal_buffer[cal_sample_count][i] = gyro_raw[i];
     }
     cal_sample_count++;
 
     if (cal_sample_count >= MAX_CAL_SAMPLES) {
       Serial.println(F("Calibration data collection complete"));
       calibration_mode = false;
       calibrateSensors();
     }
   }
 
   // Stream data if enabled
   if (streaming && accel_valid && gyro_valid) {
     // Apply calibration if available
     float accel_calibrated[3];
     float gyro_calibrated[3];
 
     applyCalibration(accel_raw, accel_calibrated, accel_bias, accel_scale);
     applyCalibration(gyro_raw, gyro_calibrated, gyro_bias, gyro_scale);
 
     // Format: "A,ax,ay,az,G,gx,gy,gz,C,cax,cay,caz,D,cgx,cgy,cgz"
     Serial.print("A,");
     for (int i = 0; i < 3; i++) {
       Serial.print(accel_raw[i]);
       if (i < 2) Serial.print(",");
     }
 
     Serial.print(",G,");
     for (int i = 0; i < 3; i++) {
       Serial.print(gyro_raw[i]);
       if (i < 2) Serial.print(",");
     }
 
     Serial.print(",C,");
     for (int i = 0; i < 3; i++) {
       Serial.print(accel_calibrated[i], 4);
       if (i < 2) Serial.print(",");
     }
 
     Serial.print(",D,");
     for (int i = 0; i < 3; i++) {
       Serial.print(gyro_calibrated[i], 4);
       if (i < 2) Serial.print(",");
     }
 
     Serial.println();
   }
 
   // Small delay to prevent flooding the serial port
   delay(10);
 }
 
 bool initIMU() {
   // 1. Reset ICM-20948
   if (!writeICMRegister(0, PWR_MGMT_1, 0x80)) return false;  // Device Reset
   delay(100);
 
   // 2. Wake up the ICM-20948 and select the best clock source
   if (!writeICMRegister(0, PWR_MGMT_1, 0x01)) return false;  // Clock auto-select
   delay(10);
 
   // 3. Configure accelerometer
   // Assuming ±4g range (FS_SEL = 1, ACCEL_FS_SEL bit 1)
   // Check ICM-20948 datasheet for ACCEL_CONFIG register (Bank 2, Address 0x14)
   // Bits [2:1] ACCEL_FS_SEL: 00 = ±2g, 01 = ±4g, 10 = ±8g, 11 = ±16g
   // Here, 0x01 sets ACCEL_FS_SEL to 01 (±4g)
   if (!writeICMRegister(2, ACCEL_CONFIG, 0x01)) return false;
   delay(10);
 
   // 4. Configure gyroscope
   // Assuming ±500 dps range (FS_SEL = 1, GYRO_FS_SEL bit 1)
   // Check ICM-20948 datasheet for GYRO_CONFIG_1 register (Bank 2, Address 0x01)
   // Bits [2:1] GYRO_FS_SEL: 00 = ±250dps, 01 = ±500dps, 10 = ±1000dps, 11 = ±2000dps
   // Here, 0x01 sets GYRO_FS_SEL to 01 (±500 dps)
   if (!writeICMRegister(2, GYRO_CONFIG_1, 0x01)) return false;
   delay(10);
 
   // 5. Set initial DLPF configuration
   if (!setDLPF(current_dlpf)) return false;
 
   return true;
 }
 
 bool setDLPF(uint8_t setting) {
   if (setting > 7) setting = 7;  // Ensure valid range
 
   // Set accelerometer DLPF
   // ACCEL_CONFIG_2 (Bank 2, Address 0x15) - Bits [5:3] A_DLPFCFG
   // setting directly maps to these bits (0-7)
   if (!writeICMRegister(2, ACCEL_CONFIG_2, setting)) return false;
 
   // Set gyroscope DLPF
   // GYRO_CONFIG_2 (Bank 2, Address 0x02) - Bits [5:3] G_DLPFCFG
   // setting directly maps to these bits (0-7)
   if (!writeICMRegister(2, GYRO_CONFIG_2, setting)) return false;
 
   return true;
 }
 
 bool readAccelerometer() {
   uint8_t data[6];
 
   // Select Bank 0
   if (!writeICMRegister(0, REG_BANK_SEL, 0x00)) return false;
 
   // Read accelerometer data
   Wire.beginTransmission(ICM_20948_ADDR);
   Wire.write(ACCEL_XOUT_H);
   if (Wire.endTransmission(false) != 0) return false;
 
   if (Wire.requestFrom(ICM_20948_ADDR, 6, true) != 6) return false;
 
   for (int i = 0; i < 6; i++) {
     data[i] = Wire.read();
   }
 
   // Combine high and low bytes
   accel_raw[0] = (int16_t)((data[0] << 8) | data[1]);  // X-axis
   accel_raw[1] = (int16_t)((data[2] << 8) | data[3]);  // Y-axis
   accel_raw[2] = (int16_t)((data[4] << 8) | data[5]);  // Z-axis
 
   return true;
 }
 
 bool readGyroscope() {
   uint8_t data[6];
 
   // Select Bank 0
   if (!writeICMRegister(0, REG_BANK_SEL, 0x00)) return false;
 
   // Read gyroscope data
   Wire.beginTransmission(ICM_20948_ADDR);
   Wire.write(GYRO_XOUT_H);
   if (Wire.endTransmission(false) != 0) return false;
 
   if (Wire.requestFrom(ICM_20948_ADDR, 6, true) != 6) return false;
 
   for (int i = 0; i < 6; i++) {
     data[i] = Wire.read();
   }
 
   // Combine high and low bytes
   gyro_raw[0] = (int16_t)((data[0] << 8) | data[1]);  // X-axis
   gyro_raw[1] = (int16_t)((data[2] << 8) | data[3]);  // Y-axis
   gyro_raw[2] = (int16_t)((data[4] << 8) | data[5]);  // Z-axis
 
   return true;
 }
 
 bool writeICMRegister(uint8_t bank, uint8_t reg, uint8_t value) {
   for (int attempt = 0; attempt < I2C_MAX_RETRIES; attempt++) {
     // Select the bank
     Wire.beginTransmission(ICM_20948_ADDR);
     Wire.write(REG_BANK_SEL);
     Wire.write(bank << 4);  // Bank selection (bits [5:4] of REG_BANK_SEL)
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
 
   if (debug_mode) {
     Serial.print(F("I2C Error: Failed to write to register 0x"));
     Serial.print(reg, HEX);
     Serial.print(F(" in bank "));
     Serial.println(bank);
   }
   return false;
 }
 
 void calibrateSensors() {
   if (cal_sample_count < 50) {
     Serial.println(F("Not enough samples for calibration"));
     return;
   }
 
   // Calculate average for bias (offset)
   float accel_sum[3] = {0, 0, 0};
   float gyro_sum[3] = {0, 0, 0};
 
   for (size_t i = 0; i < cal_sample_count; i++) {
     for (int axis = 0; axis < 3; axis++) {
       accel_sum[axis] += accel_cal_buffer[i][axis];
       gyro_sum[axis] += gyro_cal_buffer[i][axis];
     }
   }
 
   // Calculate bias (average)
   for (int axis = 0; axis < 3; axis++) {
     accel_bias[axis] = accel_sum[axis] / cal_sample_count;
     gyro_bias[axis] = gyro_sum[axis] / cal_sample_count;
   }
 
   // For accelerometer, adjust Z-axis to account for gravity (1g)
   // At ±4g range, the sensitivity is 8192 LSB/g.
   // So, 1g should be approximately 8192 LSB.
   // Assuming the device is placed flat during calibration, the Z-axis raw reading
   // will include the 1g acceleration of gravity. We subtract this to effectively
   // make the bias correspond to 0g in the Z-axis.
   accel_bias[2] -= 8192.0;
 
   // For scale calculation, you generally need to move the IMU through its full range of motion.
   // The current calibration only calculates bias (offset).
   // If you want full scale calibration, you would need to define a more complex
   // calibration procedure (e.g., rotating the sensor to get +/- readings on each axis).
   // For simplicity here, we'll keep scale factors at 1.0 (no scaling, only bias correction).
   // If you truly need scale correction, you'd calculate min/max values for each axis
   // from a set of data where the sensor has been rotated to its extremes, and then
   // calculate the scale factor to normalize the range.
   // For now, let's reset scales to 1.0 after bias calculation if they were modified elsewhere.
   accel_scale[0] = 1.0; accel_scale[1] = 1.0; accel_scale[2] = 1.0;
   gyro_scale[0] = 1.0; gyro_scale[1] = 1.0; gyro_scale[2] = 1.0;
 
 
   Serial.println(F("Calibration complete!"));
 
   if (debug_mode) {
     Serial.println(F("Accelerometer calibration:"));
     Serial.print(F("Bias: "));
     for (int i = 0; i < 3; i++) {
       Serial.print(accel_bias[i], 4); // Print with 4 decimal places for float
       if (i < 2) Serial.print(", ");
     }
     Serial.println();
 
     Serial.print(F("Scale: "));
     for (int i = 0; i < 3; i++) {
       Serial.print(accel_scale[i], 6);
       if (i < 2) Serial.print(", ");
     }
     Serial.println();
 
     Serial.println(F("Gyroscope calibration:"));
     Serial.print(F("Bias: "));
     for (int i = 0; i < 3; i++) {
       Serial.print(gyro_bias[i], 4);
       if (i < 2) Serial.print(", ");
     }
     Serial.println();
 
     Serial.print(F("Scale: "));
     for (int i = 0; i < 3; i++) {
       Serial.print(gyro_scale[i], 6);
       if (i < 2) Serial.print(", ");
     }
     Serial.println();
   }
 }
 
 void applyCalibration(int16_t raw[3], float calibrated[3], float bias[3], float scale[3]) {
   for (int i = 0; i < 3; i++) {
     // Apply bias correction (subtract bias)
     float corrected = (float)raw[i] - bias[i]; // Cast raw[i] to float before subtraction
     // Apply scale correction
     calibrated[i] = corrected * scale[i];
   }
 }
 
 void parseCalibrationValues() {
  // This function is now called *after* the 'B' marker has been received and consumed
  // by the calling context (the 'b' case in loop()).
  // So, we just need to read the rest of the line.

  // Wait for the full line to be available with a timeout
  unsigned long start_time = millis();
  const unsigned long line_timeout = 2000; // 2 second timeout for the rest of the line

  // Wait for at least one character or a newline, then read the whole string.
  // This is a common pattern: readStringUntil() will block until newline or timeout.
  String data_str = "";
  while (millis() - start_time < line_timeout) {
      if (Serial.available()) {
          data_str = Serial.readStringUntil('\n');
          break; // Line received
      }
      delay(5);
  }

  data_str.trim(); // Remove any leading/trailing whitespace, including carriage return

  // Debug output
  if (debug_mode) {
    Serial.print(F("CAL_DEBUG: Received string for parsing: '"));
    Serial.print(data_str);
    Serial.println("'");
  }

  // Expected format (after 'B' is removed): ",ax,ay,az,gx,gy,gz"
  // Remove the leading comma if present
  if (data_str.length() > 0 && data_str[0] == ',') {
    data_str = data_str.substring(1);
  } else {
    // This warning might be helpful during debugging if Python isn't sending a leading comma
    if (debug_mode) {
      Serial.print(F("Warning: No leading comma found in bias data string: "));
      Serial.println(data_str);
    }
  }

  float parsed_values[6]; // Array to temporarily store parsed float values
  int count = 0;          // Counter for successfully parsed values
  int lastPos = 0;        // Last position of a comma + 1

  // Parse comma-separated values using indexOf and substring
  // Iterate through the string, finding commas or reaching the end
  // The loop condition 'i <= data_str.length()' is important to process the last value
  // which might not be followed by a comma.
  for (int i = 0; i <= data_str.length() && count < 6; i++) {
    // If a comma is found or it's the end of the string
    if (data_str[i] == ',' || i == data_str.length()) {
      // Extract the substring for the current value
      String valStr = data_str.substring(lastPos, i);
      // Convert the substring to a float and store it
      parsed_values[count++] = valStr.toFloat();
      lastPos = i + 1; // Update lastPos to after the current comma
    }
  }

  // Check if exactly 6 values were successfully parsed
  if (count == 6) {
    // Assign parsed values to the global bias arrays
    accel_bias[0] = parsed_values[0];
    accel_bias[1] = parsed_values[1];
    accel_bias[2] = parsed_values[2];
    gyro_bias[0] = parsed_values[3];
    gyro_bias[1] = parsed_values[4];
    gyro_bias[2] = parsed_values[5];

    Serial.println(F("Calibration biases received and applied from Python:"));
    Serial.print(F("Accel bias: X=")); Serial.print(accel_bias[0], 4);
    Serial.print(F(", Y=")); Serial.print(accel_bias[1], 4);
    Serial.print(F(", Z=")); Serial.println(accel_bias[2], 4);

    Serial.print(F("Gyro bias: X=")); Serial.print(gyro_bias[0], 4);
    Serial.print(F(", Y=")); Serial.print(gyro_bias[1], 4);
    Serial.print(F(", Z=")); Serial.println(gyro_bias[2], 4);

    // Verify values: Check for NaN (Not a Number) which indicates a failed float conversion
    bool parse_error = false;
    for (int i = 0; i < 3; i++) {
      if (isnan(accel_bias[i]) || isnan(gyro_bias[i])) {
        Serial.println(F("ERROR: Failed to parse some calibration values (NaN detected)."));
        parse_error = true;
        break; // Exit loop after first warning
      }
    }
    if (!parse_error) {
        Serial.println(F("All calibration values parsed successfully."));
    }

  } else {
    // If not exactly 6 values, the format was incorrect
    Serial.print(F("ERROR: Invalid calibration data format. Expected 6 values, but parsed "));
    Serial.print(count);
    Serial.print(F(" values. Received string was: '"));
    Serial.print(data_str);
    Serial.println("'");
  }

  // Ensure scale factors remain 1.0, as Python only sends bias
  accel_scale[0] = 1.0; accel_scale[1] = 1.0; accel_scale[2] = 1.0;
  gyro_scale[0] = 1.0; gyro_scale[1] = 1.0; gyro_scale[2] = 1.0;
}