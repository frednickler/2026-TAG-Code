# ESP32-S3 ICM20948 + AK09916 Sensor Fusion Project

## Overview
This project implements robust sensor fusion (Madgwick) using an ESP32-S3 DevKit and the ICM20948 IMU (with AK09916 magnetometer). The code is modular, with user-friendly configuration for all key sensor filtering and output rates.

This project implements a robust sensor fusion solution for an ESP32-S3 microcontroller connected to an ICM-20948 9-axis IMU. It uses the Madgwick AHRS algorithm to calculate orientation (quaternions and Euler angles) and includes a comprehensive, real-time validation suite to monitor sensor performance.

## Features

- **9-DOF Sensor Fusion**: Combines accelerometer, gyroscope, and magnetometer data using the Madgwick AHRS algorithm.
- **6-DOF Fallback**: Intelligently switches to 6-DOF mode (accel + gyro) if magnetometer data becomes unreliable.
- **Real-time Validation Suite**: Prints a detailed report to the serial monitor every second, checking for:
    - Raw vs. Filtered data comparison (hardware DLPF).
    - Quaternion validity (normalization, NaN/Inf checks).
    - Euler angle round-trip consistency.
    - Static condition checks (device at rest).
    - Temperature sensor range.
- **CSV Data Logging**: Outputs sensor and fusion data in CSV format for easy analysis and plotting.
- **Configurable Settings**: Easily adjust key parameters like sensor filter levels and output data rates.

## How to Configure IMU and Magnetometer Filters

### 1. Where to Configure
All configuration for the IMU and magnetometer (output data rate and filtering) is done in the file:

```
include/imu_config.h
```

### 2. How to Change Presets
At the top of `imu_config.h`, you will see these lines:

```cpp
#define MAG_ODR_PRESET 3
#define ACCEL_DLPF_PRESET 3
#define GYRO_DLPF_PRESET 3
```

Change the number (1-5) for each sensor to select the desired preset **before uploading** the code.

- **MAG_ODR_PRESET:** Magnetometer Output Data Rate
- **ACCEL_DLPF_PRESET:** Accelerometer Digital Low Pass Filter
- **GYRO_DLPF_PRESET:** Gyroscope Digital Low Pass Filter

### 3. What Each Preset Means
Preset tables are included as comments in `imu_config.h` for quick reference. Example:

```
// =================== Magnetometer ODR Preset Table ===================
//   Preset | ODR Setting | Output Data Rate (Hz)
//   --------------------------------------------
//      1   |   0x02      |      10
//      2   |   0x04      |      20
//      3   |   0x06      |      50
//      4   |   0x08      |     100
//      5   |   0x08      |     100
//   --------------------------------------------

// =================== Accel DLPF Preset Table ===================
//   Preset | DLPF Setting | Bandwidth (Hz)
//   --------------------------------------
//      1   |   0x06       |      5
//      2   |   0x05       |     10
//      3   |   0x03       |    111
//      4   |   0x01       |    246
//      5   |   0x00       |   No filter
//   --------------------------------------

// =================== Gyro DLPF Preset Table ===================
//   Preset | DLPF Setting | Bandwidth (Hz)
//   --------------------------------------
//      1   |   0x06       |      5
//      2   |   0x05       |     10
//      3   |   0x03       |    111
//      4   |   0x01       |    246
//      5   |   0x00       |   No filter
//   --------------------------------------
```

## Advanced Accelerometer Configuration

### NEW: User-Friendly Configuration Interface

For precise accelerometer control, use the new user-friendly configuration system in `src/accel_config.h` and `src/accel_config.cpp`. This allows direct specification using intuitive numbers instead of cryptic register values.

#### Easy Configuration Function

```cpp
// In your Arduino code:
AccelCfg::applyAccelConfigDirect(fsr_g, dlpf_hz, odr_hz);

// Examples:
AccelCfg::applyAccelConfigDirect(4, 50, 225);   // General purpose
AccelCfg::applyAccelConfigDirect(2, 12, 112);   // High-res, smooth
AccelCfg::applyAccelConfigDirect(16, 0, 1125);  // Extreme shocks
AccelCfg::applyAccelConfigDirect(8, 246, 562);  // Sports/aerobatics
```

#### Parameters Explained

**1. 📊 FSR (Full-Scale Range) - Just enter the g-force:**
- `2` = ±2g (highest resolution: 16,384 LSB/g)
- `4` = ±4g (good balance: 8,192 LSB/g)
- `8` = ±8g (higher range: 4,096 LSB/g)
- `16` = ±16g (maximum range: 2,048 LSB/g)

**2. 🔧 DLPF (Digital Low-Pass Filter) - Just enter the bandwidth in Hz:**
- `0` = DISABLED (bypass mode, ~1209 Hz BW)
- `6` = 6 Hz (maximum smoothing, 27.9 ms delay)
- `12` = 12 Hz (high noise rejection, 15.5 ms delay)
- `24` = 24 Hz (smooth motion, 7.8 ms delay)
- `50` = 50 Hz (general purpose, 3.9 ms delay)
- `111` = 111 Hz (moderate filtering, 1.69 ms delay)
- `246` = 246 Hz (fast response, 0.97 ms delay)
- `473` = 473 Hz (high bandwidth, 0.66 ms delay)

**3. ⚡ ODR (Output Data Rate) - Just enter the frequency in Hz:**
- `1` = 1 Hz (ultra-low power)
- `7` = 7 Hz (low power logging)
- `14` = 14 Hz (minimal power)
- `28` = 28 Hz (very slow sampling)
- `56` = 56 Hz (slow applications)
- `112` = 112 Hz (lower bandwidth)
- `225` = 225 Hz (general purpose)
- `375` = 375 Hz (standard rate)
- `562` = 562 Hz (fast control loops)
- `1125` = 1125 Hz (maximum rate)

#### Common Use Cases

```cpp
// Human motion tracking (smooth, high resolution)
AccelCfg::applyAccelConfigDirect(2, 12, 112);

// General purpose drone/vehicle (balanced)
AccelCfg::applyAccelConfigDirect(4, 50, 225);

// Sports equipment (moderate shocks, fast response)
AccelCfg::applyAccelConfigDirect(8, 111, 375);

// Crash detection (maximum range, fast sampling)
AccelCfg::applyAccelConfigDirect(16, 246, 1125);

// Battery-powered logging (ultra-low power)
AccelCfg::applyAccelConfigDirect(4, 6, 7);
```

#### Get Help

```cpp
// Print all available options to serial monitor:
AccelCfg::printAccelConfigOptions();
```

#### Alternative Configuration Methods

**Compile-time Configuration (src/accel_config.h):**
```cpp
constexpr uint8_t USER_FS_SEL = FS_4G;           // ±4g
constexpr uint8_t USER_DLPF_CFG = DLPF_50HZ;     // 50 Hz
constexpr uint16_t USER_SMPLRT_DIV = ODR_225HZ_DIV; // 225 Hz
```

**Runtime CLI Presets:**
```
acfg 1    // ±2g, 50Hz DLPF, 225Hz ODR
acfg 2    // ±4g, 50Hz DLPF, 225Hz ODR  
acfg 3    // ±8g, 50Hz DLPF, 225Hz ODR
acfg 4    // ±16g, 50Hz DLPF, 225Hz ODR
acfg 5    // ±4g, 24Hz DLPF, 112Hz ODR
```

## Advanced Magnetometer Configuration

### NEW: User-Friendly Magnetometer Interface

For precise magnetometer control, use the new user-friendly configuration system in `src/mag_config.h` and `src/mag_config.cpp`. The AK09916 magnetometer has unique characteristics compared to accelerometer and gyroscope.

#### Easy Configuration Function

```cpp
// In your Arduino code:
MagCfg::applyMagConfigDirect(odr_hz, do_reset);

// Examples:
MagCfg::applyMagConfigDirect(100, true);  // 100Hz continuous, general purpose
MagCfg::applyMagConfigDirect(20, true);   // 20Hz continuous, standard compass
MagCfg::applyMagConfigDirect(10, true);   // 10Hz continuous, battery-powered
MagCfg::applyMagConfigDirect(1, true);    // Single-shot mode, ultra-low power
MagCfg::applyMagConfigDirect(0, true);    // Power-down mode, sleep
```

#### Parameters Explained

**1. 📊 FSR (Full-Scale Range) - FIXED:**
- **±4900 µT** (cannot be changed by user)
- **0.15 µT/LSB** sensitivity (16-bit resolution)
- Covers typical Earth magnetic field (25-65 µT) with plenty of headroom
- Unlike accelerometer/gyroscope, magnetometer FSR is hardware-fixed

**2. 🔧 DLPF (Digital Low-Pass Filter) - NOT AVAILABLE:**
- AK09916 does NOT have user-configurable DLPF settings
- Internal filtering is fixed and optimized for each operating mode
- **Alternative filtering strategies:**
  - Use lower ODR (10Hz or 20Hz instead of 100Hz)
  - Apply software filtering in your application
  - Adjust Madgwick filter beta parameter
  - Use calibration to remove hard/soft iron distortions

**3. ⚡ ODR (Output Data Rate) - Just enter the frequency in Hz:**
- `0` = Power-down mode (0.1 µA) - sleep/standby
- `1` = Single-shot mode (~10 µA per shot) - ultra-low power
- `10` = 10 Hz continuous (~100 µA) - battery-powered compass
- `20` = 20 Hz continuous (~200 µA) - standard compass
- `50` = 50 Hz continuous (~500 µA) - moderate-speed tracking
- `100` = 100 Hz continuous (~1000 µA) - high-rate fusion [DEFAULT]

**4. 🔄 Reset Option:**
- `true` = Perform soft reset before setting mode [RECOMMENDED]
- `false` = Skip reset (advanced users only)

#### Power Consumption Considerations

**Important**: Magnetometer power consumption varies dramatically:
- **100Hz mode** consumes ~10x more power than 10Hz mode
- **Single-shot mode** is most efficient for infrequent measurements
- **Power-down mode** for sleep/standby (only 0.1 µA)

#### Common Use Cases

```cpp
// High-rate sensor fusion (drones, fast motion)
MagCfg::applyMagConfigDirect(100, true);

// Standard compass applications
MagCfg::applyMagConfigDirect(20, true);

// Battery-powered compass (basic orientation)
MagCfg::applyMagConfigDirect(10, true);

// Ultra-low power (triggered measurements)
MagCfg::applyMagConfigDirect(1, true);

// Sleep mode (when magnetometer not needed)
MagCfg::applyMagConfigDirect(0, true);
```

#### Get Help

```cpp
// Print all available options to serial monitor:
MagCfg::printMagConfigOptions();
```

#### Alternative Configuration Methods

**Compile-time Configuration (src/mag_config.h):**
```cpp
constexpr uint8_t USER_MODE = MODE_CONT_100HZ;  // 100 Hz continuous
constexpr bool USER_DO_SOFT_RESET = true;      // Perform soft reset
```

**Runtime CLI Presets:**
```
mcfg 0    // Power-down mode
mcfg 1    // Single-shot mode
mcfg 2    // 10Hz continuous
mcfg 3    // 20Hz continuous
mcfg 4    // 50Hz continuous
mcfg 5    // 100Hz continuous [DEFAULT]
```

#### Key Differences from Accelerometer/Gyroscope

- **Fixed FSR**: Cannot change measurement range (±4900 µT)
- **No DLPF**: No user-configurable digital filtering
- **Power Focus**: Dramatic power differences between modes
- **Operating Modes**: Discrete modes rather than continuous parameter ranges

## Axis Mapping System

The axis mapping system ensures consistent coordinate system across all sensors and sensor fusion algorithms (including the Madgwick filter).

### Current Implementation

The system applies a unified coordinate transformation to all sensor readings:
- **X-axis**: Forward direction
- **Y-axis**: Right direction  
- **Z-axis**: Up direction (against gravity)

### Physical Sensor Orientation

Based on testing, the current board orientation mapping is:
- **Z up** = board face down
- **Z down** = board face up
- **Y up** = face to left
- **Y down** = face to right
- **X up** = nose up
- **X down** = nose down

### Adjusting Axis Mapping

The axis mapping system is now ready for use. If you need to adjust the specific axis transformations based on your physical sensor orientation test results, simply modify the `applyAxisMapping()` function in `imu_config.cpp`.

The function applies the same transformation to all sensors (accelerometer, gyroscope, magnetometer) to ensure consistency across the entire system.

### Testing Axis Mapping

Use the standalone `axis_mapping_test.ino` sketch to verify your sensor's physical orientation and generate appropriate mapping code for your specific hardware setup.

## IMU Configuration and Axis Mapping

### Axis Mapping System

The IMU system uses a consistent coordinate system across all sensors to ensure accurate sensor fusion. The axis mapping is handled automatically by the following functions:

```cpp
// In imu_config.h
void applyAxisMapping(float raw_x, float raw_y, float raw_z,
                     float& mapped_x, float& mapped_y, float& mapped_z);

// Pre-mapped sensor reading functions
bool readAccelMapped(float& ax, float& ay, float& az);
bool readGyroMapped(float& gx, float& gy, float& gz);
bool readMagMapped(float& mx, float& my, float& mz);
```

### Physical Sensor Orientation

For proper orientation understanding, the following conventions are used:
- **Z axis**: 
  - **Up** = Board face down 
  - **Down** = Board face up
- **Y axis**:
  - **Up** = Face to left
  - **Down** = Face to right
- **X axis**:
  - **Up** = Nose up
  - **Down** = Nose down

### Configuration Functions

The IMU configuration system provides low-level control over sensor settings:

#### Basic Configuration
```cpp
// Set accelerometer digital low-pass filter
void setAccelDLPF(uint8_t mode);  // 0-7 (see preset tables in imu_config.h)

// Set gyroscope digital low-pass filter
void setGyroDLPF(uint8_t mode);   // 0-7 (see preset tables in imu_config.h)

// Set magnetometer output data rate
void setMagOutputDataRate(uint8_t rate_config);  // See mag_odr_table in imu_config.cpp
```

#### Initialization
```cpp
// Initialize the IMU with default settings
bool initIMU();

// Switch between register banks (0-3)
void setBank(uint8_t bank);

// Read/write registers
void writeRegister(uint8_t reg, uint8_t val);
uint8_t readRegister(uint8_t reg);
uint8_t readRegisters(uint8_t reg, uint8_t* data, uint8_t len);
```

### Best Practices

1. **Always use mapped sensor readings** (`*Mapped` functions) for sensor fusion to ensure consistent coordinate systems.
2. After changing any sensor configuration, verify the settings by reading back the registers.
3. When developing new features that use raw sensor data, test the axis mapping thoroughly using the `axis_mapping_test.ino` utility.
4. For optimal performance, choose DLPF settings that match your application's bandwidth requirements.

### Troubleshooting

#### Common Issues

1. **Incorrect orientation**: If the orientation seems wrong, verify the physical mounting of the IMU matches the expected orientation.
2. **Noisy readings**: Try enabling or adjusting the DLPF settings to reduce noise.
3. **I2C errors**: If experiencing I2C communication issues, check wiring and consider adding pull-up resistors.

#### Debugging Tools

Use the built-in `axis_mapping_test.ino` to verify the sensor orientation and axis mapping:
1. Upload the test sketch to your board
2. Follow the on-screen instructions to verify each axis
3. The test will automatically generate the correct mapping code if needed

### Advanced Configuration

For advanced users, you can directly access the configuration tables:

```cpp
// Accelerometer DLPF settings (0-7)
extern const uint8_t accel_dlpf_table[8];

// Gyroscope DLPF settings (0-7)
extern const uint8_t gyro_dlpf_table[8];

// Temperature DLPF settings (0-7)
extern const uint8_t temp_dlpf_table[8];

// Magnetometer ODR settings (10Hz-100Hz)
extern const uint8_t mag_odr_table[6];

// Full scale range tables
extern const uint8_t accel_fss_table[4];  // 2g, 4g, 8g, 16g
extern const uint8_t gyro_fss_table[4];   // 250dps, 500dps, 1000dps, 2000dps
```

### Example: Custom IMU Configuration

```cpp
#include "imu_config.h"

void setup() {
  // Initialize I2C
  Wire.begin();
  
  // Initialize IMU with default settings
  if (!initIMU()) {
    Serial.println("IMU initialization failed!");
    while(1);
  }
  
  // Configure accelerometer for high bandwidth
  setAccelDLPF(3);  // ~111Hz bandwidth
  
  // Configure gyroscope for medium bandwidth
  setGyroDLPF(5);   // ~10Hz bandwidth
  
  // Configure magnetometer for 50Hz output
  setMagOutputDataRate(3);  // 50Hz
}

void loop() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  
  // Read sensors with automatic axis mapping
  if (readAccelMapped(ax, ay, az) && 
      readGyroMapped(gx, gy, gz) && 
      readMagMapped(mx, my, mz)) {
    
    // Use the mapped sensor data for your application
    // ...
  }
  
  delay(10);
}
```

### Version History

- **v1.0.0**: Initial implementation with basic axis mapping
- **v1.1.0**: Added comprehensive DLPF and ODR configuration
- **v1.2.0**: Improved error handling and recovery
- **v1.3.0**: Added temperature-compensated sensor readings

## How to Use

1.  **Hardware Setup**: Ensure your ICM-20948 sensor is correctly wired to the I2C pins of your ESP32-S3.
2.  **PlatformIO**: Open this project in VS Code with the PlatformIO extension.
3.  **Build & Upload**: Use the PlatformIO controls to build and upload the firmware to your ESP32.
4.  **Monitor Output**: Open the PlatformIO Serial Monitor (set to 115200 baud). You will see two types of output:
    - **Validation Reports**: A human-readable report every second that summarizes sensor health and fusion status.
    - **CSV Data**: A continuous stream of comma-separated values, perfect for logging to a file or plotting.

## Configuration

All major configuration options are located in `include/imu_config.h`. To change a setting, simply edit the file and re-upload the firmware.

### Sensor Filtering

You can control the hardware Digital Low-Pass Filter (DLPF) for the accelerometer and gyroscope. This helps reduce noise at the cost of some latency.

-   `GYRO_DLPF_LEVEL`: Sets the gyroscope filter level.
-   `ACCEL_DLPF_LEVEL`: Sets the accelerometer filter level.

**Levels (0-7):**
-   `0`: Bypasses the filter for the lowest latency and "truest" raw data.
-   `1-6`: Increasing levels of filtering (higher number = more smoothing, more lag).
-   `7`: Heaviest filtering.

### Magnetometer ODR

-   `MAG_ODR_PRESET`: Controls the Output Data Rate (ODR) of the AK09916 magnetometer. Higher rates provide more frequent updates but consume more power.

### Sensor Calibration

Raw sensor offsets and scale factors are defined in `include/imu_calibration.h`. For best performance, you should perform a calibration routine and update these values with the results for your specific sensor.

## Suggestions & Troubleshooting

-   **Improve Calibration**: The current calibration values are placeholders. For high-accuracy applications, use a dedicated calibration tool (e.g., Magneto) to find the precise offsets and scale factors for your IMU and update them in `imu_calibration.h`.
-   **No Serial Output?**: Check that the baud rate in the Serial Monitor is set to `115200`. Verify your I2C wiring.
-   **"Mag Read Failed"**: This can happen if the magnetometer is not responding. The code will automatically fall back to 6-DOF fusion. Check your hardware connections. If the issue persists, the magnetometer on your board may be faulty.

## Quick Start
1. Connect your ESP32-S3 and IMU/magnetometer as per your hardware setup.
2. Edit `include/imu_config.h` to select your desired filter/ODR presets.
3. Build and upload the code using PlatformIO.
4. Open the serial monitor to view sensor and fusion output.
5. Adjust presets as needed for your application.

---

## Support
If you have issues with sensor fusion not updating, check the serial output for warnings or errors. Most common causes are sensor read failures, miswiring, or calibration issues.

For further help, open an issue or contact the project maintainer.

## Three-Tier IMU Calibration System

The project implements a sophisticated three-tier calibration and verification system for optimal sensor performance:

### 1. Robust One-Time Calibration
- **When**: During initial setup or when full recalibration is needed
- **What**: Comprehensive calibration of all sensors
  - Accelerometer: 6-position tumble calibration with temperature compensation
  - Gyroscope: Zero-rate and temperature-compensated calibration
  - Magnetometer: Hard/soft iron calibration with temperature tracking
- **Duration**: ~5 minutes
- **Storage**: Saved to EEPROM with integrity verification
- **Fallback**: Automatic fallback to quick calibration if robust method fails

### 2. Startup Verification
- **When**: Every device startup
- **What**: Quick health check of all sensors
  - Verifies EEPROM calibration data integrity
  - Compares current sensor readings with saved calibration
  - Checks temperature changes since last calibration
  - Validates sensor communication and basic functionality
- **Duration**: ~2 seconds
- **Output**: Color-coded status display (Green/Yellow/Red)
- **Actions**: Recommends appropriate calibration level if needed

### 3. Runtime Calibration Management
- **When**: During normal operation
- **What**: Adaptive calibration suggestions based on:
  - Temperature changes (>10°C from calibration)
  - Sensor drift detection
  - Data quality metrics
- **Options**:
  - Quick Calibration (~30 seconds): Basic bias correction
  - Full Recalibration (~5 minutes): Complete sensor calibration
  - Emergency Calibration: Failsafe mode for critical errors
- **User Control**: Serial commands for manual calibration management

### Calibration Commands
Available through serial interface:
- `cal status` - Display current calibration status
- `cal quick` - Perform quick calibration
- `cal robust` - Perform full robust calibration
- `cal verify` - Run verification routine
- `cal bypass` - Skip calibration (not recommended)

### Quality Thresholds
Default thresholds (configurable in `calibration_manager.h`):
- **Good**: <2% accel deviation, <0.5 dps gyro drift, <3% mag variation
- **Marginal**: 2-5% accel, 0.5-2.0 dps gyro, 3-8% mag
- **Poor**: Above marginal thresholds

### Auto-Calibration
- Automatically detects when device is stationary
- Performs quick calibration if needed
- Can be disabled via serial command: `cal auto off`

### Troubleshooting
If calibration fails:
1. Ensure device is stationary during calibration
2. Check sensor connections and power
3. Try emergency calibration if needed
4. Consult serial output for detailed error messages

For best results:
- Perform robust calibration in a magnetically clean environment
- Allow device temperature to stabilize before calibration
- Follow on-screen instructions for sensor positioning

## IMU Calibration System - Three-Tier Startup Process

This project implements a comprehensive three-tier calibration and verification system for the IMU sensors (Accelerometer, Gyroscope, Magnetometer, and Temperature sensor). The system ensures optimal sensor accuracy while providing user-friendly fallback options.

## Overview

The calibration system operates on three levels:

1. **One-Time Robust Calibration** - Comprehensive calibration saved to EEPROM
2. **Startup Verification** - Automatic health check of existing calibration
3. **Adaptive Response** - User-guided actions based on calibration status

## Startup Process Flow

### 1. System Initialization
```
Power On → IMU Init → Calibration Manager Init → Load EEPROM Data
```

### 2. Startup Verification
The system automatically performs a health check of all sensors:

- **Accelerometer**: Checks if magnitude is close to 1g when stationary
- **Gyroscope**: Verifies zero-rate output when stationary  
- **Magnetometer**: Validates field consistency and sphere fit
- **Temperature**: Monitors environmental changes since last calibration

### 3. Status Classification

| Status | Color | Description | Action Required |
|--------|-------|-------------|-----------------|
| **GOOD** | 🟢 Green | Calibration valid, <2% deviation | None - proceed normally |
| **MARGINAL** | 🟡 Yellow | Minor drift, 2-5% deviation | Quick calibration recommended |
| **POOR** | 🔴 Red | Significant drift, >5% deviation | Robust calibration required |
| **CORRUPTED** | 🔴 Red | Data integrity failure | Emergency calibration needed |
| **NONE** | 🔴 Red | No calibration data found | Initial calibration required |

### 4. User Decision Process

Based on the status, users are presented with options:

```
Choose an action:
0 - Proceed with current calibration
1 - Perform quick calibration  
2 - Perform robust calibration
3 - Bypass checks and proceed
```

## Calibration Methods

### Robust Calibration (One-Time Setup)
- **Accelerometer**: 6-position calibration with temperature compensation
- **Gyroscope**: Bias calibration with temperature compensation
- **Magnetometer**: Advanced ellipsoid fitting (hard & soft iron correction)
- **Duration**: 10-15 minutes
- **Accuracy**: Highest precision
- **Fallback**: Automatically downgrades to simpler methods if advanced methods fail

### Quick Calibration (Maintenance)
- **Accelerometer**: Simple bias-only update
- **Gyroscope**: Zero-rate bias update
- **Magnetometer**: Hard-iron offset update
- **Duration**: 1-2 minutes
- **Accuracy**: Good for drift correction
- **Use Case**: Regular maintenance, environmental changes

### Emergency Calibration (Failsafe)
- **Accelerometer**: Minimal bias correction
- **Gyroscope**: Basic zero-rate calibration
- **Duration**: 30 seconds
- **Accuracy**: Basic functionality only
- **Use Case**: When all other methods fail

## Environmental Awareness

The system monitors environmental conditions:

- **Temperature Tracking**: Detects >10°C changes since last calibration
- **Drift Detection**: Monitors sensor performance over time
- **Adaptive Thresholds**: Adjusts quality standards based on application needs

### Precision Levels
- **Relaxed (0)**: Looser thresholds for general applications
- **Normal (1)**: Default balanced thresholds
- **High Precision (2)**: Strict thresholds for critical applications

## EEPROM Data Management

### Data Structure
```cpp
struct ExtendedCalibrationData {
    uint16_t magic;                    // Validation marker
    uint16_t version;                  // Data format version
    uint32_t timestamp;                // Calibration time
    float calibration_temp;            // Temperature during calibration
    CalibrationData cal_data;          // Actual calibration parameters
    CalibrationThresholds thresholds;  // Quality thresholds
    CalibrationHistoryEntry history[3]; // Last 3 status checks
    uint16_t checksum;                 // Data integrity check
};
```

### Data Integrity
- **Magic Number**: Validates data presence
- **Version Control**: Handles format upgrades
- **Checksum**: Detects corruption
- **Write Verification**: Confirms successful save

## Runtime Commands

Send these commands via Serial Monitor:

| Command | Description |
|---------|-------------|
| `cal` or `calibrate` | Show calibration menu |
| `status` | Display current calibration status |
| `quick` | Perform quick calibration |
| `robust` | Perform robust calibration |
| `emergency` | Perform emergency calibration |
| `precision` | Set precision level (0-2) |
| `help` | Show command help |

## Integration in Your Code

### Setup Function
```cpp
#include "startup_calibration.h"

void setup() {
    Serial.begin(115200);
    
    // Initialize IMU hardware first
    initIMU();
    
    // Perform startup calibration workflow
    if (!performStartupCalibrationWorkflow()) {
        Serial.println("Critical calibration failure - check system");
        // Handle critical failure as needed
    }
    
    // Continue with normal setup...
}
```

### Main Loop
```cpp
void loop() {
    // Handle serial commands
    if (Serial.available()) {
        String command = Serial.readString();
        if (!handleCalibrationCommand(command)) {
            // Handle other commands...
        }
    }
    
    // Optional: Check for runtime calibration needs
    if (checkRuntimeCalibrationNeeded()) {
        // Notify user or perform auto-calibration
    }
    
    // Optional: Auto-calibration when device is stable
    performAutoCalibration();
    
    // Your main application code...
}
```

## Startup Sequence Example

```
========================================
    IMU CALIBRATION STARTUP WORKFLOW    
========================================

=== Initializing Calibration Manager ===
✓ Existing calibration data loaded
=== Calibration Manager Ready ===

=== Startup Calibration Verification ===
⚠ Significant environmental change detected

=== Calibration Status Report ===
Overall Status: MARGINAL
Recommendation: Quick calibration recommended
Last Calibration: 3600 seconds ago
Calibration Temp: 22.5°C

=== Calibration Decision ===
Choose an action:
0 - Proceed with current calibration
1 - Perform quick calibration
2 - Perform robust calibration
3 - Bypass checks and proceed
Enter choice (0-3): 1

=== Quick Calibration Update ===
Performing bias-only calibration update...
Keep device stationary during calibration.
✓ Quick calibration completed!

✓ Quick calibration completed successfully
```

## Troubleshooting

### Common Issues

1. **"No valid calibration data found"**
   - First-time setup - perform robust calibration
   - EEPROM corruption - perform emergency calibration

2. **"EEPROM verification failed"**
   - Hardware issue or power loss during save
   - Try robust calibration again

3. **"Calibration drift detected"**
   - Normal over time - perform quick calibration
   - Significant environmental change - consider robust calibration

4. **"Emergency calibration also failed"**
   - Hardware malfunction
   - Check IMU connections and power supply

### Best Practices

1. **Initial Setup**: Always perform robust calibration first
2. **Regular Maintenance**: Run quick calibration weekly or after environmental changes
3. **Critical Applications**: Use high precision mode and monitor status regularly
4. **Backup**: Save calibration parameters externally for critical systems

## File Structure

```
src/
├── calibration_manager.h/cpp     # Core calibration management
├── startup_calibration.h/cpp     # Startup workflow integration
├── accel_calibration.h/cpp       # Accelerometer calibration methods
├── gyro_calibration.h/cpp        # Gyroscope calibration methods
├── mag_calibration.h/cpp         # Magnetometer calibration methods
├── temp_sensor.h/cpp             # Temperature sensor management
├── imu_validation.h/cpp          # Validation and verification routines
└── calibration_common.h/cpp      # Shared structures and utilities
```

## Advanced Features

### Auto-Calibration
- Automatically detects when device is stationary
- Performs quick calibration if needed
- Can be disabled via serial command: `cal auto off`

### Runtime Monitoring
- Periodic health checks every 5 minutes
- Environmental change detection
- Drift monitoring and alerts

### Adaptive Thresholds
- Configurable quality standards
- Application-specific precision requirements
- Dynamic adjustment based on conditions

This comprehensive system ensures your IMU maintains optimal accuracy while providing user-friendly operation and robust fallback options.
