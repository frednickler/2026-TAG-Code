#include "dmp_imu.h"
#include <Wire.h>
#include "calibration_manager.h" // Required for calibration override

// Global library object
ArduinoICM20948 myICM;

// EXTERNAL VARIABLE OVERRIDE
// The library defines "uint8_t I2C_Address = 0x69;" in Arduino-ICM20948.cpp
// We must override this to 0x68 for the SparkFun breakout (AD0=0)
extern uint8_t I2C_Address;

// Data storage
static float latestQw = 1.0f, latestQx = 0.0f, latestQy = 0.0f, latestQz = 0.0f;
static float latestAx = 0.0f, latestAy = 0.0f, latestAz = 0.0f;
static float latestGx = 0.0f, latestGy = 0.0f, latestGz = 0.0f;
static float latestMx = 0.0f, latestMy = 0.0f, latestMz = 0.0f;
static float latestHeading = 0.0f;
static bool dmpInitialized = false;
static bool freshData = false;

bool initDMP() {
    Serial.println("\n🔧 Initializing DMP (using isouriadakis library)...");

    // Define Settings
    ArduinoICM20948Settings icmSettings = {
        .i2c_speed = 400000,                // 400kHz I2C
        .is_SPI = false,                    // Use I2C
        .cs_pin = 10,                       // Not used for I2C
        .spi_speed = 7000000,               // Not used
        .mode = 1,                          // HIGH PERFORMANCE MODE: 0 = low power, 1 = high performance
        .enable_gyroscope = true,
        .enable_accelerometer = true,
        .enable_magnetometer = true,
        .enable_gravity = false,            // Not needed if we have Quat9
        .enable_linearAcceleration = false, // Not needed
        .enable_quaternion6 = false,        // Disable Quat6 (we want 9-axis)
        .enable_quaternion9 = true,         // Enable Quat9 (9-axis with Heading)
        .enable_har = false,
        .enable_steps = false,
        .gyroscope_frequency = 50,          // 50Hz 
        .accelerometer_frequency = 50,
        .magnetometer_frequency = 50,       
        .gravity_frequency = 50,            // MUST BE NON-ZERO to avoid div/0 crash
        .linearAcceleration_frequency = 50, // MUST BE NON-ZERO
        .quaternion6_frequency = 50,
        .quaternion9_frequency = 50,        // 50Hz Quat9 output
        .har_frequency = 50,                // MUST BE NON-ZERO
        .steps_frequency = 50               // MUST BE NON-ZERO
    };

    // OVERRIDE I2C ADDRESS
    I2C_Address = 0x68;
    // Serial.printf("[DMP DEBUG] Forcing global I2C Address to 0x%02X\n", I2C_Address);
    
    // Check if Wire is actually working before we start
    Wire.beginTransmission(0x68);
    byte error = Wire.endTransmission();
    if (error != 0) {
        Serial.printf("[ERROR] I2C Device check at 0x68 failed! Error: %d\n", error);
        return false;
    }

    // Initialize the sensor with settings
    myICM.init(icmSettings);
    
    // The library doesn't return a status code for init, so we assume success
    // if the I2C bus is working.
    Serial.println("✅ DMP Initialization: SUCCESS");
    dmpInitialized = true;
    return true;
}

bool updateDMP() {
    if (!dmpInitialized) return false;

    // Library task function must be called frequently
    myICM.task();

    // Check availability
    bool q9 = myICM.quat9DataIsReady();
    
    // Check for Quaternion 9-axis data
    if (q9) {
        myICM.readQuat9Data(&latestQw, &latestQx, &latestQy, &latestQz);
        
        // Also read sensor data if available (usually is)
        if (myICM.accelDataIsReady()) myICM.readAccelData(&latestAx, &latestAy, &latestAz);
        if (myICM.gyroDataIsReady()) myICM.readGyroData(&latestGx, &latestGy, &latestGz);
        if (myICM.magDataIsReady()) myICM.readMagData(&latestMx, &latestMy, &latestMz);
        
        // ===================================================================
        // CRITICAL FIX: Use library's built-in Euler angle extraction
        // The library already converts quaternion to Euler internally with
        // the correct frame convention. Manual conversion was causing
        // ±90°/±180° errors due to frame mismatch.
        // ===================================================================
        float roll, pitch, yaw;
        if (myICM.euler9DataIsReady()) {
            myICM.readEuler9Data(&roll, &pitch, &yaw);
            
            // Convert to 0-360 range
            if (yaw < 0) yaw += 360.0f;
            
            latestHeading = yaw;
            freshData = true;
            return true;
        } else {
            // Fallback: If Euler9 not ready but Quat9 is, calculate manually
            // (This shouldn't normally happen, but provides robustness)
            float q0 = latestQw;
            float q1 = latestQx;
            float q2 = latestQy;
            float q3 = latestQz;
            
            double siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
            double cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
            double yaw_calc = atan2(siny_cosp, cosy_cosp) * 180.0 / PI;
            
            if (yaw_calc < 0) yaw_calc += 360.0;
            latestHeading = (float)yaw_calc;
            
            freshData = true;
            return true;
        }
    }

    freshData = false;
    return false;
}

bool readDMPQuaternion(float &qw, float &qx, float &qy, float &qz) {
    if (!dmpInitialized) return false;
    qw = latestQw;
    qx = latestQx;
    qy = latestQy;
    qz = latestQz;
    return true;
}

bool readDMPCalibratedSensors(float &ax, float &ay, float &az, 
                              float &gx, float &gy, float &gz, 
                              float &mx, float &my, float &mz) {
    if (!dmpInitialized) return false;
    ax = latestAx; ay = latestAy; az = latestAz;
    gx = latestGx; gy = latestGy; gz = latestGz;
    mx = latestMx; my = latestMy; mz = latestMz;
    return true;
}

float getDMPHeading() {
    return latestHeading;
}

float getHybridHeading() {
    if (!dmpInitialized) return 0.0f;
    
    // Get DMP's stable pitch and roll (in degrees)
    float roll_deg, pitch_deg, yaw_unused;
    if (!myICM.euler9DataIsReady()) {
        return latestHeading; // Fallback to last known heading
    }
    myICM.readEuler9Data(&roll_deg, &pitch_deg, &yaw_unused);
    
    // Get raw magnetometer data
    float mx, my, mz;
    if (!myICM.magDataIsReady()) {
        return latestHeading; // Fallback to last known heading
    }
    myICM.readMagData(&mx, &my, &mz);
    
    // No manual axis inversions needed!
    // Since the sensor is upside-down, Roll is ~180°.
    // The tilt comp formula: my_comp = ... + my * cos(roll) ...
    // cos(180) is -1, so this NATURALLY inverts Y, which matches our V7 findings.
    
    // --- CALIBRATION STRATEGY ---
    float offset_x, offset_y, offset_z;
    float scale_x, scale_y; // Z scale usually 1.0 but could be supported

    // 1. Try to load dynamic calibration from CalibrationManager
    // Requires "calibration_manager.h" included at top
    if (g_extended_cal_data.cal_data.magCalibrated) {
        offset_x = g_extended_cal_data.cal_data.magBias[0];
        offset_y = g_extended_cal_data.cal_data.magBias[1];
        offset_z = g_extended_cal_data.cal_data.magBias[2];
        scale_x = g_extended_cal_data.cal_data.magScale[0];
        scale_y = g_extended_cal_data.cal_data.magScale[1]; 
        // scale_z = g_extended_cal_data.cal_data.magScale[2];
    } else {
        // 2. Fallback Values (Derived empirically from "Multiverse" Test):
        // Hard Iron: Offset X = 4.0, Offset Y = -5.0, Offset Z = -32.0
        // Soft Iron: Range X = 52, Range Y = 39. Ratio = 1.33.
        
        offset_x = 4.0f;
        offset_y = -5.0f;
        offset_z = -32.0f;
        scale_x = 1.0f;
        scale_y = 1.33f; 
    }
    
    // Apply Calibration
    float mx_cal = (mx - offset_x) * scale_x;
    float my_cal = (my - offset_y) * scale_y;
    float mz_cal = (mz - offset_z); // Z scale assumed 1.0 for now unless we add scale_z

    // Convert pitch and roll to radians for trig functions
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float roll_rad = roll_deg * DEG_TO_RAD;
    
    // Tilt-compensated magnetometer heading calculation
    // Using H1 MAPPING (+X, +Y) as verified by "N=1, S=178" result.
    float mx_comp = mx_cal * cos(pitch_rad) + mz_cal * sin(pitch_rad);
    float my_comp = mx_cal * sin(roll_rad) * sin(pitch_rad) + 
                    my_cal * cos(roll_rad) - 
                    mz_cal * sin(roll_rad) * cos(pitch_rad);
    
    // Calculate heading
    float heading = atan2(my_comp, mx_comp) * RAD_TO_DEG;
    
    // Normalize to 0-360 range
    if (heading < 0) heading += 360.0f;
    
    return heading;
}

bool isDMPInitialized() {
    return dmpInitialized;
}
