#include "CalibrationEngine.h"
#include "../Madgwick.h"
#include <EEPROM.h>

// V2 EEPROM Storage Address - isolated from main firmware (which uses addr 0)
#define V2_EEPROM_ADDR 512

CalStorage CalibrationEngine::calData;

void CalibrationEngine::begin() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n\n=== TAG 3D CALIBRATION TOOL V2 ===");
    
    if (!SensorDriverV2::begin()) {
        Serial.println("!!! SENSOR INIT FAILED !!!");
        Serial.println("Check connections.");
        while(1) delay(1000);
    }
    
    // EEPROM.begin(512); // DISABLED to prevent conflict with V1 Calibration (uses 1024)
    
    // Initialize calData before loading
    calData.calibration_count = 1;
    calData.valid = false;
    
    loadCalibration();
    showMenu();
}

void CalibrationEngine::loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') return;
        
        switch (c) {
            case '1': runQuickCal(); break;
            case '2': runRobustCal(); break;
            case '3': runBestCal(); break;
            case '4': runValidation(); break;
            case '5': runExtendedValidation(); break;
            case '6': printStatus(); showMenu(); break;
            case '7': identifyAxes(); break;
            case 's': saveCalibration(); showMenu(); break;
            case 'r': loadCalibration(); showMenu(); break;
            case 'h': showMenu(); break;
            default: Serial.println("Unknown command."); showMenu(); break;
        }
    }
}

void CalibrationEngine::showMenu() {
    Serial.println("\n--- MAIN MENU ---");
    Serial.println("1) Quick Cal (Gyro+Mag)");
    Serial.println("2) Robust Cal (Guided 6-Point)");
    Serial.println("3) Best Cal (Extended Sampling)");
    Serial.println("4) Validate");
    Serial.println("5) Extended Validation (with Madgwick)");
    Serial.println("6) Status");
    Serial.println("7) Help: Identify Axes");
    Serial.println("s) Save");
    Serial.println("r) Load");
    Serial.print("> ");
}

void CalibrationEngine::identifyAxes() {
    Serial.println("\n--- AXIS IDENTIFIER ---");
    Serial.println("Rotate the board. Press KEY to exit.");
    delay(1000);
    
    while (!Serial.available()) {
        RawSensorData d = SensorDriverV2::readAll();
        if (d.valid) {
            Serial.printf("Ax: %5.2f  Ay: %5.2f  Az: %5.2f  -> ", d.ax, d.ay, d.az);
            
            if (d.az > 0.8) Serial.print("[Z-UP]");
            else if (d.az < -0.8) Serial.print("[Z-DOWN]");
            
            if (d.ay > 0.8) Serial.print("[Y-UP]");
            else if (d.ay < -0.8) Serial.print("[Y-DOWN]");
            
            if (d.ax > 0.8) Serial.print("[X-UP]");
            else if (d.ax < -0.8) Serial.print("[X-DOWN]");
            
            Serial.println();
        }
        delay(200);
    }
    Serial.read();
    showMenu();
}

void CalibrationEngine::performGyroCal() {
    Serial.println("\n[GYRO CALIBRATION]");
    Serial.println("Place device flat and still.");
    Serial.println("Press ENTER...");
    while(Serial.available()) Serial.read();
    while(!Serial.available()); Serial.read();
    
    Serial.print("Sampling...");
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int samples = 0;
    
    for (int i=0; i<200; i++) {
        RawSensorData d = SensorDriverV2::readAll();
        if (d.valid) {
            gx_sum += d.gx;
            gy_sum += d.gy;
            gz_sum += d.gz;
            samples++;
        }
        delay(5);
        if(i % 20 == 0) Serial.print(".");
    }
    
    if (samples > 0) {
        calData.gyro_bias[0] = gx_sum / samples;
        calData.gyro_bias[1] = gy_sum / samples;
        calData.gyro_bias[2] = gz_sum / samples;
        Serial.println(" Done.");
    } else {
        Serial.println(" Failed.");
    }
}

void CalibrationEngine::performMagCal() {
    Serial.println("\n[MAG CALIBRATION]");
    Serial.println("Rotate device in Figure-8 (20s).");
    Serial.println("Press ENTER...");
    while(Serial.available()) Serial.read(); 
    while(!Serial.available()); Serial.read(); 
    
    float min_m[3] = {10000, 10000, 10000};
    float max_m[3] = {-10000, -10000, -10000};
    
    // Track coverage (8 octants)
    bool octants[8] = {false};
    
    Serial.println("GO!");
    uint32_t start = millis();
    int samples = 0;
    uint32_t lastCoverageUpdate = 0;
    
    while (millis() - start < 20000) {
        float x, y, z;
        if (SensorDriverV2::readMag(x, y, z)) {
            for(int i=0; i<3; i++) {
                float val = (i==0)?x:(i==1)?y:z;
                if (val < min_m[i]) min_m[i] = val;
                if (val > max_m[i]) max_m[i] = val;
            }
            
            // Track octant (XYZ sign combinations)
            int octant = 0;
            if (x > 0) octant |= 0x04;
            if (y > 0) octant |= 0x02;
            if (z > 0) octant |= 0x01;
            octants[octant] = true;
            
            samples++;
            if (samples % 10 == 0) Serial.print("+");
        }
        
        delay(20);
        
        // Update coverage display every 2 seconds
        if (millis() - lastCoverageUpdate > 2000) {
            int coverage = 0;
            for(int i=0; i<8; i++) if(octants[i]) coverage++;
            int pct = (coverage * 100) / 8;
            Serial.printf(" [%d%%] ", pct);
            lastCoverageUpdate = millis();
        }
    }
    
    // Final coverage check
    int coverage = 0;
    for(int i=0; i<8; i++) if(octants[i]) coverage++;
    int pct = (coverage * 100) / 8;
    Serial.printf("\nFinal Coverage: %d%%\n", pct);
    
    if (pct < 75) {
        Serial.println("WARNING: Low coverage! Consider repeating for better accuracy.");
    }
    
    // Calculate Offsets & Scale
    for(int i=0; i<3; i++) {
        calData.mag_offset[i] = (max_m[i] + min_m[i]) / 2.0f;
    }
    
    float range[3];
    float range_sum = 0;
    for(int i=0; i<3; i++) {
        range[i] = (max_m[i] - min_m[i]) / 2.0f;
        range_sum += range[i];
    }
    float avg_rad = range_sum / 3.0f;
    
    for(int i=0; i<3; i++) {
        if (range[i] < 1.0f) calData.mag_scale[i] = 1.0f;
        else calData.mag_scale[i] = avg_rad / range[i];
    }
    
    calData.valid = true;
    Serial.println(" Done.");
}

void CalibrationEngine::runQuickCal() {
    performGyroCal();
    performMagCal();
    Serial.println("\nQuick Cal Complete.");
    printStatus();
    showMenu();
}

void CalibrationEngine::runRobustCal() {
    Serial.println("\n=== ROBUST ACCEL CALIBRATION ===");
    
    struct AxisLimit { float min_val; float max_val; };
    AxisLimit accLimits[3];
    for(int i=0; i<3; i++) { accLimits[i].min_val = 100.0f; accLimits[i].max_val = -100.0f; }

    const char* instr[] = {
        "Z-UP (Flat)", "Z-DOWN", 
        "Y-UP", "Y-DOWN", 
        "X-UP", "X-DOWN"
    };
    
    // Target Axis indices: 2, 2, 1, 1, 0, 0
    int targetAxis[]  = {2, 2, 1, 1, 0, 0};
    // Target Signs:    +1, -1, +1, -1, +1, -1
    float targetSign[] = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};
    
    for (int step=0; step<6; step++) {
        Serial.printf("\n[Pos %d/6] Hold %s\n", step+1, instr[step]);
        waitForStableOrientation(targetAxis[step], targetSign[step], 0.8f);
        
        Serial.print("Sampling...");
        for(int i=0; i<50; i++) {
            RawSensorData d = SensorDriverV2::readAll();
            if (d.valid) {
                 if (d.ax < accLimits[0].min_val) accLimits[0].min_val = d.ax;
                 if (d.ax > accLimits[0].max_val) accLimits[0].max_val = d.ax;
                 if (d.ay < accLimits[1].min_val) accLimits[1].min_val = d.ay;
                 if (d.ay > accLimits[1].max_val) accLimits[1].max_val = d.ay;
                 if (d.az < accLimits[2].min_val) accLimits[2].min_val = d.az;
                 if (d.az > accLimits[2].max_val) accLimits[2].max_val = d.az;
            }
            delay(10);
        }
        Serial.println(" Done.");
    }
    
    for(int i=0; i<3; i++) {
        if (accLimits[i].max_val > accLimits[i].min_val) {
            calData.accel_bias[i] = (accLimits[i].max_val + accLimits[i].min_val) / 2.0f;
        } else {
            calData.accel_bias[i] = 0.0f;
        }
    }
    
    performGyroCal();
    performMagCal();
    
    Serial.println("\nRobust Cal Complete.");
    printStatus();
    showMenu();
}

void CalibrationEngine::waitForStableOrientation(int axis, float target, float threshold) {
    int stableCount = 0;
    while (stableCount < 10) { // 1 second stable
        RawSensorData d = SensorDriverV2::readAll();
        if (!d.valid) continue;
        
        float val = (axis==0)?d.ax:(axis==1)?d.ay:d.az;
        
        // Check if value matches target sign and magnitude
        bool match = false;
        if (target > 0 && val > threshold) match = true;
        if (target < 0 && val < -threshold) match = true;
        
        if (match) {
            stableCount++;
            Serial.print("*");
        } else {
            stableCount = 0;
            // Print live feedback
            Serial.printf("\r Waiting... Ax:%.2f Ay:%.2f Az:%.2f ", d.ax, d.ay, d.az);
        }
        delay(100);
    }
    Serial.println(" STABLE!");
}

void CalibrationEngine::runBestCal() {
    Serial.println("\n=== BEST CALIBRATION (Extended Precision) ===");
    Serial.println("Same 6 positions as Robust, but with:");
    Serial.println("  - 3x more samples per position (300 vs 50)");
    Serial.println("  - Extended mag capture (60s vs 20s)");
    Serial.println("  - Statistical validation");
    
    struct AxisLimit { float min_val; float max_val; };
    AxisLimit accLimits[3];
    for(int i=0; i<3; i++) { accLimits[i].min_val = 100.0f; accLimits[i].max_val = -100.0f; }

    const char* instr[] = {
        "Z-UP (Flat on table)", "Z-DOWN (Upside down)", 
        "Y-UP (Stand on edge)", "Y-DOWN (Opposite edge)", 
        "X-UP (Stand on side)", "X-DOWN (Opposite side)"
    };
    
    int targetAxis[]  = {2, 2, 1, 1, 0, 0};
    float targetSign[] = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};
    
    Serial.println("\n[PART 1: Extended 6-Point Accelerometer]");
    for (int step=0; step<6; step++) {
        Serial.printf("\n[Pos %d/6] %s\n", step+1, instr[step]);
        waitForStableOrientation(targetAxis[step], targetSign[step], 0.8f);
        
        Serial.print("Extended sampling (300 samples)...");
        
        // Collect 300 samples with statistical tracking
        float axis_sum[3] = {0, 0, 0};
        int valid_samples = 0;
        
        for(int i=0; i<300; i++) {
            RawSensorData d = SensorDriverV2::readAll();
            if (d.valid) {
                float vals[3] = {d.ax, d.ay, d.az};
                for(int j=0; j<3; j++) {
                    axis_sum[j] += vals[j];
                    if (vals[j] < accLimits[j].min_val) accLimits[j].min_val = vals[j];
                    if (vals[j] > accLimits[j].max_val) accLimits[j].max_val = vals[j];
                }
                valid_samples++;
            }
            delay(10);
            if(i % 50 == 0) Serial.print(".");
        }
        
        // Show average for this position
        Serial.printf(" Done. Avg: %.2f %.2f %.2f\n", 
                     axis_sum[0]/valid_samples, 
                     axis_sum[1]/valid_samples, 
                     axis_sum[2]/valid_samples);
    }
    
    // Calculate Bias
    Serial.println("\n[Analysis]");
    for(int i=0; i<3; i++) {
        if (accLimits[i].max_val > accLimits[i].min_val) {
            calData.accel_bias[i] = (accLimits[i].max_val + accLimits[i].min_val) / 2.0f;
            float range = (accLimits[i].max_val - accLimits[i].min_val) / 2.0f;
            Serial.printf("Axis %d: Range=%.3fg  Bias=%.3fg\n", i, range, calData.accel_bias[i]);
            
            // Validate: Range should be ~1.0g
            if (range < 0.9f || range > 1.1f) {
                Serial.printf("  WARNING: Expected ~1.0g, got %.3fg\n", range);
            }
        } else {
            calData.accel_bias[i] = 0.0f;
        }
    }
    
    performGyroCal();
    
    // Extended Magnetometer
    Serial.println("\n[PART 3: Extended Magnetometer (60s)]");
    Serial.println("Slowly rotate the device to cover ALL orientations.");
    Serial.println("Try to rotate smoothly in all 3 axes.");
    Serial.println("Press ENTER...");
    while(Serial.available()) Serial.read(); 
    while(!Serial.available()); Serial.read(); 
    
    float min_m[3] = {10000, 10000, 10000};
    float max_m[3] = {-10000, -10000, -10000};
    
    Serial.println("GO! (60 seconds)");
    uint32_t start = millis();
    int samples = 0;
    while (millis() - start < 60000) {
        float x, y, z;
        if (SensorDriverV2::readMag(x, y, z)) {
            for(int i=0; i<3; i++) {
                float val = (i==0)?x:(i==1)?y:z;
                if (val < min_m[i]) min_m[i] = val;
                if (val > max_m[i]) max_m[i] = val;
            }
            samples++;
            if (samples % 20 == 0) Serial.print("+");
        }
        delay(20);
        if ((millis() - start) % 5000 == 0) {
            Serial.printf(" %ds ", (millis()-start)/1000);
        }
    }
    
    Serial.printf("\nCaptured %d samples.\n", samples);
    
    // Calculate Offsets & Scale
    for(int i=0; i<3; i++) {
        calData.mag_offset[i] = (max_m[i] + min_m[i]) / 2.0f;
    }
    
    float range[3];
    float range_sum = 0;
    for(int i=0; i<3; i++) {
        range[i] = (max_m[i] - min_m[i]) / 2.0f;
        range_sum += range[i];
        Serial.printf("Mag Axis %d: Range=%.1f uT\n", i, range[i]*2);
    }
    float avg_rad = range_sum / 3.0f;
    
    for(int i=0; i<3; i++) {
        if (range[i] < 1.0f) calData.mag_scale[i] = 1.0f;
        else calData.mag_scale[i] = avg_rad / range[i];
    }
    
    calData.valid = true;
    Serial.println("\nBest Cal Complete.");
    printStatus();
    showMenu();
}

void CalibrationEngine::runValidation() {
    Serial.println("\n========================================");
    Serial.println("       VALIDATION STREAM        ");
    Serial.println("========================================");
    Serial.println("How to validate:");
    Serial.println("  Accel: Magnitude should = 1.00g when still");
    Serial.println("  Gyro:  Should be near 0 dps when still");
    Serial.println("  Mag:   Magnitude should stay constant while rotating");
    Serial.println("\nPress ANY KEY to stop...\n");
    delay(2000); 
    
    int sample_count = 0;
    while (!Serial.available()) {
        RawSensorData d = SensorDriverV2::readAll();
        if (d.valid) {
            float gx = d.gx - calData.gyro_bias[0];
            float gy = d.gy - calData.gyro_bias[1];
            float gz = d.gz - calData.gyro_bias[2];
            
            float mx = (d.mx - calData.mag_offset[0]) * calData.mag_scale[0];
            float my = (d.my - calData.mag_offset[1]) * calData.mag_scale[1];
            float mz = (d.mz - calData.mag_offset[2]) * calData.mag_scale[2];
            
            float ax = d.ax - calData.accel_bias[0];
            float ay = d.ay - calData.accel_bias[1];
            float az = d.az - calData.accel_bias[2];
            
            // Calculate magnitudes
            float a_mag = sqrt(ax*ax + ay*ay + az*az);
            float m_mag = sqrt(mx*mx + my*my + mz*mz);
            
            Serial.printf("A:%5.2f %5.2f %5.2f |%4.2fg| G:%5.1f %5.1f %5.1f | M:%5.1f %5.1f %5.1f |%4.1fuT|\n", 
                          ax, ay, az, a_mag, gx, gy, gz, mx, my, mz, m_mag);
            sample_count++;
        }
        delay(100);
    }
    Serial.read(); 
    Serial.printf("\nCaptured %d samples.\n", sample_count);
    showMenu();
}
void CalibrationEngine::runExtendedValidation() {
    Serial.println("\n========================================");
    Serial.println("   EXTENDED VALIDATION (COMPARISON)");
    Serial.println("========================================");
    Serial.println("Comparing:");
    Serial.println("  RAW vs CALIBRATED sensor data");
    Serial.println("  ACCEL-ONLY tilt vs MADGWICK fusion");
    Serial.println("\nIMPORTANT: Madgwick filter needs ~3000 samples");
    Serial.println("to stabilize (~30 seconds at 100Hz).");
    Serial.println("\nPress ANY KEY to stop...\n");
    delay(2000);
    
    // Initialize Madgwick filter
    Madgwick filter;
    filter.begin(100.0f);  // 100 Hz sample rate
    filter.setBeta(0.1f);
    
    unsigned long lastUpdate = micros();
    int sample_count = 0;
    bool stabilized = false;
    
    Serial.println(">>> Stabilizing filter... Please wait <<<");
    
    while (!Serial.available()) {
        RawSensorData d = SensorDriverV2::readAll();
        float mag_x, mag_y, mag_z;
        bool mag_valid = SensorDriverV2::readMag(mag_x, mag_y, mag_z);
        
        if (d.valid) {
            // Calculate delta time
            unsigned long now = micros();
            float dt = (now - lastUpdate) / 1000000.0f;
            lastUpdate = now;
            
            // 1. RAW data
            float raw_ax = d.ax, raw_ay = d.ay, raw_az = d.az;
            float raw_gx = d.gx, raw_gy = d.gy, raw_gz = d.gz;
            
            // 2. CALIBRATED data
            float cal_ax = d.ax - calData.accel_bias[0];
            float cal_ay = d.ay - calData.accel_bias[1];
            float cal_az = d.az - calData.accel_bias[2];
            
            float cal_gx = d.gx - calData.gyro_bias[0];
            float cal_gy = d.gy - calData.gyro_bias[1];
            float cal_gz = d.gz - calData.gyro_bias[2];
            
            float cal_mx = (mag_x - calData.mag_offset[0]) * calData.mag_scale[0];
            float cal_my = (mag_y - calData.mag_offset[1]) * calData.mag_scale[1];
            float cal_mz = (mag_z - calData.mag_offset[2]) * calData.mag_scale[2];
            
            // 3. ACCEL-ONLY tilt (simple trigonometry - noisy)
            float accel_roll = atan2(cal_ay, cal_az) * 57.2958f;  // rad to deg
            float accel_pitch = atan2(-cal_ax, sqrt(cal_ay*cal_ay + cal_az*cal_az)) * 57.2958f;
            
            // 4. MADGWICK fusion (gyro in rad/s)
            float gx_rad = cal_gx * 0.0174533f;
            float gy_rad = cal_gy * 0.0174533f;
            float gz_rad = cal_gz * 0.0174533f;
            
            filter.update(gx_rad, gy_rad, gz_rad, cal_ax, cal_ay, cal_az, cal_mx, cal_my, cal_mz);
            
            float yaw, pitch, roll;
            filter.getEulerAngles(yaw, pitch, roll);
            
            // Convert to degrees
            roll *= 57.2958f;
            pitch *= 57.2958f;
            yaw *= 57.2958f;
            
            // Convert yaw from -180/+180 to 0-360° (like phone compass)
            if (yaw < 0) yaw += 360.0f;
            
            // Show stabilization progress for first 3000 samples
            if (!stabilized && sample_count < 3000) {
                if (sample_count % 500 == 0) {
                    Serial.printf("Stabilizing... %d/3000 samples (%.0f%%)\n", 
                                 sample_count, (sample_count / 30.0f));
                }
            } else if (!stabilized) {
                stabilized = true;
                Serial.println("\n>>> Filter stabilized! Now showing comparison <<<\n");
            }
            
            // Only display comparison after stabilization
            if (stabilized) {
                // Display comparison every 500ms (reduce spam)
                static uint32_t lastPrint = 0;
                if (millis() - lastPrint > 500) {
                    Serial.println("--- Sample ---");
                    Serial.printf("RAW:  A:%5.2f %5.2f %5.2f | G:%5.1f %5.1f %5.1f\n",
                                 raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz);
                    Serial.printf("CAL:  A:%5.2f %5.2f %5.2f | G:%5.1f %5.1f %5.1f\n",
                                 cal_ax, cal_ay, cal_az, cal_gx, cal_gy, cal_gz);
                    Serial.printf("ACCEL-ONLY: Roll:%6.1f° Pitch:%6.1f°\n",
                                 accel_roll, accel_pitch);
                    Serial.printf("MADGWICK:   Roll:%6.1f° Pitch:%6.1f° Yaw:%6.1f°\n",
                                 roll, pitch, yaw);
                    Serial.println("            (Yaw = MAGNETIC heading, not true north)");
                    Serial.println();
                    lastPrint = millis();
                }
            }
            
            sample_count++;
        }
        delay(10);  // 100 Hz
    }
    
    Serial.read();
    Serial.printf("\nCaptured %d samples.\n", sample_count);
    showMenu();
}

void CalibrationEngine::printStatus() {
    Serial.println("\n========================================");
    Serial.println("       CALIBRATION QUALITY REPORT       ");
    Serial.println("========================================");
    
    // 1. Storage Status & Session Info
    Serial.print("Storage: ");
    if (calData.valid) {
        Serial.printf("[ OK ] Calibration Session #%d\n", calData.calibration_count);
    } else {
        Serial.println("[FAIL] Not Saved / Invalid");
    }

    // 2. Accelerometer Health
    Serial.println("\n[ ACCELEROMETER ]");
    Serial.printf("Biases (g):  X:%5.2f  Y:%5.2f  Z:%5.2f\n", 
                  calData.accel_bias[0], calData.accel_bias[1], calData.accel_bias[2]);
                  
    // Check if biases are reasonable
    bool accGood = true;
    for(int i=0; i<3; i++) if(abs(calData.accel_bias[i]) > 0.5f) accGood = false;
    
    if (accGood) Serial.println("Status: EXCELLENT (Offsets are low)");
    else Serial.println("Status: WARNING (High offsets detected)");

    // 3. Gyroscope Health
    Serial.println("\n[ GYROSCOPE ]");
    Serial.printf("Biases (dps): X:%5.2f  Y:%5.2f  Z:%5.2f\n", 
                  calData.gyro_bias[0], calData.gyro_bias[1], calData.gyro_bias[2]);
    Serial.println("Status: OK (Stationary bias captured)");

    // 4. Magnetometer Health
    Serial.println("\n[ MAGNETOMETER ]");
    Serial.printf("Offsets (uT): X:%5.2f  Y:%5.2f  Z:%5.2f\n", 
                  calData.mag_offset[0], calData.mag_offset[1], calData.mag_offset[2]);
    Serial.printf("Scales:       X:%5.2f  Y:%5.2f  Z:%5.2f\n", 
                  calData.mag_scale[0], calData.mag_scale[1], calData.mag_scale[2]);

    bool magDataFound = false;
    if (abs(calData.mag_offset[0]) > 0.1 || abs(calData.mag_offset[1]) > 0.1) magDataFound = true;
    
    if (magDataFound) {
        Serial.println("Status: GOOD (Calibration data captured)");
    } else {
        Serial.println("Status: CRITICAL FAILURE (No data captured!)");
        Serial.println("   -> Try moving device more during Mag Cal.");
    }
    
    Serial.println("========================================\n");
}

void CalibrationEngine::saveCalibration() {
    Serial.println("\n[SAVING]");
    Serial.println("Writing calibration data to EEPROM...");
    calData.magic = 0xCAFE;
    calData.version = 2;
    calData.calibration_count++; // Increment session counter
    EEPROM.put(V2_EEPROM_ADDR, calData);  // Use isolated address
    bool success = EEPROM.commit();
    
    if (success) {
        Serial.printf("SUCCESS: Calibration #%d saved to non-volatile memory.\n", calData.calibration_count);
        Serial.println("These values will persist across power cycles.");
    } else {
        Serial.println("ERROR: Failed to commit to EEPROM!");
    }
}

void CalibrationEngine::loadCalibration() {
    Serial.println("\n[LOADING]");
    Serial.println("Reading from EEPROM...");
    EEPROM.get(V2_EEPROM_ADDR, calData);  // Use isolated address
    
    if (calData.magic != 0xCAFE) {
        Serial.println("RESULT: No valid calibration found (First run).");
        Serial.println("Action: Loaded default zero-offsets.");
        calData.valid = false;
        calData.calibration_count = 1;
        for(int i=0; i<3; i++) {
            calData.mag_scale[i] = 1.0f;
            calData.mag_offset[i] = 0;
            calData.gyro_bias[i] = 0;
            calData.accel_bias[i] = 0;
        }
    } else {
        // Fix corrupted counter (can happen from previous firmware versions)
        if (calData.calibration_count < 0 || calData.calibration_count > 10000) {
            Serial.println("WARNING: Corrupted session counter detected. Resetting to 1.");
            calData.calibration_count = 1;
        }
        
        Serial.printf("RESULT: Calibration #%d Successfully Loaded.\n", calData.calibration_count);
        Serial.printf("Version: %d\n", calData.version);
        calData.valid = true;
    }
}
