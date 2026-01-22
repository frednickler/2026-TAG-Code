#include "menu_handler.h"
#include "imu_config.h" // Fix readMag error
#include "config_store.h"
#include "gps_module.h"
#include "accel_config.h"
#include "gyro_config.h"
#include "mag_config.h"
#include "esp_now_handler.h"
#include "data_logger.h"
// #include "Madgwick.h"  // For Madgwick filter configuration (Obsolete with DMP)
#include "calibration_common.h"  // For CalibrationData
#include "mag_calibration.h"  // For saveCalibration, MAG_CAL_ELLIPSOID

// Menu State
enum MenuState {
    MENU_MAIN_CONFIG,
    MENU_GPS_ROOT,      // The "Advanced" 7-item menu
    MENU_GNSS_CORE,     // The specific GNSS params (Constellation, Rate, Model)
    MENU_COMM,
    MENU_RADIO_ROOT,    // Top-Level Radio Config
    MENU_DATALOG,       // New: Data Logging
    MENU_MADGWICK,      // New: Madgwick Filter Configuration
    MENU_DLPF,          // New: Sensor Bandwidth
    MENU_MAG_CONFIG,    // New: Magnetometer Configuration Submenu
    MENU_ACCURACY,
    MENU_POWER,
    MENU_DISPLAY,
    MENU_SYSTEM,
    MENU_STATUS,
    MENU_MAG_CAL_TOOL // New Post-Processing Tool
};
static MenuState currentMenu = MENU_MAIN_CONFIG;

// --- Forward Declarations ---
void handleGNSSCoreInput(char c);
void handleCommInput(char c);
void handleRadioInput(char c);
void handleDatalogInput(char c);
void handleAccuracyInput(char c);
void handlePowerInput(char c);
void handleDisplayInput(char c);
void handleSystemInput(char c);
void printStatusDashboard();
void handleSensorConfig(int sensorType); // 1=Accel, 2=Gyro
void handleMadgwickInput(char c);
void handleMagCalToolInput(char c); // Pro Calibration Tool
void handleMagConfigInput(char c); // Mag submenu handler
void inputMagCalibrationMatrix(); // Manual matrix input

// ... (Rest of file) ...

// [Garbage Removed]
// handleMagCalToolInput definition is below


// NEW HANDLER IMPLEMENTATION:
void handleMagCalToolInput(char c) {
    if (c == 'x') { 
        currentMenu = MENU_MAG_CONFIG; // Return to Mag Config submenu
        return; 
    }
    
    if (c == '1') {
         Serial.println("\n[MAG CAL] Starting raw data stream...");
         Serial.println("RAW_MAG_START");
         Serial.println("mx,my,mz");
         
         // Stream loop
         while(true) {
             float mx, my, mz;
             
             if (readMag(mx, my, mz)) {
                 Serial.printf("%.2f,%.2f,%.2f\n", mx, my, mz);
             }
             
             // Check for exit - flush buffer to prevent stuck commands
             if (Serial.available()) {
                 char cmd = Serial.read();
                 // Flush remaining buffer
                 while (Serial.available()) Serial.read();
                 if (cmd == 'x') break;
             }
             delay(20); // 50Hz
         }
         Serial.println("RAW_MAG_END");
         Serial.println("[MAG CAL] Stream stopped. Returning to menu...\n");
    }
}
// Helper
uint8_t parseHexChar(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

// -------------------------------------------
// MAIN ENTRY POINT
// -------------------------------------------
// ... (Existing handlers)

void handleDLPFInput(char c) {
    if (c == 'x') { currentMenu = MENU_MAIN_CONFIG; return; }
    
    // Config Values (From Datasheet)
    // 0 = 246Hz
    // 2 = 111Hz
    // 3 = 50Hz (Recommended for 100Hz loop)
    // 4 = 24Hz
    // 5 = 11Hz
    
    int configVal = -1;
    const char* freq = "";
    
    switch(c) {
        case '1': configVal = 0; freq = "240Hz"; break;
        case '2': configVal = 2; freq = "110Hz"; break;
        case '3': configVal = 3; freq = "50Hz (Recommended)"; break;
        case '4': configVal = 4; freq = "20Hz"; break;
        case '5': configVal = 5; freq = "10Hz"; break;
        default: break;
    }
    
    if (configVal != -1) {
        // Enable bit (1) + Config bits (val << 3)
        // Note: setAccelDLPF and setGyroDLPF expect the RAW CONFIG VALUE?
        // Let's check imu_config.cpp usage.
        // It says: writeRegister(0x01, 0x19); // 0001 1001. 3 << 3 | 1.
        // And setGyroDLPF(mode) logic:
        // new_val = ... | ((mode & 0x07) << 3) | 0x01;
        // So setGyroDLPF takes 0,1,2,3... NOT the raw hex.
        // Confirmed via Step 607 (Lines 95-128).
        // So we pass 'configVal' (0,2,3,4,5).
        
        #include "imu_config.h" // Ensure we can call these
        setAccelDLPF(configVal);
        setGyroDLPF(configVal);
        
        Serial.printf("\n[DLPF] Set Bandwidth to %s\n", freq);
        Serial.println("[DLPF] Applied to Accel and Gyro immediately.");
        delay(500);
    }
}

void handleConfigMenu() {
    static bool printed = false;
    
    // Print Menu if switched or first run
    if (!printed) {
        switch(currentMenu) {
            case MENU_MAIN_CONFIG:
                Serial.println("\n=== CONFIGURATION MENU ===");
                Serial.printf("[1] Accelerometer (Current: Preset %d)\n", g_sysConfig.accelPreset);
                Serial.printf("[2] Gyroscope     (Current: Preset %d)\n", g_sysConfig.gyroPreset);
                Serial.printf("[3] Magnetometer  (Current: Preset %d)\n", g_sysConfig.magPreset);
                Serial.printf("[ZUPT] Zero Vel Update: %s\n", g_sysConfig.zuptEnabled ? "ENABLED" : "DISABLED [Default]");
                Serial.println("[4] GPS Advanced Configuration");
                Serial.println("[5] Radio Configuration");
                Serial.println("[6] Data Logging / Debug Mode");
                Serial.println("[7] Madgwick Filter Configuration");
                Serial.println("[8] DLPF Bandwidth Configuration (Anti-Aliasing)");
                Serial.println("[x] Exit to Tracking Mode");
                Serial.print("Select: ");
                break;
                
            case MENU_DLPF:
                Serial.println("\n--- DLPF Bandwidth Configuration ---");
                Serial.println("Digital Low Pass Filter removes high-freq noise (motor vibration).");
                Serial.println("Bandwidth (BW) must be < 50% of the Loop Rate (100Hz).");
                Serial.println("Recommended: 50Hz for 100Hz Loop.\n");
                
                Serial.println("[1] 240Hz (Raw/Fast) - Warning: High Noise");
                Serial.println("[2] 110Hz (Fast)     - Good for >200Hz loops");
                Serial.println("[3] 50Hz  (Target)   - BEST match for 100Hz loop");
                Serial.println("[4] 20Hz  (Smooth)   - Removes extreme vibration");
                Serial.println("[5] 10Hz  (Slow)     - Very smooth, slight lag");
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;

            case MENU_MAG_CONFIG:
                Serial.println("\n--- Magnetometer Configuration ---");
                Serial.printf("Current Preset: %d\n", g_sysConfig.magPreset);
                Serial.println("");
                Serial.println("[1] Change Preset (10Hz - 100Hz)");
                Serial.println("[2] Simple Calibration (Min/Max)");
                Serial.println("[3] Advanced Calibration (6-position)");
                Serial.println("[4] Pro Calibration Tool (Ellipsoid Fit)");
                Serial.println("[5] Input Calibration Matrix (from Python script)");
                Serial.println("[x] Back to Main Menu");
                Serial.print("Select: ");
                break;
                
            case MENU_MAG_CAL_TOOL:
                Serial.println("\n--- Pro Magnetometer Calibration (Ellipsoid Fit) ---");
                Serial.println("This tool streams RAW UNCALIBRATED data for post-processing.");
                Serial.println("");
                Serial.println("PROCEDURE:");
                Serial.println("1. Prepare to capture output (copy/paste or serial logger)");
                Serial.println("2. Press [1] to start streaming");
                Serial.println("3. Rotate sensor in ALL directions for 60 seconds:");
                Serial.println("   - Figure-8 patterns while tilting");
                Serial.println("   - Flip completely upside down");
                Serial.println("   - Point straight up, then straight down");
                Serial.println("   - Roll in all axes");
                Serial.println("4. Press 'x' to stop");
                Serial.println("5. Save output to 'mag.csv'");
                Serial.println("6. Run: python calibration_tool/calibrate_mag.py mag.csv");
                Serial.println("7. Use option [3] in Mag Config to input the result");
                Serial.println("");
                Serial.println("[1] Start Streaming");
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
                
            case MENU_GPS_ROOT:
                Serial.println("\n--- 4. GPS Advanced Configuration ---");
                Serial.println("[1] GNSS Config");
                Serial.println("[2] Comm Settings");
                Serial.println("[3] Accuracy & Features");
                Serial.println("[4] Power Management");
                Serial.println("[5] Display & Monitoring");
                Serial.println("[6] System");
                Serial.println("[7] Real-Time Status");
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
                
            case MENU_GNSS_CORE:
                Serial.println("\n--- GNSS Core Configuration ---");
                // 0=All(4), 1=GPS+GLO+GAL(3), 2=GPS+GLO(2)
                Serial.printf("[1] Constellation: %s\n", g_sysConfig.gnssConstellation == 0 ? "All (4 Sats)" : (g_sysConfig.gnssConstellation == 1 ? "3 Sats (GPS+GLO+GAL)" : "2 Sats (GPS+GLO)"));
                Serial.printf("[2] Update Rate:   %d Hz\n", g_sysConfig.gpsRate);
                Serial.printf("[3] Dynamic Model: %s\n", g_sysConfig.dynamicModel == 0 ? "Off (Portable)" : 
                    (g_sysConfig.dynamicModel == 2 ? "Automotive" : 
                    (g_sysConfig.dynamicModel == 3 ? "Airborne" : 
                    (g_sysConfig.dynamicModel == 5 ? "Stationary" : "Pedestrian"))));
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
                
            case MENU_COMM: {
                Serial.println("\n--- Comm Settings ---");
                const char* speedStr = "Standard (100kHz)";
                switch(g_sysConfig.i2cClockSpeed) {
                    case 0: speedStr = "Low (50kHz)"; break;
                    case 1: speedStr = "Standard (100kHz)"; break;
                    case 2: speedStr = "High (200kHz)"; break;
                    case 3: speedStr = "Fast (400kHz)"; break;
                }
                Serial.printf("[1] I2C Bus Speed: %s\n", speedStr);
                Serial.println("[2] Output Protocol: UBX-NAV-PVT (Hardcoded)");
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
            }
            case MENU_RADIO_ROOT:
                 Serial.println("\n--- Radio Configuration ---");
                 Serial.printf("Current Target: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                    g_sysConfig.targetMac[0], g_sysConfig.targetMac[1], g_sysConfig.targetMac[2],
                    g_sysConfig.targetMac[3], g_sysConfig.targetMac[4], g_sysConfig.targetMac[5]);
                 Serial.println("[1] Set Custom Target Address");
                 Serial.println("[2] Reset to Broadcast (FF:FF:FF:FF:FF:FF)");
                 Serial.printf("[3] Radio Debug: %s\n", g_sysConfig.radioDebug ? "ON" : "OFF");
                 Serial.println("[x] Back");
                 Serial.print("Select: ");
                 break;
                 
            case MENU_DATALOG:
                Serial.println("\n--- Data Logging / Debug Mode ---");
                Serial.printf("Status: %s\n", DataLogger::isLogging() ? "LOGGING ACTIVE (Do not power off)" : "STOPPED");
                // Show file size
                Serial.printf("File Size: %d bytes\n", DataLogger::getLogSize());
                Serial.println("[1] Start Logging");
                Serial.println("[2] Stop Logging");
                Serial.println("[3] Dump Log to Serial (CSV)");
                Serial.println("[4] Clear Log File");
                Serial.println("[5] Format Storage (Fix FS Issues)");
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
                
            /*
            case MENU_MADGWICK: {
                // extern Madgwick MadgwickFilter;
                // float currentBeta = MadgwickFilter.getBeta();
                Serial.println("\n--- Madgwick Filter Configuration (DISABLED) ---");
                Serial.println("DMP handles sensor fusion internally.");
                // Serial.printf("Current Beta: %.2f\n", currentBeta);
                Serial.printf("ZUPT (Zero Velocity Update): %s\n\n", g_sysConfig.zuptEnabled ? "ENABLED" : "DISABLED");
                // ...
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
            }
            */
            /*
                Serial.println("Beta controls drift correction speed:");
                Serial.println("  Lower = Smoother, slower correction, MORE drift");
                Serial.println("  Higher = Faster correction, noisier, LESS drift\n");
                Serial.println("[1] Ultra Smooth (0.04) - Slow, very stable");
                Serial.println("[2] Smooth (0.1) - Original default");
                Serial.println("[3] Balanced (0.2) - Current default, recommended");
                Serial.println("[4] Responsive (0.3) - Faster drift correction");
                Serial.println("[5] Ultra Responsive (0.5) - Maximum correction");
                Serial.println("[6] Toggle ZUPT (Fixes stationary drift)");
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
            }
            */
                
            case MENU_ACCURACY:
                Serial.println("\n--- Accuracy & Features ---");
                Serial.printf("[1] SBAS: %s\n", g_sysConfig.sbasEnabled ? "Enabled" : "Disabled");
                Serial.printf("[2] QZSS: %s\n", g_sysConfig.qzssEnabled ? "Enabled" : "Disabled");
                Serial.printf("[3] Anti-Jamming: %s\n", g_sysConfig.antiJamming == 2 ? "Adaptive" : (g_sysConfig.antiJamming == 1 ? "Fixed" : "Auto/Off"));
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
                
            case MENU_POWER:
                Serial.println("\n--- Power Management ---");
                Serial.println("[1] Power Mode: Full Power (Fixed)");
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
                
            case MENU_DISPLAY:
                Serial.println("\n--- Display & Monitoring ---");
                Serial.printf("[1] Units: %s\n", g_sysConfig.unitSystem == 0 ? "Metric" : "Imperial");
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
                
            case MENU_SYSTEM:
                Serial.println("\n--- System ---");
                Serial.println("[1] Save Configuration (Auto-Saved)");
                Serial.println("[2] Load Defaults");
                Serial.println("[3] Factory Reset");
                Serial.println("[4] Firmware Info");
                Serial.println("[x] Back");
                Serial.print("Select: ");
                break;
                
            case MENU_STATUS:
                printStatusDashboard();
                break;
        }
        printed = true;
    }

    if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') return; // Ignore newlines
        
        Serial.println(c); // Echo
        
        switch (currentMenu) {
            case MENU_MAIN_CONFIG:
                if (c == '1') handleSensorConfig(1);
                else if (c == '2') handleSensorConfig(2);
                else if (c == '3') currentMenu = MENU_MAG_CONFIG;
                else if (c == '4') currentMenu = MENU_GPS_ROOT;
                else if (c == '5') currentMenu = MENU_RADIO_ROOT;
                else if (c == '6') currentMenu = MENU_DATALOG;
                else if (c == '7') currentMenu = MENU_MADGWICK;
                else if (c == '8') currentMenu = MENU_DLPF;
                else if (c == 'x') {
                    extern int sysState; 
                    sysState = 6; // SYS_RUNNING
                }
                break;
                
            case MENU_GPS_ROOT:
                if (c == 'x') currentMenu = MENU_MAIN_CONFIG;
                else if (c == '1') currentMenu = MENU_GNSS_CORE;
                else if (c == '2') currentMenu = MENU_COMM;
                else if (c == '3') currentMenu = MENU_ACCURACY;
                else if (c == '4') currentMenu = MENU_POWER;
                else if (c == '5') currentMenu = MENU_DISPLAY;
                else if (c == '6') currentMenu = MENU_SYSTEM;
                else if (c == '7') currentMenu = MENU_STATUS;
                break;
                
            case MENU_DLPF: handleDLPFInput(c); break;
            case MENU_GNSS_CORE: handleGNSSCoreInput(c); break;
            case MENU_COMM: handleCommInput(c); break;
            case MENU_RADIO_ROOT: handleRadioInput(c); break;
            case MENU_DATALOG: handleDatalogInput(c); break;
            // case MENU_MADGWICK: handleMadgwickInput(c); break;
            case MENU_MAG_CONFIG: handleMagConfigInput(c); break;
            case MENU_MAG_CAL_TOOL: handleMagCalToolInput(c); break;
            case MENU_ACCURACY: handleAccuracyInput(c); break;
            case MENU_POWER: handlePowerInput(c); break;
            case MENU_DISPLAY: handleDisplayInput(c); break;
            case MENU_SYSTEM: handleSystemInput(c); break;
            case MENU_STATUS: 
                if (c == 'x') currentMenu = MENU_GPS_ROOT;
                break;
        }
        printed = false; // Refresh menu
        delay(10);
    }
}

// -------------------------------------------
// SUB-MENU HANDLERS
// -------------------------------------------

void handleSensorConfig(int sensorType) {
    if (sensorType == 1) { // Accel
        Serial.println("\n--- Select Accelerometer Preset ---");
        Serial.println("[1] High Precision (+/-2g, 50Hz)");
        Serial.println("[2] General Purpose (+/-4g, 50Hz) [Default]");
        Serial.println("[3] High Impact (+/-8g, 50Hz)");
        Serial.println("[4] Max Range (+/-16g, 50Hz)");
        Serial.println("[5] Noise Filtered (+/-4g, 24Hz)");
        Serial.print("Select (1-5): ");
        while (!Serial.available()) delay(10);
        char p = Serial.read();
        uint8_t val = p - '0';
        if (val >= 1 && val <= 5) {
            AccelCfg::applyAccelPreset(val);
            g_sysConfig.accelPreset = val;
            saveConfigStore();
            Serial.println("Preset Applied & Saved.");
        } else Serial.println("Invalid.");
    } 
    else if (sensorType == 2) { // Gyro
        Serial.println("\n--- Select Gyroscope Preset ---");
        Serial.println("[1] High Res (+/-250dps, 92Hz)");
        Serial.println("[2] General (+/-500dps, 92Hz) [Default]");
        Serial.println("[3] Fast Maneuvers (+/-1000dps, 92Hz)");
        Serial.println("[4] Extreme (+/-2000dps, 92Hz)");
        Serial.println("[5] Aggressive Filtering (+/-500dps, 41Hz)");
        Serial.print("Select (1-5): ");
        while (!Serial.available()) delay(10);
        char p = Serial.read();
        uint8_t val = p - '0';
        if (val >= 1 && val <= 5) {
            GyroCfg::applyGyroPreset(val);
            g_sysConfig.gyroPreset = val;
            saveConfigStore();
            Serial.println("Preset Applied & Saved.");
        } else Serial.println("Invalid.");
    }
    else if (sensorType == 3) { // Mag
        Serial.println("\n--- Select Magnetometer Preset ---");
        Serial.println("[2] Low Power (10Hz)");
        Serial.println("[3] Standard (20Hz)");
        Serial.println("[4] High Speed (50Hz)");
        Serial.println("[5] Max Performance (100Hz) [Default]");
        Serial.print("Select (2-5): ");
        while (!Serial.available()) delay(10);
        char p = Serial.read();
        uint8_t val = p - '0';
        if (val >= 2 && val <= 5) { 
            MagCfg::applyMagPreset(val);
            g_sysConfig.magPreset = val;
            saveConfigStore();
            Serial.println("Preset Applied & Saved.");
        } else Serial.println("Invalid.");
    }
}

void handleGNSSCoreInput(char c) {
    if (c == 'x') { currentMenu = MENU_GPS_ROOT; return; }
    
    if (c == '1') {
        g_sysConfig.gnssConstellation++;
        if (g_sysConfig.gnssConstellation > 2) g_sysConfig.gnssConstellation = 0;
        setGPSConstellation(g_sysConfig.gnssConstellation);
    } 
    else if (c == '2') {
        if (g_sysConfig.gpsRate == 1) g_sysConfig.gpsRate = 5;
        else if (g_sysConfig.gpsRate == 5) g_sysConfig.gpsRate = 10;
        else if (g_sysConfig.gpsRate == 10) g_sysConfig.gpsRate = 25;
        else g_sysConfig.gpsRate = 1;
        setGPSRefreshRate(g_sysConfig.gpsRate);
    }
    else if (c == '3') {
        // Cycle 0(Portable) -> 2(Automotive) -> 3(Airborne) -> 5(Stationary) -> 0
        switch(g_sysConfig.dynamicModel) {
            case 0: g_sysConfig.dynamicModel = 5; break; // Portable -> Stationary
            case 5: g_sysConfig.dynamicModel = 2; break; // Stationary -> Automotive
            case 2: g_sysConfig.dynamicModel = 3; break; // Automotive -> Airborne
            case 3: g_sysConfig.dynamicModel = 0; break; // Airborne -> Portable
            default: g_sysConfig.dynamicModel = 0; break;
        }
        setGPSDynamicModel(g_sysConfig.dynamicModel);
    }
    saveConfigStore();
    Serial.println("Updated.");
}

void handleCommInput(char c) {
    if (c == 'x') { currentMenu = MENU_GPS_ROOT; return; }
    if (c == '1') {
        g_sysConfig.i2cClockSpeed++;
        if (g_sysConfig.i2cClockSpeed > 3) g_sysConfig.i2cClockSpeed = 0;
        setGPSI2CClock(g_sysConfig.i2cClockSpeed);
        saveConfigStore();
        Serial.println("Updated.");
    }
}

void handleRadioInput(char c) {
    if (c == 'x') { currentMenu = MENU_MAIN_CONFIG; return; }
    
    if (c == '2') {
        // Reset to Broadcast
        memset(g_sysConfig.targetMac, 0xFF, 6);
        updatePeerAddress(g_sysConfig.targetMac);
        saveConfigStore();
        Serial.println("Reset to Broadcast.");
    } 
    else if (c == '1') {
        // --- INPUT FIX: FLUSH BUFFER ---
        while(Serial.available()) Serial.read(); 
        
        Serial.println("\nEnter 12 Hex Characters (e.g. AABBCCDDEEFF):");
        Serial.setTimeout(10000); // 10s timeout
        
        // Blocking wait for ANY data to arrive first, to avoid premature timeout empty string?
        // readStringUntil waits for timeout or char.
        // Let's rely on timeout.
        
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() == 12) {
            uint8_t newMac[6];
            for (int i=0; i<6; i++) {
                uint8_t high = parseHexChar(input.charAt(i*2));
                uint8_t low = parseHexChar(input.charAt(i*2+1));
                newMac[i] = (high << 4) | low;
            }
            memcpy(g_sysConfig.targetMac, newMac, 6);
            updatePeerAddress(g_sysConfig.targetMac);
            saveConfigStore();
            Serial.println("Address Updated!");
        } else {
            Serial.print("Error: Received length ");
            Serial.print(input.length());
            Serial.println(". Must be 12 chars.");
        }
    }
    else if (c == '3') {
        g_sysConfig.radioDebug = !g_sysConfig.radioDebug;
        extern bool g_debugMode;
        g_debugMode = g_sysConfig.radioDebug;
        saveConfigStore();
        Serial.println("Debug Toggled.");
    }
}

void handleAccuracyInput(char c) {
    if (c == 'x') { currentMenu = MENU_GPS_ROOT; return; }
    if (c == '1') {
        g_sysConfig.sbasEnabled = !g_sysConfig.sbasEnabled;
        setGPSSBAS(g_sysConfig.sbasEnabled);
    } else if (c == '2') {
        g_sysConfig.qzssEnabled = !g_sysConfig.qzssEnabled;
        setGPSQZSS(g_sysConfig.qzssEnabled);
    } else if (c == '3') {
        g_sysConfig.antiJamming++;
        // Cycle 0, 1, 2
        if (g_sysConfig.antiJamming > 2) g_sysConfig.antiJamming = 0;
        setGPSAntiJamming(g_sysConfig.antiJamming);
    }
    saveConfigStore();
    Serial.println("Updated.");
}

void handlePowerInput(char c) {
    if (c == 'x') { currentMenu = MENU_GPS_ROOT; return; }
    Serial.println("Fixed to Full Power.");
}

void handleDisplayInput(char c) {
    if (c == 'x') { currentMenu = MENU_GPS_ROOT; return; }
    if (c == '1') {
        g_sysConfig.unitSystem = !g_sysConfig.unitSystem;
        Serial.println("Units Toggled.");
    }
    saveConfigStore();
    Serial.println("Updated.");
}

void handleSystemInput(char c) {
    if (c == 'x') { currentMenu = MENU_GPS_ROOT; return; }
    if (c == '1') {
        saveConfigStore();
        Serial.println("Saved.");
    } else if (c == '2') {
        // Defaults
        g_sysConfig.magic = 0; // Invalidate
        initConfigStore(); // Reload defaults
        Serial.println("Defaults Loaded. Please Reboot.");
    } else if (c == '4') {
        Serial.println("\nFIRMWARE: v1.1.1 (RADIO FIX)");
        Serial.println("BUILD: Dec 16 2025");
        delay(2000);
    }
}

void printStatusDashboard() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        Serial.println("\n--- STATUS DASHBOARD ---");
        
        const GPSData& gps = getLatestGPSData();
        const GPSConfigStatus& cfg = getGPSConfigStatus();
        
        Serial.printf("FIX: %s (Sats: %d)\n", gps.valid ? "3D LOCK" : "SEARCHING", gps.numSV);
        Serial.printf("POS: %.6f, %.6f\n", gps.latitude, gps.longitude);
        Serial.printf("ACC: H:%.2fm V:%.2fm\n", gps.horizontalAccuracy, gps.verticalAccuracy);
        Serial.printf("CFG: Rate:%dHz (Target:%dHz)\n", cfg.actualRefreshRate, g_sysConfig.gpsRate);
        Serial.println("[x] Back");
    }
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'x') currentMenu = MENU_GPS_ROOT;
    }
}

void handleDatalogInput(char c) {
    if (c == 'x') { currentMenu = MENU_MAIN_CONFIG; return; }
    
    if (c == '1') {
        DataLogger::startLogging();
    } else if (c == '2') {
        DataLogger::stopLogging();
    } else if (c == '3') {
        Serial.println("\n--- DUMPING LOG FROM LITTLEFS ---");
        // Wait for serial to catch up? No just dump.
        DataLogger::dumpLogToSerial();
        Serial.println("--- DUMP COMPLETE ---");
    } else if (c == '4') {
        DataLogger::clearLog();
    } else if (c == '5') {
        Serial.println("\n[WARN] Formatting LittleFS. This takes time...");
        if (LittleFS.format()) {
            Serial.println("[DONE] Format Complete. File System is clean.");
            DataLogger::init(); // Re-init
        } else {
            Serial.println("[FAIL] Format Failed.");
        }
    }
}

void handleMadgwickInput(char c) {
    if (c == 'x') { currentMenu = MENU_MAIN_CONFIG; return; }
    
/*
    // extern Madgwick MadgwickFilter;
    // extern Madgwick MadgwickFilter6DOF;
    
    // float newBeta = 0.2f; // Default
    bool validChoice = true;
    
    switch(c) {
        case '1': // newBeta = 0.04f; break;  // Ultra Smooth
        case '2': // newBeta = 0.1f; break;   // Smooth
        case '3': // newBeta = 0.2f; break;   // Balanced (default)
        case '4': // newBeta = 0.3f; break;   // Responsive
        case '5': // newBeta = 0.5f; break;   // Ultra Responsive
        case '6': 
            g_sysConfig.zuptEnabled = !g_sysConfig.zuptEnabled;
            saveConfigStore();
            Serial.printf("\n[ZUPT] Zero Velocity Update %s\n", g_sysConfig.zuptEnabled ? "ENABLED" : "DISABLED");
            delay(1000);
            validChoice = false; // Don't update beta
            break;
        default: validChoice = false; break;
    }
    
    
    if (validChoice) {
        // MadgwickFilter.setBeta(newBeta);
        // MadgwickFilter6DOF.setBeta(newBeta);
        // Serial.printf("\n[MADGWICK] Beta set to %.2f\n", newBeta);
        Serial.println("[MADGWICK] Applied to both 9-DOF and 6-DOF filters");
        Serial.println("[MADGWICK] Changes take effect immediately");
        delay(1000);
    }
    */
}
// Magnetometer configuration submenu handler
void handleMagConfigInput(char c) {
    if (c == 'x') { 
        currentMenu = MENU_MAIN_CONFIG; 
        return; 
    }
    
    if (c == '1') {
        // Change Preset
        Serial.println("\\n--- Select Magnetometer Preset ---");
        Serial.println("[2] Low Power (10Hz)");
        Serial.println("[3] Standard (20Hz)");
        Serial.println("[4] High Speed (50Hz)");
        Serial.println("[5] Max Performance (100Hz) [Default]");
        Serial.print("Select (2-5): ");
        while (!Serial.available()) delay(10);
        char p = Serial.read();
        uint8_t val = p - '0';
        if (val >= 2 && val <= 5) { 
            MagCfg::applyMagPreset(val);
            g_sysConfig.magPreset = val;
            saveConfigStore();
            Serial.println("Preset Applied & Saved.");
        } else Serial.println("Invalid.");
        delay(1000);
    }
    else if (c == '2') {
        // Simple Calibration (Min/Max) - TODO: Integrate existing simple cal
        Serial.println("\\n[TODO] Simple Min/Max calibration");
        Serial.println("This will call the existing simple calibration routine.");
        delay(2000);
    }
    else if (c == '3') {
        // Advanced Calibration (6-pos) - TODO: Integrate existing advanced cal  
        Serial.println("\\n[TODO] Advanced 6-position calibration");
        Serial.println("This will call the existing advanced calibration routine.");
        delay(2000);
    }
    else if (c == '4') {
        // Pro Calibration Tool (was [2])
        currentMenu = MENU_MAG_CAL_TOOL;
    }
    else if (c == '5') {
        // Input Calibration Matrix (was [3])
        inputMagCalibrationMatrix();
    }
}

// Helper: Read line with character echo
String readLineWithEcho() {
    String result = "";
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (result.length() > 0) {
                    Serial.println(); // Newline
                    return result;
                }
            } else if (c == '\b' || c == 127) { // Backspace
                if (result.length() > 0) {
                    result.remove(result.length() - 1);
                    Serial.print("\b \b"); // Erase character
                }
            } else if (c >= 32 && c <= 126) { // Printable
                result += c;
                Serial.print(c); // Echo
            }
        }
        delay(1);
    }
}

// Manual input of calibration matrix from Python script
void inputMagCalibrationMatrix() {
    CalibrationData& cal = const_cast<CalibrationData&>(getCalibrationData());
    
    Serial.println("\n=== Input Magnetometer Calibration ===");
    Serial.println("Paste values from Python script (comma-separated):");
    Serial.println("");
    
    // Input Hard Iron Bias
    Serial.println("Hard Iron Bias (3 values):");
    Serial.println("Example: -31.0514, 16.7783, -15.7077");
    Serial.print("> ");
    
    String biasStr = readLineWithEcho();
    biasStr.trim();
    
    // Parse bias values
    int idx1 = biasStr.indexOf(',');
    int idx2 = biasStr.indexOf(',', idx1 + 1);
    if (idx1 > 0 && idx2 > idx1) {
        cal.magBias[0] = biasStr.substring(0, idx1).toFloat();
        cal.magBias[1] = biasStr.substring(idx1 + 1, idx2).toFloat();
        cal.magBias[2] = biasStr.substring(idx2 + 1).toFloat();
        
        Serial.printf("✓ Bias: [%.4f, %.4f, %.4f]\n", 
            cal.magBias[0], cal.magBias[1], cal.magBias[2]);
    } else {
        Serial.println("❌ Invalid format! Calibration aborted.");
        delay(2000);
        return;
    }
    
    // Input Soft Iron Matrix (3 rows)
    Serial.println("\nSoft Iron Matrix (3 rows, 3 values each):");
    for (int row = 0; row < 3; row++) {
        Serial.printf("Row %d: ", row + 1);
        Serial.print("> ");
        
        String rowStr = readLineWithEcho();
        rowStr.trim();
        
        int idx1 = rowStr.indexOf(',');
        int idx2 = rowStr.indexOf(',', idx1 + 1);
        if (idx1 > 0 && idx2 > idx1) {
            cal.magSoftIron[row][0] = rowStr.substring(0, idx1).toFloat();
            cal.magSoftIron[row][1] = rowStr.substring(idx1 + 1, idx2).toFloat();
            cal.magSoftIron[row][2] = rowStr.substring(idx2 + 1).toFloat();
            
            Serial.printf("✓ Row %d: [%.4f, %.4f, %.4f]\n", row + 1,
                cal.magSoftIron[row][0], cal.magSoftIron[row][1], cal.magSoftIron[row][2]);
        } else {
            Serial.println("❌ Invalid format! Calibration aborted.");
            delay(2000);
            return;
        }
    }
    
    // Mark as calibrated with ellipsoid method
    cal.magCalibrated = true;
    cal.magCalMethod = MAG_CAL_ELLIPSOID;
    
    // VERIFICATION: Print what we're about to save
    Serial.println("\n[VERIFY] Values being saved:");
    Serial.printf("  magBias: %.4f, %.4f, %.4f\n", 
        cal.magBias[0], cal.magBias[1], cal.magBias[2]);
    Serial.printf("  magSoftIron[0]: %.4f, %.4f, %.4f\n",
        cal.magSoftIron[0][0], cal.magSoftIron[0][1], cal.magSoftIron[0][2]);
    Serial.printf("  magSoftIron[1]: %.4f, %.4f, %.4f\n",
        cal.magSoftIron[1][0], cal.magSoftIron[1][1], cal.magSoftIron[1][2]);
    Serial.printf("  magSoftIron[2]: %.4f, %.4f, %.4f\n",
        cal.magSoftIron[2][0], cal.magSoftIron[2][1], cal.magSoftIron[2][2]);
    
    // CRITICAL: Save to EEPROM
    if (saveCalibrationToEEPROM()) {
        Serial.println("\n========================================");
        Serial.println("✅ CALIBRATION SAVED TO EEPROM!");
        Serial.println("========================================");
        Serial.println("New calibration is active and will persist across reboots.");
    } else {
        Serial.println("\n========================================");
        Serial.println("⚠️ CALIBRATION LOADED (EEPROM SAVE FAILED)");
        Serial.println("========================================");
        Serial.println("Calibration is active but may not persist across reboots.");
    }
    Serial.println("\nPress any key to return to menu...");
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read(); // Flush
}

