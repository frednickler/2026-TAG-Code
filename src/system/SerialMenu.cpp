#include "SerialMenu.h"
#include "Watchdog.h"
#include "../sensors/IMUManager.h"
#include "../sensors/GPSManager.h"
#include "../calibration/CalibrationManager.h"
#include "../calibration/AlignmentManager.h"
#include "../config/SystemSettings.h"
#include "../radio/RadioManager.h"
#include "../servo/ServoController.h"

// Static member initialization
bool SerialMenu::active = false;
bool SerialMenu::logging = false;
unsigned long SerialMenu::logStartTime = 0;
unsigned long SerialMenu::logSamples = 0;
String SerialMenu::inputBuffer = "";
unsigned long SerialMenu::lastInputTime = 0;

void SerialMenu::init() {
    DEBUG_INFO("SerialMenu initialized");
    active = false;
}

void SerialMenu::update() {
    // If logging is active, check for any key to stop (non-blocking)
    if (logging) {
        if (Serial.available()) {
            while(Serial.available()) { Serial.read(); } // Clear buffer
            stopLogging();
            showMainMenu();
        }
        return; // Don't process normal menu input while logging
    }
    
    // Non-blocking: process characters available (limit to 64 per cycle to prevent blocking)
    int count = 0;
    while (Serial.available() > 0 && count < 64) {
        char c = Serial.read();
        count++;
        lastInputTime = millis();
        
        // Handle newline/carriage return
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                processCommand(inputBuffer);
                inputBuffer = "";
            }
            continue;
        }
        
        // Handle backspace
        if (c == '\b' || c == 127) {
            if (inputBuffer.length() > 0) {
                inputBuffer.remove(inputBuffer.length() - 1);
                Serial.print("\b \b");  // Visual feedback
            }
            continue;
        }
        
        // Accumulate printable characters
        if (isPrintable(c)) {
            inputBuffer += c;
            Serial.print(c);  // Echo back
        }
    }
    
    // Auto-timeout after 30 seconds of inactivity
    if (active && (millis() - lastInputTime > 30000)) {
        DEBUG_INFO("Menu timeout - returning to normal mode");
        active = false;
    }
}

void SerialMenu::processCommand(const String& cmd) {
    String lower = cmd;
    lower.toLowerCase();
    lower.trim();
    
    DEBUG_INFO("Processing command: '%s'", cmd.c_str());
    
    // Main menu commands
    if (lower == "m" || lower == "menu") {
        showMainMenu();
    }
    else if (lower == "h" || lower == "help") {
        showHelp();
    }
    else if (lower == "s" || lower == "status") {
        showStatus();
    }
    else if (lower == "c" || lower == "cal" || lower == "calibrate") {
        showCalibrationMenu();
    }
    else if (lower == "a" || lower == "align") {
        showAlignmentMenu();
    }
    else if (lower == "d" || lower == "output" || lower == "debug") {
        showOutputModeMenu();
    }
    else if (lower == "cfg" || lower == "config") {
        showConfigMenu();
    }
    else if (lower == "l" || lower == "log" || lower == "logger") {
        showDataLoggerMenu();
    }
    else if (lower == "r" || lower == "radio") {
        showRadioMenu();
    }
    else if (lower == "audit") {
        IMUManager::printAudit();
        Serial.print("\nEnter command: ");
    }
    else if (lower == "diag") {
        // Interactive Axis Alignment Diagnostic (Live Data)
        IMUManager::runAxisAlignmentDiagnostic();
        Serial.print("\nEnter command: ");
    }
    else if (lower == "report" || lower == "verify") {
        // Alias for audit report
        IMUManager::printAudit();
        Serial.print("\nEnter command: ");
    }
    else if (lower == "hdg" || lower == "heading") {
        // VQF Heading debug command
        IMUManager::printHeadingDebug();
        Serial.print("\nEnter command: ");
    }
    else if (lower == "gpsdiag") {
        // GPS UART diagnostics
        GPSManager::printDebug();
        Serial.print("\nEnter command: ");
    }
    else if (lower == "exit" || lower == "q" || lower == "quit") {
        Serial.println("\nExiting menu mode");
        active = false;
        // Print prompt for when they come back
        Serial.println("\n(System running... type 'menu' to return)");
    }
    else {
        Serial.printf("\nUnknown command: '%s'. Type 'help' for options.\n", cmd.c_str());
        Serial.print("\nEnter command: ");
    }
}

void SerialMenu::showMainMenu() {
    active = true;
    Serial.println("\n========================================");
    Serial.println("   Secure Base Station V2 - Main Menu");
    Serial.println("========================================");
    Serial.println("  [s] Status       - Show sensor status");
    Serial.println("  [c] Calibrate    - Calibration menu");
    Serial.println("  [a] Alignment    - Position/Heading Alignment");
    Serial.println("  [r] Radio        - ESP-NOW Settings");
    Serial.println("  [l] Logger       - Data Logger (CSV)");
    Serial.println("  [cfg] Config     - Configuration menu");
    Serial.println("  [h] Help         - Show all commands");
    Serial.println("  [q] Quit         - Exit menu");
    Serial.println("========================================");
    Serial.print("\nEnter command: ");
}

void SerialMenu::showHelp() {
    Serial.println("\n========== Help ==========");
    Serial.println("Quick Commands:");
    Serial.println("  m, menu   - Show main menu");
    Serial.println("  h, help   - Show this help");
    Serial.println("  s, status - Sensor status");
    Serial.println("  c, cal    - Calibration menu");
    Serial.println("  d, output - Toggle output mode");
    Serial.println("  cfg       - Configuration menu");
    Serial.println("  audit     - Interactive Axis Diagnostic");
    Serial.println("  report    - Static System Verification Report");
    Serial.println("  hdg       - VQF Heading Debug");
    Serial.println("  gpsdiag   - GPS UART Diagnostics");
    Serial.println("  q, exit   - Exit menu mode");
    Serial.println();
    Serial.println("Notes:");
    Serial.println("  - Menu is non-blocking");
    Serial.println("  - Sensors continue updating");
    Serial.println("  - Watchdog remains active");
    Serial.println("===========================");
    Serial.print("\nEnter command: ");
}

void SerialMenu::showStatus() {
    Serial.println("\n======== Sensor Status ========");
    
    // IMU Status
    Serial.print("IMU (ICM-20948/BMM350): ");
    if (IMUManager::isAvailable()) {
        Serial.print("OK | Health: ");
        Serial.println(IMUManager::checkHealth() ? "GOOD" : "DEGRADED");
        Serial.printf("  Accel: (%.2f, %.2f, %.2f) m/s²\n",
                     IMUManager::getAccelX(), IMUManager::getAccelY(), IMUManager::getAccelZ());
        Serial.printf("  Gyro:  (%.2f, %.2f, %.2f) rad/s\n",
                     IMUManager::getGyroX(), IMUManager::getGyroY(), IMUManager::getGyroZ());
        Serial.printf("  Mag:   (%.0f, %.0f, %.0f) µT\n",
                     IMUManager::getMagX(), IMUManager::getMagY(), IMUManager::getMagZ());
    } else {
        Serial.println("NOT AVAILABLE");
    }
    
    // Barometer: Not available on TAG hardware
    
    // GPS Status
    Serial.print("\nGPS (NEO-M9N): ");
    if (GPSManager::isAvailable()) {
        if (GPSManager::hasFix()) {
            Serial.println("OK | FIX ACQUIRED");
            Serial.printf("  Position: %.6f°, %.6f°\n",
                         GPSManager::getLatitude(), GPSManager::getLongitude());
            Serial.printf("  Altitude: %.1f m\n", GPSManager::getAltitudeMSL());
            Serial.printf("  Speed:    %.1f m/s\n", GPSManager::getSpeed());
            Serial.printf("  Heading:  %.0f°\n", GPSManager::getHeading());
            Serial.printf("  Sats:     %d\n", GPSManager::getSatelliteCount());
        } else {
            Serial.printf("OK | NO FIX (%d sats visible)\n", GPSManager::getSatelliteCount());
        }
    } else {
        Serial.println("NOT AVAILABLE");
    }
    
    Serial.printf("\nSystem Heap: %d bytes free\n", ESP.getFreeHeap());
    Serial.println("================================");
    Serial.print("\nEnter command: ");
}

// Helper to reliably read a line for blocking sub-menus
String readLine() {
    String line = "";
    while(true) {
        Watchdog::feed();
        if (Serial.available()) {
            char c = Serial.read();
            
            // Handle line endings (CR, LF, CRLF)
            if (c == '\n') {
                return line;
            }
            if (c == '\r') {
                // Peek for following \n
                delay(2);
                if (Serial.available() && Serial.peek() == '\n') {
                    Serial.read(); // Consume \n
                }
                return line;
            }
            
            if (isPrintable(c)) {
                line += c;
                Serial.print(c); // Echo
            }
        }
        delay(5);
    }
}

void SerialMenu::showCalibrationMenu() {
    while (true) {
        Serial.println("\n========================================");
        Serial.println("       IMU Calibration Menu");
        Serial.println("========================================");
        Serial.println("  [1] Calibrate Gyroscope (bias only)");
        Serial.println("  [2] Calibrate Accelerometer (simple - bias)");
        Serial.println("  [3] Calibrate Accelerometer (6-position - bias+scale)");
        Serial.println("  [4] Full Calibration (gyro + accel 6-pos)");
        Serial.println("  [5] Calibrate Magnetometer (simple - hard iron)");
        Serial.println("  [6] Calibrate Magnetometer (precision - hard+soft iron)");
        Serial.println("  [7] View Current Calibration");
        Serial.println("  [8] Reset Calibration");
        Serial.println("  [9] Axis Alignment Diagnostic");
        Serial.println("  [b] Back to main menu");
        Serial.println("========================================");
        Serial.print("\nEnter choice: ");
        
        while(!Serial.available()) {
            Watchdog::feed();
            delay(10);
        }
        
        String input = readLine();
        input.trim();
        Serial.println(); // Newline after input
        
        if (input == "1") CalibrationManager::calibrateGyro();
        else if (input == "2") CalibrationManager::calibrateAccelSimple();
        else if (input == "3") CalibrationManager::calibrateAccel6Position();
        else if (input == "4") CalibrationManager::calibrateFull();
        else if (input == "5") CalibrationManager::calibrateMagSimple();
        else if (input == "6") CalibrationManager::calibrateMagPrecision();
        else if (input == "7") CalibrationManager::printCalibration();
        else if (input == "8") {
            Serial.println("\nWARNING: Erase all calibration data? [y/N]: ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String confirm = readLine();
            confirm.trim();
            Serial.println();
            if (confirm.equalsIgnoreCase("y")) {
                 if (CalibrationManager::resetCalibration(true)) Serial.println("Reset complete.");
                 else Serial.println("Reset failed.");
            } else Serial.println("Cancelled.");
        }
        else if (input == "9") {
            IMUManager::runAxisAlignmentDiagnostic();
        }
        else if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) {
            showMainMenu();
            return;
        }
    }
}

void SerialMenu::showConfigMenu() {
    while (true) {
        Serial.println("\n==== Configuration Menu ====");
        Serial.println("  [1] IMU Settings");
        Serial.println("  [2] GPS Settings");
        Serial.println("  [3] System Settings");
        Serial.println("  [4] VQF Tuning");
        Serial.println("  [5] Reset to Defaults");
        Serial.println("  [b] Back to main menu");
        Serial.println("============================");
        Serial.print("\nEnter command: ");

        while(!Serial.available()) {
            Watchdog::feed();
            delay(10);
        }

        String input = readLine();
        input.trim();
        Serial.println();

        if (input == "1") showIMUConfig();
        else if (input == "2") showGPSConfig();
        else if (input == "3") showSystemConfig();
        else if (input == "4") showVQFConfig();
        else if (input == "5") {
            Serial.println("\nWARNING: Reset all settings to defaults AND REBOOT? [y/N]: ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String confirm = readLine();
            confirm.trim();
            if (confirm.equalsIgnoreCase("y")) {
                Serial.println("Resetting to defaults...");
                SystemSettings::loadDefaults();
                Serial.println("Rebooting...");
                delay(1000);
                ESP.restart();
            } else {
                Serial.println("Cancelled.");
            }
        }
        else if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) {
            showMainMenu();
            return;
        }
    }
}

void SerialMenu::showIMUConfig() {
    while(true) {
        RuntimeConfig& cfg = SystemSettings::getConfig();
        Serial.println("\n---- IMU Configuration ----");
        
        // Accelerometer
        Serial.printf("[1] Accel Preset: %d ", cfg.accelPreset);
        if(cfg.accelPreset==1) Serial.print("(Precise - 2g range)");
        else if(cfg.accelPreset==2) Serial.print("(Default - 4g range)");
        else if(cfg.accelPreset==3) Serial.print("(Sports - 8g range)");
        else if(cfg.accelPreset==4) Serial.print("(Extreme - 16g range)");
        Serial.println();
        
        // Gyroscope
        Serial.printf("[2] Gyro Preset:  %d ", cfg.gyroPreset);
        if(cfg.gyroPreset==1) Serial.print("(Precise - 250dps)");
        else if(cfg.gyroPreset==2) Serial.print("(Default - 500dps)");
        else if(cfg.gyroPreset==3) Serial.print("(Fast - 1000dps)");
        else if(cfg.gyroPreset==4) Serial.print("(Extreme - 2000dps)");
        Serial.println();
        
        // Magnetometer - show current config
        Serial.print("[3] Magnetometer Configuration");
        if(cfg.magPreset == 1) Serial.print(" (LOW POWER)");
        else if(cfg.magPreset == 2) Serial.print(" (REGULAR)");
        else if(cfg.magPreset == 3) Serial.print(" (LOW NOISE)");
        else if(cfg.magPreset == 4) Serial.print(" (ULTRA LOW NOISE)");
        else Serial.print(" (Custom/Advanced)");
        Serial.printf(" @ %.4f Hz\n", cfg.magODR);
        
        Serial.println("[b] Back");
        Serial.print("Select setting to change: ");

        while(!Serial.available()) { Watchdog::feed(); delay(10); }
        String input = readLine();
        input.trim();
        Serial.println();

        if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) return;
        
        if (input == "1") {
            // Accel preset
            Serial.println("\nAccelerometer Presets:");
            Serial.println("  1 = Precise  (2g range)  - Low impact activities");
            Serial.println("  2 = Default  (4g range)  - Walking, running");
            Serial.println("  3 = Sports   (8g range)  - High impact sports");
            Serial.println("  4 = Extreme  (16g range) - Extreme sports, crashes");
            Serial.print("Enter Accel Preset (1-4): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String valStr = readLine();
            Serial.println();
            int val = valStr.toInt();
            if (val >= 1 && val <= 4) {
                cfg.accelPreset = val;
                IMUManager::applyAccelPreset(val);
                SystemSettings::save();
                Serial.printf("✓ Saved: Preset %d\n", val);
            } else Serial.println("Invalid.");
        }
        else if (input == "2") {
            // Gyro preset
            Serial.println("\nGyroscope Presets:");
            Serial.println("  1 = Precise  (250dps)  - Slow rotation");
            Serial.println("  2 = Default  (500dps)  - Normal movement");
            Serial.println("  3 = Fast     (1000dps) - Fast rotation");
            Serial.println("  4 = Extreme  (2000dps) - Very fast rotation");
            Serial.print("Enter Gyro Preset (1-4): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String valStr = readLine();
            Serial.println();
            int val = valStr.toInt();
            if (val >= 1 && val <= 4) {
                cfg.gyroPreset = val;
                IMUManager::applyGyroPreset(val);
                SystemSettings::save();
                Serial.printf("✓ Saved: Preset %d\n", val);
            } else Serial.println("Invalid.");
        }
        else if (input == "3") {
            // Magnetometer submenu
            showMagConfig();
        }
    }
}

void SerialMenu::showMagConfig() {
    while(true) {
        RuntimeConfig& cfg = SystemSettings::getConfig();
        Serial.println("\n---- Magnetometer Configuration ----");
        Serial.printf("Current: ");
        if(cfg.magPreset == 1) Serial.print("LOW POWER");
        else if(cfg.magPreset == 2) Serial.print("REGULAR");
        else if(cfg.magPreset == 3) Serial.print("LOW NOISE");
        else if(cfg.magPreset == 4) Serial.print("ULTRA LOW NOISE");
        Serial.printf(" @ %.4f Hz (avg=%d)\n\n", cfg.magODR, cfg.magAveraging);
        
        Serial.println("[1] Simple Configuration (Presets)");
        Serial.println("[2] Advanced Configuration (Manual)");
        Serial.println("[b] Back");
        Serial.print("Select: ");

        while(!Serial.available()) { Watchdog::feed(); delay(10); }
        String input = readLine();
        input.trim();
        Serial.println();

        if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) return;
        
        if (input == "1") {
            showMagSimpleConfig();
        }
        else if (input == "2") {
            showMagAdvancedConfig();
        }
    }
}

void SerialMenu::showMagSimpleConfig() {
    RuntimeConfig& cfg = SystemSettings::getConfig();
    Serial.println("\n---- Magnetometer Presets ----");
    Serial.println("[1] LOW POWER       (avg=0, 1 sample)  - Battery saving");
    Serial.println("[2] REGULAR         (avg=1, 2 samples) - Good balance (DEFAULT)");
    Serial.println("[3] LOW NOISE       (avg=2, 4 samples) - Better accuracy");
    Serial.println("[4] ULTRA LOW NOISE (avg=3, 8 samples) - Best accuracy");
    
    if (cfg.magPreset == 0) Serial.println("\nCurrent: Custom / Advanced");
    else Serial.printf("\nCurrent: Preset %d\n", cfg.magPreset);
    
    Serial.print("Enter Preset (1-4): ");
    
    while(!Serial.available()) { Watchdog::feed(); delay(10); }
    String valStr = readLine();
    Serial.println();
    int val = valStr.toInt();
    
    if (val >= 1 && val <= 4) {
        cfg.magPreset = val;
        IMUManager::applyMagPreset(val);
        SystemSettings::save();
        Serial.printf("✓ Saved: Preset %d (averaging=%d)\n", val, cfg.magAveraging);
    } else {
        Serial.println("Invalid preset.");
    }
}

void SerialMenu::showMagAdvancedConfig() {
    while(true) {
        RuntimeConfig& cfg = SystemSettings::getConfig();
        Serial.println("\n---- Magnetometer Advanced Config ----");
        
        // Get current power mode string
        const char* powerModeStr = "Unknown";
        if (IMUManager::getMagPowerMode() == 1) powerModeStr = "Normal";
        else if (IMUManager::getMagPowerMode() == 4) powerModeStr = "Forced Fast";
        
        Serial.printf("Current: ODR=%.4f Hz, Averaging=%d (%d samples), Power=%s\n\n", 
                     cfg.magODR, cfg.magAveraging, (1 << cfg.magAveraging), powerModeStr);
        
        while(!Serial.available()) { Watchdog::feed(); delay(10); }
        String input = readLine();
        input.trim();
        Serial.println();

        if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) return;
        
        if (input == "1") {
            // Set ODR
            Serial.println("\nAvailable ODR options:");
            Serial.println("  1.5625 Hz  - Ultra slow monitoring");
            Serial.println("  3.125 Hz   - Very slow updates");
            Serial.println("  6.25 Hz    - Slow compass");
            Serial.println("  12.5 Hz    - Default (RECOMMENDED)");
            Serial.println("  25 Hz      - Medium speed");
            Serial.println("  50 Hz      - Fast updates");
            Serial.println("  100 Hz     - Very fast");
            Serial.println("  200 Hz     - Ultra fast (limited averaging)");
            Serial.println("  400 Hz     - Maximum (avg 0-1 only)");
            Serial.print("\nEnter ODR (1.5625, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400): ");
            
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String valStr = readLine();
            Serial.println();
            float odr = valStr.toFloat();
            
            // Validate ODR value
            const float validODRs[] = {1.5625f, 3.125f, 6.25f, 12.5f, 25.0f, 50.0f, 100.0f, 200.0f, 400.0f};
            bool validODR = false;
            for(int i = 0; i < 9; i++) {
                if(fabs(odr - validODRs[i]) < 0.01f) {
                    validODR = true;
                    break;
                }
            }
            
            if(!validODR) {
                Serial.println("❌ Invalid ODR value!");
                continue;
            }
            
            // Validate combination with current averaging
            if(!IMUManager::isValidBMM350Config(odr, cfg.magAveraging)) {
                const char* error = IMUManager::getBMM350ValidationError(odr, cfg.magAveraging);
                Serial.printf("❌ Invalid combination!\n   %s\n", error);
                continue;
            }
            
            cfg.magODR = odr;
            IMUManager::applyMagODR(odr);
            SystemSettings::save();
            Serial.printf("✓ Saved: ODR = %.4f Hz\n", odr);
        }
        else if (input == "2") {
            // Set Averaging
            Serial.println("\nAvailable Averaging options:");
            Serial.println("  0 - No averaging (1 sample)  - Fastest, noisiest");
            Serial.println("  1 - 2 samples                - Good balance (RECOMMENDED)");
            Serial.println("  2 - 4 samples                - Better accuracy");
            Serial.println("  3 - 8 samples                - Best accuracy");
            Serial.print("\nEnter Averaging (0-3): ");
            
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String valStr = readLine();
            Serial.println();
            int avg = valStr.toInt();
            
            if(avg < 0 || avg > 3) {
                Serial.println("❌ Invalid averaging value!");
                continue;
            }
            
            // Validate combination with current ODR
            if(!IMUManager::isValidBMM350Config(cfg.magODR, avg)) {
                const char* error = IMUManager::getBMM350ValidationError(cfg.magODR, avg);
                Serial.printf("❌ Invalid combination!\n   %s\n", error);
                continue;
            }
            
            // cfg.magAveraging = avg; // Handled by setMagAveraging
            IMUManager::setMagAveraging(avg);
            SystemSettings::save();
            Serial.printf("✓ Saved: Averaging = %d (%d samples)\n", avg, (1 << avg));
        }
        else if (input == "3") {
            // Set Power Mode
            Serial.println("\nAvailable Power Modes:");
            Serial.println("  1 - Normal Mode         - Power optimized, ~37Hz max ODR");
            Serial.println("  2 - Forced Mode Fast    - Speed optimized, ~200Hz max ODR");
            Serial.print("\nEnter Mode (1-2): ");
            
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String valStr = readLine();
            Serial.println();
            int mode = valStr.toInt();
            
            if(mode < 1 || mode > 2) {
                Serial.println("❌ Invalid mode!");
                continue;
            }
            
            // Map to enum: 1=Normal (0x01), 2=ForcedFast (0x04)
            uint8_t powerMode = (mode == 1) ? 1 : 4;
            
            IMUManager::setMagPowerMode(powerMode);
            SystemSettings::save();
            Serial.printf("✓ Saved: Power Mode = %s\n", (mode == 1) ? "Normal" : "Forced Fast");
            Serial.println("  Note: Reconfigure ODR for mode to take full effect");
        }
    }
}

void SerialMenu::showGPSConfig() {
    while(true) {
        RuntimeConfig& cfg = SystemSettings::getConfig();
        Serial.println("\n========================================");
        Serial.println("       GPS CONFIGURATION MENU");
        Serial.println("========================================");
        Serial.printf("[1] I2C Clock Speed: %d kHz\n", cfg.gpsI2CClockKHz);
        
        // Show clock speed recommendation
        const char* recommendation = GPSManager::getClockSpeedRecommendation(cfg.gpsRate);
        Serial.printf("    (Current: %d Hz → Recommended: %s)\n", cfg.gpsRate, recommendation);
        
        Serial.printf("[2] Update Rate:     %d Hz\n", cfg.gpsRate);
        Serial.printf("[3] Protocol Mode:   %s\n", cfg.gpsProtocolMode == 0 ? "UBX (Binary)" : "NMEA Debug");
        Serial.printf("[4] Constellation:   %s\n", cfg.gnssConstellation == 0 ? "All (4 Sats)" : (cfg.gnssConstellation == 1 ? "GPS+GLO+GAL" : "GPS+GLO"));
        Serial.printf("[5] Dynamic Model:   %s\n", cfg.dynamicModel == 0 ? "Portable" : (cfg.dynamicModel == 2 ? "Automotive" : "Other"));
        Serial.printf("[6] SBAS:            %s\n", cfg.sbasEnabled ? "Enabled" : "Disabled");
        Serial.printf("[7] QZSS:            %s\n", cfg.qzssEnabled ? "Enabled" : "Disabled");
        Serial.printf("[8] Anti-Jamming:    %s\n", cfg.antiJamming == 0 ? "No" : (cfg.antiJamming == 1 ? "Fixed" : "Adaptive"));
        Serial.println("[9] Factory Reset GPS");
        Serial.println("[b] Back");
        Serial.println("========================================");
        Serial.print("Select option: ");
        
        while(!Serial.available()) { Watchdog::feed(); delay(10); }
        String input = readLine();
        input.trim();
        Serial.println();
        
        if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) return;
        
        // [1] I2C Clock Speed submenu
        if (input == "1") {
            Serial.println("\n---- I2C Clock Speed Selection ----");
            Serial.printf("Current: %d kHz\n", cfg.gpsI2CClockKHz);
            Serial.printf("Recommended for %d Hz update rate: %s\n\n", cfg.gpsRate, recommendation);
            Serial.println("  [1] 100 kHz  (Standard Mode - safest)");
            Serial.println("  [2] 200 kHz  (Good for 1-5 Hz)");
            Serial.println("  [3] 400 kHz  (Good for 6-10 Hz) ← DEFAULT");
            Serial.println("  [4] 600 kHz  (Good for 11-15 Hz)");
            Serial.println("  [5] 800 kHz  (Good for 16-20 Hz)");
            Serial.println("  [6] 1000 kHz (Good for 21-25 Hz, max performance)");
            Serial.println("\nNOTE: Higher speeds = lower latency but more sensitive to wire length/noise");
            Serial.print("\nSelect (1-6): ");
            
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String sVal = readLine();
            int choice = sVal.toInt();
            Serial.println();
            
            uint16_t speeds[] = {100, 200, 400, 600, 800, 1000};
            if (choice >= 1 && choice <= 6) {
                cfg.gpsI2CClockKHz = speeds[choice - 1];
                GPSManager::setI2CClockSpeed(cfg.gpsI2CClockKHz);
                SystemSettings::save();
                Serial.printf("✓ I2C clock set to %d kHz\n", cfg.gpsI2CClockKHz);
            } else {
                Serial.println("Invalid selection");
            }
        }
        
        // [2] Update Rate (Hz)
        else if (input == "2") {
            Serial.print("Enter GPS Update Rate (1-25 Hz): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String sVal = readLine();
            int val = sVal.toInt();
            Serial.println();
            if (val > 0 && val <= 25) {
                cfg.gpsRate = val;
                GPSManager::setRefreshRate(val);
                SystemSettings::save();
                Serial.printf("✓ GPS update rate set to %d Hz\n", cfg.gpsRate);
                
                // Show new recommendation
                const char* newRec = GPSManager::getClockSpeedRecommendation(val);
                Serial.printf("ℹ Recommended I2C clock for %d Hz: %s\n", val, newRec);
                if (cfg.gpsI2CClockKHz < 100 || cfg.gpsI2CClockKHz > 1000) {
                    Serial.printf("⚠ Current clock speed (%d kHz) may not be optimal\n", cfg.gpsI2CClockKHz);
                }
            } else {
                Serial.printf("Invalid rate: '%s' (must be 1-25)\n", sVal.c_str());
            }
        }
        
        // [3] Protocol Mode
        else if (input == "3") {
            Serial.println("\n---- Protocol Mode Selection ----");
            Serial.printf("Current: %s\n\n", cfg.gpsProtocolMode == 0 ? "UBX (Binary)" : "NMEA Debug");
            Serial.println("  [0] UBX (Binary)   - Default, efficient, recommended");
            Serial.println("  [1] NMEA Debug     - Text mode for debugging");
            Serial.println("\nNOTE: Changing protocol requires GPS restart");
            Serial.print("\nSelect (0/1): ");
            
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String sVal = readLine();
            int val = sVal.toInt();
            Serial.println();
            
            if (val == 0 || val == 1) {
                cfg.gpsProtocolMode = val;
                GPSManager::setProtocolMode(val);  // This handles restart confirmation
                SystemSettings::save();
            } else {
                Serial.println("Invalid selection");
            }
        }
        
        // [4] Constellation
        else if (input == "4") {
            Serial.print("Constellation (0=All, 1=3Sat, 2=2Sat): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String sVal = readLine();
            int val = sVal.toInt();
            Serial.println();
            if (sVal.length() > 0 && isDigit(sVal[0]) && val >= 0 && val <= 2) {
                cfg.gnssConstellation = val;
                GPSManager::setConstellation(val);
                SystemSettings::save();
                Serial.printf("✓ Saved. New value: %d\n", cfg.gnssConstellation);
            } else {
                Serial.println("Invalid.");
            }
        }
        
        // [5] Dynamic Model
        else if (input == "5") {
            Serial.print("Model (0=Port, 2=Auto, 3=Air, 5=Stat): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String sVal = readLine();
            int val = sVal.toInt();
            Serial.println();
            if (val == 0 || val == 2 || val == 3 || val == 5) {
                cfg.dynamicModel = val;
                GPSManager::setDynamicModel(val);
                SystemSettings::save();
                Serial.printf("✓ Saved. New value: %d\n", cfg.dynamicModel);
            }
        }
        
        // [6] SBAS Toggle
        else if (input == "6") {
            cfg.sbasEnabled = !cfg.sbasEnabled;
            GPSManager::setSBAS(cfg.sbasEnabled);
            SystemSettings::save();
            Serial.printf("✓ SBAS now %s\n", cfg.sbasEnabled ? "Enabled" : "Disabled");
        }
        
        // [7] QZSS Toggle
        else if (input == "7") {
            cfg.qzssEnabled = !cfg.qzssEnabled;
            GPSManager::setQZSS(cfg.qzssEnabled);
            SystemSettings::save();
            Serial.printf("✓ QZSS now %s\n", cfg.qzssEnabled ? "Enabled" : "Disabled");
        }
        
        // [8] Anti-Jamming
        else if (input == "8") {
            Serial.print("Mode (0=No, 1=Fixed, 2=Adaptive): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String sVal = readLine();
            int val = sVal.toInt();
            Serial.println();
            if (sVal.length() > 0 && isDigit(sVal[0]) && val >= 0 && val <= 2) {
                cfg.antiJamming = val;
                GPSManager::setAntiJamming(val);
                SystemSettings::save();
                Serial.printf("✓ Saved. New value: %d\n", cfg.antiJamming);
            }
        }
        
        // [9] Factory Reset
        else if (input == "9") {
            Serial.println();
            if (GPSManager::factoryReset()) {
                // Reset was successful - update config to defaults
                cfg.gpsRate = 1;  // GPS reverted to 1 Hz
                cfg.gpsProtocolMode = 0;  // UBX mode
                SystemSettings::save();
            }
        }
    }
}

void SerialMenu::showSystemConfig() {
    while(true) {
        RuntimeConfig& cfg = SystemSettings::getConfig();
        Serial.println("\n========================================");
        Serial.println("         SYSTEM CONFIGURATION");
        Serial.println("========================================");
        Serial.printf("Project: %s\n", PROJECT_NAME);
        Serial.printf("Version: %s\n", PROJECT_VERSION);
        Serial.printf("Heap:    %d bytes\n", ESP.getFreeHeap());
        Serial.println("----------------------------------------");
        Serial.printf("[1] Sensor I2C Clock: %d kHz\n", cfg.sensorI2CClockKHz);
        Serial.printf("[2] Mounting:         %s\n", cfg.mountUpsideDown ? "UPSIDE DOWN" : "STANDARD");
        Serial.printf("[3] Loop Rate:        %d Hz\n", SystemSettings::getLoopRate());
        Serial.println("[4] Factory Reset");
        Serial.println("[b] Back");
        Serial.println("========================================");
        Serial.print("Select option: ");
        
        while(!Serial.available()) { Watchdog::feed(); delay(10); }
        String input = readLine();
        input.trim();
        Serial.println();
        
        if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) return;
        
        // [1] Sensor I2C Clock Speed
        if (input == "1") {
            Serial.println("\n---- Sensor I2C Clock Speed ----");
            Serial.printf("Current: %d kHz\n\n", cfg.sensorI2CClockKHz);
            Serial.println("  [1] 100 kHz  (Standard Mode - safest)");
            Serial.println("  [2] 200 kHz  (Low speed)");
            Serial.println("  [3] 400 kHz  (Fast Mode) ← DEFAULT");
            Serial.println("  [4] 600 kHz  (Fast Mode+)");
            Serial.println("  [5] 800 kHz  (High speed)");
            Serial.println("  [6] 1000 kHz (Maximum speed)");
            Serial.println("\nNOTE: IMU, Magnetometer, Display on this bus");
            Serial.println("      Higher speeds = faster updates but sensitivity to noise");
            Serial.print("\nSelect (1-6): ");
            
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String sVal = readLine();
            int choice = sVal.toInt();
            Serial.println();
            
            uint16_t speeds[] = {100, 200, 400, 600, 800, 1000};
            if (choice >= 1 && choice <= 6) {
                cfg.sensorI2CClockKHz = speeds[choice - 1];
                SystemSettings::save();
                Serial.printf("✓ Sensor I2C clock set to %d kHz\n", cfg.sensorI2CClockKHz);
                Serial.println("⚠  Restart device for change to take effect");
            } else {
                Serial.println("Invalid selection");
            }
        }
        
        // [2] Mounting
        else if (input == "2") {
            cfg.mountUpsideDown = !cfg.mountUpsideDown;
            SystemSettings::save();
            Serial.printf("✓ Mounting set to: %s\n", cfg.mountUpsideDown ? "UPSIDE DOWN" : "STANDARD");
        }
        
        // [3] Loop Rate
        else if (input == "3") {
            Serial.print("Enter Loop Rate (25, 50, 100, 200): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String valStr = readLine();
            Serial.println();
            int val = valStr.toInt();
            if (IMUManager::setLoopRate(val)) {
                Serial.printf("✓ Loop Rate set to %d Hz\n", val);
            } else {
                Serial.println("Invalid rate.");
            }
        }
        
        // [4] Factory Reset
        else if (input == "4") {
            Serial.println("\n⚠  WARNING: Resetting to Defaults...");
            Serial.println("This will erase all configurations!");
            Serial.print("Continue? (Y/N): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String confirm = readLine();
Serial.println();
            if (confirm.equalsIgnoreCase("Y")) {
                SystemSettings::loadDefaults();
                Serial.println("✓ Reset complete. Please reboot.");
            } else {
                Serial.println("Cancelled.");
            }
        }
    }
}

bool SerialMenu::isActive() {
    return active;
}

void SerialMenu::showAlignmentMenu() {
    while(true) {
        RuntimeConfig& cfg = SystemSettings::getConfig();
        bool Acquiring = AlignmentManager::isAcquiring();
        
        Serial.println("\n==== Alignment Menu ====");
        if (Acquiring) {
             Serial.printf("STATUS: ACQUIRING GPS (%d sec remaining)\n", AlignmentManager::getAcquisitionRemaining());
        } else {
             Serial.printf("STATUS: %s\n", AlignmentManager::isBaseSet() ? "BASE SET (Ready)" : "WAITING FOR FIX");
             if (AlignmentManager::isBaseSet()) {
                 Serial.printf("  Home: %.7f, %.7f\n", AlignmentManager::getHomeLat(), AlignmentManager::getHomeLon());
             }
        }
        
        Serial.printf("  Offset: %.1f deg\n", cfg.savedHeadingOffset);
        Serial.println("------------------------");
        Serial.printf("[1] GPS Mode:      %s\n", cfg.gpsAlignmentMode == 1 ? "AVERAGE (60s)" : "LAST POSITION");
        Serial.println("[2] Reset/Start Acquisition");
        Serial.println("[3] TARE (Set Visual Target)");
        Serial.println("[4] Manual Offset Entry");
        Serial.println("[b] Back");
        Serial.print("\nEnter choice: ");
        
        while(!Serial.available()) {
            Watchdog::feed();
            if (Acquiring && !AlignmentManager::isAcquiring()) {
                // Acquisition finished while waiting? Refresh menu
                break; 
            }
            delay(100);
        }
        
        if (Serial.available()) {
            String input = readLine();
            input.trim();
            Serial.println();
            
            if (input == "1") {
                cfg.gpsAlignmentMode = (cfg.gpsAlignmentMode == 0) ? 1 : 0;
                SystemSettings::save();
                Serial.printf("Mode set to: %s\n", cfg.gpsAlignmentMode == 1 ? "AVERAGE" : "LAST POS");
            }
            else if (input == "2") {
                AlignmentManager::startAcquisition();
            }
            else if (input == "3") {
                 // Simulate known tag location (Mock for now, or need real tag data)
                 // Since we don't have Radio/Tag data here yet, we ask user
                 Serial.println("Enter Tag Latitude (e.g. -33.12345):");
                 while(!Serial.available()) { Watchdog::feed(); delay(10); }
                 double tLat = readLine().toDouble();
                 
                 Serial.println("Enter Tag Longitude:");
                 while(!Serial.available()) { Watchdog::feed(); delay(10); }
                 double tLon = readLine().toDouble();
                 
                 AlignmentManager::setTare(tLat, tLon);
            }
            else if (input == "4") {
                Serial.print("Enter Offset (deg): ");
                while(!Serial.available()) { Watchdog::feed(); delay(10); }
                float off = readLine().toFloat();
                AlignmentManager::setVisualOffset(off);
            }
            else if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) {
                showMainMenu();
                return;
            }
        }
    }
}

void SerialMenu::showVQFConfig() {
    while(true) {
        RuntimeConfig& cfg = SystemSettings::getConfig();
        Serial.println("\n---- VQF Tuning ----");
        Serial.printf("[1] Tau Accel:   %.1f s (Horizon Speed)\n", cfg.vqfTauAcc);
        Serial.printf("[2] Tau Mag:     %.1f s (Heading Speed)\n", cfg.vqfTauMag);
        Serial.printf("[3] Mag Reject:  %s\n", cfg.vqfMagRejection ? "ON" : "OFF");
        Serial.println("[b] Back");
        Serial.print("Select setting: ");
        
        while(!Serial.available()) { Watchdog::feed(); delay(10); }
        String input = readLine();
        input.trim();
        Serial.println();
        
        if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) return;
        
        if (input == "1") {
            Serial.print("Enter Tau Accel (0.1 - 20.0): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String s = readLine();
            float val = s.toFloat();
            if (val >= 0.1 && val <= 20.0) {
                cfg.vqfTauAcc = val;
                IMUManager::setVQFParams(cfg.vqfTauAcc, cfg.vqfTauMag, cfg.vqfMagRejection);
                SystemSettings::save();
            } else Serial.println("Invalid value.");
        }
        else if (input == "2") {
            Serial.print("Enter Tau Mag (0.1 - 60.0): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String s = readLine();
            float val = s.toFloat();
            if (val >= 0.1 && val <= 60.0) {
                cfg.vqfTauMag = val;
                IMUManager::setVQFParams(cfg.vqfTauAcc, cfg.vqfTauMag, cfg.vqfMagRejection);
                SystemSettings::save();
            } else Serial.println("Invalid value.");
        }
        else if (input == "3") {
            cfg.vqfMagRejection = !cfg.vqfMagRejection;
            IMUManager::setVQFParams(cfg.vqfTauAcc, cfg.vqfTauMag, cfg.vqfMagRejection);
            SystemSettings::save();
        }
    }
}

void SerialMenu::showOutputModeMenu() {
    // Flush any leftover input (e.g. newline from previous command)
    while(Serial.available()) { Serial.read(); }
    
    active = true;  // Prevent update() from consuming our input
    
    const char* modeName[] = {"DEBUG", "COMPACT", "SILENT"};
    OutputMode currentMode = SystemSettings::getOutputMode();
    
    Serial.println("\n==== Output Mode ====");
    Serial.printf("Current Mode: %s\n", modeName[static_cast<int>(currentMode)]);
    Serial.println();
    Serial.println("1. DEBUG   - All sensor data");
    Serial.println("2. COMPACT - GPS + Heading only");
    Serial.println("3. SILENT  - No periodic output");
    Serial.println();
    Serial.println("Note: ERROR/WARN messages always display");
    Serial.println("=====================");
    Serial.print("\nSelect mode (1-3) or [b] to go back: ");
    
    while(!Serial.available()) { 
        Watchdog::feed(); 
        delay(10); 
    }
    
    String input = readLine();
    input.trim();
    Serial.println();
    
    if (input == "1" || input == "2" || input == "3") {
        OutputMode newMode = static_cast<OutputMode>(input.toInt() - 1);
        SystemSettings::setOutputMode(newMode);
        Serial.printf("Output mode changed to: %s\n", modeName[static_cast<int>(newMode)]);
    } else if (!input.equalsIgnoreCase("b") && !input.equalsIgnoreCase("back") && input.length() > 0) {
        Serial.println("Invalid choice.");
    }
    
    active = false;  // Re-enable normal processing
    Serial.println();
}

// =============================================================================
// DATA LOGGER IMPLEMENTATION
// =============================================================================

bool SerialMenu::isDataLogging() {
    return logging;
}

void SerialMenu::startLogging() {
    logging = true;
    logStartTime = millis();
    logSamples = 0;
    
    // Print CSV header
    Serial.println();
    Serial.println("# DATA LOGGING STARTED");
    Serial.println("# Press any key to stop logging");
    Serial.println("#");
    Serial.println("millis,heading_deg,latitude,longitude,speed_mps,satellites");
}

void SerialMenu::stopLogging() {
    if (!logging) return;
    
    logging = false;
    unsigned long duration = millis() - logStartTime;
    
    Serial.println();
    Serial.println("# DATA LOGGING STOPPED");
    Serial.printf("# Duration: %lu ms (%.1f seconds)\n", duration, duration / 1000.0);
    Serial.printf("# Samples: %lu\n", logSamples);
    if (duration > 0) {
        Serial.printf("# Average Rate: %.2f Hz\n", logSamples * 1000.0 / duration);
    }
    Serial.println("#");
    Serial.println("# Copy the above CSV data and save to a .csv file");
    Serial.println("# Or use: pio device monitor -f log2file");
}

void SerialMenu::incrementLogSample() {
    logSamples++;
}

void SerialMenu::showDataLoggerMenu() {
    // Flush any leftover input
    while(Serial.available()) { Serial.read(); }
    
    active = true;
    
    Serial.println("\n========================================");
    Serial.println("       DATA LOGGER (CSV to Serial)");
    Serial.println("========================================");
    Serial.println();
    Serial.println("This logs sensor data as CSV to Serial.");
    Serial.println("Use 'pio device monitor -f log2file' to");
    Serial.println("automatically save output to a .log file.");
    Serial.println();
    Serial.println("Data logged: millis, heading, lat, lon,");
    Serial.println("             speed, satellites");
    Serial.println();
    Serial.println("Sample rate: 1 Hz (same as display rate)");
    Serial.println("========================================");
    Serial.println();
    Serial.println("[s] START logging");
    Serial.println("[b] Back to main menu");
    Serial.print("\nEnter choice: ");
    
    while(!Serial.available()) { 
        Watchdog::feed(); 
        delay(10); 
    }
    
    String input = readLine();
    input.trim();
    Serial.println();
    
    if (input.equalsIgnoreCase("s") || input == "1") {
        startLogging();
        // Return immediately - main loop will output CSV data
        // update() will check for keypress to stop logging
        active = false;  // Allow main loop display to run
        return;
    }
    
    active = false;
    showMainMenu();
}

// =============================================================================
// RADIO (ESP-NOW) MENU IMPLEMENTATION
// =============================================================================

void SerialMenu::showRadioMenu() {
    while (true) {
        RuntimeConfig& cfg = SystemSettings::getConfig();
        
        // Status line
        Serial.println("\n=========== Radio (ESP-NOW) Menu ===========");
        if (!RadioManager::isEnabled()) {
            Serial.println("Status: DISABLED");
        } else if (!RadioManager::isInitialized()) {
            Serial.println("Status: NOT INITIALIZED");
        } else if (RadioManager::isPairingActive()) {
            Serial.println("Status: PAIRING SCAN ACTIVE...");
        } else if (RadioManager::isConnected()) {
            Serial.printf("Status: CONNECTED | Last RX: %ds ago | RSSI: %d dBm\n",
                         RadioManager::getSecondsSinceLastPacket(),
                         RadioManager::getLastRSSI());
        } else {
            Serial.println("Status: WAITING FOR CONNECTION");
        }
        
        // Statistics
        Serial.printf("Packets: %lu recv | %lu sent | %lu lost\n",
                     RadioManager::getPacketsReceived(),
                     RadioManager::getPacketsSent(),
                     RadioManager::getPacketsLost());
        Serial.println("---------------------------------------------");
        
        // Current settings
        const char* roleStr = (cfg.radioRole == 0) ? "BASE (Receiver)" : "TAG (Transmitter)";
        Serial.printf("[1] Role:          %s\n", roleStr);
        Serial.printf("[2] Channel:       %d\n", cfg.radioChannel);
        
        const char* powerStr[] = {"LOW (11 dBm)", "MEDIUM (15 dBm)", "HIGH (20 dBm)"};
        uint8_t txPwrIdx = cfg.radioTxPower < 3 ? cfg.radioTxPower : 2;  // Bounds guard
        Serial.printf("[3] TX Power:      %s\n", powerStr[txPwrIdx]);
        
        Serial.printf("[4] ACK on RX:     %s\n", cfg.radioAckEnabled ? "ON" : "OFF");
        
        Serial.printf("[5] TAG TX Rate:   %d Hz\n", cfg.radioTagTxRate);
        
        // Target MAC
        uint8_t mac[6];
        RadioManager::getTargetMAC(mac);
        bool isBroadcast = RadioManager::isBroadcast();
        Serial.printf("[6] Target Mode:   %s\n", isBroadcast ? "BROADCAST" : "SPECIFIC");
        Serial.printf("[7] Target MAC:    %02X:%02X:%02X:%02X:%02X:%02X%s\n",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                     isBroadcast ? " (all devices)" : "");
        
        Serial.println("---------------------------------------------");
        Serial.println("[8] Scan for TAGs (auto-pair)");
        Serial.println("[9] Enter MAC Manually");
        Serial.println("[m] Show My MAC Address");
        Serial.println("[a] Advanced Settings");
        Serial.printf("[e] %s Radio\n", cfg.radioEnabled ? "Disable" : "Enable");
        Serial.println("[b] Back");
        Serial.println("=============================================");
        Serial.print("\nEnter choice: ");
        
        while(!Serial.available()) { Watchdog::feed(); delay(10); }
        String input = readLine();
        input.trim();
        Serial.println();
        
        if (input == "1") {
            // Toggle role
            uint8_t newRole = (cfg.radioRole == 0) ? 1 : 0;
            RadioManager::setRole(static_cast<RadioRole>(newRole));
            Serial.printf("Role changed to: %s\n", newRole == 0 ? "BASE" : "TAG");
            Serial.println("NOTE: Restart device for role change to take full effect.");
        }
        else if (input == "2") {
            Serial.print("Enter channel (1-14): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String chStr = readLine();
            int ch = chStr.toInt();
            if (ch >= 1 && ch <= 14) {
                RadioManager::setChannel(ch);
            } else {
                Serial.println("Invalid channel. Must be 1-14.");
            }
        }
        else if (input == "3") {
            Serial.println("TX Power Options:");
            Serial.println("  0 = LOW (11 dBm) - ~100m range");
            Serial.println("  1 = MEDIUM (15 dBm) - ~200m range");
            Serial.println("  2 = HIGH (20 dBm) - ~300m+ range");
            Serial.print("Enter choice (0-2): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String pStr = readLine();
            int p = pStr.toInt();
            if (p >= 0 && p <= 2) {
                RadioManager::setTxPower(static_cast<TxPower>(p));
                Serial.printf("TX Power set to: %s\n", powerStr[p]);
            } else {
                Serial.println("Invalid choice.");
            }
        }
        else if (input == "4") {
            bool newAck = !cfg.radioAckEnabled;
            RadioManager::setAckEnabled(newAck);
            Serial.printf("ACK %s\n", newAck ? "enabled (useful for debugging)" : "disabled");
        }
        else if (input == "5") {
            Serial.println("TAG TX Rate Options:");
            Serial.println("  1 Hz  - Low power, standard tracking");
            Serial.println("  2 Hz  - Moderate tracking");
            Serial.println("  5 Hz  - Fast tracking");
            Serial.println("  10 Hz - Real-time tracking");
            Serial.print("Enter rate (1, 2, 5, or 10): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String rStr = readLine();
            int r = rStr.toInt();
            if (r == 1 || r == 2 || r == 5 || r == 10) {
                RadioManager::setTagTxRate(r);
            } else {
                Serial.println("Invalid rate. Must be 1, 2, 5, or 10 Hz.");
            }
        }
        else if (input == "6") {
            // Toggle broadcast/specific
            if (isBroadcast) {
                Serial.println("Currently BROADCAST. Enter specific MAC to change.");
                showRadioMACMenu();
            } else {
                Serial.println("Reset to BROADCAST mode? [y/N]: ");
                while(!Serial.available()) { Watchdog::feed(); delay(10); }
                String confirm = readLine();
                if (confirm.equalsIgnoreCase("y")) {
                    RadioManager::resetTobroadcast();
                }
            }
        }
        else if (input == "7") {
            showRadioMACMenu();
        }
        else if (input == "8") {
            // Start pairing scan
            Serial.println("\nStarting pairing scan (30 seconds)...");
            Serial.println("Make sure TAG device is also in pairing mode!");
            Serial.println("Press any key to cancel.\n");
            
            RadioManager::startPairingScan();
            
            unsigned long startTime = millis();
            while (RadioManager::isPairingActive()) {
                Watchdog::feed();
                if (Serial.available()) {
                    while(Serial.available()) Serial.read();
                    RadioManager::stopPairingScan();
                    Serial.println("Scan cancelled.");
                    break;
                }
                
                // Check for discovered device
                uint8_t discovered[6];
                RadioManager::getLastDiscoveredMAC(discovered);
                if (discovered[0] != 0 || discovered[1] != 0 || discovered[2] != 0) {
                    Serial.printf("\nDiscovered: %02X:%02X:%02X:%02X:%02X:%02X\n",
                                 discovered[0], discovered[1], discovered[2],
                                 discovered[3], discovered[4], discovered[5]);
                    Serial.println("Pairing complete!");
                    RadioManager::stopPairingScan();
                    break;
                }
                
                // Show countdown
                int remaining = 30 - (millis() - startTime) / 1000;
                Serial.printf("\rScanning... %d seconds remaining  ", remaining);
                delay(500);
            }
            Serial.println();
        }
        else if (input == "9") {
            showRadioMACMenu();
        }
        else if (input.equalsIgnoreCase("m")) {
            uint8_t myMac[6];
            RadioManager::getMyMAC(myMac);
            Serial.printf("\n>>> MY MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X <<<\n",
                         myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
            Serial.println("(Copy this to configure other devices)");
        }
        else if (input.equalsIgnoreCase("a")) {
            showRadioAdvancedMenu();
        }
        else if (input.equalsIgnoreCase("e")) {
            bool newEnabled = !cfg.radioEnabled;
            RadioManager::setEnabled(newEnabled);
        }
        else if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) {
            showMainMenu();
            return;
        }
    }
}

void SerialMenu::showRadioMACMenu() {
    while (true) {
        uint8_t mac[6];
        RadioManager::getTargetMAC(mac);
        bool isBroadcast = RadioManager::isBroadcast();
        
        Serial.println("\n======== Target MAC Configuration ========");
        Serial.printf("Current Target: %02X:%02X:%02X:%02X:%02X:%02X%s\n",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                     isBroadcast ? " (BROADCAST)" : "");
        Serial.println("-------------------------------------------");
        Serial.println("[1] Enter MAC manually");
        Serial.println("[2] Scan for nearby devices");
        Serial.println("[3] Reset to BROADCAST");
        Serial.println("[b] Back");
        Serial.println("==========================================");
        Serial.print("\nEnter choice: ");
        
        while(!Serial.available()) { Watchdog::feed(); delay(10); }
        String input = readLine();
        input.trim();
        Serial.println();
        
        if (input == "1") {
            Serial.println("Enter MAC address in format: AA:BB:CC:DD:EE:FF");
            Serial.print("MAC: ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String macStr = readLine();
            macStr.trim();
            macStr.toUpperCase();
            
            // Parse MAC address
            uint8_t newMac[6];
            int parsed = sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                               &newMac[0], &newMac[1], &newMac[2],
                               &newMac[3], &newMac[4], &newMac[5]);
            
            if (parsed == 6) {
                RadioManager::setTargetMAC(newMac);
                Serial.printf("Target MAC set to: %02X:%02X:%02X:%02X:%02X:%02X\n",
                             newMac[0], newMac[1], newMac[2],
                             newMac[3], newMac[4], newMac[5]);
            } else {
                Serial.println("Invalid format. Use AA:BB:CC:DD:EE:FF");
            }
        }
        else if (input == "2") {
            Serial.println("Returning to main radio menu to start scan...");
            return; // Let user use option 8 from main radio menu
        }
        else if (input == "3") {
            RadioManager::resetTobroadcast();
            Serial.println("Reset to BROADCAST mode.");
        }
        else if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) {
            return;
        }
    }
}

void SerialMenu::showRadioAdvancedMenu() {
    while (true) {
        RuntimeConfig& cfg = SystemSettings::getConfig();
        
        Serial.println("\n======== Advanced Radio Settings ========");
        
        const char* rateStr[] = {"1 Mbps", "2 Mbps", "5.5 Mbps", "11 Mbps"};
        const char* rateDesc[] = {"(best range)", "(good range)", "(moderate)", "(shortest range)"};
        uint8_t rateIdx = cfg.radioDataRate < 4 ? cfg.radioDataRate : 0;  // Bounds guard
        Serial.printf("[1] Data Rate:     %s %s\n", rateStr[rateIdx], rateDesc[rateIdx]);
        
        Serial.println("[2] Encryption:    [Coming Soon - Not Implemented]");
        
        Serial.println("[3] Show Statistics");
        Serial.println("[b] Back");
        Serial.println("=========================================");
        Serial.print("\nEnter choice: ");
        
        while(!Serial.available()) { Watchdog::feed(); delay(10); }
        String input = readLine();
        input.trim();
        Serial.println();
        
        if (input == "1") {
            Serial.println("Data Rate Options:");
            Serial.println("  0 = 1 Mbps   - Best range & reliability (RECOMMENDED)");
            Serial.println("  1 = 2 Mbps   - Good range");
            Serial.println("  2 = 5.5 Mbps - Moderate");
            Serial.println("  3 = 11 Mbps  - Shortest range");
            Serial.print("Enter choice (0-3): ");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            String rStr = readLine();
            int r = rStr.toInt();
            if (r >= 0 && r <= 3) {
                RadioManager::setDataRate(static_cast<DataRate>(r));
                Serial.printf("Data rate set to: %s\n", rateStr[r]);
            } else {
                Serial.println("Invalid choice.");
            }
        }
        else if (input == "2") {
            Serial.println("\n========================================");
            Serial.println("       ENCRYPTION - COMING SOON");
            Serial.println("========================================");
            Serial.println("Encryption will be added in a future update.");
            Serial.println("This will use ESP-NOW's CCMP encryption with");
            Serial.println("Local Master Key (LMK) for secure communication.");
            Serial.println("========================================");
            Serial.println("\nPress any key to continue...");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            while(Serial.available()) Serial.read();
        }
        else if (input == "3") {
            Serial.println("\n======== Radio Statistics ========");
            Serial.printf("Packets Received: %lu\n", RadioManager::getPacketsReceived());
            Serial.printf("Packets Sent:     %lu\n", RadioManager::getPacketsSent());
            Serial.printf("Packets Lost:     %lu\n", RadioManager::getPacketsLost());
            Serial.printf("Last RSSI:        %d dBm\n", RadioManager::getLastRSSI());
            Serial.printf("Connected:        %s\n", RadioManager::isConnected() ? "Yes" : "No");
            if (RadioManager::isConnected()) {
                Serial.printf("Last packet:      %d seconds ago\n", RadioManager::getSecondsSinceLastPacket());
                Serial.printf("TAG Position:     %.9f, %.9f\n",
                             RadioManager::getTagLatitude(),
                             RadioManager::getTagLongitude());
                Serial.printf("TAG Heading:      %d°\n", RadioManager::getTagHeading());
            }
            Serial.println("==================================");
            Serial.println("\nPress any key to continue...");
            while(!Serial.available()) { Watchdog::feed(); delay(10); }
            while(Serial.available()) Serial.read();
        }
        else if (input.equalsIgnoreCase("b") || input.equalsIgnoreCase("back")) {
            return;
        }
    }
}
// Temporary file to hold new menu implementation
// This will replace showIMUConfig() and add new submenu functions

