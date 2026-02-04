#include <Arduino.h>
#include <LittleFS.h>
#include "config/SystemConfig.h"
#include "config/SystemSettings.h"
#include "system/Watchdog.h"
#include "system/SerialMenu.h"
#include "sensors/IMUManager.h"
#include "sensors/GPSManager.h"
#include "calibration/CalibrationManager.h"
#include "calibration/AlignmentManager.h"
#include "radio/RadioManager.h"
// Note: ServoController not included - TAG has no servo (BASE only)
#include "servo/ServoController.h"
#include "utils/GeoMath.h"

// =============================================================================
// GLOBAL STATE
// =============================================================================
unsigned long lastLoopTime = 0;
unsigned long loopCounter = 0;
unsigned long lastDisplayUpdate = 0;

void setup() {
    // Initialize Serial with high baud rate to prevent debug output blocking
    Serial.begin(921600);
    delay(2000); // Wait for serial monitor (matching BASE Code)
    
    // Initialize LED pin
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    
    DEBUG_INFO("======================================");
    DEBUG_INFO("%s v%s", PROJECT_NAME, PROJECT_VERSION);
    DEBUG_INFO("Hardware: %s", HARDWARE_VERSION);
    DEBUG_INFO("======================================");
    DEBUG_INFO("");
    
    
    // ========================================================================
    // CRITICAL: Initialize SystemSettings FIRST (initializes NVS partition)
    // All other managers depend on NVS being ready
    // ========================================================================
    SystemSettings::init();
    
    // Initialize LittleFS for data logging
    if (!LittleFS.begin(true)) {  // Format if mount fails
        DEBUG_ERROR("LittleFS mount failed - continuing without filesystem");
    } else {
        DEBUG_INFO("LittleFS mounted successfully");
    }
    
    // Initialize Watchdog
    if (!Watchdog::init()) {
        DEBUG_ERROR("Watchdog init failed - continuing without watchdog");
    } else {
        DEBUG_INFO("Watchdog initialized (%d sec timeout)", WATCHDOG_TIMEOUT_SEC);
    }
    
    // Initialize Calibration Manager (loads from NVS - requires SystemSettings first!)
    CalibrationManager::init();
    
    // Initialize IMU (ICM-20948 + BMM350 + VQF)
    DEBUG_INFO("");
    if (!IMUManager::init()) {
        DEBUG_ERROR("IMU initialization FAILED");
        DEBUG_WARN("System will continue with limited functionality");
    } else {
        DEBUG_INFO("IMU initialized successfully");
    }
    
    // Initialize GPS
    DEBUG_INFO("");
    if (!GPSManager::init()) {
        DEBUG_ERROR("GPS initialization FAILED");
        DEBUG_WARN("System will continue without GPS");
    } else {
        DEBUG_INFO("GPS initialized successfully");
    }
    
    // Initialize Radio (ESP-NOW)
    DEBUG_INFO("");
    if (!RadioManager::init()) {
        DEBUG_ERROR("Radio initialization FAILED");
        DEBUG_WARN("System will continue without radio");
    } else {
        DEBUG_INFO("Radio initialized successfully");
    }
    
    // Initialize Alignment Manager
    AlignmentManager::init();
    
    DEBUG_INFO("");
    DEBUG_INFO("======================================");
    DEBUG_INFO("System initialization complete!");
    DEBUG_INFO("======================================");
    DEBUG_INFO("");
    DEBUG_INFO("Type 'help' for commands");
    DEBUG_INFO("Type 'info' for system status");
    DEBUG_INFO("");
    
    
    // Show initial status
    SerialMenu::showStatus();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Calculate loop timing
    unsigned long loopDuration = currentTime - lastLoopTime;
    lastLoopTime = currentTime;
    loopCounter++;
    
    // Feed the watchdog (CRITICAL - must be called regularly)
    Watchdog::feed();
    
    // Profiling variables
    static uint32_t lastProfile = 0;
    static uint32_t imuSum = 0, gpsSum = 0, radioSum = 0, totalSum = 0, samples = 0;
    static uint32_t lastBlink = 0; // For LED blink
    
    uint32_t startTotal = micros();
    
    // 1. IMU Update
    uint32_t t0 = micros();
    IMUManager::update();
    uint32_t t1 = micros();
    
    // 2. GPS Update
    GPSManager::update();
    uint32_t t2 = micros();

    // 3. Radio Update
    RadioManager::update();
    uint32_t t3 = micros();
    
    // 4. Alignment Manager (Lower priority)
    AlignmentManager::update();
    
    // 5. Serial Menu
    SerialMenu::update();
    
    uint32_t endTotal = micros();
    
    // Accumulate stats
    imuSum += (t1 - t0);
    gpsSum += (t2 - t1);
    radioSum += (t3 - t2);
    totalSum += (endTotal - startTotal);
    samples++;
    
    // Print stats every 5 seconds
    if (millis() - lastProfile > 5000) {
        if (samples > 0) {
            Serial.printf("[PROFILE] Loop: %uus | IMU: %uus | GPS: %uus | Radio: %uus | Hz: %.1f\n", 
                totalSum/samples, imuSum/samples, gpsSum/samples, radioSum/samples, 
                1000000.0f / (totalSum/samples));
        }
        lastProfile = millis();
        imuSum = gpsSum = radioSum = totalSum = samples = 0;
    }
    
    // Blink LED to indicate life
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); // Using STATUS_LED_PIN instead of LED_BUILTIN
        
        // Debug output for update rate
        // DEBUG_INFO("Loop running...");
    }
    
    // TAG mode: Send GPS data to BASE
    RuntimeConfig& cfg = SystemSettings::getConfig();
    if (cfg.radioRole == 1 && cfg.radioEnabled) {  // TAG mode
        static uint32_t lastTagTx = 0;
        uint8_t txRate = cfg.radioTagTxRate > 0 ? cfg.radioTagTxRate : 1;
        uint32_t txInterval = 1000 / txRate;
        if (currentTime - lastTagTx >= txInterval) {
            lastTagTx = currentTime;
            // Send data - gpsValid is auto-detected in RadioManager
            RadioManager::sendTagData(
                GPSManager::getLatitude(),
                GPSManager::getLongitude(),
                (uint16_t)IMUManager::getHeading(),
                GPSManager::hasFix()
            );
        }
    }
    
    // Heartbeat LED (blink every second)
    // Use getLoopRate() because MAIN_LOOP_RATE_HZ is now just a default
    uint16_t loopRate = SystemSettings::getLoopRate();
    if (loopCounter % loopRate == 0) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }
    
    // Display sensor data based on output mode (only if menu is not active)
    uint32_t outputInterval = 1000;
    if (SystemSettings::getOutputMode() == OutputMode::DEBUG) {
        outputInterval = 20; // 50Hz update rate for debug
    }

    if (!SerialMenu::isActive() && (currentTime - lastDisplayUpdate >= outputInterval)) {
        lastDisplayUpdate = currentTime;
        
        // DATA LOGGER: Output CSV if logging is active
        if (SerialMenu::isDataLogging()) {
            float heading = IMUManager::getHeading();
            double lat = GPSManager::getLatitude();
            double lon = GPSManager::getLongitude();
            float speed = GPSManager::getSpeed();
            uint8_t sats = GPSManager::getSatelliteCount();
            
            // Output CSV line: millis,heading,lat,lon,speed,sats
            Serial.printf("%lu,%.1f,%.9f,%.9f,%.2f,%d\n",
                          millis(), heading, lat, lon, speed, sats);
            SerialMenu::incrementLogSample();
        }
        else {
            // Normal display output based on mode
            OutputMode mode = SystemSettings::getOutputMode();
            
            // SILENT mode: no periodic output
            if (mode == OutputMode::SILENT) {
                // Do nothing - only ERROR/WARN macros will print
            }
            // COMPACT mode: GPS + Heading only
            else if (mode == OutputMode::COMPACT) {
                float heading = IMUManager::getHeading();
                if (GPSManager::hasFix()) {
                    DEBUG_INFO("GPS: %.9f, %.9f | %.1f m/s | %d sats | Hdg: %.0f°",
                             GPSManager::getLatitude(), GPSManager::getLongitude(),
                             GPSManager::getSpeed(), GPSManager::getSatelliteCount(),
                             heading);
                } else if (GPSManager::isAvailable()) {
                    DEBUG_INFO("GPS: No fix (%d sats) | Hdg: %.0f°", 
                             GPSManager::getSatelliteCount(), heading);
                } else {
                    DEBUG_INFO("GPS: Not Detected | Hdg: %.0f°", heading);
                }
            }
            // RADIO mode: Radio debug info
            else if (mode == OutputMode::RADIO) {
                Serial.println("\n========== TAG RADIO DEBUG ==========");
                
                // Radio status
                Serial.printf("Radio: %s | Role: TAG (Transmitter)\n", 
                             RadioManager::isEnabled() ? "ENABLED" : "DISABLED");
                
                if (RadioManager::isEnabled()) {
                    uint8_t mac[6];
                    RadioManager::getMyMAC(mac);
                    Serial.printf("My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                    
                    RadioManager::getTargetMAC(mac);
                    Serial.printf("Target: %02X:%02X:%02X:%02X:%02X:%02X %s\n",
                                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                                 RadioManager::isBroadcast() ? "(BROADCAST)" : "");
                    
                    RuntimeConfig& cfg = SystemSettings::getConfig();
                    Serial.printf("Channel: %d | TX Rate: %d Hz | Power: ", 
                                 cfg.radioChannel, cfg.radioTagTxRate);
                    
                    const char* powerStr[] = {"LOW", "MED", "HIGH"};
                    Serial.println(powerStr[cfg.radioTxPower < 3 ? cfg.radioTxPower : 2]);
                    
                    Serial.printf("Packets Sent: %lu | Lost: %lu\n",
                                 RadioManager::getPacketsSent(),
                                 RadioManager::getPacketsLost());
                }
                
                // GPS status (required for transmission)
                Serial.println("---");
                if (GPSManager::hasFix()) {
                    Serial.printf("GPS: FIX | Pos: %.6f, %.6f\n",
                                 GPSManager::getLatitude(), GPSManager::getLongitude());
                    Serial.printf("     %d sats | %.1f m/s | Hdg: %d°\n",
                                 GPSManager::getSatelliteCount(),
                                 GPSManager::getSpeed(),
                                 (int)GPSManager::getHeading());
                    Serial.println("✓ TRANSMITTING (GPS fix acquired)");
                } else if (GPSManager::isAvailable()) {
                    Serial.printf("GPS: NO FIX (%d sats visible)\n", 
                                 GPSManager::getSatelliteCount());
                    Serial.println("⚠ NOT TRANSMITTING (waiting for fix)");
                } else {
                    Serial.println("GPS: ERROR (not detected)");
                    Serial.println("✗ NOT TRANSMITTING");
                }
                
                Serial.printf("IMU Heading: %.1f°\n", IMUManager::getHeading());
                Serial.println("=====================================");
            }
            // DEBUG mode: Full sensor data
            else {
                // IMU Data
                DEBUG_INFO("CHAIN[%lu] Accel: (%.2f, %.2f, %.2f) m/s² | Gyro: (%.2f, %.2f, %.2f) rad/s | Mag: (%.1f, %.1f, %.1f) µT",
                         IMUManager::getUpdateCount(),
                         IMUManager::getAccelX(), IMUManager::getAccelY(), IMUManager::getAccelZ(),
                         IMUManager::getGyroX(), IMUManager::getGyroY(), IMUManager::getGyroZ(),
                         IMUManager::getMagX(), IMUManager::getMagY(), IMUManager::getMagZ());
                
                DEBUG_INFO("EULER: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°",
                         IMUManager::getRoll(), IMUManager::getPitch(), IMUManager::getHeading());
                
                DEBUG_INFO("VQF_INPUT: Accel=(%.2f,%.2f,%.2f) Gyro=(%.2f,%.2f,%.2f) Mag=(%.1f,%.1f,%.1f)",
                         IMUManager::getAccelX(), IMUManager::getAccelY(), IMUManager::getAccelZ(),
                         IMUManager::getGyroX(), IMUManager::getGyroY(), IMUManager::getGyroZ(),
                         IMUManager::getMagX(), IMUManager::getMagY(), IMUManager::getMagZ());
                
                // GPS Data
                if (GPSManager::hasFix()) {
                    DEBUG_INFO("GPS: %.9f°, %.9f° | Alt=%.1f m | Speed=%.1f m/s | Hdg=%d° | %d sats",
                             GPSManager::getLatitude(), GPSManager::getLongitude(),
                             GPSManager::getAltitudeMSL(), GPSManager::getSpeed(),
                             (int)GPSManager::getHeading(), GPSManager::getSatelliteCount());
                } else if (GPSManager::isAvailable()) {
                    DEBUG_INFO("GPS: No fix (%d sats visible)", GPSManager::getSatelliteCount());
                } else {
                    DEBUG_INFO("GPS: Not Detected (Check Wiring)");
                }
            }
        }
    }
    
    // Maintain loop rate
    delay(1000 / loopRate);
}
