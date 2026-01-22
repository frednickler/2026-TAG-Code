#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include "dmp_imu.h"  // DMP-based quaternion fusion
#include <EEPROM.h>
#include "config.h"
#include "accel_config.h"
#include "gyro_config.h"
#include "mag_config.h"
#include "temp_config.h"
#include "imu_config.h"
#include "imu_validation.h"
#include "gps_module.h"
#include "sensor_fusion.h"
#include "position_tracking.h"
#include "calibration_common.h"
#include "startup_calibration.h"
#include "accel_calibration.h"
#include "gyro_calibration.h"
#include "mag_calibration.h"
#include "temp_sensor.h"
#include "calibration_manager.h" // Include for calibration data structures and functions
#include "radio_packet.h"
#include "radio_packet.h"
#include "esp_now_handler.h"
#include "menu_handler.h"
#include "zupt.h"  // Added ZUPT module
#include "accel_config.h" // For AccelCfg namespace
#include "gyro_config.h" // For GyroCfg namespace
#include "gyro_config.h" // For GyroCfg namespace
#include "mag_config.h"  // For MagCfg namespace
#include "config_store.h" // For persistent settings
#include "menu_handler.h" // For comprehensive configuration menu

// ========== DMP ISOLATION TEST ==========
// Uncomment this to test DMP without I2C bus contention
#define DMP_ISOLATION_TEST  
// ========================================

// DMP quaternion update flag
bool dmpReady = false;

// Calibration state machine
enum CalibrationState { CAL_IDLE, CAL_ACCEL_GYRO, CAL_MAG };
CalibrationState calState = CAL_IDLE;
unsigned long calStartTime = 0;
const int CALIBRATION_DURATION = 30000; // 30 seconds

// DMP Auto-Calibration Warm-Up State Machine
enum DMP_WarmupPhase {
    WARMUP_IDLE,
    WARMUP_GYRO,        // 30s - Keep device still
    WARMUP_ACCEL,       // 10s - Varied orientations
    WARMUP_MAG,         // 30s - Figure-8 motion
    WARMUP_COMPLETE
};
DMP_WarmupPhase dmpWarmupPhase = WARMUP_IDLE;
unsigned long warmupPhaseStartTime = 0;

// ANSI color codes for terminal output
#define ANSI_RESET   "\033[0m"
#define ANSI_BLACK   "\033[30m"
#define ANSI_RED     "\033[31m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_BLUE    "\033[34m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_CYAN    "\033[36m"
#define ANSI_WHITE   "\033[37m"

#include "data_logger.h"

// When set to true, the sensor fusion will use the magnetometer data.
// This is enabled by default, but can be disabled if the magnetometer is not calibrated.
bool useMagnetometer = true;

// --- AXIS IDENTIFICATION MODE ---
// Set this to 'true' and re-upload to easily identify sensor axes.
// It will print only raw accelerometer data to the Serial Monitor.
const bool AXIS_IDENTIFICATION_MODE = false;

// Wi-Fi Access Point Configuration
const char* ssid = "ESP32_IMU_Sensor";
const char* password = "password123"; // CHANGE THIS!

// Web Server & WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- I2C Addresses ---
#define ICM_20948_ADDR 0x68
#define AK09916_ADDR   0x0C

// Add these defines at the top of your main file
#define PROCESSING_VISUALIZATION   // Enable serial output formatted for Processing

// --- Forward Declarations ---
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void handleSerialCommand(const String& command);
#if ENABLE_EEPROM_SELFTEST
void eepromSelfTest();
#endif

// --- Global Variables ---
bool streamActive = false;

// --- Sensor Data structure to prevent shadowing and ensure alignment ---
struct SensorReadings {
    float raw_acc[3];  // mg
    float raw_gyro[3]; // dps
    float raw_mag[3];  // uT
    float cal_acc[3];  // mg
    float cal_gyro[3]; // dps
    float cal_mag[3];  // uT
    float mag_mag;     // uT
} g_sensors = {0};

// Legacy variables for compatibility with AXIS_IDENTIFICATION_MODE and data logger
float ax_mg = 0, ay_mg = 0, az_mg = 0;
float gx_dps = 0, gy_dps = 0, gz_dps = 0;
float mx_uT = 0, my_uT = 0, mz_uT = 0;

float q6[4] = {1, 0, 0, 0}; // Quaternion for 6-DOF fusion
float q9[4] = {1, 0, 0, 0}; // Quaternion for 9-DOF fusion

// Timing
unsigned long lastStreamTime = 0;
unsigned long lastUpdateTime = 0;
unsigned long lastFilterUpdate = 0;

// Function to send calibration values to WebSocket clients
void sendCalibrationValues() {
    const CalibrationData& calibration = getCalibrationData();
    
    // Create a JSON document for the calibration data
    JsonDocument doc;
    
    // Use the recommended syntax to create nested objects and arrays
    JsonObject calObj = doc["calibration"].to<JsonObject>();
    
    JsonArray accelBias = calObj["accelBias"].to<JsonArray>();
    accelBias.add(calibration.accelBias[0]);
    accelBias.add(calibration.accelBias[1]);
    accelBias.add(calibration.accelBias[2]);
    
    JsonArray gyroBias = calObj["gyroBias"].to<JsonArray>();
    gyroBias.add(calibration.gyroBias[0]);
    gyroBias.add(calibration.gyroBias[1]);
    gyroBias.add(calibration.gyroBias[2]);
    
    JsonArray magBias = calObj["magBias"].to<JsonArray>();
    magBias.add(calibration.magBias[0]);
    magBias.add(calibration.magBias[1]);
    magBias.add(calibration.magBias[2]);
    
    JsonArray magScale = calObj["magScale"].to<JsonArray>();
    magScale.add(calibration.magScale[0]);
    magScale.add(calibration.magScale[1]);
    magScale.add(calibration.magScale[2]);
    
    // Serialize JSON to string
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Send to all connected WebSocket clients
    ws.textAll(jsonString);
}

// --- WebSocket Event Handler ---
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        // Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    } else if (type == WS_EVT_DISCONNECT) {
        // Serial.printf("WebSocket client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_PONG) {
        // Serial.printf("Pong from client #%u\n", client->id());
    } else if (type == WS_EVT_DATA) {
        JsonDocument doc; // Use modern JsonDocument
        DeserializationError error = deserializeJson(doc, (char*)data, len);
        if (error) {
            return;
        }

        // Check if the 'command' key exists before trying to access it
        if (!doc["command"].isNull()) {
            const char* command = doc["command"];
            
            if (strcmp(command, "startStream") == 0) {
                streamActive = true;
            } else if (strcmp(command, "stopStream") == 0) {
                streamActive = false;
            } else if (strcmp(command, "calibrateAccel") == 0) {
                bool success = calibrateAccelSimple();
            } else if (strcmp(command, "calibrateMag") == 0) {
                bool success = calibrateMagnetometer();
            } else if (strcmp(command, "getCalibrationValues") == 0) {
                sendCalibrationValues();
            }
        }
    }
}

// --- Radio Packet Helper ---
void sendRadioPacket(const TrackingPacket& packet) {
    sendTrackingData(packet);
    
    // Debug print of the packet size (once)
    static bool printedSize = false;
    if (!printedSize) {
        Serial.printf("[RADIO] Packet Size: %d bytes\n", sizeof(TrackingPacket));
        printedSize = true;
    }
}

// --- Global Flags ---
bool g_debugMode = false;

// --- State Machine ---
enum SystemState {
    SYS_BOOT,
    SYS_INIT_SENSORS,
    SYS_DMP_WARMUP,     // DMP auto-calibration warm-up procedure
    SYS_SAFE_MODE,
    SYS_MENU,
    SYS_CONFIG_MENU,    // New State
    SYS_WAIT_FIX,
    SYS_RUNNING
};
SystemState sysState = SYS_BOOT;

// --- Function Prototypes ---
bool initializeSensors();
// Forward Declarations
void handleMainMenu();
void handleWaitFix();
void handleConfigMenu(); 
void handleSafeMode();
void handleDMPWarmup();  // DMP warm-up procedure
void scanI2C();
void softReset();
#ifdef PROCESSING_VISUALIZATION
void updateVisualization();
#endif

// --- Main Setup ---
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- ESP32 IMU/GPS WebSocket Server ---");
    
    // Initialize LittleFS
    if (!LittleFS.begin()) {
        Serial.println("An Error has occurred while mounting LittleFS");
    } else {
        Serial.println("LittleFS mounted successfully");
        if (DataLogger::init()) Serial.println("[LOG] Found existing log file.");
    }

    // Initialize I2C Buses
    Serial.println("Initializing I2C...");
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN); // IMU I2C
    Wire.setClock(100000); // Lowered to 100kHz for stability check
    Wire1.begin(GPS_SDA_PIN, GPS_SCL_PIN); // GPS I2C

    Wire1.setClock(400000);
    Serial.println("I2C initialized");

    // DMP will be initialized in initializeSensors()
    Serial.println("DMP initialization deferred to sensor init phase");

    sysState = SYS_INIT_SENSORS;
}

void loop() {
    switch (sysState) {
        case SYS_INIT_SENSORS:
            if (initializeSensors()) {
                // SUCCESS: Transition to DMP Warm-Up instead of Menu
                Serial.println("\n[DMP AUTO-CALIBRATION REQUIRED]");
                Serial.println("The DMP learns calibration values during operation.");
                Serial.println("Please follow the on-screen instructions.\n");
                
                // Start warm-up procedure
                dmpWarmupPhase = WARMUP_GYRO;
                warmupPhaseStartTime = millis();
                sysState = SYS_DMP_WARMUP;
                
                // Init Network/Server 
                Serial.printf("Setting up AP: %s\n", ssid);
                WiFi.mode(WIFI_AP);
                WiFi.softAP(ssid, password);
                
                ws.onEvent(onWsEvent);
                server.addHandler(&ws);
                server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
                server.begin();
            } else {
                Serial.println("\n[ERROR] Sensor Initialization Failed. Entering SAFE MODE.");
                sysState = SYS_SAFE_MODE;
            }
            break;

        case SYS_DMP_WARMUP:
            handleDMPWarmup();
            break;

        case SYS_MENU:
            handleMainMenu();
            break;

        case SYS_WAIT_FIX:
            handleWaitFix();
            break;

        case SYS_SAFE_MODE:
            handleSafeMode();
            break;

        case SYS_RUNNING:
            {
            // --- Normal Operation Loop ---
            // --- AXIS IDENTIFICATION MODE ---
            if (AXIS_IDENTIFICATION_MODE) {
                if (readAccel(ax_mg, ay_mg, az_mg)) {
                    Serial.printf("X: %-8.2f Y: %-8.2f Z: %-8.2f\n", ax_mg, ay_mg, az_mg);
                }
                delay(250);
                return;
            }

            ws.cleanupClients();
            
            #ifndef DMP_ISOLATION_TEST
            // Poll GPS module
            updateGPS();
            #endif

            // Check for exit command
            if (Serial.available() > 0) {
                char cmd = Serial.read();
                if (cmd == 'x' || cmd == 'q') {
                    Serial.println("\n[STOP] Stopping Stream. Returning to Menu.");
                    sysState = SYS_MENU;
                    return; 
                } else if (cmd == 'r') {
                    softReset();
                    return;
                }
            }
            
            // --- DMP SENSOR PIPELINE ---
            
            // Poll DMP for new data
            if (updateDMP()) {
                // Get Quaternion (9-axis with Heading Accuracy)
                float qw, qx, qy, qz;
                if (readDMPQuaternion(qw, qx, qy, qz)) {
                    // Update global quaternion for visualization/logging
                    // DMP output is already fused (North-East-Down or similar depending on mounting)
                    q9[0] = qw; q9[1] = qx; q9[2] = qy; q9[3] = qz;
                    
                    // Also populate 6DOF quaternion variable for compatibility
                    q6[0] = qw; q6[1] = qx; q6[2] = qy; q6[3] = qz;
                }
                
                // Get Calibrated Sensor Data (for logging/debug)
                float d_ax, d_ay, d_az;
                float d_gx, d_gy, d_gz;
                float d_mx, d_my, d_mz;
                
                if (readDMPCalibratedSensors(d_ax, d_ay, d_az, d_gx, d_gy, d_gz, d_mx, d_my, d_mz)) {
                    g_sensors.cal_acc[0] = d_ax; g_sensors.cal_acc[1] = d_ay; g_sensors.cal_acc[2] = d_az;
                    g_sensors.cal_gyro[0] = d_gx; g_sensors.cal_gyro[1] = d_gy; g_sensors.cal_gyro[2] = d_gz;
                    g_sensors.cal_mag[0] = d_mx; g_sensors.cal_mag[1] = d_my; g_sensors.cal_mag[2] = d_mz;
                    
                    // Update legacy variables
                    ax_mg = d_ax; ay_mg = d_ay; az_mg = d_az;
                    gx_dps = d_gx; gy_dps = d_gy; gz_dps = d_gz;
                    mx_uT = d_mx; my_uT = d_my; mz_uT = d_mz;
                    
                    g_sensors.mag_mag = sqrt(d_mx*d_mx + d_my*d_my + d_mz*d_mz);
                }
                
                // DEBUG: Print quaternion every 100 updates
                static int debugCount = 0;
                if (++debugCount >= 100) {
                    debugCount = 0;
                    Serial.printf("[DEBUG] DMP Quat: %.3f, %.3f, %.3f, %.3f | Heading: %.1f\n", 
                                 q9[0], q9[1], q9[2], q9[3], getHybridHeading());
                }

                #ifdef PROCESSING_VISUALIZATION
                updateVisualization();
                #endif
            }
            }
            break;

            
        case SYS_CONFIG_MENU:
            handleConfigMenu(); // We haven't defined this yet, wait... defining below
            break;

        default:
            sysState = SYS_MENU;
            break;
    }
}

// --- Helper Functions ---



void softReset() {
    Serial.println("\n[RESET] Rebooting ESP32 in 1 second...");
    // GPS lock will be preserved by Smart Init on next boot!
    delay(1000);
    ESP.restart();
}

void handleMainMenu() {
    static bool printedMenu = false;
    if (!printedMenu) {
        Serial.println("\n=== MAIN MENU ===");
        Serial.println("[1] Start Tracking (Waits for GPS Fix)");
        Serial.println("[2] Calibrate IMU");
        Serial.println("[3] Restart System (Keeps GPS Lock)");
        Serial.println("[4] System Status");
        Serial.println("[6] Configure Sensors"); // Option 6
        Serial.println("=================");
        Serial.print("Select: ");
        printedMenu = true;
    }

    if (Serial.available()) {
        char cmd = Serial.read();
        // serial echo
        if (cmd != '\n' && cmd != '\r') Serial.println(cmd); 

        switch (cmd) {
            case '1':
                Serial.println("[CMD] Starting Tracking Engine...");
                sysState = SYS_WAIT_FIX;
                printedMenu = false;
                break;
            case '2':
                Serial.println("[CMD] Starting Calibration (Standard Workflow)...");
                if (performStartupCalibrationWorkflow()) {
                    Serial.println("[CAL] Success. Calibration complete.");
                    // Safe to reset - EEPROM.commit() is synchronous so data is already saved
                    softReset();
                }
                printedMenu = false;
                break;
            case '3':
                softReset();
                break;
            case '4':
                // Status check
                Serial.printf("\n[STATUS] GPS Fix: %d (Sats: %d)\n", getLatestGPSData().fixType, getLatestGPSData().numSV);
                Serial.printf("[STATUS] Mag Calibrated: %s\n", isMagCalibrated() ? "YES" : "NO");
                printedMenu = false; // Reprint menu
                break;
            case '6':
                sysState = SYS_CONFIG_MENU;
                printedMenu = false;
                break;
            default:
                // Ignore newline chars
                if (cmd != '\n' && cmd != '\r') printedMenu = false; 
                break;
        }
    }
}

void handleWaitFix() {
    static unsigned long lastPrint = 0;
    
    // Always poll GPS in background
    updateGPS();
    
    const GPSData& data = getLatestGPSData();
    
    // Exit Condition: 3D Fix (3) or GNSS+DeadReckoning (4) or Time Only (5 - wait, no)
    // We want at least FixWait (3).
    if (data.fixType >= 3) {
        Serial.printf("\n[GPS] 3D FIX ACQUIRED! (Sats: %d). Starting Stream.\n", data.numSV);
        Serial.println("Press 'x' to stop streaming.");
        sysState = SYS_RUNNING;
        return;
    }
    
    // User Input
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'x') {
            Serial.println("\n[ABORT] Returning to Menu.");
            sysState = SYS_MENU;
            return;
        }
        if (c == 'p' || c == 'P') {
            Serial.println("\n[PROCEED] Starting stream WITHOUT GPS fix (heading/position unavailable).");
            Serial.println("Press 'x' to stop streaming.");
            sysState = SYS_RUNNING;
            return;
        }
    }
    
    // Progress Report (1Hz)
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        Serial.printf("[WAIT] Searching for satellites... (Fix: %d, Sats: %d) [Press 'x' to abort | 'p' to proceed without GPS]\r", 
                      data.fixType, data.numSV);
    }
}

// DMP Auto-Calibration Warm-Up Handler
void handleDMPWarmup() {
    unsigned long phaseElapsed = millis() - warmupPhaseStartTime;
    unsigned long phaseRemaining;
    const unsigned long GYRO_DURATION = 30000;   // 30 seconds
    const unsigned long ACCEL_DURATION = 30000;  // 30 seconds (10s instructions + 20s movement)
    const unsigned long MAG_DURATION = 60000;    // 60 seconds
    
    switch (dmpWarmupPhase) {
        case WARMUP_IDLE:
            // Should not reach here - transition initiated from SYS_INIT_SENSORS
            break;
            
        case WARMUP_GYRO:
            phaseRemaining = (GYRO_DURATION - phaseElapsed) / 1000;
            
            // Print instructions at start
            if (phaseElapsed < 100) {
                Serial.println("\\n" ANSI_YELLOW "═══════════════════════════════════════════════════════" ANSI_RESET);
                Serial.println(ANSI_CYAN "   DMP AUTO-CALIBRATION: GYROSCOPE" ANSI_RESET);
                Serial.println(ANSI_YELLOW "═══════════════════════════════════════════════════════" ANSI_RESET);
                Serial.println(ANSI_WHITE "┌─ INSTRUCTIONS ─────────────────────────────────────┐" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  1. Place device on a FLAT, STABLE surface       " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  2. DO NOT TOUCH or MOVE the device              " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  3. Keep still for 30 seconds                    " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "└────────────────────────────────────────────────────┘" ANSI_RESET "\\n");
            }
            
            // Progress indicator every second
            static unsigned long lastGyroUpdate = 0;
            if (millis() - lastGyroUpdate > 1000) {
                lastGyroUpdate = millis();
                Serial.printf(ANSI_CYAN "[GYRO CAL]" ANSI_RESET " %lu seconds remaining...\\n", phaseRemaining);
            }
            
            // Transition to ACCEL phase
            if (phaseElapsed >= GYRO_DURATION) {
                Serial.println(ANSI_GREEN "\\n✓ Gyroscope calibration complete!" ANSI_RESET "\\n");
                dmpWarmupPhase = WARMUP_ACCEL;
                warmupPhaseStartTime = millis();
            }
            break;
            
        case WARMUP_ACCEL:
            phaseRemaining = (ACCEL_DURATION - phaseElapsed) / 1000;
            
            // Print instructions at start (and keep displayed for 10 seconds)
            if (phaseElapsed < 100) {
                Serial.println(ANSI_YELLOW "═══════════════════════════════════════════════════════" ANSI_RESET);
                Serial.println(ANSI_CYAN "   DMP AUTO-CALIBRATION: ACCELEROMETER" ANSI_RESET);
                Serial.println(ANSI_YELLOW "═══════════════════════════════════════════════════════" ANSI_RESET);
                Serial.println(ANSI_WHITE "┌─ INSTRUCTIONS ─────────────────────────────────────┐" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  1. Keep device FLAT for 5 seconds                " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  2. Rotate to VERTICAL (standing) for 5 seconds  " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  3. Rotate to SIDE for 5 seconds                  " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  4. Repeat varied orientations for 5 seconds     " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "└────────────────────────────────────────────────────┘" ANSI_RESET "\\n");
                Serial.println(ANSI_CYAN "[ACCEL CAL]" ANSI_RESET " Reading instructions (10 seconds)...");
            }
            
            // Progress indicator every second (after initial 10s instruction display)
            static unsigned long lastAccelUpdate = 0;
            if (millis() - lastAccelUpdate > 1000 && phaseElapsed >= 10000) {
                lastAccelUpdate = millis();
                Serial.printf(ANSI_CYAN "[ACCEL CAL]" ANSI_RESET " %lu seconds remaining...\\n", phaseRemaining);
            }
            
            // Transition to MAG phase
            if (phaseElapsed >= ACCEL_DURATION) {
                Serial.println(ANSI_GREEN "\\n✓ Accelerometer calibration complete!" ANSI_RESET "\\n");
                dmpWarmupPhase = WARMUP_MAG;
                warmupPhaseStartTime = millis();
            }
            break;
            
        case WARMUP_MAG:
            phaseRemaining = (MAG_DURATION - phaseElapsed) / 1000;
            
            // Print instructions at start
            if (phaseElapsed < 100) {
                Serial.println(ANSI_YELLOW "═══════════════════════════════════════════════════════" ANSI_RESET);
                Serial.println(ANSI_CYAN "   DMP AUTO-CALIBRATION: MAGNETOMETER" ANSI_RESET);
                Serial.println(ANSI_YELLOW "═══════════════════════════════════════════════════════" ANSI_RESET);
                Serial.println(ANSI_WHITE "┌─ INSTRUCTIONS ─────────────────────────────────────┐" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  1. Move device in FIGURE-8 pattern               " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  2. Tilt and ROTATE through all orientations      " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  3. Continue smooth motion for 60 seconds         " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "│" ANSI_GREEN "  4. Cover all 3D space with movement              " ANSI_WHITE "│" ANSI_RESET);
                Serial.println(ANSI_WHITE "└────────────────────────────────────────────────────┘" ANSI_RESET "\\n");
            }
            
            // Progress indicator every second
            static unsigned long lastMagUpdate = 0;
            if (millis() - lastMagUpdate > 1000) {
                lastMagUpdate = millis();
                Serial.printf(ANSI_CYAN "[MAG CAL]" ANSI_RESET " %lu seconds remaining...\\n", phaseRemaining);
            }
            
            // Transition to COMPLETE
            if (phaseElapsed >= MAG_DURATION) {
                Serial.println(ANSI_GREEN "\\n✓ Magnetometer calibration complete!" ANSI_RESET);
                dmpWarmupPhase = WARMUP_COMPLETE;
                warmupPhaseStartTime = millis();
            }
            break;
            
        case WARMUP_COMPLETE:
            // Display completion message and transition to menu
            if (phaseElapsed < 100) {
                Serial.println("\\n" ANSI_GREEN "═══════════════════════════════════════════════════════" ANSI_RESET);
                Serial.println(ANSI_GREEN "   ✓ DMP AUTO-CALIBRATION COMPLETE!" ANSI_RESET);
                Serial.println(ANSI_GREEN "═══════════════════════════════════════════════════════" ANSI_RESET);
                Serial.println(ANSI_WHITE "DMP is now calibrated and ready for accurate heading." ANSI_RESET);
                Serial.println(ANSI_YELLOW "Note: Calibration is lost on power-off and will need" ANSI_RESET);
                Serial.println(ANSI_YELLOW "      to be repeated on next boot." ANSI_RESET "\\n");
            }
            
            // Wait 2 seconds to let user read message, then go to menu
            if (phaseElapsed >= 2000) {
                sysState = SYS_MENU;
                dmpWarmupPhase = WARMUP_IDLE;  // Reset for next time
            }
            break;
    }
    
    // Always call updateDMP during warm-up to keep data flowing
    updateDMP();
}

bool initializeSensors() {
    bool success = true;

    // === CRITICAL: Apply sensor configuration BEFORE DMP initialization ===
    // DMP initialization locks sensor state. Any config changes AFTER DMP breaks it!
    // === CRITICAL: Apply sensor configuration BEFORE DMP initialization ===
    // DMP initialization locks sensor state. Any config changes AFTER DMP breaks it!
    // Serial.println("Configuring sensors (BEFORE DMP)...");
    
    // NOTE: Conflicted with isouriadakis library. Library sets own defaults.
    /*
    if (!AccelCfg::applyAccelPreset(g_sysConfig.accelPreset)) {
        Serial.println("[WARN] Failed to apply Accel Preset");
    }
    
    if (!GyroCfg::applyGyroPreset(g_sysConfig.gyroPreset)) {
        Serial.println("[WARN] Failed to apply Gyro Preset");
    }
    
    if (!MagCfg::applyMagPreset(g_sysConfig.magPreset)) {
        Serial.println("[WARN] Failed to apply Mag Preset");
    }
    */
    
    // === NOW initialize DMP (sensor config already applied) ===
    Serial.println("Initializing DMP...");
    if (!initDMP()) {
        Serial.println("[ERROR] IMU initialization failed!");
        success = false;
    } else {
        Serial.println("DMP initialized successfully"); 
        dmpReady = true;
    }
    
    // === Load calibration data (does NOT touch sensor registers) ===
    // Optionally clear calibration data 
    #if CLEAR_CALIBRATION_EEPROM_ON_BOOT
    Serial.println("Clearing calibration data from EEPROM...");
    clearCalibrationEEPROM();
    #endif

    Serial.println("Starting calibration (SILENT LOAD)...");
    
    // Initialize Calibration Manager
    initCalibrationManager();
    
    // Initialize ZUPT
    initZUPT();

    // Check I2C bus status and Apply System Config (GPS/Debug only, NOT sensor presets!)
    initConfigStore();
    
    // NOTE: Sensor presets already applied above before DMP init
    // DO NOT re-apply them here - it breaks DMP!
    // Only apply non-sensor settings:
    
    // 6. Radio Debug
    g_debugMode = g_sysConfig.radioDebug;
    
    // 6. Radio Debug
    g_debugMode = g_sysConfig.radioDebug;
    
    // Check status but DO NOT BLOCK
    CalibrationStatus calStatus = performStartupVerification();
    
    if (calStatus == CAL_STATUS_GOOD) {
        Serial.println("[CAL] Verification Passed");
    } else {
        Serial.printf("[WARN] Calibration Status: %d (Not Optimal)\n", calStatus);
        Serial.println("[WARN] Select '2' in Main Menu to recalibrate.");
    }
    
    lastUpdateTime = micros(); 

    Serial.println("Setting up Radio...");
    if (!initESPNow()) {
         Serial.println("[ERROR] Radio initialization failed!");
         // success = false; // Don't block boot for radio?
    }

    Serial.println("Setting up GPS...");
    if (!setupGPS()) {
        Serial.println("[ERROR] GPS initialization failed!");
        success = false;
    } else {
        Serial.println("GPS initialized successfully");
        
        // --- APPLY GPS CONFIGURATION HERE (Safe now) ---
        Serial.println("[CFG] Syncing GPS Advanced Settings...");
        setGPSRefreshRate(g_sysConfig.gpsRate);
        setGPSConstellation(g_sysConfig.gnssConstellation);
        setGPSDynamicModel(g_sysConfig.dynamicModel);
        setGPSI2CClock(g_sysConfig.i2cClockSpeed);
        setGPSSBAS(g_sysConfig.sbasEnabled);
        setGPSQZSS(g_sysConfig.qzssEnabled);
        // setGPSAntiJamming(g_sysConfig.antiJamming);
    }

    Serial.println("Initializing position tracking...");
    initPositionTracking();
    
    #if ENABLE_EEPROM_SELFTEST
        eepromSelfTest();
    #endif

    return success;
}

void handleSafeMode() {
    Serial.println("\n=== SAFE MODE DIAGNOSTIC MENU ===");
    Serial.println("[1] Retry Full Init");
    Serial.println("[2] Retry IMU");
    Serial.println("[3] Retry GPS");
    Serial.println("[4] Scan I2C Buses");
    Serial.println("[5] Force Run (Risk of instability)");
    Serial.println("=================================");
    
    while (true) {
        if (Serial.available()) {
            char cmd = Serial.read();
            switch (cmd) {
                case '1':
                    sysState = SYS_INIT_SENSORS;
                    return;
                case '2':
                    initIMU();
                    break;
                case '3':
                    setupGPS();
                    break;
                case '4':
                    scanI2C();
                    break;
                case '5':
                    sysState = SYS_MENU; // Go to Menu instead of Run
                    return;
            }
            // Re-print menu after action
            Serial.println("\n=== SAFE MODE DIAGNOSTIC MENU ===");
            Serial.println("[1] Retry Full Init");
            Serial.println("[2] Retry IMU");
            Serial.println("[3] Retry GPS");
            Serial.println("[4] Scan I2C Buses");
            Serial.println("[5] Force Run");
        }
        delay(10);
    }
}

void scanI2C() {
    Serial.println("\n--- I2C SCANNER ---");
    Serial.print("Scanning Wire (IMU): SDA="); Serial.print(IMU_SDA_PIN); Serial.print(" SCL="); Serial.println(IMU_SCL_PIN);
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Found I2C device at 0x%02X\n", address);
        }
    }
    
    Serial.print("Scanning Wire1 (GPS): SDA="); Serial.print(GPS_SDA_PIN); Serial.print(" SCL="); Serial.println(GPS_SCL_PIN);
    for (byte address = 1; address < 127; address++) {
        Wire1.beginTransmission(address);
        if (Wire1.endTransmission() == 0) {
            Serial.printf("Found I2C device at 0x%02X\n", address);
        }
    }
    Serial.println("--- SCAN COMPLETE ---");
}

// --- Helper: Algebraic Tilt-Compensated Heading (The "Truth Check") ---
// Returns heading in degrees (0-360), or -1.0 if invalid input
float getTiltCompensatedHeading(float ax, float ay, float az, float mx, float my, float mz) {
    // Input validation - check for NaN or inf
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        !isfinite(mx) || !isfinite(my) || !isfinite(mz)) {
        // Silent failure - return error code
        return -1.0f;
    }
    
    // Check for zero-magnitude cases (sensor failure or freefall)
    float acc_mag = sqrt(ax*ax + ay*ay + az*az);
    if (acc_mag < 500.0f || acc_mag > 1500.0f) {
        // Acceleration not near 1000mg (1G) - tilt compensation unreliable
        // Could be: freefall, high-G maneuver, or sensor error
        return -1.0f;
    }
    
    // 1. Calculate Pitch and Roll from Accelerometer (Rad)
    // Assumes Standard NED/FRD Frame: X=Fwd, Y=Right, Z=Down
    float roll_rad = atan2(ay, az);
    float pitch_rad = atan2(-ax, sqrt(ay * ay + az * az));
    
    // 2. De-rotate Magnetometer vector (Tilt Compensation)
    float cos_roll = cos(roll_rad);
    float sin_roll = sin(roll_rad);
    float cos_pitch = cos(pitch_rad);
    float sin_pitch = sin(pitch_rad);
    
    // The "Horizontal" Mag components (FIXED per NXP AN4248):
    // Standard tilt-compensated heading formula - verified against multiple sources
    float Xh = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
    float Yh = my * cos_roll - mz * sin_roll;
    
    // Check for degenerate case (both Xh and Yh near zero)
    if (abs(Xh) < 0.01f && abs(Yh) < 0.01f) {
        // Horizontal mag field too weak - can't determine heading
        return -1.0f;
    }
    
    // 3. Calculate Heading
    float heading_rad = atan2(Yh, Xh);
    float heading_deg = heading_rad * 180.0f / PI;
    
    // 4. Wrap to 0-360
    if (heading_deg < 0) heading_deg += 360.0f;
    
    // Final sanity check
    if (!isfinite(heading_deg)) {
        return -1.0f;
    }
    
    return heading_deg;
}

#ifdef PROCESSING_VISUALIZATION
void updateVisualization() {

    // Send data periodically for Processing visualization
    static unsigned long lastProcessingUpdate = 0;
    if (millis() - lastProcessingUpdate > 50) { // 20 Hz
      lastProcessingUpdate = millis();

      // Use the global quaternion updated by DMP
      float qw = q9[0];
      float qx = q9[1];
      float qy = q9[2];
      float qz = q9[3];
      
      // Check if we have valid data (quaternion should block sum to ~1.0)
      // If all zero, don't send
      if (qw == 0 && qx == 0 && qy == 0 && qz == 0) return;
      
      // Use Hybrid heading - DMP Pitch/Roll + corrected Magnetometer
      float yawDeg = getHybridHeading();  // ✅ Already 0-360, tilt-compensated
      float roll = 0.0f;
      float pitch = 0.0f;

      const GPSData& gpsData = getLatestGPSData();
      
      // Update global position tracking state
      updatePositionData(gpsData, yawDeg, 0, 0, qw, qx, qy, qz, isMagCalibrated(), isMagCalibrated() || gpsData.valid); 

      // Magnetic and True Heading
      float magHeading = atan2(g_sensors.cal_mag[1], g_sensors.cal_mag[0]) * 180.0f / PI;
      if (magHeading < 0) magHeading += 360.0f;
 
      float trueHeading = magHeading;
      if (gpsData.valid) {
          trueHeading = magHeading + gpsData.magDec_deg;
          while (trueHeading < 0) trueHeading += 360.0f;
          while (trueHeading >= 360.0f) trueHeading -= 360.0f;
      }
      


      // --- 1. CSV OUTPUT (Debug/USB) ---
      // Format: DATA, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz, raw_mx, raw_my, raw_mz, cal_ax, ...
      // Total 28 values
      Serial.print("DATA,");
      
      // 0-2: Raw Accel
      Serial.print(g_sensors.raw_acc[0]); Serial.print(",");
      Serial.print(g_sensors.raw_acc[1]); Serial.print(",");
      Serial.print(g_sensors.raw_acc[2]); Serial.print(",");
      
      // 3-5: Raw Gyro
      Serial.print(g_sensors.raw_gyro[0]); Serial.print(",");
      Serial.print(g_sensors.raw_gyro[1]); Serial.print(",");
      Serial.print(g_sensors.raw_gyro[2]); Serial.print(",");
      
      // 6-8: Raw Mag
      Serial.print(g_sensors.raw_mag[0]); Serial.print(",");
      Serial.print(g_sensors.raw_mag[1]); Serial.print(",");
      Serial.print(g_sensors.raw_mag[2]); Serial.print(",");
      
      // 9-11: Calibrated Accel
      Serial.print(g_sensors.cal_acc[0]); Serial.print(",");
      Serial.print(g_sensors.cal_acc[1]); Serial.print(",");
      Serial.print(g_sensors.cal_acc[2]); Serial.print(",");
      
      // 12-14: Calibrated Gyro
      Serial.print(g_sensors.cal_gyro[0]); Serial.print(",");
      Serial.print(g_sensors.cal_gyro[1]); Serial.print(",");
      Serial.print(g_sensors.cal_gyro[2]); Serial.print(",");
      
      // 15-17: Calibrated Mag
      Serial.print(g_sensors.cal_mag[0]); Serial.print(",");
      Serial.print(g_sensors.cal_mag[1]); Serial.print(",");
      Serial.print(g_sensors.cal_mag[2]); Serial.print(",");
      
      // 18: Mag Magnitude
      Serial.print(g_sensors.mag_mag); Serial.print(",");
      
      // --- 1. VISUALIZATION OUTPUT (CSV) ---
      // USER REQUEST: Offset Heading by 180 degrees (Forward was pointing backward)
      // We apply this to both Quaternion and Euler angles.
      
      // 1. Rotate Quaternion 180 degrees around Z-axis
      // Q_rot = [0, 0, 0, 1] (180 deg around Z)
      // New Q = Q_orig * Q_rot
      // Result: w'=-z, x'=y, y'=-x, z'=w
      float qw_out = -qz;
      float qx_out =  qy;
      float qy_out = -qx;
      float qz_out =  qw;
      
      // 2. Use DMP Euler Yaw (already correct reference frame)
      float yaw_orig = yawDeg;
      
      // No offset needed - yaw_orig is already in correct reference frame
      float yaw_out = yaw_orig;
      if (yaw_out < 0.0f) yaw_out += 360.0f;
      if (yaw_out >= 360.0f) yaw_out -= 360.0f;
      
      // 3. Output headings (no artificial offsets)
      float magHeading_out = magHeading;
      if (magHeading_out < 0.0f) magHeading_out += 360.0f;

      // 19-22: Quaternion (Offset)
      Serial.print(qw_out, 4); Serial.print(",");
      Serial.print(qx_out, 4); Serial.print(",");
      Serial.print(qy_out, 4); Serial.print(",");
      Serial.print(qz_out, 4); Serial.print(",");
      
      // 23-25: Euler (Offset Yaw)
      Serial.print(roll); Serial.print(",");
      Serial.print(pitch); Serial.print(",");
      Serial.print(yaw_out); Serial.print(",");
      
      // 26-27: Headings (Offset)
      Serial.print(magHeading_out); Serial.print(",");
      Serial.print(trueHeading); Serial.print(",");
      
      // 28: Algorithmic Heading (Tilt Compensated) - New!
      // Use MAPPED BODY FRAME Data (g_sensors.cal_acc/mag)
      // Returns -1.0 if sensors invalid (NaN, non-1G acceleration, etc.)
      float algHead = getTiltCompensatedHeading(
          g_sensors.cal_acc[0], g_sensors.cal_acc[1], g_sensors.cal_acc[2],
          g_sensors.cal_mag[0], g_sensors.cal_mag[1], g_sensors.cal_mag[2]
      );
      // Note: -1.0 indicates error (invalid sensors, freefall, high-G, etc.)
      // Output raw value (no artificial offset)
      Serial.print(algHead);
      
      // EOL
      Serial.println("");

      // --- 2. PACKET PREPARATION (Radio) ---
      TrackingPacket packet;
      packet.ms_time = millis();
      packet.lat = (int32_t)(gpsData.latitude * 1e7);
      packet.lon = (int32_t)(gpsData.longitude * 1e7);
      packet.alt = (int16_t)gpsData.altitude;
      packet.speed = (int16_t)(gpsData.speed_m_s * 100.0f); // m/s to cm/s
      packet.gps_course = (uint16_t)(gpsData.heading_deg * 100.0f);
      packet.imu_heading = (uint16_t)(yawDeg * 100.0f);
      packet.pitch = (int8_t)pitch;
      packet.roll = (int8_t)roll;
      packet.flags = 0;
      if (gpsData.valid) packet.flags |= PACKET_FLAG_VALID;
      if (gpsData.fixType == 3) packet.flags |= PACKET_FLAG_FIX_3D;
      
      #ifndef DMP_ISOLATION_TEST
      sendRadioPacket(packet);
      #endif
    }
    #endif

    // 5. Data Logging (1Hz Throttled)
    if (DataLogger::isLogging()) {
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 1000) { 
             lastLog = millis();
             
             LogPacket pkt;
             pkt.timestamp = millis();
             const GPSData& g = getLatestGPSData();
             pkt.lat = g.latitude;
             pkt.lon = g.longitude;
             pkt.alt = g.altitude;
             pkt.accX = ax_mg; pkt.accY = ay_mg; pkt.accZ = az_mg;
             pkt.gyroX = gx_dps; pkt.gyroY = gy_dps; pkt.gyroZ = gz_dps;
             pkt.magX = mx_uT; pkt.magY = my_uT; pkt.magZ = mz_uT;
             
             DataLogger::log(pkt);
        }
    }
}

#if ENABLE_EEPROM_SELFTEST
/**
 * @brief EEPROM self-test: verifies that the currently stored calibration blob
 * can be written back and read again without corruption. No dummy values are
 * injected, so the user’s calibration data remains intact.
 */
void eepromSelfTest() {
    Serial.println("\n--- EEPROM Self-Test ---");

    // Backup the in-memory calibration so we can restore it after the test.
    ExtendedCalibrationData backup = g_extended_cal_data;

    // 1. Ensure EEPROM is initialised and the latest data is loaded.
    if (!loadCalibrationFromEEPROM()) {
        Serial.println("[SELFTEST] No valid data in EEPROM – skipping test.");
        return;
    }

    // 2. Re-compute checksum in case RAM copy differs.
    g_extended_cal_data.checksum = calculateChecksum(&g_extended_cal_data);

    Serial.print("[SELFTEST] Saving current calibration data... ");
    if (!saveCalibrationToEEPROM()) {
        Serial.println("FAILED");
        g_extended_cal_data = backup; // restore
        return;
    }
    Serial.println("OK");

    // 3. Clear RAM copy and load again to verify round-trip.
    memset(&g_extended_cal_data, 0, sizeof(ExtendedCalibrationData));

    Serial.print("[SELFTEST] Re-loading calibration data... ");
    if (!loadCalibrationFromEEPROM()) {
        Serial.println("FAILED");
        g_extended_cal_data = backup;
        return;
    }
    Serial.println("OK");

    // 4. Compare checksums / sizes.
    if (g_extended_cal_data.checksum == backup.checksum) {
        Serial.println("[RESULT] EEPROM self-test PASSED");
    } else {
        Serial.println("[RESULT] EEPROM self-test FAILED – checksum mismatch");
    }

    // 5. Restore backup to leave system unchanged.
    g_extended_cal_data = backup;
}
#endif
