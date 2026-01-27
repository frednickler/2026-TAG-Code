/**
 * GPS Auto-Discovery Tool
 * 
 * Automatically tests different GPIO pins and baud rates to find
 * the correct GPS configuration. Tests pins first, then baud rates.
 * 
 * USAGE:
 * 1. Upload this sketch: pio run -e gps_autodiscover --target upload
 * 2. Monitor output: pio device monitor
 * 3. Wait for results - it will tell you which pin/baud works
 * 4. Update SystemConfig.h with the working configuration
 */

#include <Arduino.h>

// Test these GPIO pins (common ESP32-S3 UART pins)
const uint8_t TEST_PINS[] = {16, 18, 17, 9, 10, 11, 12, 13, 14, 15};
const int NUM_TEST_PINS = 10;

// Test these baud rates (from most common to least)
const uint32_t TEST_BAUDS[] = {9600, 38400, 115200, 4800, 19200, 57600};
const int NUM_TEST_BAUDS = 6;

// GPS TX pin - assuming this is fixed at GPIO17
const uint8_t GPS_TX_PIN = 17;

// Test duration per configuration
const unsigned long TEST_DURATION_MS = 8000; // 8 seconds per test

// Statistics
struct TestResult {
    uint8_t rxPin;
    uint32_t baudRate;
    unsigned long bytesReceived;
    unsigned long validNMEA;
    uint8_t satellites;
    bool hasFix;
    String sampleSentence;
};

TestResult bestResult = {0, 0, 0, 0, 0, false, ""};
TestResult currentTest = {0, 0, 0, 0, 0, false, ""};

String nmeaBuffer = "";

// Simple NMEA validator
bool isValidNMEA(const String& sentence) {
    if (sentence.length() < 10) return false;
    if (sentence.charAt(0) != '$') return false;
    
    int asteriskPos = sentence.indexOf('*');
    if (asteriskPos < 0) return false;
    
    String checksumStr = sentence.substring(asteriskPos + 1, asteriskPos + 3);
    uint8_t expectedChecksum = strtoul(checksumStr.c_str(), NULL, 16);
    
    uint8_t actualChecksum = 0;
    for (int i = 1; i < asteriskPos; i++) {
        actualChecksum ^= sentence.charAt(i);
    }
    
    return (actualChecksum == expectedChecksum);
}

// Extract satellite count from GGA sentence
uint8_t getSatelliteCount(const String& sentence) {
    if (sentence.indexOf("GGA") < 0) return 0;
    
    int commaCount = 0;
    int satStart = 0, satEnd = 0;
    
    for (int i = 0; i < sentence.length() && commaCount < 8; i++) {
        if (sentence.charAt(i) == ',') {
            commaCount++;
            if (commaCount == 7) satStart = i + 1;
            if (commaCount == 8) satEnd = i;
        }
    }
    
    if (satEnd > satStart) {
        String sats = sentence.substring(satStart, satEnd);
        return sats.toInt();
    }
    return 0;
}

void testConfiguration(uint8_t rxPin, uint32_t baudRate) {
    Serial.printf("\n========================================\n");
    Serial.printf("Testing: RX=GPIO%d, Baud=%lu\n", rxPin, baudRate);
    Serial.printf("========================================\n");
    
    // Reset current test
    currentTest = {rxPin, baudRate, 0, 0, 0, false, ""};
    nmeaBuffer = "";
    
    // Initialize Serial1 with test configuration
    Serial1.end();
    delay(100);
    Serial1.setRxBufferSize(2048);
    Serial1.begin(baudRate, SERIAL_8N1, rxPin, GPS_TX_PIN);
    
    // Clear any buffered data
    delay(200);
    while (Serial1.available()) {
        Serial1.read();
    }
    
    unsigned long startTime = millis();
    unsigned long lastReport = millis();
    
    while (millis() - startTime < TEST_DURATION_MS) {
        // Read data
        while (Serial1.available()) {
            char c = Serial1.read();
            currentTest.bytesReceived++;
            
            // Build NMEA sentences
            if (c == '\n' || c == '\r') {
                if (nmeaBuffer.length() > 0) {
                    if (isValidNMEA(nmeaBuffer)) {
                        currentTest.validNMEA++;
                        
                        // Save first valid sentence as sample
                        if (currentTest.sampleSentence.length() == 0) {
                            currentTest.sampleSentence = nmeaBuffer;
                        }
                        
                        // Extract satellite count
                        uint8_t sats = getSatelliteCount(nmeaBuffer);
                        if (sats > 0) {
                            currentTest.satellites = sats;
                            currentTest.hasFix = true;
                        }
                        
                        // Print first few valid sentences
                        if (currentTest.validNMEA <= 3) {
                            Serial.printf("  ✓ Valid: %s\n", nmeaBuffer.c_str());
                        }
                    }
                    nmeaBuffer = "";
                }
            } else if (c >= 32 && c <= 126) {
                nmeaBuffer += c;
                if (nmeaBuffer.length() > 256) nmeaBuffer = "";
            }
        }
        
        // Progress report every 2 seconds
        if (millis() - lastReport > 2000) {
            lastReport = millis();
            Serial.printf("  Progress: %lu bytes, %lu valid NMEA\n", 
                         currentTest.bytesReceived, currentTest.validNMEA);
        }
        
        delay(10);
    }
    
    // Results
    Serial.printf("\n--- Test Results ---\n");
    Serial.printf("Bytes Received: %lu\n", currentTest.bytesReceived);
    Serial.printf("Valid NMEA: %lu\n", currentTest.validNMEA);
    Serial.printf("Satellites: %d\n", currentTest.satellites);
    Serial.printf("Fix: %s\n", currentTest.hasFix ? "YES" : "NO");
    
    if (currentTest.validNMEA > 0) {
        Serial.printf("\n✓✓✓ SUCCESS! This configuration works! ✓✓✓\n");
        Serial.printf("Sample: %s\n", currentTest.sampleSentence.c_str());
        
        // Update best result if this is better
        if (currentTest.validNMEA > bestResult.validNMEA) {
            bestResult = currentTest;
        }
    } else if (currentTest.bytesReceived > 0) {
        Serial.printf("⚠ Data received but no valid NMEA (wrong baud rate?)\n");
    } else {
        Serial.printf("❌ No data received on this pin\n");
    }
    
    Serial1.end();
    delay(100);
}

void setup() {
    Serial.begin(921600);
    delay(2000);
    
    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("   GPS AUTO-DISCOVERY TOOL");
    Serial.println("========================================");
    Serial.println("This tool will automatically find your");
    Serial.println("GPS pin and baud rate configuration.");
    Serial.println("");
    Serial.println("Strategy:");
    Serial.println("1. Test each GPIO pin at 9600 baud");
    Serial.println("2. If pin shows activity, test all bauds");
    Serial.println("3. Report working configuration");
    Serial.println("========================================");
    Serial.println("");
    
    delay(2000);
    
    // PHASE 1: Test each pin at default GPS baud (9600)
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  PHASE 1: Testing Pins at 9600 baud   ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    for (int i = 0; i < NUM_TEST_PINS; i++) {
        testConfiguration(TEST_PINS[i], 9600);
        delay(500);
    }
    
    // PHASE 2: If we found a pin with activity, test all baud rates on that pin
    if (bestResult.bytesReceived > 0 && bestResult.validNMEA == 0) {
        Serial.println("\n╔════════════════════════════════════════╗");
        Serial.printf("║  PHASE 2: Testing Bauds on GPIO%d     ║\n", bestResult.rxPin);
        Serial.println("╚════════════════════════════════════════╝\n");
        
        for (int i = 0; i < NUM_TEST_BAUDS; i++) {
            if (TEST_BAUDS[i] == 9600) continue; // Already tested
            testConfiguration(bestResult.rxPin, TEST_BAUDS[i]);
            delay(500);
        }
    }
    
    // FINAL RESULTS
    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║          FINAL RESULTS                 ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    if (bestResult.validNMEA > 0) {
        Serial.println("✓✓✓ GPS FOUND AND WORKING! ✓✓✓\n");
        Serial.printf("Correct Configuration:\n");
        Serial.printf("  RX Pin: GPIO%d\n", bestResult.rxPin);
        Serial.printf("  TX Pin: GPIO%d\n", GPS_TX_PIN);
        Serial.printf("  Baud Rate: %lu\n", bestResult.baudRate);
        Serial.printf("  Satellites: %d\n", bestResult.satellites);
        Serial.printf("  Fix Status: %s\n", bestResult.hasFix ? "ACQUIRED" : "Searching");
        Serial.printf("\nSample NMEA: %s\n", bestResult.sampleSentence.c_str());
        
        Serial.println("\n--- NEXT STEPS ---");
        Serial.println("Update your SystemConfig.h:");
        Serial.printf("  #define GPS_RX_PIN  %d\n", bestResult.rxPin);
        Serial.printf("  #define GPS_TX_PIN  %d\n", GPS_TX_PIN);
        Serial.printf("  #define GPS_BAUD_RATE  %lu\n", bestResult.baudRate);
        
    } else if (bestResult.bytesReceived > 0) {
        Serial.println("⚠ PARTIAL SUCCESS\n");
        Serial.printf("Data found on GPIO%d but baud rate unknown\n", bestResult.rxPin);
        Serial.println("Try manual baud rate testing");
        
    } else {
        Serial.println("❌ NO GPS DETECTED\n");
        Serial.println("Possible issues:");
        Serial.println("1. GPS TX not connected to any tested pin");
        Serial.println("2. GPS module not powered");
        Serial.println("3. GPS module faulty");
        Serial.println("4. Try different GPIO pins manually");
        Serial.println("\nTested pins: ");
        for (int i = 0; i < NUM_TEST_PINS; i++) {
            Serial.printf("  GPIO%d%s", TEST_PINS[i], (i < NUM_TEST_PINS-1) ? ", " : "\n");
        }
    }
    
    Serial.println("\n========================================");
    Serial.println("Auto-discovery complete!");
    Serial.println("========================================\n");
}

void loop() {
    // Keep running the best configuration for monitoring
    if (bestResult.validNMEA > 0) {
        static unsigned long lastInit = 0;
        
        if (millis() - lastInit > 1000) {
            lastInit = millis();
            
            // Re-init on best config and show live data
            Serial1.end();
            Serial1.setRxBufferSize(2048);
            Serial1.begin(bestResult.baudRate, SERIAL_8N1, bestResult.rxPin, GPS_TX_PIN);
        }
        
        while (Serial1.available()) {
            char c = Serial1.read();
            if (c == '\n' || c == '\r') {
                if (nmeaBuffer.length() > 0 && isValidNMEA(nmeaBuffer)) {
                    Serial.printf("[LIVE] %s\n", nmeaBuffer.c_str());
                }
                nmeaBuffer = "";
            } else if (c >= 32 && c <= 126) {
                nmeaBuffer += c;
                if (nmeaBuffer.length() > 256) nmeaBuffer = "";
            }
        }
    }
    
    delay(10);
}
