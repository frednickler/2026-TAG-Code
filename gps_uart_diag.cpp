/**
 * GPS UART Diagnostic Tool
 * 
 * Tests GPS communication at different baud rates and displays raw NMEA data.
 * Use this to verify GPS hardware and determine correct baud rate.
 * 
 * USAGE:
 * 1. Comment out src/main.cpp in platformio.ini
 * 2. Uncomment this file in platformio.ini
 * 3. Upload and monitor serial output
 * 4. Observe which baud rate shows valid NMEA sentences
 */

#include <Arduino.h>

// Pin definitions (must match your hardware)
#define GPS_RX_PIN  16  // GPIO16 - ESP32 receives data FROM GPS
#define GPS_TX_PIN  17  // GPIO17 - ESP32 sends data TO GPS

// Test these baud rates in sequence
const uint32_t BAUD_RATES[] = {9600, 38400, 115200, 57600, 19200, 4800};
const int NUM_BAUD_RATES = 6;

int currentBaudIndex = 0;
unsigned long lastBaudSwitch = 0;
unsigned long lastDataReceived = 0;
unsigned long totalBytesReceived = 0;
unsigned long validNMEASentences = 0;
unsigned long invalidNMEASentences = 0;
unsigned long checksumErrors = 0;

String nmeaBuffer = "";

// Simple checksum validator
bool validateNMEAChecksum(const String& sentence) {
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

void switchBaudRate() {
    currentBaudIndex = (currentBaudIndex + 1) % NUM_BAUD_RATES;
    uint32_t newBaud = BAUD_RATES[currentBaudIndex];
    
    Serial1.end();
    delay(100);
    Serial1.begin(newBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial1.setRxBufferSize(2048);
    
    // Clear any buffered data
    while (Serial1.available()) {
        Serial1.read();
    }
    
    totalBytesReceived = 0;
    validNMEASentences = 0;
    invalidNMEASentences = 0;
    checksumErrors = 0;
    nmeaBuffer = "";
    
    Serial.println();
    Serial.println("========================================");
    Serial.printf("TESTING BAUD RATE: %lu\n", newBaud);
    Serial.println("Waiting for GPS data...");
    Serial.println("========================================");
    
    lastBaudSwitch = millis();
}

void setup() {
    Serial.begin(921600);
    delay(2000);
    
    Serial.println();
    Serial.println("========================================");
    Serial.println("GPS UART DIAGNOSTIC TOOL");
    Serial.println("========================================");
    Serial.println("Testing GPS communication at multiple baud rates");
    Serial.printf("GPS RX Pin: GPIO%d\n", GPS_RX_PIN);
    Serial.printf("GPS TX Pin: GPIO%d\n", GPS_TX_PIN);
    Serial.println();
    Serial.println("Each baud rate will be tested for 15 seconds");
    Serial.println("Look for valid NMEA sentences ($GNGGA, $GNRMC, etc.)");
    Serial.println("========================================");
    Serial.println();
    
    // Start with first baud rate
    switchBaudRate();
}

void loop() {
    unsigned long now = millis();
    
    // Switch baud rate every 15 seconds
    if (now - lastBaudSwitch > 15000) {
        Serial.println();
        Serial.println("--- Test Results ---");
        Serial.printf("Total Bytes: %lu\n", totalBytesReceived);
        Serial.printf("Valid NMEA: %lu\n", validNMEASentences);
        Serial.printf("Invalid NMEA: %lu\n", invalidNMEASentences);
        Serial.printf("Checksum Errors: %lu\n", checksumErrors);
        
        if (totalBytesReceived == 0) {
            Serial.println("❌ NO DATA RECEIVED - Check wiring!");
        } else if (validNMEASentences > 0) {
            Serial.printf("✓ SUCCESS! Valid baud rate: %lu\n", BAUD_RATES[currentBaudIndex]);
        } else {
            Serial.println("⚠ Data received but no valid NMEA - Wrong baud rate");
        }
        
        switchBaudRate();
    }
    
    // Read and process GPS data
    while (Serial1.available()) {
        char c = Serial1.read();
        totalBytesReceived++;
        lastDataReceived = now;
        
        // Echo raw bytes (as hex for debugging)
        if (totalBytesReceived <= 100) {
            Serial.printf("%02X ", (uint8_t)c);
            if (totalBytesReceived % 16 == 0) Serial.println();
        }
        
        // Build NMEA sentences
        if (c == '\n' || c == '\r') {
            if (nmeaBuffer.length() > 0) {
                // Check if it looks like NMEA
                if (nmeaBuffer.charAt(0) == '$' && nmeaBuffer.indexOf('*') > 0) {
                    if (validateNMEAChecksum(nmeaBuffer)) {
                        validNMEASentences++;
                        Serial.println();
                        Serial.printf("[VALID] %s\n", nmeaBuffer.c_str());
                        
                        // Parse and display key data
                        if (nmeaBuffer.indexOf("GGA") >= 0) {
                            // Extract satellite count (field 7)
                            int commaCount = 0;
                            int satStart = 0, satEnd = 0;
                            for (int i = 0; i < nmeaBuffer.length() && commaCount < 8; i++) {
                                if (nmeaBuffer.charAt(i) == ',') {
                                    commaCount++;
                                    if (commaCount == 7) satStart = i + 1;
                                    if (commaCount == 8) satEnd = i;
                                }
                            }
                            if (satEnd > satStart) {
                                String sats = nmeaBuffer.substring(satStart, satEnd);
                                Serial.printf("  → Satellites: %s\n", sats.c_str());
                            }
                        }
                    } else {
                        checksumErrors++;
                        Serial.println();
                        Serial.printf("[CHECKSUM ERROR] %s\n", nmeaBuffer.c_str());
                    }
                } else if (nmeaBuffer.length() > 5) {
                    invalidNMEASentences++;
                    if (invalidNMEASentences <= 5) {
                        Serial.println();
                        Serial.printf("[INVALID] %s\n", nmeaBuffer.c_str());
                    }
                }
                nmeaBuffer = "";
            }
        } else if (c >= 32 && c <= 126) {
            nmeaBuffer += c;
            if (nmeaBuffer.length() > 256) {
                nmeaBuffer = ""; // Prevent overflow
            }
        }
    }
    
    // Status indicator every 5 seconds
    static unsigned long lastStatus = 0;
    if (now - lastStatus > 5000) {
        lastStatus = now;
        if (totalBytesReceived == 0 && now - lastBaudSwitch > 3000) {
            Serial.println("⏳ No data yet... Check GPS is powered and TX is connected to RX");
        }
    }
    
    delay(1);
}
