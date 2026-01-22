#include "GPSManager.h"

// Initialize static members
GPSData GPSManager::data = {};
bool GPSManager::initialized = false;
unsigned long GPSManager::lastUpdate = 0;
String GPSManager::nmeaBuffer = "";
NMEAData GPSManager::tempParserData = {};

bool GPSManager::init() {
    DEBUG_INFO("Initializing GPSManager (NMEA Mode)...");
    
    // Initialize Serial Port
    // RX=16, TX=17, Baud=38400 (Standard for M5Stack GNSS Unit)
    GPS_SERIAL.setRxBufferSize(1024); // Large buffer for bursts
    GPS_SERIAL.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    // Give valid "Initialized" status immediately because we can't query the device
    // If it sends data, we process it. If not, "lastUpdate" stays old.
    initialized = true;
    
    // Clear buffer
    while (GPS_SERIAL.available()) {
        GPS_SERIAL.read();
    }
    
    DEBUG_INFO("GPS UART initialized at %d baud", GPS_BAUD_RATE);
    return true;
}

void GPSManager::update() {
    if (!initialized) return;
    
    int bytesProcessed = 0;
    while (GPS_SERIAL.available() && bytesProcessed < 2048) { // Safety limit
        char c = GPS_SERIAL.read();
        bytesProcessed++;
        
        // Basic line buffering
        if (c == '\n' || c == '\r') {
            if (nmeaBuffer.length() > 0) {
                // We have a sentence
                if (NMEAParser::parseSentence(nmeaBuffer, tempParserData)) {
                    // Packet parsed successfully, update internal state
                    if (tempParserData.fixValid) {
                        data.latitude = tempParserData.latitude;
                        data.longitude = tempParserData.longitude;
                        data.altitude = tempParserData.altitude;
                        data.satellites = tempParserData.satellites;
                        data.fixValid = true;
                        
                        // Update timing
                        data.lastUpdate = millis();
                        lastUpdate = millis();
                    }
                    
                    // Always update these if available (even without fix sometimes)
                    if (tempParserData.heading > 0) data.heading = tempParserData.heading;
                    if (tempParserData.speed > 0) data.speed = tempParserData.speed;
                    data.satellites = tempParserData.satellites; // Update sats even if no fix
                }
                nmeaBuffer = ""; // Clear buffer
            }
        } else if (c >= 32 && c <= 126) {
            // Valid ASCII char
            nmeaBuffer += c;
            if (nmeaBuffer.length() > 256) {
                nmeaBuffer = ""; // Prevent overflow
            }
        }
    }
    
    // Debug print if we hit the limit
    if (bytesProcessed >= 2048) {
        DEBUG_WARN("GPS RX Flood: Processed 2048 bytes in one loop!");
    }
}

bool GPSManager::hasFix() {
    if (!initialized) return false;
    // Check if data is stale (older than 10 seconds)
    if (millis() - lastUpdate > 10000) return false;
    return data.fixValid;
}

bool GPSManager::isAvailable() {
    return initialized;
}

// Data Accessors
double GPSManager::getLatitude() { return data.latitude; }
double GPSManager::getLongitude() { return data.longitude; }
float GPSManager::getAltitudeMSL() { return data.altitude; }
float GPSManager::getSpeed() { return data.speed; }
float GPSManager::getHeading() { return data.heading; }
uint8_t GPSManager::getSatelliteCount() { return data.satellites; }

bool GPSManager::checkHealth() {
    // Return true if we have received ANY valid update in the last 5 seconds
    return (millis() - lastUpdate < 5000);
}

// Configuration Stubs (Cannot reliably configure if RX is broken)
void GPSManager::setRefreshRate(uint8_t hz) {
    // Attempt to send NMEA command to change rate? 
    // For u-blox, it's complex binary or proprietary NMEA.
    // Given the issues, we accept default 1Hz.
    DEBUG_WARN("GPS setRefreshRate not supported in NMEA Fallback Mode");
}

void GPSManager::setConstellation(uint8_t preset) {
    DEBUG_WARN("GPS setConstellation not supported in NMEA Fallback Mode");
}

void GPSManager::setDynamicModel(uint8_t model) {
    DEBUG_WARN("GPS setDynamicModel not supported in NMEA Fallback Mode");
}

void GPSManager::setSBAS(bool enable) {
    DEBUG_WARN("GPS setSBAS not supported in NMEA Fallback Mode");
}

void GPSManager::setQZSS(bool enable) {
    DEBUG_WARN("GPS setQZSS not supported in NMEA Fallback Mode");
}

void GPSManager::setAntiJamming(uint8_t mode) {
    DEBUG_WARN("GPS setAntiJamming not supported in NMEA Fallback Mode");
}
