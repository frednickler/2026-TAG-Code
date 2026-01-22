#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <Arduino.h>

/**
 * @brief Simple NMEA sentence parser for debugging/verification
 * 
 * This is a lightweight parser for basic GPS data extraction.
 * Production code will use optimized binary protocol.
 */
struct NMEAData {
    // Position
    double latitude = 0.0;
    double longitude = 0.0;
    float altitude = 0.0;
    
    // Fix quality
    bool fixValid = false;
    uint8_t fixQuality = 0;  // 0=invalid, 1=GPS, 2=DGPS
    uint8_t satellites = 0;
    
    // Velocity
    float speed = 0.0;        // m/s
    float heading = 0.0;      // degrees
    
    // Time
    unsigned long lastUpdate = 0;
};

class NMEAParser {
public:
    /**
     * @brief Parse a single NMEA sentence
     * @param sentence The NMEA sentence (e.g., "$GNGGA,...")
     * @param data Output data structure to update
     * @return true if sentence was successfully parsed
     */
    static bool parseSentence(const String& sentence, NMEAData& data);

private:
    static bool parseGGA(const String& sentence, NMEAData& data);
    static bool parseRMC(const String& sentence, NMEAData& data);
    static bool parseVTG(const String& sentence, NMEAData& data);
    
    static double parseCoordinate(const String& coord, const String& dir);
    static uint8_t calculateChecksum(const String& sentence);
    static bool validateChecksum(const String& sentence);
};

#endif // NMEA_PARSER_H
