#include "NMEAParser.h"

bool NMEAParser::parseSentence(const String& sentence, NMEAData& data) {
    if (sentence.length() < 10) return false;
    if (sentence.charAt(0) != '$') return false;
    
    // Validate checksum
    if (!validateChecksum(sentence)) return false;
    
    // Determine sentence type
    if (sentence.indexOf("GGA") >= 0) {
        return parseGGA(sentence, data);
    } else if (sentence.indexOf("RMC") >= 0) {
        return parseRMC(sentence, data);
    } else if (sentence.indexOf("VTG") >= 0) {
        return parseVTG(sentence, data);
    }
    
    return false;
}

bool NMEAParser::parseGGA(const String& sentence, NMEAData& data) {
    // $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    // Format: $GNGGA,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,...
    
    int commaPos[15];
    int commaCount = 0;
    
    // Find all commas
    for (int i = 0; i < sentence.length() && commaCount < 15; i++) {
        if (sentence.charAt(i) == ',') {
            commaPos[commaCount++] = i;
        }
    }
    
    if (commaCount < 10) return false;
    
    // Extract fields
    String lat = sentence.substring(commaPos[1] + 1, commaPos[2]);
    String latDir = sentence.substring(commaPos[2] + 1, commaPos[3]);
    String lon = sentence.substring(commaPos[3] + 1, commaPos[4]);
    String lonDir = sentence.substring(commaPos[4] + 1, commaPos[5]);
    String quality = sentence.substring(commaPos[5] + 1, commaPos[6]);
    String sats = sentence.substring(commaPos[6] + 1, commaPos[7]);
    String alt = sentence.substring(commaPos[8] + 1, commaPos[9]);
    
    // Parse data
    if (lat.length() > 0 && lon.length() > 0) {
        data.latitude = parseCoordinate(lat, latDir);
        data.longitude = parseCoordinate(lon, lonDir);
    }
    
    data.fixQuality = quality.toInt();
    data.fixValid = (data.fixQuality > 0);
    data.satellites = sats.toInt();
    data.altitude = alt.toFloat();
    data.lastUpdate = millis();
    
    return true;
}

bool NMEAParser::parseRMC(const String& sentence, NMEAData& data) {
    // $GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    // Format: $GNRMC,time,status,lat,N/S,lon,E/W,speed,course,date,...
    
    int commaPos[15];
    int commaCount = 0;
    
    for (int i = 0; i < sentence.length() && commaCount < 15; i++) {
        if (sentence.charAt(i) == ',') {
            commaPos[commaCount++] = i;
        }
    }
    
    if (commaCount < 8) return false;
    
    String status = sentence.substring(commaPos[1] + 1, commaPos[2]);
    String speed = sentence.substring(commaPos[6] + 1, commaPos[7]);
    String course = sentence.substring(commaPos[7] + 1, commaPos[8]);
    
    if (status == "A") { // Active/Valid
        data.speed = speed.toFloat() * 0.514444; // knots to m/s
        data.heading = course.toFloat();
    }
    
    return true;
}

bool NMEAParser::parseVTG(const String& sentence, NMEAData& data) {
    // $GNVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
    // Format: $GNVTG,course_true,T,course_mag,M,speed_knots,N,speed_kph,K
    
    int commaPos[10];
    int commaCount = 0;
    
    for (int i = 0; i < sentence.length() && commaCount < 10; i++) {
        if (sentence.charAt(i) == ',') {
            commaPos[commaCount++] = i;
        }
    }
    
    if (commaCount < 8) return false;
    
    String course = sentence.substring(commaPos[0] + 1, commaPos[1]);
    String speedKph = sentence.substring(commaPos[6] + 1, commaPos[7]);
    
    if (course.length() > 0) {
        data.heading = course.toFloat();
    }
    if (speedKph.length() > 0) {
        data.speed = speedKph.toFloat() / 3.6; // kph to m/s
    }
    
    return true;
}

double NMEAParser::parseCoordinate(const String& coord, const String& dir) {
    if (coord.length() < 4) return 0.0;
    
    // NMEA format: DDMM.MMMMM or DDDMM.MMMMM
    int dotPos = coord.indexOf('.');
    if (dotPos < 3) return 0.0;
    
    // Extract degrees and minutes
    String degreesStr = coord.substring(0, dotPos - 2);
    String minutesStr = coord.substring(dotPos - 2);
    
    double degrees = degreesStr.toDouble();
    double minutes = minutesStr.toDouble();
    
    // Convert to decimal degrees
    double decimal = degrees + (minutes / 60.0);
    
    // Apply direction
    if (dir == "S" || dir == "W") {
        decimal = -decimal;
    }
    
    return decimal;
}

uint8_t NMEAParser::calculateChecksum(const String& sentence) {
    uint8_t checksum = 0;
    
    // Start after '$', end before '*'
    int startPos = 1;
    int endPos = sentence.indexOf('*');
    
    if (endPos < 0) endPos = sentence.length();
    
    for (int i = startPos; i < endPos; i++) {
        checksum ^= sentence.charAt(i);
    }
    
    return checksum;
}

bool NMEAParser::validateChecksum(const String& sentence) {
    int asteriskPos = sentence.indexOf('*');
    if (asteriskPos < 0) return false; // No checksum
    
    // Extract checksum from sentence
    String checksumStr = sentence.substring(asteriskPos + 1, asteriskPos + 3);
    uint8_t expectedChecksum = strtoul(checksumStr.c_str(), NULL, 16);
    
    // Calculate actual checksum
    uint8_t actualChecksum = calculateChecksum(sentence);
    
    return (actualChecksum == expectedChecksum);
}
