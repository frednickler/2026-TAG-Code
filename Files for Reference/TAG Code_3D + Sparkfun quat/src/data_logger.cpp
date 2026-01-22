#include "data_logger.h"

bool DataLogger::_loggingActive = false;
const char* DataLogger::_logFileName = "/tracking_log.csv";

bool DataLogger::init() {
    // LittleFS mounted in main.cpp
    return LittleFS.exists(_logFileName);
}

void DataLogger::startLogging() {
    File f = LittleFS.open(_logFileName, "a");
    if (f) {
        if (f.size() == 0) {
            // Write Header
            f.println("Millis,Lat,Lon,Alt,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ");
        }
        f.close();
        _loggingActive = true;
        Serial.println("[LOG] Logging Started.");
    } else {
        Serial.println("[LOG] Error opening log file!");
    }
}

void DataLogger::stopLogging() {
    _loggingActive = false;
    Serial.println("[LOG] Logging Stopped.");
}

bool DataLogger::isLogging() {
    return _loggingActive;
}

void DataLogger::log(const LogPacket& packet) {
    if (!_loggingActive) return;

    File f = LittleFS.open(_logFileName, "a"); // Append mode
    if (f) {
        f.printf("%lu,%.6f,%.6f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            packet.timestamp,
            packet.lat, packet.lon, packet.alt,
            packet.accX, packet.accY, packet.accZ,
            packet.gyroX, packet.gyroY, packet.gyroZ,
            packet.magX, packet.magY, packet.magZ
        );
        f.close();
    }
}

void DataLogger::dumpLogToSerial() {
    if (!LittleFS.exists(_logFileName)) {
        Serial.println("[LOG] No log file found.");
        return;
    }

    File f = LittleFS.open(_logFileName, "r");
    if (!f) {
        Serial.println("[LOG] Failed to open log file.");
        return;
    }

    Serial.println("\n--- BEGIN LOG DUMP ---");
    while (f.available()) {
        Serial.write(f.read());
    }
    Serial.println("\n--- END LOG DUMP ---");
    f.close();
}

void DataLogger::clearLog() {
    if (LittleFS.remove(_logFileName)) {
        Serial.println("[LOG] Log file cleared.");
    } else {
        Serial.println("[LOG] Failed to clear log file (or it didn't exist).");
    }
}

size_t DataLogger::getLogSize() {
    if (LittleFS.exists(_logFileName)) {
        File f = LittleFS.open(_logFileName, "r");
        size_t s = f.size();
        f.close();
        return s;
    }
    return 0;
}
