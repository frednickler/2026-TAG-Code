#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <Arduino.h>
#include <LittleFS.h>
#include "gps_module.h" // For GPSData struct

// Structure to hold one frame of sensor data
struct LogPacket {
    unsigned long timestamp;
    double lat;
    double lon;
    int32_t alt;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
};

class DataLogger {
public:
    static bool init();
    static void startLogging();
    static void stopLogging();
    static bool isLogging();
    static void log(const LogPacket& packet);
    static void dumpLogToSerial();
    static void clearLog();
    static size_t getLogSize();

private:
    static bool _loggingActive;
    static const char* _logFileName;
};

#endif
