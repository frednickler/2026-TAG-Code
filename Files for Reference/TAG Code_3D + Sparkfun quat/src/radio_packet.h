#ifndef RADIO_PACKET_H
#define RADIO_PACKET_H

#include <Arduino.h>

// Compact tracking packet for LoRa/Radio transmission (32 bytes)
struct __attribute__((packed)) TrackingPacket {
    uint32_t ms_time;    // 4 bytes: Timestamp (millis) for sync/latency
    int32_t lat;         // 4 bytes: Latitude * 10^7
    int32_t lon;         // 4 bytes: Longitude * 10^7
    int16_t alt;         // 2 bytes: Altitude in meters
    int16_t speed;       // 2 bytes: Ground Speed in cm/s
    uint16_t gps_course; // 2 bytes: GPS Course (Motion) in degrees * 100
    uint16_t imu_heading;// 2 bytes: IMU Heading (Facing) in degrees * 100
    int8_t pitch;        // 1 byte:  Pitch angle in degrees
    int8_t roll;         // 1 byte:  Roll angle in degrees
    uint8_t flags;       // 1 byte:  Bitmask (Bit 0: Valid, Bit 1-2: Fix Type)
    uint8_t reserved[9]; // 9 bytes: Padding to reach 32 bytes (optional, or use for future)
};

// Flags bit definitions
#define PACKET_FLAG_VALID     0x01
#define PACKET_FLAG_FIX_2D    0x02
#define PACKET_FLAG_FIX_3D    0x04

#endif
