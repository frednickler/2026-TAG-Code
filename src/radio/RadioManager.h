#ifndef RADIOMANAGER_H
#define RADIOMANAGER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// =============================================================================
// ENUMS
// =============================================================================
enum class RadioRole {
    BASE = 0,
    TAG = 1
};

enum class TxPower {
    PWR_LOW = 0,
    PWR_MEDIUM = 1,
    PWR_HIGH = 2,
    PWR_MAX = 3
};

enum class DataRate {
    // Legacy placeholders
    RATE_250KBPS = 0,
    RATE_1MBPS = 1,
    RATE_2MBPS = 2
};

// =============================================================================
// PACKET STRUCTURE
// =============================================================================
#pragma pack(push, 1)
enum PacketType {
    PACKET_TYPE_GPS_DATA = 0x01,
    PACKET_TYPE_CONFIG = 0x02,
    PACKET_TYPE_ACK = 0x03
};

struct TagPacket {
    uint8_t packetType;       // 1 byte
    uint8_t gpsValid;         // 1 byte (MOVED HERE TO MATCH TAG)
    double latitude;          // 8 bytes
    double longitude;         // 8 bytes
    uint16_t heading;         // 2 bytes
    uint32_t timestamp;       // 4 bytes
};
#pragma pack(pop)

// =============================================================================
// RADIO MANAGER CLASS
// =============================================================================
class RadioManager {
public:
    // Core Functions
    static bool init();
    static void update();
    static void deinit();

    // Configuration
    static void setRole(RadioRole role);
    static void setChannel(uint8_t channel);
    static void setTxPower(TxPower power);
    static void setDataRate(DataRate rate);
    static void setAckEnabled(bool enabled);
    static void setTagTxRate(uint8_t rate); // Hz
    static void setEnabled(bool enabled);
    static void setTargetMAC(const uint8_t* mac);
    
    // Transmission
    static bool sendTagData(float lat, float lon, uint16_t heading, bool gpsValid); // FIXED SIGNATURE
    static void sendAck(const uint8_t* targetMac, uint32_t packetTimestamp);

    // Getters
    static bool isInitialized();
    static bool isConnected();
    static bool isEnabled();
    static bool hasNewData();
    static void getTargetMAC(uint8_t* mac);
    static void getMyMAC(uint8_t* mac);
    
    // Telemetry getters
    static double getTagLatitude();
    static double getTagLongitude();
    static uint16_t getTagHeading();
    static bool getTagGPSValid();
    static uint32_t getTagTimestamp();
    static unsigned long getLastRxTime();
    static int8_t getLastRSSI();
    static uint32_t getPacketsSent();
    static uint32_t getPacketsReceived();
    static uint32_t getPacketsLost();
    static int getLastPacketLength();

    // Advanced Peer Management (Placeholders for compatibility)
    static void resetTobroadcast();
    static bool isBroadcast();
    static void startPairingScan();
    static void stopPairingScan();
    static bool isPairingActive();
    static uint8_t getPairedDeviceCount(); 
    static void getLastDiscoveredMAC(uint8_t* mac);
    static int getSecondsSinceLastPacket();

    // Internal Callback Handlers (Public for C callbacks to access)
    static void processPacket(const uint8_t* data, size_t len, int rssiVal); // FIXED SIGNATURE
    static void updateTxStats(bool success);

    // Callbacks (Optional)
    static void onReceiveCallback(const uint8_t* mac, const uint8_t* data, int len);
    static void onSendCallback(const uint8_t* mac, esp_now_send_status_t status);

    // Static State Members (MUST BE PUBLIC or FRIEND for C callbacks if using getters, but here simple public static)
    // To solve linker errors, we declare them here.
    static const RadioRole defaultRole;
    static RadioRole role;
    static uint8_t channel;
    static bool initialized;
    static bool enabled;
    static uint8_t tagTxRate;
    static uint8_t targetMAC[6];
    static uint8_t myMAC[6];
    
    // Telemetry
    static volatile unsigned long lastPacketTime;
    static volatile int rssi;
    static int rxPacketCount;
    static int txPacketCount;
    static int txSuccessCount;
    static int txFailCount;
    
    static TagPacket lastTagPacket;
    static bool newDataFlag; // Matches source usage
    static bool myDataFlag;  // Matches source usage if duplicate? We used newDataFlag in source.
    
    static double rxLatitude;
    static double rxLongitude;
    static uint16_t rxHeading;
    static uint32_t rxTimestamp;
    static bool rxGpsValid;
    static unsigned long lastRxTime;
    static unsigned long lastTagTxTime;
    
    static uint32_t packetsSent;
    static uint32_t packetsReceived;
    static uint32_t packetsLost;
    static int lastRSSI;
    static int lastPacketLength;

private:
    RadioManager() = delete; // Static class
};

#endif // RADIOMANAGER_H
