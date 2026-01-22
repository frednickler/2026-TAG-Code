#ifndef RADIO_MANAGER_H
#define RADIO_MANAGER_H

#include <Arduino.h>
#include <esp_now.h>

// =============================================================================
// ENUMS
// =============================================================================

enum class RadioRole : uint8_t {
    BASE = 0,   // Receiver - listens for TAG data
    TAG = 1     // Transmitter - sends GPS/heading data
};

enum class TxPower : uint8_t {
    POWER_LOW = 0,    // 11 dBm (~100m range)
    POWER_MEDIUM = 1, // 15 dBm (~200m range)
    POWER_HIGH = 2    // 20 dBm (~300m+ range)
};

enum class DataRate : uint8_t {
    RATE_1M = 0,    // 1 Mbps - best range (default)
    RATE_2M = 1,    // 2 Mbps
    RATE_5M = 2,    // 5.5 Mbps
    RATE_11M = 3    // 11 Mbps - shortest range
};

// =============================================================================
// PACKET TYPES
// =============================================================================

#define PACKET_TYPE_GPS_DATA        0x01
#define PACKET_TYPE_ACK             0x02
#define PACKET_TYPE_PAIR_BEACON     0x10
#define PACKET_TYPE_PAIR_REQUEST    0x11
#define PACKET_TYPE_PAIR_CONFIRM    0x12

// =============================================================================
// PACKET STRUCTURES
// =============================================================================

// TAG -> BASE: GPS/Heading data (uses double for 9-decimal precision)
struct TagPacket {
    uint8_t  type;        // PACKET_TYPE_GPS_DATA
    double   latitude;    // degrees (9 decimal places)
    double   longitude;   // degrees (9 decimal places)
    uint16_t heading;     // degrees (0-360, integer)
    uint32_t timestamp;   // millis() on TAG
} __attribute__((packed)); // 23 bytes

// BASE -> TAG: Acknowledgment
struct AckPacket {
    uint8_t  type;           // PACKET_TYPE_ACK
    uint32_t echoTimestamp;  // Echo back TAG's timestamp for latency calc
} __attribute__((packed)); // 5 bytes

// Pairing handshake packets
struct PairingPacket {
    uint8_t  type;        // PAIR_BEACON, PAIR_REQUEST, or PAIR_CONFIRM
    uint8_t  mac[6];      // Sender's MAC address
    uint8_t  channel;     // Current WiFi channel
    uint8_t  role;        // RadioRole of sender
} __attribute__((packed)); // 10 bytes

// =============================================================================
// RADIO MANAGER CLASS
// =============================================================================

class RadioManager {
public:
    // Lifecycle
    static bool init();
    static void update();
    static void deinit();
    
    // Configuration
    static void setRole(RadioRole role);
    static void setChannel(uint8_t channel);
    static void setTxPower(TxPower power);
    static void setDataRate(DataRate rate);
    static void setAckEnabled(bool enabled);
    static void setTagTxRate(uint8_t hz);
    static void setEnabled(bool enabled);
    
    // MAC Address Management
    static void setTargetMAC(const uint8_t* mac);
    static void getTargetMAC(uint8_t* mac);
    static void getMyMAC(uint8_t* mac);
    static void resetTobroadcast();
    static bool isBroadcast();
    
    // Pairing
    static void startPairingScan();
    static void stopPairingScan();
    static bool isPairingActive();
    static uint8_t getPairedDeviceCount();
    static void getLastDiscoveredMAC(uint8_t* mac);
    
    // TAG Mode: Send data
    static bool sendTagData(double lat, double lon, uint16_t heading);
    
    // BASE Mode: Get received data
    static bool hasNewData();
    static double getTagLatitude();
    static double getTagLongitude();
    static uint16_t getTagHeading();
    static uint32_t getTagTimestamp();
    static int getSecondsSinceLastPacket();
    static bool isConnected();
    
    // Statistics
    static int8_t getLastRSSI();
    static uint32_t getPacketsReceived();
    static uint32_t getPacketsSent();
    static uint32_t getPacketsLost();
    
    // Status
    static bool isInitialized();
    static bool isEnabled();
    static RadioRole getRole();
    static uint8_t getChannel();
    
    // Callbacks (internal use, but public for ESP-NOW C API)
    static void onReceiveCallback(const uint8_t* mac, const uint8_t* data, int len);
    static void onSendCallback(const uint8_t* mac, esp_now_send_status_t status);

private:
    static bool initialized;
    static bool enabled;
    static RadioRole role;
    static uint8_t channel;
    static TxPower txPower;
    static DataRate dataRate;
    static bool ackEnabled;
    static uint8_t tagTxRate;
    static uint8_t targetMAC[6];
    static uint8_t myMAC[6];
    
    // Received data (BASE mode)
    static double rxLatitude;
    static double rxLongitude;
    static uint16_t rxHeading;
    static uint32_t rxTimestamp;
    static uint32_t lastRxTime;
    static bool newDataFlag;
    
    // Pairing state
    static bool pairingActive;
    static uint32_t pairingStartTime;
    static uint8_t lastDiscoveredMAC[6];
    static uint8_t pairedDeviceCount;
    
    // Statistics
    static int8_t lastRSSI;
    static uint32_t packetsReceived;
    static uint32_t packetsSent;
    static uint32_t packetsLost;
    
    // TAG mode timing
    static uint32_t lastTagTxTime;
    
    // Internal helpers
    static bool applyWiFiSettings();
    static bool addPeer(const uint8_t* mac);
    static void sendAck(const uint8_t* mac, uint32_t timestamp);
    static void sendPairingBeacon();
    static void handlePairingPacket(const uint8_t* mac, const PairingPacket* pkt);
};

#endif // RADIO_MANAGER_H
