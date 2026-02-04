#include "RadioManager.h"
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include "config/SystemSettings.h"

// Define Verbose Debugging
#define RADIO_DEBUG_VERBOSE 1

// =============================================================================
// RADIO MANAGER STATIC MEMBERS
// =============================================================================

// Broadcast address = All FF
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Internal State
bool RadioManager::initialized = false;
bool RadioManager::enabled = true;
const RadioRole RadioManager::defaultRole = RadioRole::BASE;
RadioRole RadioManager::role = RadioRole::BASE;
uint8_t RadioManager::channel = 1; // LR Mode on Channel 1
uint8_t RadioManager::tagTxRate = 1;
uint8_t RadioManager::targetMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // BROADCAST (MVE Method)
uint8_t RadioManager::myMAC[6] = {0};

// Telemetry Data (Getters support)
volatile unsigned long RadioManager::lastPacketTime = 0;
volatile int RadioManager::rssi = 0;
int RadioManager::rxPacketCount = 0;
int RadioManager::txPacketCount = 0;
int RadioManager::txSuccessCount = 0;
int RadioManager::txFailCount = 0;
TagPacket RadioManager::lastTagPacket;
bool RadioManager::myDataFlag = false; // Renamed from newDataFlag to match getter if needed? No, getter uses newDataFlag?
// checking getter: bool RadioManager::hasNewData() { return newDataFlag; } 
// Let's declare variable names matching the getters errors seen in log.
// Log said: bool RadioManager::hasNewData() { return newDataFlag; }
// So variable is likely newDataFlag.
bool RadioManager::newDataFlag = false;

double RadioManager::rxLatitude = 0;
double RadioManager::rxLongitude = 0;
uint16_t RadioManager::rxHeading = 0;
uint32_t RadioManager::rxTimestamp = 0;
bool RadioManager::rxGpsValid = false;
unsigned long RadioManager::lastRxTime = 0;
unsigned long RadioManager::lastTagTxTime = 0;

uint32_t RadioManager::packetsSent = 0;
uint32_t RadioManager::packetsReceived = 0;
uint32_t RadioManager::packetsLost = 0;
int RadioManager::lastRSSI = 0;
int RadioManager::lastPacketLength = 0;


// =============================================================================
// NATIVE CALLBACKS (C-Style)
// =============================================================================
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *data, int len) {
    #if RADIO_DEBUG_VERBOSE
    Serial.printf("\n[RX] Msg from " MACSTR " len=%d RSSI=%d\n", 
                  MAC2STR(info->src_addr), len, info->rx_ctrl->rssi);
    #endif
    
    // Pass to RadioManager
    RadioManager::processPacket(data, len, info->rx_ctrl->rssi);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    RadioManager::updateTxStats(status == ESP_NOW_SEND_SUCCESS);
}

// =============================================================================
// INIT (Native API Implementation)
// =============================================================================
bool RadioManager::init() {
    if (initialized) {
        Serial.println("[RadioManager] Already initialized");
        return true;
    }

    Serial.println("");
    Serial.println("╔══════════════════════════════════════════════════════════════════╗");
    Serial.println("║            RADIO MANAGER INITIALIZATION (NATIVE API)            ║");
    Serial.println("╚══════════════════════════════════════════════════════════════════╝");

    // Load Settings
    RuntimeConfig& cfg = SystemSettings::getConfig();
    role = static_cast<RadioRole>(cfg.radioRole);
    channel = 1; // LR Mode
    
    Serial.printf("│  ├─ Role: %s\n", (role == RadioRole::BASE) ? "BASE (Receiver)" : "TAG (Sender)");
    Serial.printf("│  └─ Channel: %d (Hardcoded)\n", channel);

    // Initialize WiFi
    WiFi.mode(WIFI_STA);
    
    // FORCE CHANNEL (MVE METHOD)
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    
    // VERIFY CHANNEL
    Serial.printf("  ✓ WiFi Channel Solidiied: %d (Actual: %d)\n", channel, WiFi.channel());
    
    // DISABLE POWER SAVE (CRITICAL)
    esp_wifi_set_ps(WIFI_PS_NONE);

    // SET MAX TX POWER (20dBm)
    esp_wifi_set_max_tx_power(80); 

    // FORCE LONG RANGE PROTOCOL (MVE Verified)
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    
    // Wait for WiFi
    while (!WiFi.STA.started()) delay(100);
    
    WiFi.macAddress(myMAC);
    Serial.printf("  MAC Address: " MACSTR "\n", MAC2STR(myMAC));
    
    // Verify Power Save
    wifi_ps_type_t ps_type;
    int retryCount = 0;
    do {
        esp_wifi_set_ps(WIFI_PS_NONE);
        delay(10);
        esp_wifi_get_ps(&ps_type);
        if (ps_type != WIFI_PS_NONE) {
            Serial.printf("  ⚠️ Power Save ENABLED! Retrying... (%d)\n", ++retryCount);
            if (retryCount > 5) {
                Serial.println("  ❌ COULD NOT DISABLE POWER SAVE! (Continuing anyway...)");
                break;
            }
        }
    } while (ps_type != WIFI_PS_NONE);
    
    if (ps_type == WIFI_PS_NONE) {
         Serial.println("  ✓ Power Save DISABLED");
    }

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("└─ ❌ esp_now_init() FAILED!");
        return false;
    }
    Serial.println("│  ✓ esp_now_init() Success");

    // Register Callbacks
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);

    // Register Peer (Broadcast)
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, targetMAC, 6); // Use TARGET MAC for Peer
    peerInfo.channel = channel;  
    peerInfo.ifidx = WIFI_IF_STA; 
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("└─ ❌ Failed to add broadcast peer");
        return false;
    }
    Serial.println("└─ ✓ Broadcast Peer Added");
    
    initialized = true;
    return true;
}

// =============================================================================
// MAIN UPDATE LOOP
// =============================================================================
void RadioManager::update() {
    // nothing to do for receiver, driven by callbacks
}

// =============================================================================
// TRANSMISSION
// =============================================================================
bool RadioManager::sendTagData(float lat, float lon, uint16_t heading, bool gpsValid) {
    if (!initialized) return false;

    TagPacket pkt;
    pkt.packetType = PACKET_TYPE_GPS_DATA;
    pkt.latitude = lat;
    pkt.longitude = lon;
    pkt.heading = heading;
    pkt.gpsValid = gpsValid; // Using the bool logic here, but struct might define it as uint8_t.
    // In Header: uint8_t gpsValid;
    // Assignment bool -> uint8_t is fine.
    pkt.timestamp = millis();

    #if RADIO_DEBUG_VERBOSE
    Serial.printf("[TX] GPS: Lat=%.6f Lon=%.6f Hdg=%d Valid=%d\n", lat, lon, heading, gpsValid);
    #endif

    // Native Send UNICAST
    esp_err_t result = esp_now_send(targetMAC, (uint8_t*)&pkt, sizeof(pkt));
    
    if (result != ESP_OK) {
        Serial.printf("[TX] Error: 0x%x\n", result);
        updateTxStats(false);
        return false;
    }
    return true;
}

void RadioManager::updateTxStats(bool success) {
    if (success) {
        txSuccessCount++;
        packetsSent++;
    } else {
        txFailCount++;
        packetsLost++;
    }
}

// =============================================================================
// PACKET PROCESSING
// =============================================================================
void RadioManager::processPacket(const uint8_t* data, size_t len, int rssiVal) {
    rssi = rssiVal;
    lastRSSI = rssiVal;
    lastRxTime = millis();
    lastPacketLength = len;
    
    Serial.printf("[RX] PARSING NATIVE PACKET len=%d\n", len);
    
    if (len < 1) {
        Serial.println("     └─ ERROR: Packet too small");
        return;
    }
    
    uint8_t packetType = data[0];
    if (packetType != PACKET_TYPE_GPS_DATA) return;
    
    if (len != sizeof(TagPacket)) {
        Serial.printf("     └─ ERROR: Size mismatch %d vs %d\n", len, sizeof(TagPacket));
        return;
    }
    
    memcpy(&lastTagPacket, data, sizeof(TagPacket));
    
    // Update internal state
    rxLatitude = lastTagPacket.latitude;
    rxLongitude = lastTagPacket.longitude;
    rxHeading = lastTagPacket.heading;
    rxGpsValid = lastTagPacket.gpsValid;
    rxTimestamp = lastTagPacket.timestamp;
    newDataFlag = true;
    rxPacketCount++;
    packetsReceived++;
    
    Serial.printf("     └─ ✓ DATA: Lat=%.6f Lon=%.6f Hdg=%d\n", rxLatitude, rxLongitude, rxHeading);
}

// =============================================================================
// GETTERS
// =============================================================================
uint32_t RadioManager::getPacketsReceived() { return rxPacketCount; } // Fixed variable name mismatch
uint32_t RadioManager::getPacketsSent() { return packetsSent; }
uint32_t RadioManager::getPacketsLost() { return packetsLost; }
int RadioManager::getLastPacketLength() { return lastPacketLength; }
bool RadioManager::isInitialized() { return initialized; }
bool RadioManager::hasNewData() { return newDataFlag; }

double RadioManager::getTagLatitude() { 
    newDataFlag = false; 
    return rxLatitude; 
}
double RadioManager::getTagLongitude() { return rxLongitude; }
uint16_t RadioManager::getTagHeading() { return rxHeading; }
bool RadioManager::getTagGPSValid() { return rxGpsValid; }
uint32_t RadioManager::getTagTimestamp() { return rxTimestamp; }
unsigned long RadioManager::getLastRxTime() { return lastRxTime; }
int8_t RadioManager::getLastRSSI() { return (int8_t)rssi; }
bool RadioManager::isConnected() { return (millis() - lastRxTime) < 5000; }
bool RadioManager::isEnabled() { return enabled; }

// Dummy implementations for compile compatibility
void RadioManager::deinit() { initialized=false; }
void RadioManager::setRole(RadioRole r) { role = r; }
void RadioManager::setChannel(uint8_t c) { channel = c; }
void RadioManager::setTxPower(TxPower p) {}
void RadioManager::setDataRate(DataRate r) {}
void RadioManager::setAckEnabled(bool e) {}
void RadioManager::setTagTxRate(uint8_t r) {}
void RadioManager::setEnabled(bool e) {}
void RadioManager::setTargetMAC(const uint8_t* m) {}
void RadioManager::sendAck(const uint8_t* mac, uint32_t ts) {}
void RadioManager::getTargetMAC(uint8_t* mac) { 
    memcpy(mac, targetMAC, 6); 
}
void RadioManager::getMyMAC(uint8_t* mac) { memcpy(mac, myMAC, 6); }
void RadioManager::resetTobroadcast() {}
bool RadioManager::isBroadcast() { 
    return (targetMAC[0] == 0xFF); 
}
void RadioManager::startPairingScan() {}
void RadioManager::stopPairingScan() {}
bool RadioManager::isPairingActive() { return false; }
uint8_t RadioManager::getPairedDeviceCount() { return 0; }
void RadioManager::getLastDiscoveredMAC(uint8_t* mac) {}
int RadioManager::getSecondsSinceLastPacket() { return (millis() - lastRxTime) / 1000; }
void RadioManager::onReceiveCallback(const uint8_t* mac, const uint8_t* data, int len) {}
void RadioManager::onSendCallback(const uint8_t* mac, esp_now_send_status_t status) {}
