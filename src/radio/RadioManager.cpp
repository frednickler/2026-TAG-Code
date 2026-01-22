#include "RadioManager.h"
#include "../config/SystemConfig.h"
#include "../config/SystemSettings.h"
#include "../system/Watchdog.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// =============================================================================
// STATIC MEMBER INITIALIZATION
// =============================================================================
bool RadioManager::initialized = false;
bool RadioManager::enabled = false;
RadioRole RadioManager::role = RadioRole::BASE;
uint8_t RadioManager::channel = 1;
TxPower RadioManager::txPower = TxPower::POWER_HIGH;
DataRate RadioManager::dataRate = DataRate::RATE_1M;
bool RadioManager::ackEnabled = true;
uint8_t RadioManager::tagTxRate = 1;
uint8_t RadioManager::targetMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t RadioManager::myMAC[6] = {0};

// Received data
double RadioManager::rxLatitude = 0.0;
double RadioManager::rxLongitude = 0.0;
uint16_t RadioManager::rxHeading = 0;
uint32_t RadioManager::rxTimestamp = 0;
uint32_t RadioManager::lastRxTime = 0;
bool RadioManager::newDataFlag = false;

// Pairing state
bool RadioManager::pairingActive = false;
uint32_t RadioManager::pairingStartTime = 0;
uint8_t RadioManager::lastDiscoveredMAC[6] = {0};
uint8_t RadioManager::pairedDeviceCount = 0;

// Statistics
int8_t RadioManager::lastRSSI = 0;
uint32_t RadioManager::packetsReceived = 0;
uint32_t RadioManager::packetsSent = 0;
uint32_t RadioManager::packetsLost = 0;

// TAG mode timing
uint32_t RadioManager::lastTagTxTime = 0;

// =============================================================================
// LIFECYCLE
// =============================================================================

bool RadioManager::init() {
    if (initialized) {
        return true;
    }
    
    DEBUG_INFO("Initializing RadioManager (ESP-NOW)...");
    
    // Load settings from NVS
    RuntimeConfig& cfg = SystemSettings::getConfig();
    role = static_cast<RadioRole>(cfg.radioRole);
    channel = cfg.radioChannel;
    txPower = static_cast<TxPower>(cfg.radioTxPower);
    dataRate = static_cast<DataRate>(cfg.radioDataRate);
    ackEnabled = cfg.radioAckEnabled;
    tagTxRate = cfg.radioTagTxRate;
    memcpy(targetMAC, cfg.radioTargetMAC, 6);
    enabled = cfg.radioEnabled;
    
    if (!enabled) {
        DEBUG_INFO("  Radio is DISABLED in settings");
        return true; // Not an error, just disabled
    }
    
    // Initialize WiFi in Station mode (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(); // Don't connect to AP, just use for ESP-NOW
    
    // Set channel
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    delay(10); // Stabilize WiFi radio
    
    // Get our MAC address
    WiFi.macAddress(myMAC);
    DEBUG_INFO("  My MAC: %02X:%02X:%02X:%02X:%02X:%02X",
              myMAC[0], myMAC[1], myMAC[2], myMAC[3], myMAC[4], myMAC[5]);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        DEBUG_ERROR("ESP-NOW initialization failed!");
        return false;
    }
    
    // Register callbacks
    esp_now_register_recv_cb(onReceiveCallback);
    esp_now_register_send_cb(onSendCallback);
    
    // Apply TX power and data rate
    applyWiFiSettings();
    
    // Add broadcast peer (always needed for pairing)
    uint8_t broadcastMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    addPeer(broadcastMAC);
    
    // Add specific target peer if not broadcast
    if (!isBroadcast()) {
        addPeer(targetMAC);
    }
    
    initialized = true;
    
    const char* roleStr = (role == RadioRole::BASE) ? "BASE (Receiver)" : "TAG (Transmitter)";
    const char* powerStr[] = {"LOW (11dBm)", "MEDIUM (15dBm)", "HIGH (20dBm)"};
    const char* rateStr[] = {"1 Mbps", "2 Mbps", "5.5 Mbps", "11 Mbps"};
    uint8_t pwrIdx = static_cast<uint8_t>(txPower) < 3 ? static_cast<uint8_t>(txPower) : 2;
    uint8_t rateIdx = static_cast<uint8_t>(dataRate) < 4 ? static_cast<uint8_t>(dataRate) : 0;
    
    DEBUG_INFO("  Role: %s", roleStr);
    DEBUG_INFO("  Channel: %d", channel);
    DEBUG_INFO("  TX Power: %s", powerStr[pwrIdx]);
    DEBUG_INFO("  Data Rate: %s", rateStr[rateIdx]);
    DEBUG_INFO("  ACK: %s", ackEnabled ? "Enabled" : "Disabled");
    DEBUG_INFO("  Target MAC: %02X:%02X:%02X:%02X:%02X:%02X%s",
              targetMAC[0], targetMAC[1], targetMAC[2], 
              targetMAC[3], targetMAC[4], targetMAC[5],
              isBroadcast() ? " (BROADCAST)" : "");
    
    DEBUG_INFO("RadioManager initialized");
    return true;
}

void RadioManager::update() {
    if (!initialized || !enabled) {
        return;
    }
    
    // Handle pairing timeout
    if (pairingActive) {
        if (millis() - pairingStartTime > RADIO_PAIRING_TIMEOUT_MS) {
            DEBUG_INFO("Pairing scan timeout");
            pairingActive = false;
        }
        // Send pairing beacons periodically
        static uint32_t lastBeacon = 0;
        if (millis() - lastBeacon > RADIO_BEACON_INTERVAL_MS) {
            lastBeacon = millis();
            sendPairingBeacon();
        }
    }
    
    // TAG mode: Send data at configured rate
    if (role == RadioRole::TAG && !pairingActive) {
        uint8_t rate = tagTxRate > 0 ? tagTxRate : 1;  // Guard against div-by-zero
        uint32_t txInterval = 1000 / rate;
        if (millis() - lastTagTxTime >= txInterval) {
            // TAG will call sendTagData() externally with GPS data
            // This is just timing tracking - actual send is triggered from main loop
            lastTagTxTime = millis();
        }
    }
}

void RadioManager::deinit() {
    if (!initialized) {
        return;
    }
    
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();
    
    initialized = false;
    DEBUG_INFO("RadioManager deinitialized");
}

// =============================================================================
// CONFIGURATION
// =============================================================================

void RadioManager::setRole(RadioRole newRole) {
    role = newRole;
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.radioRole = static_cast<uint8_t>(newRole);
    SystemSettings::save();
    DEBUG_INFO("Radio role set to: %s", newRole == RadioRole::BASE ? "BASE" : "TAG");
}

void RadioManager::setChannel(uint8_t newChannel) {
    if (newChannel < 1 || newChannel > 14) {
        DEBUG_WARN("Invalid channel %d (must be 1-14)", newChannel);
        return;
    }
    channel = newChannel;
    if (initialized) {
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    }
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.radioChannel = newChannel;
    SystemSettings::save();
    DEBUG_INFO("Radio channel set to: %d", channel);
}

void RadioManager::setTxPower(TxPower power) {
    txPower = power;
    if (initialized) {
        applyWiFiSettings();
    }
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.radioTxPower = static_cast<uint8_t>(power);
    SystemSettings::save();
}

void RadioManager::setDataRate(DataRate rate) {
    dataRate = rate;
    if (initialized) {
        applyWiFiSettings();
    }
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.radioDataRate = static_cast<uint8_t>(rate);
    SystemSettings::save();
}

void RadioManager::setAckEnabled(bool en) {
    ackEnabled = en;
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.radioAckEnabled = en;
    SystemSettings::save();
    DEBUG_INFO("ACK %s", en ? "enabled" : "disabled");
}

void RadioManager::setTagTxRate(uint8_t hz) {
    // Valid rates: 1, 2, 5, 10 Hz
    if (hz != 1 && hz != 2 && hz != 5 && hz != 10) {
        DEBUG_WARN("Invalid TX rate %d Hz (valid: 1, 2, 5, 10)", hz);
        return;
    }
    tagTxRate = hz;
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.radioTagTxRate = hz;
    SystemSettings::save();
    DEBUG_INFO("TAG TX rate set to: %d Hz", hz);
}

void RadioManager::setEnabled(bool en) {
    enabled = en;
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.radioEnabled = en;
    SystemSettings::save();
    
    if (en && !initialized) {
        init();
    } else if (!en && initialized) {
        deinit();
    }
    DEBUG_INFO("Radio %s", en ? "enabled" : "disabled");
}

// =============================================================================
// MAC ADDRESS MANAGEMENT
// =============================================================================

void RadioManager::setTargetMAC(const uint8_t* mac) {
    memcpy(targetMAC, mac, 6);
    RuntimeConfig& cfg = SystemSettings::getConfig();
    memcpy(cfg.radioTargetMAC, mac, 6);
    SystemSettings::save();
    
    // Add as peer if initialized
    if (initialized && !isBroadcast()) {
        addPeer(mac);
    }
    
    DEBUG_INFO("Target MAC set to: %02X:%02X:%02X:%02X:%02X:%02X",
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void RadioManager::getTargetMAC(uint8_t* mac) {
    memcpy(mac, targetMAC, 6);
}

void RadioManager::getMyMAC(uint8_t* mac) {
    memcpy(mac, myMAC, 6);
}

void RadioManager::resetTobroadcast() {
    uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    setTargetMAC(broadcast);
    DEBUG_INFO("Reset to broadcast mode");
}

bool RadioManager::isBroadcast() {
    return (targetMAC[0] == 0xFF && targetMAC[1] == 0xFF && 
            targetMAC[2] == 0xFF && targetMAC[3] == 0xFF && 
            targetMAC[4] == 0xFF && targetMAC[5] == 0xFF);
}

// =============================================================================
// PAIRING
// =============================================================================

void RadioManager::startPairingScan() {
    if (!initialized) {
        DEBUG_WARN("Cannot start pairing - radio not initialized");
        return;
    }
    pairingActive = true;
    pairingStartTime = millis();
    memset(lastDiscoveredMAC, 0, 6);
    DEBUG_INFO("Pairing scan started (30 seconds)...");
}

void RadioManager::stopPairingScan() {
    pairingActive = false;
    DEBUG_INFO("Pairing scan stopped");
}

bool RadioManager::isPairingActive() {
    return pairingActive;
}

uint8_t RadioManager::getPairedDeviceCount() {
    return pairedDeviceCount;
}

void RadioManager::getLastDiscoveredMAC(uint8_t* mac) {
    memcpy(mac, lastDiscoveredMAC, 6);
}

// =============================================================================
// TAG MODE: SEND DATA
// =============================================================================

bool RadioManager::sendTagData(double lat, double lon, uint16_t heading) {
    if (!initialized || !enabled || role != RadioRole::TAG) {
        return false;
    }
    
    TagPacket pkt;
    pkt.type = PACKET_TYPE_GPS_DATA;
    pkt.latitude = lat;
    pkt.longitude = lon;
    pkt.heading = heading;
    pkt.timestamp = millis();
    
    esp_err_t result = esp_now_send(targetMAC, (uint8_t*)&pkt, sizeof(pkt));
    
    if (result == ESP_OK) {
        packetsSent++;
        lastTagTxTime = millis();
        return true;
    } else {
        packetsLost++;
        DEBUG_WARN("ESP-NOW send failed: %d", result);
        return false;
    }
}

// =============================================================================
// BASE MODE: GET RECEIVED DATA
// =============================================================================

bool RadioManager::hasNewData() {
    bool result = newDataFlag;
    newDataFlag = false;
    return result;
}

double RadioManager::getTagLatitude() {
    return rxLatitude;
}

double RadioManager::getTagLongitude() {
    return rxLongitude;
}

uint16_t RadioManager::getTagHeading() {
    return rxHeading;
}

uint32_t RadioManager::getTagTimestamp() {
    return rxTimestamp;
}

int RadioManager::getSecondsSinceLastPacket() {
    if (lastRxTime == 0) {
        return -1; // Never received
    }
    return (millis() - lastRxTime) / 1000;
}

bool RadioManager::isConnected() {
    if (lastRxTime == 0) {
        return false;
    }
    return (millis() - lastRxTime) < RADIO_CONNECTION_TIMEOUT_MS;
}

// =============================================================================
// STATISTICS
// =============================================================================

int8_t RadioManager::getLastRSSI() {
    return lastRSSI;
}

uint32_t RadioManager::getPacketsReceived() {
    return packetsReceived;
}

uint32_t RadioManager::getPacketsSent() {
    return packetsSent;
}

uint32_t RadioManager::getPacketsLost() {
    return packetsLost;
}

// =============================================================================
// STATUS
// =============================================================================

bool RadioManager::isInitialized() {
    return initialized;
}

bool RadioManager::isEnabled() {
    return enabled;
}

RadioRole RadioManager::getRole() {
    return role;
}

uint8_t RadioManager::getChannel() {
    return channel;
}

// =============================================================================
// CALLBACKS
// =============================================================================

void RadioManager::onReceiveCallback(const uint8_t* mac, const uint8_t* data, int len) {
    if (len < 1) return;
    
    uint8_t packetType = data[0];
    
    // Get RSSI from esp_now_recv_info (ESP-IDF 4.4+)
    // Note: For Arduino ESP32 core, RSSI may not be directly available in callback
    // We'll use a workaround or leave as 0 for now
    
    switch (packetType) {
        case PACKET_TYPE_GPS_DATA:
            if (len >= sizeof(TagPacket)) {
                TagPacket* pkt = (TagPacket*)data;
                rxLatitude = pkt->latitude;
                rxLongitude = pkt->longitude;
                rxHeading = pkt->heading;
                rxTimestamp = pkt->timestamp;
                lastRxTime = millis();
                newDataFlag = true;
                packetsReceived++;
                
                // Send ACK if enabled and we're BASE
                if (ackEnabled && role == RadioRole::BASE) {
                    sendAck(mac, pkt->timestamp);
                }
                
                // Store sender MAC if in pairing mode
                if (pairingActive) {
                    memcpy(lastDiscoveredMAC, mac, 6);
                }
            }
            break;
            
        case PACKET_TYPE_ACK:
            // TAG receives ACK from BASE - could track latency here
            if (len >= sizeof(AckPacket)) {
                // AckPacket* ack = (AckPacket*)data;
                // uint32_t latency = millis() - ack->echoTimestamp;
                // Could expose this via getter if needed
            }
            break;
            
        case PACKET_TYPE_PAIR_BEACON:
        case PACKET_TYPE_PAIR_REQUEST:
        case PACKET_TYPE_PAIR_CONFIRM:
            if (len >= sizeof(PairingPacket)) {
                handlePairingPacket(mac, (PairingPacket*)data);
            }
            break;
            
        default:
            DEBUG_WARN("Unknown packet type: 0x%02X", packetType);
            break;
    }
}

void RadioManager::onSendCallback(const uint8_t* mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        packetsLost++;
    }
}

// =============================================================================
// INTERNAL HELPERS
// =============================================================================

bool RadioManager::applyWiFiSettings() {
    // Apply TX power (units of 0.25 dBm)
    int8_t powerValue;
    switch (txPower) {
        case TxPower::POWER_LOW:    powerValue = 44;  break; // 11 dBm
        case TxPower::POWER_MEDIUM: powerValue = 60;  break; // 15 dBm
        case TxPower::POWER_HIGH:   powerValue = 80;  break; // 20 dBm
        default:                    powerValue = 80;  break;
    }
    esp_wifi_set_max_tx_power(powerValue);
    
    // Apply data rate
    wifi_phy_rate_t rate;
    switch (dataRate) {
        case DataRate::RATE_1M:  rate = WIFI_PHY_RATE_1M_L;  break;
        case DataRate::RATE_2M:  rate = WIFI_PHY_RATE_2M_L;  break;
        case DataRate::RATE_5M:  rate = WIFI_PHY_RATE_5M_L;  break;
        case DataRate::RATE_11M: rate = WIFI_PHY_RATE_11M_L; break;
        default:                 rate = WIFI_PHY_RATE_1M_L;  break;
    }
    esp_wifi_config_espnow_rate(WIFI_IF_STA, rate);
    
    return true;
}

bool RadioManager::addPeer(const uint8_t* mac) {
    // Check if peer already exists
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;  // 0 = use current channel
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    
    esp_err_t result = esp_now_add_peer(&peerInfo);
    if (result != ESP_OK) {
        DEBUG_WARN("Failed to add peer: %d", result);
        return false;
    }
    
    pairedDeviceCount++;
    return true;
}

void RadioManager::sendAck(const uint8_t* mac, uint32_t timestamp) {
    AckPacket ack;
    ack.type = PACKET_TYPE_ACK;
    ack.echoTimestamp = timestamp;
    
    // Send ACK to specific MAC (not broadcast)
    if (!esp_now_is_peer_exist(mac)) {
        addPeer(mac);
    }
    esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
}

void RadioManager::sendPairingBeacon() {
    PairingPacket pkt;
    pkt.type = (role == RadioRole::BASE) ? PACKET_TYPE_PAIR_BEACON : PACKET_TYPE_PAIR_REQUEST;
    memcpy(pkt.mac, myMAC, 6);
    pkt.channel = channel;
    pkt.role = static_cast<uint8_t>(role);
    
    esp_now_send(targetMAC, (uint8_t*)&pkt, sizeof(pkt));
}

void RadioManager::handlePairingPacket(const uint8_t* mac, const PairingPacket* pkt) {
    if (!pairingActive) {
        return;
    }
    
    // Store discovered device
    memcpy(lastDiscoveredMAC, mac, 6);
    
    if (pkt->type == PACKET_TYPE_PAIR_BEACON && role == RadioRole::TAG) {
        // TAG received beacon from BASE - respond with request
        DEBUG_INFO("Discovered BASE: %02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        
        PairingPacket response;
        response.type = PACKET_TYPE_PAIR_REQUEST;
        memcpy(response.mac, myMAC, 6);
        response.channel = channel;
        response.role = static_cast<uint8_t>(role);
        
        addPeer(mac);
        esp_now_send(mac, (uint8_t*)&response, sizeof(response));
    }
    else if (pkt->type == PACKET_TYPE_PAIR_REQUEST && role == RadioRole::BASE) {
        // BASE received request from TAG - confirm and save
        DEBUG_INFO("Discovered TAG: %02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        
        // Save TAG's MAC as target
        setTargetMAC(mac);
        addPeer(mac);
        
        // Send confirmation
        PairingPacket confirm;
        confirm.type = PACKET_TYPE_PAIR_CONFIRM;
        memcpy(confirm.mac, myMAC, 6);
        confirm.channel = channel;
        confirm.role = static_cast<uint8_t>(role);
        
        esp_now_send(mac, (uint8_t*)&confirm, sizeof(confirm));
        
        pairingActive = false;
        DEBUG_INFO("Pairing complete!");
    }
    else if (pkt->type == PACKET_TYPE_PAIR_CONFIRM && role == RadioRole::TAG) {
        // TAG received confirmation from BASE
        DEBUG_INFO("Paired with BASE: %02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        
        setTargetMAC(mac);
        pairingActive = false;
        DEBUG_INFO("Pairing complete!");
    }
}
