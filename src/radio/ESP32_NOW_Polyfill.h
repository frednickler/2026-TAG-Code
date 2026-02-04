#ifndef ESP32_NOW_POLYFILL_H
#define ESP32_NOW_POLYFILL_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Forward declaration
class ESP_NOW_Peer;

// Global callback adapters
extern void _polyfill_rx_cb(const uint8_t *mac, const uint8_t *data, int len);
extern void _polyfill_tx_cb(const uint8_t *mac, esp_now_send_status_t status);

// =============================================================================
// ESP_NOW_Peer Class
// =============================================================================
class ESP_NOW_Peer {
protected:
    uint8_t mac[6];
    uint8_t channel;
    wifi_interface_t iface;
    bool added;

public:
    ESP_NOW_Peer(const uint8_t *mac_addr, uint8_t channel = 0, wifi_interface_t iface = WIFI_IF_STA, const uint8_t *lmk = NULL) 
        : channel(channel), iface(iface), added(false) {
        if (mac_addr) memcpy(this->mac, mac_addr, 6);
        else memset(this->mac, 0xFF, 6);
    }

    virtual ~ESP_NOW_Peer() {
        remove();
    }

    bool add() {
        Serial.println("     ┌──────────────────────────────────────────────┐");
        Serial.println("     │ [ESP_NOW_Peer::add()] Adding peer           │");
        Serial.println("     └──────────────────────────────────────────────┘");
        
        if (added) {
            Serial.println("        ├─ Already added, returning true");
            return true;
        }
        
        Serial.printf("        ├─ MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        Serial.printf("        ├─ Channel: %d\n", channel);
        Serial.printf("        ├─ Interface: %s\n", (iface == WIFI_IF_STA) ? "STA" : "AP");
        
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, mac, 6);
        peerInfo.channel = channel;
        peerInfo.ifidx = iface;
        peerInfo.encrypt = false;
        
        Serial.println("        ├─ Calling esp_now_add_peer()...");
        esp_err_t result = esp_now_add_peer(&peerInfo);
        
        if (result == ESP_OK) {
            Serial.println("        ├─ ✓ esp_now_add_peer() returned ESP_OK");
            added = true;
            Serial.println("        └─ Peer added successfully");
            return true;
        } else if (result == ESP_ERR_ESPNOW_EXIST) {
            Serial.println("        ├─ ⚠ Peer already exists (ESP_ERR_ESPNOW_EXIST)");
            added = true;
            Serial.println("        └─ Treating as success");
            return true;
        } else {
            Serial.printf("        ├─ ❌ esp_now_add_peer() FAILED with error: %d (0x%X)\n", result, result);
            Serial.println("        └─ Peer add FAILED");
            return false;
        }
    }

    bool remove() {
        if (added) {
            Serial.println("[ESP_NOW_Peer::remove()] Removing peer");
            esp_now_del_peer(mac);
            added = false;
        }
        return true;
    }

    bool send(const uint8_t *data, size_t len) {
        esp_err_t result = esp_now_send(mac, (uint8_t*)data, len);
        
        #ifdef RADIO_DEBUG_VERBOSE
        Serial.printf("[ESP_NOW_Peer::send()] esp_now_send() returned: %s\n",
                      (result == ESP_OK) ? "ESP_OK" : "ERROR!");
        #endif
        
        return (result == ESP_OK);
    }
    
    // Virtual callbacks
    virtual void onReceive(const uint8_t *data, size_t len, bool broadcast) {}
    virtual void onSent(bool success) {}
    
    bool matches(const uint8_t* other_mac) {
        if (!other_mac) return false;
        return (memcmp(mac, other_mac, 6) == 0) || (mac[0]==0xFF);
    }
};

// =============================================================================
// ESP_NOW_Class
// =============================================================================
class ESP_NOW_Class {
public:
    ESP_NOW_Peer* active_peer = nullptr;

    bool begin() {
        Serial.println("     ┌──────────────────────────────────────────────┐");
        Serial.println("     │ [ESP_NOW_Class::begin()] Initializing        │");
        Serial.println("     └──────────────────────────────────────────────┘");
        
        // Init ESP-NOW
        Serial.println("        ├─ Calling esp_now_init()...");
        esp_err_t result = esp_now_init();
        
        if (result != ESP_OK) {
            Serial.printf("        ├─ ❌ esp_now_init() FAILED with error: %d (0x%X)\n", result, result);
            Serial.println("        └─ ESP-NOW init FAILED");
            return false;
        }
        Serial.println("        ├─ ✓ esp_now_init() returned ESP_OK");
        
        // Register RX Callback
        Serial.println("        ├─ Registering RX callback (_polyfill_rx_cb)...");
        result = esp_now_register_recv_cb(_polyfill_rx_cb);
        
        if (result != ESP_OK) {
            Serial.printf("        ├─ ❌ esp_now_register_recv_cb() FAILED: %d (0x%X)\n", result, result);
            Serial.println("        └─ RX callback registration FAILED");
            return false;
        }
        Serial.println("        ├─ ✓ RX callback registered successfully");
        
        // Register TX Callback
        Serial.println("        ├─ Registering TX callback (_polyfill_tx_cb)...");
        result = esp_now_register_send_cb(_polyfill_tx_cb);
        
        if (result != ESP_OK) {
            Serial.printf("        ├─ ❌ esp_now_register_send_cb() FAILED: %d (0x%X)\n", result, result);
            Serial.println("        └─ TX callback registration FAILED");
            return false;
        }
        Serial.println("        ├─ ✓ TX callback registered successfully");
        
        Serial.println("        └─ ✓ ESP_NOW_Class::begin() complete");
        return true;
    }
    
    void registerPeer(ESP_NOW_Peer* peer) {
        Serial.println("        ├─ Registering peer for callbacks...");
        active_peer = peer;
        Serial.printf("        └─ active_peer set to: 0x%p\n", (void*)peer);
    }
};

extern ESP_NOW_Class ESP_NOW;

#endif // ESP32_NOW_POLYFILL_H
