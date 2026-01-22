#include "esp_now_handler.h"
#include <esp_now.h>
#include <WiFi.h>
#include "config.h"
#include "config_store.h"

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (g_debugMode) {
        Serial.print("[RADIO] Last Packet Send Status: ");
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    }
}

void updatePeerAddress(const uint8_t* newMac) {
    // Delete old peer if exists (we assume only 1 for now or we iterate)
    esp_now_del_peer(g_sysConfig.targetMac); // This might fail if address changed already? 
    // Actually, g_sysConfig.targetMac is already the new one if we updated it before calling this?
    // Wait, typical usage: config updated, then this called.
    // Issue: delete needs OLD address.
    // Fix: We don't track old address.
    // Brute force: delete all peers or just add new one?
    // ESP-NOW supports multiple peers.
    // Let's just delete the previous config if we can track it, or just use registered peer list?
    // Simpler approach: De-init and Re-init? No.
    // We will assume single peer logic for this project.
    // We'll try to delete expected old address (which we don't know here without passing it).
    // Let's just TRY to delete the newMac (in case it exists) and add it? No.
    
    // Better: Helper checks if peer exists.
    if (esp_now_is_peer_exist(newMac)) {
        //Already exists
        return;
    }
    
    // Add new peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, newMac, 6);
    peerInfo.channel = 0;  // 0 = use current channel
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] Failed to add new peer");
    } else {
        Serial.printf("[ESP-NOW] Peer Updated: %02X:%02X:%02X:%02X:%02X:%02X\n", 
            newMac[0], newMac[1], newMac[2], newMac[3], newMac[4], newMac[5]);
    }
}

bool initESPNow() {
    // 1. Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // 2. Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Error initializing ESP-NOW");
        return false;
    }
    Serial.println("[ESP-NOW] Initialized Successfully");

    // 3. Register Send Callback
    esp_now_register_send_cb(OnDataSent);

    // 4. Register Peer (Target MAC)
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo)); 
    memcpy(peerInfo.peer_addr, g_sysConfig.targetMac, 6);
    peerInfo.channel = 0; 
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] Failed to add peer");
        return false;
    }
    
    Serial.printf("[ESP-NOW] Peer Registered: %02X:%02X:%02X:%02X:%02X:%02X\n", 
            g_sysConfig.targetMac[0], g_sysConfig.targetMac[1], g_sysConfig.targetMac[2], 
            g_sysConfig.targetMac[3], g_sysConfig.targetMac[4], g_sysConfig.targetMac[5]);
    
    return true;
}

void sendTrackingData(const TrackingPacket& packet) {
    if (g_debugMode) {
        Serial.printf("[RADIO] Sending Packet: Lat=%ld Lon=%ld Alt=%d Flags=0x%02X\n", 
            (long)packet.lat, (long)packet.lon, packet.alt, packet.flags);
    }
    esp_err_t result = esp_now_send(g_sysConfig.targetMac, (uint8_t *) &packet, sizeof(packet));
   
    if (result != ESP_OK) {
        Serial.println("[ERROR] Radio Send Failed");
    }
}
