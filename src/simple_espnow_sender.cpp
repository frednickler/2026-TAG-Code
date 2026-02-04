#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/*
 * SIMPLE ESP-NOW SENDER (Native API)
 * Configuration:
 * - Channel: 1
 * - Protocol: Long Range (LR)
 * - Power: Max (20dBm)
 * - Mode: Broadcast (FF:FF...)
 */

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("TX Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("--- MVE SENDER STARTING (Native LR) ---");

  WiFi.mode(WIFI_STA);

  // 1. FORCE CHANNEL 1
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // 2. MAX POWER & NO POWER SAVE
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(80); // 20dBm

  // 3. LONG RANGE PROTOCOL
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Sender Ready. Channel: 1 (LR Mode)");
  Serial.println("MAC: " + WiFi.macAddress());
}

int i = 0;
void loop() {
  char msg[32];
  sprintf(msg, "MVE Pkt %d", i++);
  
  Serial.print("Sending: ");
  Serial.println(msg);
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)msg, strlen(msg)); // V3.0 change: use uint8_t*
  
  delay(1000);
}
