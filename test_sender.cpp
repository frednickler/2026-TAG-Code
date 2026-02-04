/**
 * ESP-NOW Sender - AP_STA Mode + Broadcast
 * Uses broadcast to reach any listening device
 */
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;
uint8_t counter = 0;

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== SENDER AP_STA BROADCAST ===");
  
  WiFi.mode(WIFI_AP_STA);
  
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Add peer failed");
    return;
  }
  
  Serial.println("Sending broadcasts...");
}

void loop() {
  counter++;
  Serial.printf("Send %d: ", counter);
  esp_now_send(broadcastAddr, &counter, sizeof(counter));
  delay(2000);
}
