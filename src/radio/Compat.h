#ifndef ESP_NOW_COMPAT_H
#define ESP_NOW_COMPAT_H

#include <esp_now.h>
#include <esp_wifi.h>

// Compatibility for Arduino Core 2.x (ESP-IDF 4.4.x)
// to support libraries written for Core 3.x (ESP-IDF 5.x)

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3

// Define Core 3.x structs that are missing in Core 2.x

typedef struct {
    wifi_phy_mode_t phymode;
    wifi_phy_rate_t rate;
    bool ersu;
    bool dcm;
} esp_now_rate_config_t;

typedef struct {
    uint8_t * src_addr;
    uint8_t * des_addr;
    wifi_pkt_rx_ctrl_t * rx_ctrl;
} esp_now_recv_info_t;

#endif

#endif // ESP_NOW_COMPAT_H
