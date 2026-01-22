#ifndef ESP_NOW_HANDLER_H
#define ESP_NOW_HANDLER_H

#include <Arduino.h>
#include "radio_packet.h"

// Initialize ESP-NOW and register broadcast peer
bool initESPNow();

// Core Functions
bool initESPNow();
void sendTrackingData(const TrackingPacket& packet);
void updatePeerAddress(const uint8_t* newMac);

#endif
