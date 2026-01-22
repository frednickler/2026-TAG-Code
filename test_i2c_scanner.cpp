/**
 * I2C Scanner - GPIO 4 & 7
 * Testing BOTH orientations with internal pull-ups
 */

#include <Arduino.h>
#include <Wire.h>

void scan(int sda, int scl, const char* desc) {
    Serial.printf("\n--- Testing: %s ---\n", desc);
    Serial.printf("SDA: %d, SCL: %d\n", sda, scl);
    
    Wire.end();
    delay(100);
    
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
    delay(50);
    
    Wire.begin(sda, scl);
    Wire.setClock(100000);
    delay(200);
    
    int count = 0;
    for (byte i = 1; i < 127; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf("✓ Found device at 0x%02X", i);
            if (i == 0x3C) Serial.print(" (OLED)");
            if (i == 0x68) Serial.print(" (ICM-20948)");
            if (i == 0x14) Serial.print(" (BMM350)");
            Serial.println();
            count++;
        }
    }
    
    if (count == 0) Serial.println("No devices found.");
    else Serial.printf("Found %d devices.\n", count);
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("\n\n========================================");
    Serial.println("I2C SCANNER - GPIO 4 & 7");
    Serial.println("Testing BOTH orientations");
    Serial.println("========================================");
    
    scan(4, 7, "SDA=GPIO 4, SCL=GPIO 7");
    delay(1000);
    scan(7, 4, "SDA=GPIO 7, SCL=GPIO 4");
    
    Serial.println("\n========================================");
    Serial.println("TEST COMPLETE");
    Serial.println("========================================");
}

void loop() {
    delay(1000);
}
