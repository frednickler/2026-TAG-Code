/**
 * I2C Scanner for GPIO 8/9 (Default ESP32-S3 I2C pins)
 * Tests if GPS is on this bus
 */

#include <Arduino.h>
#include <Wire.h>

#define TEST_SDA 8
#define TEST_SCL 9

void setup() {
    Serial.begin(921600);
    delay(2000);
    
    Serial.println("\n========================================");
    Serial.println("   I2C Scanner - GPIO 8/9");
    Serial.println("========================================\n");
    
    Wire.begin(TEST_SDA, TEST_SCL);
    Wire.setClock(400000);
    
    Serial.printf("Scanning I2C: SDA=GPIO%d, SCL=GPIO%d\n\n", TEST_SDA, TEST_SCL);
    
    int count = 0;
    bool gpsFound = false;
    
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            count++;
            Serial.printf("✓ Device at 0x%02X", addr);
            
            if (addr == 0x42) {
                Serial.print(" ★★★ GPS! ★★★");
                gpsFound = true;
            }
            Serial.println();
        }
    }
    
    Serial.println("\n========================================");
    Serial.printf("Found %d device(s)\n", count);
    
    if (gpsFound) {
        Serial.println("\n✓✓✓ GPS CONFIRMED on GPIO 8/9! ✓✓✓");
        Serial.println("\nNext step: Update SystemConfig.h");
        Serial.println("Change I2C_SDA_PIN to 8");
        Serial.println("Change I2C_SCL_PIN to 9");
    } else if (count > 0) {
        Serial.println("\n⚠ Devices found but NO GPS at 0x42");
    } else {
        Serial.println("\n❌ NO devices found on this bus");
        Serial.println("GPS might be on different pins");
    }
    Serial.println("========================================\n");
}

void loop() {
    delay(1000);
}
