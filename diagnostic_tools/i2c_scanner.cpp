/**
 * I2C Scanner + GPS I2C Test
 * 
 * Scans I2C bus to find all connected devices
 * Then specifically tests for GPS at 0x42
 */

#include <Arduino.h>
#include <Wire.h>

// I2C pins from SystemConfig.h
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 7

void setup() {
    Serial.begin(921600);
    delay(2000);
    
    Serial.println("\n\n========================================");
    Serial.println("   I2C SCANNER + GPS DETECTOR");
    Serial.println("========================================\n");
    
    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000); // 400kHz
    
    Serial.printf("I2C Bus: SDA=GPIO%d, SCL=GPIO%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.println("\nScanning I2C bus...\n");
    
    int deviceCount = 0;
    bool gpsFound = false;
    
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            deviceCount++;
            Serial.printf("✓ Device found at 0x%02X", address);
            
            // Identify known devices
            if (address == 0x14) Serial.print(" (BMM350 Magnetometer)");
            else if (address == 0x3C) Serial.print(" (OLED Display)");
            else if (address == 0x42) {
                Serial.print(" ★★★ GPS MODULE ★★★");
                gpsFound = true;
            }
            else if (address == 0x68) Serial.print(" (ICM-20948 IMU)");
            
            Serial.println();
        }
    }
    
    Serial.println("\n========================================");
    Serial.printf("Scan complete! Found %d device(s)\n", deviceCount);
    Serial.println("========================================\n");
    
    if (gpsFound) {
        Serial.println("✓✓✓ GPS FOUND ON I2C BUS! ✓✓✓\n");
        Serial.println("Your GPS is using I2C communication,");
        Serial.println("NOT UART!");
        Serial.println("\nNext Steps:");
        Serial.println("1. GPS is already on I2C (address 0x42)");
        Serial.println("2. Update GPSManager to use I2C mode");
        Serial.println("3. Use SparkFun u-blox library (already included)");
        Serial.println("\nGPS is wired to:");
        Serial.printf("  SDA: GPIO%d\n", I2C_SDA_PIN);
        Serial.printf("  SCL: GPIO%d\n", I2C_SCL_PIN);
        Serial.printf("  I2C Address: 0x42\n");
    } else if (deviceCount > 0) {
        Serial.println("⚠ I2C devices found, but NO GPS at 0x42");
        Serial.println("\nPossible issues:");
        Serial.println("1. GPS not connected to I2C bus");
        Serial.println("2. GPS using different I2C address");
        Serial.println("3. GPS powered but not responding");
    } else {
        Serial.println("❌ NO I2C devices detected!");
        Serial.println("\nCheck:");
        Serial.printf("1. I2C wiring (SDA=GPIO%d, SCL=GPIO%d)\n", I2C_SDA_PIN, I2C_SCL_PIN);
        Serial.println("2. Pull-up resistors on SDA/SCL");
        Serial.println("3. Device power");
    }
    
    Serial.println("\n========================================\n");
}

void loop() {
    delay(1000);
}
