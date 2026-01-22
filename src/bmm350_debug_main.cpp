#include <Arduino.h>
#include <Wire.h>
#include "sensors/bmm350_defs.h"
#include "sensors/bmm350.h"

// =========================================================
// CONFIGURATION VALIDATOR - FINAL REVISION (v3.1)
// =========================================================
#define SDA_PIN 4
#define SCL_PIN 7
#define I2C_ADDR 0x14

struct bmm350_dev dev;
uint8_t dev_addr = I2C_ADDR;

void runConfigTest(float targetHz, uint8_t odr_enum, uint8_t avg_enum);

// =========================================================
// I2C DRIVER INTERFACE (RAW PASSTHROUGH - DRIVER HANDLES DUMMY)
// =========================================================
int8_t bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission() != 0) return BMM350_E_COM_FAIL;
    
    // IMPORTANT: The Bosch bmm350.c library automatically adds BMM350_DUMMY_BYTES (2)
    // to the 'len' argument and handles stripping them internally through bmm350GetRegs.
    // Our interface function MUST be a raw passthrough.
    Wire.requestFrom(addr, (uint8_t)len);
    
    for (uint32_t i = 0; i < len; i++) {
        if (Wire.available()) reg_data[i] = Wire.read();
        else return BMM350_E_COM_FAIL;
    }
    return BMM350_OK;
}

int8_t bmm350_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(addr);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) Wire.write(reg_data[i]);
    if (Wire.endTransmission() != 0) return BMM350_E_COM_FAIL;
    return BMM350_OK;
}

void bmm350_delay_us(uint32_t period, void *intf_ptr) {
    delayMicroseconds(period);
}

// =========================================================
// SETUP
// =========================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);
    
    Serial.println("\n\n╔════════════════════════════════════════╗");
    Serial.println("║  BMM350 CONFIGURATION VALIDATOR v3.1   ║");
    Serial.println("║      PROTOCOL: RAW (DRIVER DUMMY)      ║");
    Serial.println("╚════════════════════════════════════════╝");
    
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    
    dev.read = bmm350_i2c_read;
    dev.write = bmm350_i2c_write;
    dev.delayUs = bmm350_delay_us;
    dev.intfPtr = &dev_addr;
    
    // Initializing properly through driver
    if (bmm350Init(&dev) != BMM350_OK) {
        Serial.println("❌ BMM350 Init Failed! (Maybe not responding?)");
        while(1);
    }
    Serial.println("✓ BMM350 Initialized Successfully\n");

    // Enable all axes
    if (bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev) != BMM350_OK) {
        Serial.println("❌ Failed to enable axes!");
        while(1);
    }
    Serial.println("✓ Axes Enabled (X, Y, Z)");

    // Enable Data Ready Interrupt (needed for status register update?)
    if (bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &dev) != BMM350_OK) {
        Serial.println("❌ Failed to enable interrupt!");
        while(1);
    }
    Serial.println("✓ Interrupt Enabled");
    
    delay(1000);

    runConfigTest(12.5,  BMM350_DATA_RATE_12_5HZ, BMM350_AVERAGING_2);
    delay(500);
    runConfigTest(25.0,  BMM350_DATA_RATE_25HZ,   BMM350_AVERAGING_2);
    delay(500);
    runConfigTest(50.0,  BMM350_DATA_RATE_50HZ,   BMM350_AVERAGING_2);
    delay(500);
    runConfigTest(100.0, BMM350_DATA_RATE_100HZ,  BMM350_AVERAGING_2);
    delay(500);
    runConfigTest(200.0, BMM350_DATA_RATE_200HZ,  BMM350_AVERAGING_2);
    delay(500);
    runConfigTest(400.0, BMM350_DATA_RATE_400HZ,  BMM350_NO_AVERAGING);
    
    Serial.println("\n✓ ALL TESTS DONE.");
}

void loop() {}

// =========================================================
// TEST LOGIC
// =========================================================
void runConfigTest(float targetHz, uint8_t odr_enum, uint8_t avg_enum) {
    Serial.println("----------------------------------------------");
    Serial.printf("TEST: Requesting %.1f Hz (ODR=0x%X), Avg=0x%X\n", targetHz, odr_enum, avg_enum);
    
    // 1. SUSPEND MODE
    bmm350SetPowerMode(eBmm350SuspendMode, &dev);
    delay(10);
    
    // 2. APPLY CONFIGURATION
    bmm350SetOdrPerformance((eBmm350DataRates_t)odr_enum, (bmm350_performance_parameters)avg_enum, &dev);
    delay(10);
    
    // 3. NORMAL MODE
    bmm350SetPowerMode(eBmm350NormalMode, &dev);
    delay(70); 
    
    // 4. VERIFY CONFIG REGISTER (0x04) - USE bmm350GetRegs to handle dummy bytes!
    uint8_t reg_val = 0;
    bmm350GetRegs(BMM350_REG_PMU_CMD_AGGR_SET, &reg_val, 1, &dev);
    
    uint8_t read_odr = reg_val & 0x0F;
    uint8_t read_avg = (reg_val >> 4) & 0x03;
    
    Serial.printf("  > Config (0x04): 0x%02X [ODR:0x%X v 0x%X] [AVG:0x%X v 0x%X]\n", 
                  reg_val, read_odr, odr_enum, read_avg, avg_enum);
                  
    if (read_odr == odr_enum && read_avg == avg_enum) Serial.println("    ✓ Configuration MATCH");
    else Serial.println("    ✗ Configuration MISMATCH");
    
    // 5. VERIFY POWER MODE (0x07) - USE bmm350GetRegs!
    uint8_t status_0 = 0;
    bmm350GetRegs(BMM350_REG_PMU_CMD_STATUS_0, &status_0, 1, &dev);
    bool is_normal = (status_0 & BMM350_PWR_MODE_IS_NORMAL_MSK) != 0;
    Serial.printf("  > Status (0x07): 0x%02X [%s]\n", status_0, is_normal ? "NORMAL" : "NOT NORMAL");
    
    if (!is_normal) return;

    // 6. MEASURE RATE
    Serial.print("  > Measuring Rate (1s)... ");
    
    uint32_t start = millis();
    uint32_t samples = 0;
    sBmm350MagTempData_t data;
    
    while(millis() - start < 1000) {
        uint8_t int_status = 0;
        bmm350GetRegs(BMM350_REG_INT_STATUS, &int_status, 1, &dev); 
        
        if (int_status & 0x04) {
            bmm350GetCompensatedMagXYZTempData(&data, &dev);
            samples++;
            
            // Clear DRDY
            uint8_t clear = 0x04;
            bmm350SetRegs(BMM350_REG_INT_STATUS, &clear, 1, &dev);
        }
        delayMicroseconds(100);
    }
    
    Serial.printf("%d Hz", samples);
    
    if (samples > 0) {
        Serial.printf(" [Data: X=%.2f Y=%.2f Z=%.2f uT]", data.x, data.y, data.z);
    }

    if (samples > (targetHz * 0.9)) Serial.println(" -> ✓ PASS");
    else Serial.println(" -> ✗ FAIL (Too Slow)");
}
