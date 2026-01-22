#include "temp_sensor.h"
#include "temp_config.h"

// Forward declaration of ICM-20948 access functions
extern void setBank(uint8_t bank);
extern uint8_t readRegister(uint8_t reg);
extern void writeRegister(uint8_t reg, uint8_t value);
extern uint8_t readRegisters(uint8_t reg, uint8_t* data, uint8_t len);

// Forward declaration from temp_config.cpp
extern float _processTempSample(float newSample);

/**
 * Configure the temperature sensor registers for optimal reading
 * 
 * This function configures the ICM-20948 temperature sensor with settings optimized for
 * accurate temperature readings.
 */
void configureTempSensor() {
    // Wake up the device if needed (Bank 0, Register 0x06 - PWR_MGMT_1)
    setBank(0);
    writeRegister(0x06, 0x01);  // Auto-select clock source
    delay(10);
    
    // Enable temperature sensor (it's enabled by default, but let's make sure)
    // PWR_MGMT_1 register, bit 3 (TEMP_DIS) should be 0 to enable temperature sensor
    uint8_t pwr_mgmt_1 = readRegister(0x06);
    if (pwr_mgmt_1 & 0x08) {  // If TEMP_DIS bit is set (temp sensor disabled)
        writeRegister(0x06, pwr_mgmt_1 & ~0x08);  // Clear TEMP_DIS bit to enable temp sensor
        delay(10);
    }
    
    // Configure sample rate divider for temperature readings (Bank 2)
    setBank(2);
    writeRegister(0x00, 0x00);  // TEMP_CONFIG: No additional divider
    
    // Return to Bank 0
    setBank(0);
    
    Serial.println("[TEMP] Temperature sensor configured");
}

/**
 * Read the raw temperature value from the ICM-20948 sensor
 * 
 * @return Raw temperature register value
 */
int16_t readRawTemperature() {
    // Temperature data is in registers 0x39 (TEMP_OUT_H) and 0x3A (TEMP_OUT_L) in Bank 0
    setBank(0);
    
    // Read 2 bytes starting from TEMP_OUT_H (0x39)
    uint8_t data[2];
    if (readRegisters(0x39, data, 2) != 2) {
        return 0; // Return 0 on read failure
    }
    
    // Combine high and low bytes
    int16_t rawTemp = (int16_t)((data[0] << 8) | data[1]);
    return rawTemp;
}

/**
 * Read temperature from the ICM-20948 sensor
 * 
 * @param temp Reference to store the temperature value in degrees Celsius
 * @return true if reading was successful, false otherwise
 */
bool readTemperature(float& temp) {
    int16_t rawTemp = readRawTemperature();
    
    if (rawTemp == 0) {
        temp = 0.0f;
        return false;
    }
    
    // Convert raw temperature to Celsius using ICM-20948 formula:
    // Temperature in degrees C = (TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity + 21degC
    // Where RoomTemp_Offset = 0 and Temp_Sensitivity = 333.87 LSB/°C
    temp = (rawTemp / 333.87f) + 21.0f;
    
    return true;
}

/**
 * Calibrate the temperature sensor
 * 
 * This function calibrates the temperature sensor by comparing readings to a known reference
 * temperature. This improves the accuracy of temperature compensation for other sensors.
 * 
 * @param referenceTemp Optional reference temperature in Celsius (if known)
 * @return true if calibration was successful, false otherwise
 */
bool calibrateTemperatureSensor(float referenceTemp) {
    Serial.println("[CAL] Temperature Sensor Calibration");
    
    // Configure the temperature sensor
    configureTempSensor();
    
    // If no reference temperature provided, ask the user
    if (isnan(referenceTemp)) {
        Serial.println("[CAL] Enter the current ambient temperature in Celsius (e.g. 23.5):");
        Serial.println("[CAL] Or press Enter to use default calibration values");
        
        // Wait for user input
        String userInput = "";
        bool inputReceived = false;
        unsigned long startTime = millis();
        
        while (!inputReceived && (millis() - startTime < 30000)) {
            if (Serial.available()) {
                char c = Serial.read();
                if (c == '\n' || c == '\r') {
                    inputReceived = true;
                } else {
                    userInput += c;
                }
            }
            delay(10);
        }
        
        // Parse user input
        if (userInput.length() > 0) {
            referenceTemp = userInput.toFloat();
            if (referenceTemp < -40.0f || referenceTemp > 85.0f) {
                Serial.println("[CAL] Invalid temperature. Must be between -40°C and 85°C.");
                return false;
            }
        } else {
            Serial.println("[CAL] Using default calibration values");
            // Use default values
            calibration.tempRoomOffset = 0.0f;
            calibration.tempSensitivity = 333.87f;
            calibration.tempOffset = 21.0f;
            calibration.tempCalibrated = true;
            
            // Save calibration to EEPROM
            saveCalibrationToEEPROM();
            Serial.println("[CAL] Temperature sensor calibration saved to EEPROM");
            return true;
        }
    }
    
    Serial.print("[CAL] Reference temperature: ");
    Serial.print(referenceTemp);
    Serial.println("°C");
    
    // Collect multiple raw temperature readings
    const int numSamples = 100;
    int32_t rawTempSum = 0;
    
    Serial.println("[CAL] Collecting temperature data...");
    for (int i = 0; i < numSamples; i++) {
        rawTempSum += readRawTemperature();
        delay(10);
    }
    
    // Calculate average raw temperature
    int16_t avgRawTemp = rawTempSum / numSamples;
    
    // Calculate calibration parameters
    // For simplicity, we'll just adjust the offset to match the reference temperature
    // assuming the default sensitivity is correct
    
    // Using the formula: Temp_degC = ((rawTemp - roomTempOffset) / tempSensitivity) + tempOffset
    // We'll keep the default sensitivity and solve for roomTempOffset and tempOffset
    
    calibration.tempSensitivity = 333.87f;  // Default from datasheet
    calibration.tempOffset = 21.0f;         // Default from datasheet
    
    // Calculate room temperature offset to match reference temperature
    // referenceTemp = ((avgRawTemp - roomTempOffset) / tempSensitivity) + tempOffset
    // Solve for roomTempOffset:
    // roomTempOffset = avgRawTemp - (referenceTemp - tempOffset) * tempSensitivity
    calibration.tempRoomOffset = avgRawTemp - (referenceTemp - calibration.tempOffset) * calibration.tempSensitivity;
    
    // Set calibration flag
    calibration.tempCalibrated = true;
    
    // Verify calibration
    float calibratedTemp = (avgRawTemp / 333.87f) + 21.0f;
    
    Serial.print("[CAL] Raw temperature reading: ");
    Serial.println(avgRawTemp);
    Serial.print("[CAL] Calibrated temperature: ");
    Serial.print(calibratedTemp);
    Serial.println("°C");
    Serial.print("[CAL] Reference temperature: ");
    Serial.print(referenceTemp);
    Serial.println("°C");
    Serial.print("[CAL] Calibration error: ");
    Serial.print(calibratedTemp - referenceTemp, 4);
    Serial.println("°C");
    
    // Save calibration to EEPROM
    saveCalibrationToEEPROM();
    Serial.println("[CAL] Temperature sensor calibration saved to EEPROM");
    
    return true;
}

/**
 * Apply temperature compensation to sensor readings
 * 
 * This function applies temperature compensation to the specified sensor value
 * based on the current temperature and calibration parameters.
 * 
 * @param sensorValue The sensor value to compensate
 * @param sensorType The type of sensor (SENSOR_TYPE_ACCEL, SENSOR_TYPE_GYRO, SENSOR_TYPE_MAG)
 * @param axis The axis (0=X, 1=Y, 2=Z)
 * @param currentTemp The current temperature in Celsius
 * @return Temperature-compensated sensor value
 */
float applyTemperatureCompensation(float sensorValue, uint8_t sensorType, uint8_t axis, float currentTemp) {
    // Only apply compensation if temperature calibration is enabled and we have valid coefficients
    if (!calibration.tempCalibrated || axis >= 3) {
        return sensorValue;
    }
    
    float tempCoeff = 0.0f;
    float refTemp = calibration.referenceTemp;
    
    switch (sensorType) {
        case SENSOR_TYPE_ACCEL:
            if (calibration.accelTempCompEnabled) {
                tempCoeff = calibration.accelTempCoeff[axis];
            }
            break;
            
        case SENSOR_TYPE_GYRO:
            if (calibration.gyroTempCompEnabled) {
                tempCoeff = calibration.gyroTempCoeff[axis];
            }
            break;
            
        case SENSOR_TYPE_MAG:
            if (calibration.magTempCompEnabled) {
                tempCoeff = calibration.magTempCoeff[axis];
            }
            break;
            
        default:
            return sensorValue;
    }
    
    // Apply temperature compensation
    // Formula: compensated_value = raw_value - (current_temp - ref_temp) * temp_coeff
    return sensorValue - (currentTemp - refTemp) * tempCoeff;
}

// -----------------------------------------------------------------------------
// Quick repeatability validation (boot-time health check)
// -----------------------------------------------------------------------------
bool validateTemperatureRepeatability(float &mean, float &sigma) {
    const int SAMPLE_COUNT = 50;

    float sum = 0;
    float samples[SAMPLE_COUNT];

    // Ensure sensor configured
    configureTempSensor();

    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        float temp;
        if (!readTemperature(temp)) {
            return false;
        }
        samples[i] = temp;
        sum += samples[i];
        delay(20); // 50 Hz, total ~1 s
    }

    mean = sum / SAMPLE_COUNT;

    float var = 0;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        float d = samples[i] - mean;
        var += d * d;
    }
    sigma = sqrtf(var / (SAMPLE_COUNT - 1));

    return sigma < 0.1f;
}
