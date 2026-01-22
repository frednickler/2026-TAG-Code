#include <Arduino.h>
#include <Wire.h>
#include "Arduino-ICM20948.h"

ArduinoICM20948 myICM;

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 0

// Library uses this global to set address - MUST BE 0x68
extern uint8_t I2C_Address;
bool isInitialized = false;

// --- DYNAMIC RUNTIME CALIBRATION ---
// We will track Min/Max in real-time to find the center
float min_x = 1000, max_x = -1000;
float min_y = 1000, max_y = -1000;
float min_z = 1000, max_z = -1000;

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT);
  delay(1000);
  SERIAL_PORT.println("\n--- MULTIVERSE HEADING DIAGNOSTIC (AUTO-CAL) ---");
  SERIAL_PORT.println("1. ROTATE sensor 360 degrees in all directions to calibrate.");
  SERIAL_PORT.println("2. Then alignments will become accurate.");

  // 1. Initialize I2C with Explicit Pins (Matches config.h)
  WIRE_PORT.begin(4, 5);
  WIRE_PORT.setClock(400000);
  delay(100); 

  // 2. FORCE Address Override
  I2C_Address = 0x68;

  // 3. Manual I2C Bus Check
  WIRE_PORT.beginTransmission(I2C_Address);
  byte error = WIRE_PORT.endTransmission();
  if (error != 0) {
    SERIAL_PORT.printf("CRITICAL: Device not found at 0x%02X (Error %d)\n", I2C_Address, error);
    while(1) delay(1000); 
  }
  SERIAL_PORT.println("Sensor Found.");

  // 4. Initialize Library with SAFE SETTINGS
  ArduinoICM20948Settings settings = {
    .i2c_speed = 400000,
    .is_SPI = false,
    .cs_pin = 10,
    .spi_speed = 7000000,
    .mode = 1,                          
    .enable_gyroscope = true,
    .enable_accelerometer = true,
    .enable_magnetometer = true,
    .enable_gravity = false,            
    .enable_linearAcceleration = false, 
    .enable_quaternion6 = false,        
    .enable_quaternion9 = true,         
    .enable_har = false,
    .enable_steps = false,
    .gyroscope_frequency = 50,
    .accelerometer_frequency = 50,
    .magnetometer_frequency = 50,
    .gravity_frequency = 50,            
    .linearAcceleration_frequency = 50, 
    .quaternion6_frequency = 50,
    .quaternion9_frequency = 50,
    .har_frequency = 50,                
    .steps_frequency = 50               
  };
  
  myICM.init(settings);
  isInitialized = true;
  SERIAL_PORT.println("Library Initialized.");
  SERIAL_PORT.println("-----------------------------------------------------------------------");
  SERIAL_PORT.println("RawMag | Offset | H1-H8: Headings (1-4:Std, 5-8:Swap)");
  SERIAL_PORT.println("-----------------------------------------------------------------------");
}

// Helper: Calculate Tilt-Compensated Heading
float getHeading(float mx, float my, float mz, float roll_deg, float pitch_deg) {
  float roll_rad = roll_deg * DEG_TO_RAD;
  float pitch_rad = pitch_deg * DEG_TO_RAD;

  float mx_comp = mx * cos(pitch_rad) + mz * sin(pitch_rad);
  float my_comp = mx * sin(roll_rad) * sin(pitch_rad) + 
                  my * cos(roll_rad) - 
                  mz * sin(roll_rad) * cos(pitch_rad);
  
  float heading = atan2(my_comp, mx_comp) * RAD_TO_DEG;
  if (heading < 0) heading += 360.0f;
  return heading;
}

void loop() {
  if (!isInitialized) return;

  myICM.task();

  if (myICM.magDataIsReady() && myICM.euler9DataIsReady()) {
    float mx_raw, my_raw, mz_raw;
    float roll, pitch, yaw;
    
    myICM.readMagData(&mx_raw, &my_raw, &mz_raw);
    myICM.readEuler9Data(&roll, &pitch, &yaw);

    // --- DYNAMIC CALIBRATION UPDATE ---
    if (mx_raw < min_x) min_x = mx_raw;
    if (mx_raw > max_x) max_x = mx_raw;
    if (my_raw < min_y) min_y = my_raw;
    if (my_raw > max_y) max_y = my_raw;
    if (mz_raw < min_z) min_z = mz_raw;
    if (mz_raw > max_z) max_z = mz_raw;

    // Calculate Dynamic Offsets
    float off_x = (min_x + max_x) / 2.0f;
    float off_y = (min_y + max_y) / 2.0f;
    float off_z = (min_z + max_z) / 2.0f;

    // Apply Calibration
    float mx = mx_raw - off_x;
    float my = my_raw - off_y;
    float mz = mz_raw - off_z;

    // --- COMBINATIONS ---
    // Standard Axis
    float h1 = getHeading( mx,  my, mz, roll, pitch); // +X +Y
    float h2 = getHeading( mx, -my, mz, roll, pitch); // +X -Y
    float h3 = getHeading(-mx,  my, mz, roll, pitch); // -X +Y
    float h4 = getHeading(-mx, -my, mz, roll, pitch); // -X -Y
    
    // Swapped Axis (X becomes Y)
    float h5 = getHeading( my,  mx, mz, roll, pitch); // +Y +X
    float h6 = getHeading( my, -mx, mz, roll, pitch); // +Y -X
    float h7 = getHeading(-my,  mx, mz, roll, pitch); // -Y +X
    float h8 = getHeading(-my, -mx, mz, roll, pitch); // -Y -X

    SERIAL_PORT.printf("M(%3.0f,%3.0f) Off(%3.0f,%3.0f) | Std:%3.0f %3.0f %3.0f %3.0f | Swp:%3.0f %3.0f %3.0f %3.0f\n", 
                        mx_raw, my_raw, off_x, off_y,
                        h1, h2, h3, h4, h5, h6, h7, h8);

    delay(200); 
  }
}
