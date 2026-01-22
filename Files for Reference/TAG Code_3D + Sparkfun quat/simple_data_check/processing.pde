import processing.serial.*;
import java.text.DecimalFormat;

// Serial communication variables
Serial esp32Serial;
String serialBuffer = "";
String[] serialPorts;
int selectedPort = -1;
long lastDataReceived = 0;
boolean connectionActive = false;

// 3D visualization settings
float rotX = 0, rotY = 0, rotZ = 0;
PMatrix3D transform = new PMatrix3D();
int bgColor = color(135, 206, 235); // Sky blue background

// Data received from ESP32
// Raw accelerometer (mg)
float raw_ax, raw_ay, raw_az;
// Calibrated accelerometer (mg)
float cal_ax, cal_ay, cal_az;
// Accelerometer magnitude (should be ~1000mg when stationary)
float accel_mag;

// Raw gyroscope (dps)
float raw_gx, raw_gy, raw_gz;
// Calibrated gyroscope (dps)
float cal_gx, cal_gy, cal_gz;
// Gyro magnitude (should be ~0 when stationary)
float gyro_mag;

// Raw magnetometer (uT)
float raw_mx, raw_my, raw_mz;
// Calibrated magnetometer (uT)
float cal_mx, cal_my, cal_mz;
// Magnetometer magnitude (should be constant regardless of orientation)
float mag_mag;

// Quaternion (orientation)
float qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;

// Euler angles (degrees)
float roll, pitch, yaw;

// Headings (degrees)
float magHeading, trueHeading, algHeading;

// UI state variables
boolean showRawData = true;
boolean showCalibratedData = true;
boolean showMetrics = true;

// Formatting for numbers
DecimalFormat df1 = new DecimalFormat("0.0");
DecimalFormat df2 = new DecimalFormat("0.00");

// =============================================================================
//  Setup and Main Loop
// =============================================================================

void setup() {
  size(1280, 720, P3D);
  smooth();

  // List available serial ports
  serialPorts = Serial.list();
  println("Available serial ports:");
  for (int i = 0; i < serialPorts.length; i++) {
    println("[" + i + "] " + serialPorts[i]);
  }
}

void draw() {
  background(bgColor);
  
  // If no port is selected, show the selection screen
  if (selectedPort == -1) {
    drawPortSelectionScreen();
    return;
  }
  
  // Check if we're receiving data
  if (connectionActive && millis() - lastDataReceived > 3000) {
    println("WARNING: No data received for 3 seconds. Connection may be lost.");
    connectionActive = false;
  }
  
  // Print quaternion values for debugging
  if (connectionActive) {
    println("Quaternion: w=" + qw + ", x=" + qx + ", y=" + qy + ", z=" + qz);
  }
  
  // Draw the 3D scene
  draw3DScene();
  
  // Draw UI panels
  if (showRawData) drawRawDataPanel();
  if (showCalibratedData) drawCalibratedDataPanel();
  if (showMetrics) drawMetricsPanel();
  
  // Draw Compass Roses (User Request)
  drawCompassPanel();
  
  // Draw connection status
  fill(0);
  textAlign(LEFT, TOP);
  textSize(14);
  if (connectionActive) {
    fill(0, 150, 0);
    text("Connected to " + serialPorts[selectedPort] + " - Last data: " + (millis() - lastDataReceived) + "ms ago", 10, height - 20);
  } else {
    fill(150, 0, 0);
    text("No data from " + serialPorts[selectedPort] + " - Press 'R' to reconnect", 10, height - 20);
  }
}

// =============================================================================
//  3D Scene Drawing
// =============================================================================

void draw3DScene() {
  // Set up the 3D camera and lighting
  camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);
  lights();
  
  // Move to the center of the screen to establish the origin for 3D drawing
  pushMatrix();
  translate(width / 2, height / 2, 0);
  
  // --- Apply Airplane Rotation ---
  // Normalize quaternion to prevent scaling issues
  float qMag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  if (qMag > 0) {
    qw /= qMag; qx /= qMag; qy /= qMag; qz /= qMag;
  }
  
  // Convert quaternion to a rotation matrix and apply it
  applyQuaternionRotation(qw, qx, qy, qz);

  // --- Draw the Airplane ---
  // This function draws the user-provided detailed airplane model
  drawAirplane();
  
  // Draw coordinate system (uncomment for debugging)
  // drawCoordinateSystem();
  
  // Rotation is now handled by Tare/Quaternion logic
  
  // Restore the matrix to its state before we moved to the center
  popMatrix();
}

// ==========================================
// AUTOMATED MAPPING WIZARD
// ==========================================
int wizardState = 0; // 0=Off, 1=Flat, 2=NoseUp, 3=RollRight
float[] baseAccel = new float[3];

// Calculated Mapping (Verified by User Screenshot 2)
int mapX = 1; // Roll = RawY
int signX = 1;
int mapY = 2; // Pitch = RawZ
int signY = -1;
int mapZ = 0; // Yaw = RawX
int signZ = -1;

// TARE STATE (Zero Offset)
float[] tareQuat = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z conjugate
boolean isTared = false;

void tareSensor() {
    float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    float w = qw/norm;
    float x = qx/norm;
    float y = qy/norm;
    float z = qz/norm;
    
    // Step 1: Calculate Conjugate (Inverse of current orientation)
    float cw = w;
    float cx = -x;
    float cy = -y;
    float cz = -z;
    
    // Step 2: Apply 180° roll (around X axis) to flip plane right-side up
    // Quaternion for 180° X rotation: (0, 1, 0, 0)
    float roll_w = 0;
    float roll_x = 1;
    float roll_y = 0;
    float roll_z = 0;
    
    // Multiply: Roll * Conjugate
    float temp_w = roll_w*cw - roll_x*cx - roll_y*cy - roll_z*cz;
    float temp_x = roll_w*cx + roll_x*cw + roll_y*cz - roll_z*cy;
    float temp_y = roll_w*cy - roll_x*cz + roll_y*cw + roll_z*cx;
    float temp_z = roll_w*cz + roll_x*cy - roll_y*cx + roll_z*cw;
    
    // Step 3: Apply 180° yaw (around Z axis) to reverse nose direction
    // Quaternion for 180° Z rotation: (0, 0, 0, 1)
    float yaw_w = 0;
    float yaw_x = 0;
    float yaw_y = 0;
    float yaw_z = 1;
    
    // Multiply: Yaw * (Roll * Conjugate)
    tareQuat[0] = yaw_w*temp_w - yaw_x*temp_x - yaw_y*temp_y - yaw_z*temp_z;
    tareQuat[1] = yaw_w*temp_x + yaw_x*temp_w + yaw_y*temp_z - yaw_z*temp_y;
    tareQuat[2] = yaw_w*temp_y - yaw_x*temp_z + yaw_y*temp_w + yaw_z*temp_x;
    tareQuat[3] = yaw_w*temp_z + yaw_x*temp_y - yaw_y*temp_x + yaw_z*temp_w;

    isTared = true;
    println("SENSOR TARED - Plane flipped right-side up and nose reversed!");
}

String getAxisName(int idx) {
  if (idx == 0) return "RawX";
  if (idx == 1) return "RawY";
  if (idx == 2) return "RawZ";
  return "?";
}

void runWizardStep() {
  float[] curr = {raw_ax, raw_ay, raw_az};
  
  if (wizardState == 1) { // READING FLAT (Reference)
    baseAccel = curr;
    // Auto-detect Gravity Axis (Z)
    int maxAxis = 0;
    float maxVal = 0;
    for(int i=0; i<3; i++) {
        if(abs(curr[i]) > maxVal) { maxVal = abs(curr[i]); maxAxis = i; }
    }
    mapZ = maxAxis;
    // If gravity is negative (normal), sign is +, if positive, sign is - (to invert)
    // We want internal Z to be UP (+1g). If sensor shows -1000, we need -1 sign to make it +1000
    // Wait, standard convention: Gravity vector is DOWN. Accelerometer measures UP force. 
    // Variable 'signZ' corrects it to Standard Frame.
    // Let's assume we want +Z Up.
    signZ = (curr[maxAxis] > 0) ? 1 : -1; 
    
    wizardState = 2; // Next Step
    
  } else if (wizardState == 2) { // READING NOSE UP (Pitch)
    // Find axis that changed the most relative to base
    int maxAxis = -1;
    float maxDiff = 0;
    for(int i=0; i<3; i++) {
       if (i == mapZ) continue; // Skip gravity axis
       float diff = curr[i] - baseAccel[i]; // Change
       if (abs(diff) > maxDiff) { maxDiff = abs(diff); maxAxis = i; }
    }
    
    if (maxAxis != -1) {
        mapY = maxAxis; // This axis controls Pitch
        // Nose UP should result in NEGATIVE Pitch in some frames? 
        // Let's standard: Nose UP -> Pitch Increase.
        // We will refine sign later or let user flip it. 
        // For now, capture the mapping.
        float diff = curr[maxAxis] - baseAccel[maxAxis];
        // Heuristic: If value increased, sign is X.
        signY = (diff > 0) ? 1 : -1;
        wizardState = 3;
    }
    
  } else if (wizardState == 3) { // READING ROLL RIGHT
     // Find the remaining axis
     for(int i=0; i<3; i++) {
         if (i != mapZ && i != mapY) {
             mapX = i;
             float diff = curr[i] - baseAccel[i];
             signX = (diff > 0) ? 1 : -1;
             break;
         }
     }
     wizardState = 4; // Done
     printConfig();
  }
}

void printConfig() {
    println("\n=== WIZARD RESULTS ===");
    println("mapX = " + mapX + "; signX = " + signX + ";");
    println("mapY = " + mapY + "; signY = " + signY + ";");
    println("mapZ = " + mapZ + "; signZ = " + signZ + ";");
    println("======================\n");
}

void applyQuaternionRotation(float w, float x, float y, float z) {
  // Normalize Input
  float qMag = sqrt(w*w + x*x + y*y + z*z);
  if (qMag > 0) { w/=qMag; x/=qMag; y/=qMag; z/=qMag; }
  
  // APPLY TARE (If enabled)
  // Q_final = Q_tare * Q_current
  if (isTared) {
      float tw = tareQuat[0];
      float tx = tareQuat[1];
      float ty = tareQuat[2];
      float tz = tareQuat[3];
      
      // Hamiltonian Product
      float newW = tw*w - tx*x - ty*y - tz*z;
      float newX = tw*x + tx*w + ty*z - tz*y;
      float newY = tw*y - tx*z + ty*w + tz*x;
      float newZ = tw*z + tx*y - ty*x + tz*w;
      
      w = newW; x = newX; y = newY; z = newZ;
  }

  // Convert quaternion directly to rotation matrix
  // Use the quaternion as-is from ESP32
  transform.set(
    1 - 2*y*y - 2*z*z,   2*x*y - 2*w*z,     2*x*z + 2*w*y,     0,
    2*x*y + 2*w*z,     1 - 2*x*x - 2*z*z,   2*y*z - 2*w*x,     0,
    2*x*z - 2*w*y,     2*y*z + 2*w*x,     1 - 2*x*x - 2*y*y,   0,
    0,                 0,                 0,                 1
  );
  
  applyMatrix(transform);
  
  // FIX PLANE UPSIDE DOWN: Apply 180° roll to flip plane right-side up
  rotateX(PI);
  
  // TARE HUD
  pushMatrix();
  camera(); 
  hint(DISABLE_DEPTH_TEST);
  noLights();
  fill(0, 0, 0, 150);
  rect(0, height-80, width, 80);
  fill(255);
  textSize(18);
  textAlign(CENTER, CENTER);
  
  if (isTared) {
      fill(0, 255, 0);
      text("SENSOR TARED - Press [R] to Reset Tare", width/2, height - 40);
  } else {
      fill(255, 255, 0);
      text("Press [T] to TARE sensor (zero current orientation)", width/2, height - 40);
  }
  
  hint(ENABLE_DEPTH_TEST);
  popMatrix();
}

void drawCoordinateSystem() {
  pushMatrix();
  
  // Draw coordinate axes to match the new Arduino axis mapping
  // Arduino mapping: X=-Y, Y=X, Z=-Z applied to all sensors
  
  // Draw X axis (RED) - Forward direction (Arduino mapped X)
  stroke(255, 0, 0);
  strokeWeight(3);
  line(0, 0, 0, 100, 0, 0);
  
  // Draw Y axis (GREEN) - Right direction (Arduino mapped Y)
  stroke(0, 255, 0);
  strokeWeight(3);
  line(0, 0, 0, 0, 100, 0);
  
  // Draw Z axis (BLUE) - Up direction (Arduino mapped Z)
  stroke(0, 0, 255);
  strokeWeight(3);
  line(0, 0, 0, 0, 0, 100);
  
  // Add axis labels for clarity
  textSize(12);
  fill(255, 0, 0);
  text("X (Forward)", 105, 0, 0);
  fill(0, 255, 0);
  text("Y (Right)", 0, 105, 0);
  fill(0, 0, 255);
  text("Z (Up)", 0, 0, 105);
  
  // Reset stroke and fill
  strokeWeight(1);
  stroke(0);
  fill(255);
  
  popMatrix();
}

void drawAirplane() {
  // Single-piece airplane using triangles
  beginShape(TRIANGLES);
  
  // Define colors for different parts
  // RED for nose (front), BLUE for tail (back), GREEN for wings
  
  // === FUSELAGE (BODY) ===
  fill(200, 200, 200); // Gray fuselage
  
  // Top of fuselage
  vertex(0, -15, 100);    // nose top
  vertex(-10, -15, -100); // tail top left
  vertex(10, -15, -100);  // tail top right
  
  // Bottom of fuselage  
  vertex(0, 15, 100);     // nose bottom
  vertex(10, 15, -100);   // tail bottom right
  vertex(-10, 15, -100);  // tail bottom left
  
  // Left side of fuselage
  vertex(0, -15, 100);    // nose top
  vertex(-10, -15, -100); // tail top left
  vertex(0, 15, 100);     // nose bottom
  
  vertex(-10, -15, -100); // tail top left
  vertex(-10, 15, -100);  // tail bottom left
  vertex(0, 15, 100);     // nose bottom
  
  // Right side of fuselage
  vertex(0, -15, 100);    // nose top
  vertex(0, 15, 100);     // nose bottom
  vertex(10, -15, -100);  // tail top right
  
  vertex(10, -15, -100);  // tail top right
  vertex(0, 15, 100);     // nose bottom
  vertex(10, 15, -100);   // tail bottom right
  
  // === NOSE (FRONT - RED) ===
  fill(255, 0, 0); // Red nose for front identification
  
  // Nose cone triangles
  vertex(0, -15, 100);    // top of nose
  vertex(0, 15, 100);     // bottom of nose
  vertex(0, 0, 150);      // pointed tip (FRONT)
  
  // === MAIN WINGS (GREEN) ===
  fill(0, 255, 0); // Green wings
  
  // Left wing top
  vertex(-10, -15, 20);   // wing root top
  vertex(-80, -15, 0);    // wing tip top
  vertex(-80, -15, 40);   // wing tip back top
  
  vertex(-10, -15, 20);   // wing root top
  vertex(-80, -15, 40);   // wing tip back top
  vertex(-10, -15, 40);   // wing root back top
  
  // Left wing bottom
  vertex(-10, -10, 20);   // wing root bottom
  vertex(-80, -10, 40);   // wing tip back bottom
  vertex(-80, -10, 0);    // wing tip bottom
  
  vertex(-10, -10, 20);   // wing root bottom
  vertex(-10, -10, 40);   // wing root back bottom
  vertex(-80, -10, 40);   // wing tip back bottom
  
  // Left wing leading edge
  vertex(-10, -15, 20);   // wing root top
  vertex(-80, -15, 0);    // wing tip top
  vertex(-10, -10, 20);   // wing root bottom
  
  vertex(-80, -15, 0);    // wing tip top
  vertex(-80, -10, 0);    // wing tip bottom
  vertex(-10, -10, 20);   // wing root bottom
  
  // Left wing trailing edge
  vertex(-10, -15, 40);   // wing root back top
  vertex(-10, -10, 40);   // wing root back bottom
  vertex(-80, -15, 40);   // wing tip back top
  
  vertex(-80, -15, 40);   // wing tip back top
  vertex(-10, -10, 40);   // wing root back bottom
  vertex(-80, -10, 40);   // wing tip back bottom
  
  // Right wing top
  vertex(10, -15, 20);    // wing root top
  vertex(80, -15, 40);    // wing tip back top
  vertex(80, -15, 0);     // wing tip top
  
  vertex(10, -15, 20);    // wing root top
  vertex(10, -15, 40);    // wing root back top
  vertex(80, -15, 40);    // wing tip back top
  
  // Right wing bottom
  vertex(10, -10, 20);    // wing root bottom
  vertex(80, -10, 0);     // wing tip bottom
  vertex(80, -10, 40);    // wing tip back bottom
  
  vertex(10, -10, 20);    // wing root bottom
  vertex(80, -10, 40);    // wing tip back bottom
  vertex(10, -10, 40);    // wing root back bottom
  
  // Right wing leading edge
  vertex(10, -15, 20);    // wing root top
  vertex(10, -10, 20);    // wing root bottom
  vertex(80, -15, 0);     // wing tip top
  
  vertex(80, -15, 0);     // wing tip top
  vertex(10, -10, 20);    // wing root bottom
  vertex(80, -10, 0);     // wing tip bottom
  
  // Right wing trailing edge
  vertex(10, -15, 40);    // wing root back top
  vertex(80, -15, 40);    // wing tip back top
  vertex(10, -10, 40);    // wing root back bottom
  
  vertex(80, -15, 40);    // wing tip back top
  vertex(80, -10, 40);    // wing tip back bottom
  vertex(10, -10, 40);    // wing root back bottom
  
  // === TAIL FIN (BACK - BLUE) ===
  fill(0, 0, 255); // Blue tail for back identification
  
  // Vertical tail fin
  vertex(0, -15, -100);   // tail root top
  vertex(0, -60, -80);    // tail tip top
  vertex(0, -15, -120);   // tail root back
  
  vertex(0, -60, -80);    // tail tip top
  vertex(0, -60, -120);   // tail tip back
  vertex(0, -15, -120);   // tail root back
  
  // === HORIZONTAL STABILIZERS (YELLOW) ===
  fill(255, 255, 0); // Yellow tail wings
  
  // Left horizontal stabilizer
  vertex(-5, -15, -90);   // root
  vertex(-30, -15, -85);  // tip
  vertex(-30, -15, -105); // tip back
  
  vertex(-5, -15, -90);   // root
  vertex(-30, -15, -105); // tip back
  vertex(-5, -15, -110);  // root back
  
  // Right horizontal stabilizer
  vertex(5, -15, -90);    // root
  vertex(30, -15, -105);  // tip back
  vertex(30, -15, -85);   // tip
  
  vertex(5, -15, -90);    // root
  vertex(5, -15, -110);   // root back
  vertex(30, -15, -105);  // tip back
  
  endShape();
}

// =============================================================================
//  UI Drawing Functions
// =============================================================================

void drawRawDataPanel() {
  fill(0, 0, 0, 200);
  rect(20, 20, 280, 220, 10);
  fill(255);
  textSize(16);
  textAlign(LEFT, TOP);
  text("Raw Sensor Data", 35, 35);
  
  textSize(14);
  text("Accel (mg):     X: " + df2.format(raw_ax) + " Y: " + df2.format(raw_ay) + " Z: " + df2.format(raw_az), 35, 65);
  text("Gyro (dps):     X: " + df2.format(raw_gx) + " Y: " + df2.format(raw_gy) + " Z: " + df2.format(raw_gz), 35, 90);
  text("Mag (uT):       X: " + df2.format(raw_mx) + " Y: " + df2.format(raw_my) + " Z: " + df2.format(raw_mz), 35, 115);
  text("Quaternion:     W: " + df2.format(qw) + " X: " + df2.format(qx) + " Y: " + df2.format(qy) + " Z: " + df2.format(qz), 35, 140);
  text("Euler (deg):    R: " + df1.format(roll) + " P: " + df1.format(pitch) + " Y: " + df1.format(yaw), 35, 165);
  text("Heading (deg):  Mag: " + df1.format(magHeading) + " True: " + df1.format(trueHeading), 35, 190);
}

void drawCalibratedDataPanel() {
  fill(0, 0, 0, 200);
  rect(20, 260, 280, 150, 10);
  fill(255);
  textSize(16);
  textAlign(LEFT, TOP);
  text("Calibrated Sensor Data", 35, 275);
  
  textSize(14);
  text("Accel (mg):     X: " + df2.format(cal_ax) + " Y: " + df2.format(cal_ay) + " Z: " + df2.format(cal_az), 35, 305);
  text("Gyro (dps):     X: " + df2.format(cal_gx) + " Y: " + df2.format(cal_gy) + " Z: " + df2.format(cal_gz), 35, 330);
  text("Mag (uT):       X: " + df2.format(cal_mx) + " Y: " + df2.format(cal_my) + " Z: " + df2.format(cal_mz), 35, 355);
  text("Heading (deg):  Mag: " + df1.format(magHeading) + " True: " + df1.format(trueHeading), 35, 380);
}

void drawMetricsPanel() {
  fill(0, 0, 0, 200);
  rect(width - 300, 20, 280, 200, 10);
  fill(255);
  textSize(16);
  textAlign(LEFT, TOP);
  text("Calibration Metrics", width - 285, 35);
  
  textSize(14);
  
  // Accel magnitude check
  float accel_mag_g = accel_mag / 1000.0f;
  fill(abs(accel_mag_g - 1.0) > 0.1 ? color(255, 100, 100) : color(100, 255, 100));
  text("Accel Magnitude:  " + df2.format(accel_mag) + " mg", width - 285, 65);
  text(abs(accel_mag_g - 1.0) > 0.1 ? "BAD" : "GOOD", width - 80, 65);
  
  // Gyro magnitude check
  fill(gyro_mag > 5.0 ? color(255, 100, 100) : color(100, 255, 100));
  text("Gyro Magnitude:   " + df2.format(gyro_mag) + " dps", width - 285, 90);
  text(gyro_mag > 5.0 ? "BAD" : "GOOD", width - 80, 90);
  
  // Mag magnitude check
  fill(mag_mag < 25 || mag_mag > 65 ? color(255, 100, 100) : color(100, 255, 100));
  text("Mag Magnitude:    " + df2.format(mag_mag) + " uT", width - 285, 115);
  text(mag_mag < 25 || mag_mag > 65 ? "BAD" : "GOOD", width - 80, 115);
  
  // Quat magnitude check
  float qMag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  fill(abs(qMag - 1.0) > 0.05 ? color(255, 100, 100) : color(100, 255, 100));
  text("Quat Magnitude:   " + df2.format(qMag), width - 285, 140);
  text(abs(qMag - 1.0) > 0.05 ? "BAD" : "GOOD", width - 80, 140);
  
  // True heading
  fill(255);
  text("True Heading:       " + df1.format(trueHeading) + "°", width - 285, 165);
  
  // Overall status
  boolean isCalibrated = abs(accel_mag_g - 1.0) <= 0.1 && gyro_mag <= 5.0 && mag_mag >= 25 && mag_mag <= 65;
  fill(isCalibrated ? color(100, 255, 100) : color(255, 100, 100));
  text("Overall Status:   " + (isCalibrated ? "CALIBRATED" : "NEEDS CALIBRATION"), width - 285, 190);
}

void drawCompassPanel() {
  int y = height - 200; // Lowered by 100px (was -300)
  int spacing = 160; // Spacing between compass roses
  float startXFloat = width / 2.0 - spacing * 1.5; // Float calculation
  int startX = int(startXFloat); // Cast to int

  // Display 1: True Heading
  drawCompassRose(trueHeading, "TRUE HEADING", startX, y);
  
  // Display 2: Magnetic Heading
  drawCompassRose(magHeading, "MAG HEADING", startX + spacing, y);
  
  // Display 3: Tilt Compensated Heading (Yaw)
  float displayYaw = yaw;
  if (displayYaw < 0) displayYaw += 360;
  drawCompassRose(displayYaw, "TILT COMP (YAW)", startX + spacing * 2, y);
  
  // Display 4: Algorithmic Heading (NEW!)
  float displayAlg = algHeading;
  // Check for error code (-1.0)
  String algLabel = (displayAlg < 0) ? "ALG (ERROR)" : "ALG HEADING";
  if (displayAlg < 0) displayAlg = 0; // Show 0 instead of -1
  drawCompassRose(displayAlg, algLabel, startX + spacing * 3, y);
}

void drawCompassRose(float heading, String label, int x, int y) {
  pushMatrix();
  translate(x, y);
  
  // Background - Lighter for visibility of black text
  fill(240, 240, 240, 200); 
  stroke(0);
  strokeWeight(3);
  ellipse(0, 0, 130, 130); // Slightly larger
  
  // Ticks and Numbers
  textSize(8); // Small font for numbers
  textAlign(CENTER, CENTER);
  
  for (int i = 0; i < 360; i+=30) {
    float rad = radians(i);
    float r1 = 50;
    float r2 = 58;
    float rText = 40; // Radius for numbers
    
    // Ticks
    stroke(0);
    strokeWeight(1);
    line(cos(rad)*r1, sin(rad)*r1, cos(rad)*r2, sin(rad)*r2);
    
    // Numbers (0, 30, 60...)
    fill(0);
    if (i % 90 != 0) { // Don't draw numbers over N/E/S/W
       text(str(i), cos(rad-HALF_PI)*rText, sin(rad-HALF_PI)*rText);
    }
  }
  
  // Cardinal Directions Labels (BOLD RED/BLACK)
  textSize(14); // "Bold" size
  fill(0); // Black for E/W/S
  text("E", 48, 0);  
  text("S", 0, 48);  
  text("W", -48, 0); 
  
  fill(255, 0, 0); // RED for North
  text("N", 0, -48); 
  
  // Needle/Pointer (Rotates opposite to heading)
  pushMatrix();
  rotate(radians(-heading));
  
  // Red North Point
  fill(255, 0, 0);
  noStroke();
  beginShape();
  vertex(0, -50);
  vertex(8, 0);
  vertex(-8, 0);
  endShape(CLOSE);
  
  // Black South Point
  fill(0);
  beginShape();
  vertex(0, 50);
  vertex(8, 0);
  vertex(-8, 0);
  endShape(CLOSE);
  
  // Center Pin
  fill(200);
  ellipse(0, 0, 12, 12);
  
  popMatrix();
  
  // External Text Label
  fill(0); // Black label
  textSize(14);
  text(label, 0, 80);
  
  // Heading Value
  fill(255, 0, 0); // Red Value
  textSize(18); // Bold/Large
  text(df1.format(heading) + "°", 0, 100);
  
  popMatrix();
}

void drawPortSelectionScreen() {
  fill(255);
  textAlign(CENTER, CENTER);
  textSize(24);
  text("Select Serial Port:", width/2, height/2 - 100);
  
  textSize(16);
  for (int i = 0; i < serialPorts.length; i++) {
    text("Press '" + i + "' for: " + serialPorts[i], width/2, height/2 - 50 + i * 25);
  }
}

// =============================================================================
//  Event Handlers
// =============================================================================

void keyPressed() {
  // Handle serial port selection
  if (selectedPort == -1) {
    if (key >= '0' && key <= '9') {
      int portIndex = key - '0';
      if (portIndex < serialPorts.length) {
        selectedPort = portIndex;
        connectToPort(portIndex);
      }
    }
    return;
  }
  
  // TARE CONTROLS
  if (key == 't' || key == 'T') {
    tareSensor();
  }
  
  if (key == 'r' || key == 'R') {
    if (isTared) {
      // Reset tare
      tareQuat[0] = 1.0;
      tareQuat[1] = 0.0;
      tareQuat[2] = 0.0;
      tareQuat[3] = 0.0;
      isTared = false;
      println("TARE RESET!");
    } else if (!connectionActive) {
      println("Attempting to reconnect to port " + serialPorts[selectedPort]);
      connectToPort(selectedPort);
    }
  }
  
  // Handle UI toggles
  if (key == 'd' || key == 'D') showRawData = !showRawData;
  if (key == 'c' || key == 'C') showCalibratedData = !showCalibratedData;
  if (key == 'm' || key == 'M') showMetrics = !showMetrics;
  
  if (key == 'h' || key == 'H') {
    println("Airplane Orientation:");
    println("RED = Front (Nose)");
    println("BLUE = Back (Tail)");
    println("GREEN = Wings");
    println("YELLOW = Tail stabilizers");
    println("Move mouse to rotate view");
  }
}

// Helper function to connect to a serial port
void connectToPort(int portIndex) {
  try {
    // Close existing connection if any
    if (esp32Serial != null) {
      esp32Serial.stop();
      println("Closed existing serial connection");
    }
    
    // Open new connection
    println("Connecting to " + serialPorts[portIndex] + "...");
    esp32Serial = new Serial(this, serialPorts[portIndex], 115200);
    esp32Serial.bufferUntil('\n');
    selectedPort = portIndex;
    println("Connected to " + serialPorts[portIndex]);
  } catch (Exception e) {
    println("Error opening serial port: " + e.getMessage());
    selectedPort = -1; // Reset selection on error
    connectionActive = false;
  }
}

void serialEvent(Serial port) {
  try {
    String inString = port.readStringUntil('\n');
    if (inString != null) {
      inString = trim(inString);
      
      // Update connection status
      lastDataReceived = millis();
      connectionActive = true;
      
      // Debug - print raw data (uncomment for detailed debugging)
      // println("Raw data: " + inString);
      
      // Check if this is a data packet
      if (!inString.startsWith("DATA,")) {
        println("Received non-data message: " + inString);
        return;
      }
      
      // Remove the "DATA," prefix
      inString = inString.substring(5);
      
      // Parse CSV data
      String[] values = split(inString, ',');
      
      // Debug - print number of values (uncomment for detailed debugging)
      // println("Received " + values.length + " values");
      
      if (values.length >= 28) { // We expect at least 28 values
        // Raw accel
        raw_ax = float(values[0]);
        raw_ay = float(values[1]);
        raw_az = float(values[2]);
        
        // Raw gyro
        raw_gx = float(values[3]);
        raw_gy = float(values[4]);
        raw_gz = float(values[5]);
        
        // Raw mag
        raw_mx = float(values[6]);
        raw_my = float(values[7]);
        raw_mz = float(values[8]);
        
        // Calibrated accel
        cal_ax = float(values[9]);
        cal_ay = float(values[10]);
        cal_az = float(values[11]);
        
        // Calibrated gyro
        cal_gx = float(values[12]);
        cal_gy = float(values[13]);
        cal_gz = float(values[14]);
        
        // Calibrated mag
        cal_mx = float(values[15]);
        cal_my = float(values[16]);
        cal_mz = float(values[17]);
        
        // Mag magnitude
        mag_mag = float(values[18]);
        
        // Quaternion
        qw = float(values[19]);
        qx = float(values[20]);
        qy = float(values[21]);
        qz = float(values[22]);
        
        // Euler angles
        roll = float(values[23]);
        pitch = float(values[24]);
        yaw = float(values[25]);
        
        // Headings
        magHeading = float(values[26]);
        trueHeading = float(values[27]);
        
        // Algorithmic Heading (column 28 - may be -1.0 if error)
        if (values.length >= 29) {
          algHeading = float(values[28]);
        } else {
          algHeading = -1.0; // Error/not available
        }
        
        // Calculate magnitudes for display
        accel_mag = sqrt(cal_ax*cal_ax + cal_ay*cal_ay + cal_az*cal_az);
        gyro_mag = sqrt(cal_gx*cal_gx + cal_gy*cal_gy + cal_gz*cal_gz);
        
        // Debug - print quaternion values for detailed debugging
        // println("Quat: w=" + qw + ", x=" + qx + ", y=" + qy + ", z=" + qz);
      } else {
        println("ERROR: Not enough values in data packet. Expected 28+, got " + values.length);
      }
    }
  } catch (Exception e) {
    println("Error parsing serial data: " + e.getMessage());
    e.printStackTrace();
  }
}