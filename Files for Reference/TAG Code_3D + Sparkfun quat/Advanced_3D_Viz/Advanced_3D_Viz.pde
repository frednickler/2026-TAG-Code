/*
  3D IMU Airplane Visualizer
  EXACT COPY of working simple_data_check implementation
  With automated axis calibration wizard
*/

import processing.serial.*;

Serial myPort;
boolean connected = false;

// Sensor data
float[] q = {1, 0, 0, 0};
float[] rawAcc = {0,0,0}, rawGyro = {0,0,0}, rawMag = {0,0,0};
float[] calAcc = {0,0,0}, calGyro = {0,0,0}, calMag = {0,0,0};
float roll, pitch, yaw, magHeading;

// TARE
boolean isTared = false;
float[] tareQ = {1, 0, 0, 0};

// AXIS MAPPING (from working sketch)
int mapX = 1, mapY = 2, mapZ = 0;
float signX = 1, signY = 1, signZ = -1;

// WIZARD STATE
int wizardState = 0; // 0=Off, 1=Flat, 2=NoseUp, 3=RollRight, 4=Done
float[] baseAccel = new float[3];

void setup() {
  size(1400, 900, P3D);
  
  println("Available ports:");
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) {
    println("[" + i + "] " + ports[i]);
  }
  
  for (String p : ports) {
    if (p.toLowerCase().contains("usb") || p.toLowerCase().contains("slab")) {
      try {
        println("Attempting connection to: " + p + " at 115200 baud");
        myPort = new Serial(this, p, 115200);
        myPort.bufferUntil('\n');
        connected = true;
        println("✓ Connected successfully!");
        break;
      } catch (Exception e) {
        println("✗ Connection failed: " + e.getMessage());
      }
    }
  }
  
  if (!connected) {
    println("ERROR: Could not connect. Press 0-9 to select port manually.");
  }
}

void draw() {
  background(135, 206, 235);
  
  // 3D Scene
  pushMatrix();
  translate(250, 350, 0);
  camera(500, -300, 500, 0, 0, 0, 0, 1, 0);
  lights();
  
  drawGrid();
  applyQuaternion();
  drawAirplane();
  
  popMatrix();
  
  // Data panels
  camera();
  noLights();
  drawDataPanels();
  
  // WIZARD OVERLAY
  if (wizardState > 0) drawWizard();
}

void drawGrid() {
  stroke(100); strokeWeight(1);
  for (int i = -500; i <= 500; i += 100) {
    line(i, 0, -500, i, 0, 500);
    line(-500, 0, i, 500, 0, i);
  }
  strokeWeight(4);
  stroke(255,0,0); line(0,0,0, 150,0,0);
  stroke(0,255,0); line(0,0,0, 0,150,0);
  stroke(0,0,255); line(0,0,0, 0,0,150);
}

void applyQuaternion() {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  float mag = sqrt(w*w + x*x + y*y + z*z);
  if (mag > 0) { w/=mag; x/=mag; y/=mag; z/=mag; }
  
  // Apply tare
  if (isTared) {
    float tw = tareQ[0], tx = tareQ[1], ty = tareQ[2], tz = tareQ[3];
    float nw = tw*w - tx*x - ty*y - tz*z;
    float nx = tw*x + tx*w + ty*z - tz*y;
    float ny = tw*y - tx*z + ty*w + tz*x;
    float nz = tw*z + tx*y - ty*x + tz*w;
    w = nw; x = nx; y = ny; z = nz;
  }
  
  // CRITICAL: Axis mapping on quaternion components
  float[] raw = {x, y, z};
  x = raw[mapX] * signX;
  y = raw[mapY] * signY;
  z = raw[mapZ] * signZ;
  
  // Convert to matrix
  PMatrix3D m = new PMatrix3D();
  m.set(
    1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y, 0,
    2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x, 0,
    2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y, 0,
    0, 0, 0, 1
  );
  applyMatrix(m);
}

void drawAirplane() {
  scale(2);
  beginShape(TRIANGLES);
  
  fill(200);
  vertex(0,-15,100); vertex(-10,-15,-100); vertex(10,-15,-100);
  vertex(0,15,100); vertex(10,15,-100); vertex(-10,15,-100);
  vertex(0,-15,100); vertex(-10,-15,-100); vertex(0,15,100);
  vertex(-10,-15,-100); vertex(-10,15,-100); vertex(0,15,100);
  vertex(0,-15,100); vertex(0,15,100); vertex(10,-15,-100);
  vertex(10,-15,-100); vertex(0,15,100); vertex(10,15,-100);
  
  fill(255,0,0);
  vertex(0,-15,100); vertex(0,15,100); vertex(0,0,150);
  
  fill(0,255,0);
  vertex(-10,-15,20); vertex(-80,-15,0); vertex(-80,-15,40);
  vertex(-10,-15,20); vertex(-80,-15,40); vertex(-10,-15,40);
  vertex(10,-15,20); vertex(80,-15,40); vertex(80,-15,0);
  vertex(10,-15,20); vertex(10,-15,40); vertex(80,-15,40);
  
  fill(0,0,255);
  vertex(0,-15,-100); vertex(0,-60,-80); vertex(0,-15,-120);
  
  fill(255,255,0);
  vertex(-5,-15,-90); vertex(-30,-15,-85); vertex(-30,-15,-105);
  vertex(5,-15,-90); vertex(30,-15,-105); vertex(30,-15,-85);
  
  endShape();
}

void drawDataPanels() {
  int x = 800, y = 20;
  
  drawPanel(x, y, "RAW SENSOR", 560);
  y += 30; fill(0);
  text("Acc: " + nf(rawAcc[0],0,1) + ", " + nf(rawAcc[1],0,1) + ", " + nf(rawAcc[2],0,1), x+10, y+=20);
  text("Gyr: " + nf(rawGyro[0],0,1) + ", " + nf(rawGyro[1],0,1) + ", " + nf(rawGyro[2],0,1), x+10, y+=20);
  text("Mag: " + nf(rawMag[0],0,1) + ", " + nf(rawMag[1],0,1) + ", " + nf(rawMag[2],0,1), x+10, y+=20);
  
  y += 30;
  drawPanel(x, y, "CALIBRATED", 560);
  y += 30; fill(0);
  text("Acc: " + nf(calAcc[0],0,1) + ", " + nf(calAcc[1],0,1) + ", " + nf(calAcc[2],0,1), x+10, y+=20);
  text("Gyr: " + nf(calGyro[0],0,1) + ", " + nf(calGyro[1],0,1) + ", " + nf(calGyro[2],0,1), x+10, y+=20);
  text("Mag: " + nf(calMag[0],0,1) + ", " + nf(calMag[1],0,1) + ", " + nf(calMag[2],0,1), x+10, y+=20);
  
  y += 30;
  drawPanel(x, y, "QUATERNION", 560);
  y += 30; fill(0);
  text("W: " + nf(q[0],0,4) + "  X: " + nf(q[1],0,4), x+10, y+=20);
  text("Y: " + nf(q[2],0,4) + "  Z: " + nf(q[3],0,4), x+10, y+=20);
  
  y += 30;
  drawPanel(x, y, "EULER", 560);
  y += 30; fill(0);
  text("Roll: " + nf(roll,0,1) + "°  Pitch: " + nf(pitch,0,1) + "°", x+10, y+=20);
  text("Yaw: " + nf(yaw,0,1) + "°  Heading: " + nf(magHeading,0,1) + "°", x+10, y+=20);
  
  y += 30;
  drawPanel(x, y, "CONTROLS", 560);
  y += 30; fill(0,100,200);
  text("[T] Tare: " + (isTared?"ON":"OFF") + "  [Space] Reset", x+10, y+=20);
  text("[SPACE] Start Axis Wizard", x+10, y+=20);
  text("[1,2,3] Manual axis  [Q,W,E] Flip sign", x+10, y+=20);
  
  y += 30;
  drawPanel(x, y, "MAPPING", 560);
  y += 30; fill(0);
  String[] n = {"X","Y","Z"};
  text("X→" + n[mapX] + "(" + (signX>0?"+":"-") + ") Y→" + n[mapY] + "(" + (signY>0?"+":"-") + ") Z→" + n[mapZ] + "(" + (signZ>0?"+":"-") + ")", x+10, y+=20);
  
  fill(0);
  text("FPS: " + int(frameRate), x+10, height-40);
  text(connected ? "✓ Connected" : "✗ Disconnected", x+10, height-20);
}

void drawWizard() {
  fill(0, 200);
  rect(0, height-200, width, 200);
  fill(255); textSize(24); textAlign(CENTER);
  
  if (wizardState == 1) {
    fill(255,255,0);
    text("STEP 1: Place sensor FLAT on table", width/2, height-150);
    fill(255); textSize(16);
    text("Press [ENTER] when ready", width/2, height-100);
  } else if (wizardState == 2) {
    fill(255,255,0);
    text("STEP 2: Tilt NOSE UP 45°", width/2, height-150);
    fill(255); textSize(16);
    text("Press [ENTER] when ready", width/2, height-100);
  } else if (wizardState == 3) {
    fill(255,255,0);
    text("STEP 3: Bank RIGHT (right wing down)", width/2, height-150);
    fill(255); textSize(16);
    text("Press [ENTER] when ready", width/2, height-100);
  } else if (wizardState == 4) {
    fill(0,255,0);
    text("✓ CALIBRATION COMPLETE!", width/2, height-150);
    fill(255); textSize(16);
    text("mapX=" + mapX + " signX=" + (int)signX + " | mapY=" + mapY + " signY=" + (int)signY + " | mapZ=" + mapZ + " signZ=" + (int)signZ, width/2, height-100);
    text("Press [Q,W,E] to flip if backwards | Press [SPACE] to restart", width/2, height-70);
  }
  textAlign(LEFT);
}

void runWizardStep() {
  float[] curr = {rawAcc[0], rawAcc[1], rawAcc[2]};
  
  if (wizardState == 1) {
    baseAccel = curr;
    int maxAxis = 0;
    float maxVal = 0;
    for(int i=0; i<3; i++) {
      if(abs(curr[i]) > maxVal) { maxVal = abs(curr[i]); maxAxis = i; }
    }
    mapZ = maxAxis;
    signZ = (curr[maxAxis] > 0) ? 1 : -1;
    wizardState = 2;
    println("Z-axis detected: " + mapZ);
  } else if (wizardState == 2) {
    int maxAxis = -1;
    float maxDiff = 0;
    for(int i=0; i<3; i++) {
      if (i == mapZ) continue;
      float diff = curr[i] - baseAccel[i];
      if (abs(diff) > maxDiff) { maxDiff = abs(diff); maxAxis = i; }
    }
    if (maxAxis != -1) {
      mapY = maxAxis;
      float diff = curr[maxAxis] - baseAccel[maxAxis];
      signY = (diff > 0) ? 1 : -1;
      wizardState = 3;
      println("Y-axis detected: " + mapY);
    }
  } else if (wizardState == 3) {
    for(int i=0; i<3; i++) {
      if (i != mapZ && i != mapY) {
        mapX = i;
        float diff = curr[i] - baseAccel[i];
        signX = (diff > 0) ? 1 : -1;
        break;
      }
    }
    wizardState = 4;
    println("X-axis detected: " + mapX);
    println("DONE! mapX=" + mapX + " mapY=" + mapY + " mapZ=" + mapZ);
  }
}

void drawPanel(int x, int y, String title, int w) {
  fill(255, 250); noStroke();
  rect(x, y, w, 25);
  fill(0); textSize(14);
  text(title, x+10, y+17);
}

void keyPressed() {
  if (key == ' ') {
    if (wizardState == 0 || wizardState == 4) wizardState = 1;
  }
  if (key == ENTER || key == RETURN) {
    if (wizardState >= 1 && wizardState <= 3) runWizardStep();
  }
  
  if (key == 't' || key == 'T') {
    if (isTared) {
      isTared = false;
      println("TARE: OFF");
    } else {
      float mag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
      tareQ[0] = q[0]/mag;
      tareQ[1] = -q[1]/mag;
      tareQ[2] = -q[2]/mag;
      tareQ[3] = -q[3]/mag;
      isTared = true;
      println("TARE: ON");
    }
  }
  
  if (key == '1') { mapX = (mapX + 1) % 3; println("mapX=" + mapX); }
  if (key == '2') { mapY = (mapY + 1) % 3; println("mapY=" + mapY); }
  if (key == '3') { mapZ = (mapZ + 1) % 3; println("mapZ=" + mapZ); }
  if (key == 'q' || key == 'Q') { signX *= -1; println("signX=" + (int)signX); }
  if (key == 'w' || key == 'W') { signY *= -1; println("signY=" + (int)signY); }
  if (key == 'e' || key == 'E') { signZ *= -1; println("signZ=" + (int)signZ); }
}

void serialEvent(Serial p) {
  try {
    String s = p.readStringUntil('\n');
    if (s == null || !s.startsWith("DATA,")) return;
    
    String[] v = split(s.substring(5), ',');
    if (v.length >= 28) {
      rawAcc[0] = float(v[0]); rawAcc[1] = float(v[1]); rawAcc[2] = float(v[2]);
      rawGyro[0] = float(v[3]); rawGyro[1] = float(v[4]); rawGyro[2] = float(v[5]);
      rawMag[0] = float(v[6]); rawMag[1] = float(v[7]); rawMag[2] = float(v[8]);
      calAcc[0] = float(v[9]); calAcc[1] = float(v[10]); calAcc[2] = float(v[11]);
      calGyro[0] = float(v[12]); calGyro[1] = float(v[13]); calGyro[2] = float(v[14]);
      calMag[0] = float(v[15]); calMag[1] = float(v[16]); calMag[2] = float(v[17]);
      q[0] = float(v[19]); q[1] = float(v[20]); q[2] = float(v[21]); q[3] = float(v[22]);
      roll = float(v[23]); pitch = float(v[24]); yaw = float(v[25]);
      magHeading = float(v[26]);
    }
  } catch (Exception e) {}
}
