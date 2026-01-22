class Visuals {
  boolean isTared = false;
  float[] tareQ = {1, 0, 0, 0};
  
  // Axis mapping (defaults from working sketch)
  int[] axisMap = {1, 2, 0}; // X->Y, Y->Z, Z->X
  float[] axisSigns = {1, 1, -1}; // Z inverted
  float[] baseRot = {90, 0, 0}; // Pitch, Roll, Yaw offsets (Z=0 for correct yaw)
  
  void draw3D(float[] q) {
    // Grid
    stroke(40); strokeWeight(1);
    for (int i = -500; i <= 500; i += 100) {
      line(i, 0, -500, i, 0, 500);
      line(-500, 0, i, 500, 0, i);
    }
    
    // Axes
    strokeWeight(4);
    stroke(255, 0, 0); line(0,0,0, 150,0,0); // X
    stroke(0, 255, 0); line(0,0,0, 0,150,0); // Y
    stroke(0, 0, 255); line(0,0,0, 0,0,150); // Z
    
    // Apply quaternion rotation
    pushMatrix();
    float[] qUse = q.clone();
    normalize(qUse);
    if (isTared) qUse = multiply(tareQ, qUse);
    
    float[] m = toMatrix(qUse);
    float[] r = remap(m);
    applyMatrix(r[0],r[1],r[2],0, r[3],r[4],r[5],0, r[6],r[7],r[8],0, 0,0,0,1);
    
    rotateX(radians(baseRot[0]));
    rotateY(radians(baseRot[1]));
    rotateZ(radians(baseRot[2]));
    
    drawCube();
    popMatrix();
  }
  
  void drawCube() {
    lights();
    
    // Simple colored cube - easy to see orientation
    noStroke();
    
    // Front (RED)
    fill(255, 0, 0, 200);
    beginShape();
    vertex(-50, -50, 100);
    vertex(50, -50, 100);
    vertex(50, 50, 100);
    vertex(-50, 50, 100);
    endShape(CLOSE);
    
    // Back (DARK RED)
    fill(100, 0, 0, 200);
    beginShape();
    vertex(-50, -50, -100);
    vertex(-50, 50, -100);
    vertex(50, 50, -100);
    vertex(50, -50, -100);
    endShape(CLOSE);
    
    // Top (GREEN)
    fill(0, 255, 0, 200);
    beginShape();
    vertex(-50, -50, -100);
    vertex(50, -50, -100);
    vertex(50, -50, 100);
    vertex(-50, -50, 100);
    endShape(CLOSE);
    
    // Bottom (DARK GREEN)
    fill(0, 100, 0, 200);
    beginShape();
    vertex(-50, 50, -100);
    vertex(-50, 50, 100);
    vertex(50, 50, 100);
    vertex(50, 50, -100);
    endShape(CLOSE);
    
    // Right (BLUE)
    fill(0, 0, 255, 200);
    beginShape();
    vertex(50, -50, -100);
    vertex(50, 50, -100);
    vertex(50, 50, 100);
    vertex(50, -50, 100);
    endShape(CLOSE);
    
    // Left (DARK BLUE)
    fill(0, 0, 100, 200);
    beginShape();
    vertex(-50, -50, -100);
    vertex(-50, -50, 100);
    vertex(-50, 50, 100);
    vertex(-50, 50, -100);
    endShape(CLOSE);
    
    // Arrow on front face
    fill(255, 255, 0);
    pushMatrix();
    translate(0, 0, 101);
    beginShape();
    vertex(0, -30);
    vertex(20, 0);
    vertex(0, 30);
    endShape(CLOSE);
    popMatrix();
  }
  
  void drawDataPanels(SerialHandler sh) {
    int x = 720;
    int y = 20;
    
    // Panel 1: RAW SENSOR DATA
    drawPanel(x, y, "RAW SENSOR DATA");
    y += 25;
    text("Accel: " + nf(sh.rawAcc[0],0,1) + ", " + nf(sh.rawAcc[1],0,1) + ", " + nf(sh.rawAcc[2],0,1), x+10, y+=15);
    text("Gyro:  " + nf(sh.rawGyro[0],0,1) + ", " + nf(sh.rawGyro[1],0,1) + ", " + nf(sh.rawGyro[2],0,1), x+10, y+=15);
    text("Mag:   " + nf(sh.rawMag[0],0,1) + ", " + nf(sh.rawMag[1],0,1) + ", " + nf(sh.rawMag[2],0,1), x+10, y+=15);
    
    // Panel 2: CALIBRATED DATA
    y += 30;
    drawPanel(x, y, "CALIBRATED DATA");
    y += 25;
    text("Accel: " + nf(sh.acc[0],0,1) + ", " + nf(sh.acc[1],0,1) + ", " + nf(sh.acc[2],0,1), x+10, y+=15);
    text("Gyro:  " + nf(sh.gyro[0],0,1) + ", " + nf(sh.gyro[1],0,1) + ", " + nf(sh.gyro[2],0,1), x+10, y+=15);
    text("Mag:   " + nf(sh.mag[0],0,1) + ", " + nf(sh.mag[1],0,1) + ", " + nf(sh.mag[2],0,1), x+10, y+=15);
    
    // Panel 3: QUATERNION (FUSED)
    y += 30;
    drawPanel(x, y, "QUATERNION (FUSED)");
    y += 25;
    text("W: " + nf(sh.q[0],0,4), x+10, y+=15);
    text("X: " + nf(sh.q[1],0,4), x+10, y+=15);
    text("Y: " + nf(sh.q[2],0,4), x+10, y+=15);
    text("Z: " + nf(sh.q[3],0,4), x+10, y+=15);
    
    // Panel 4: CONTROLS
    y += 30;
    drawPanel(x, y, "CONTROLS");
    y += 25;
    fill(255, 255, 0);
    text("[T] Tare: " + (isTared?"ON":"OFF"), x+10, y+=15);
    text("[Space] Reset Tare", x+10, y+=15);
    text("[R] Reset Camera", x+10, y+=15);
    text("[Mouse] Rotate View", x+10, y+=15);
    y += 10;
    text("[1,2,3] Cycle Axis", x+10, y+=15);
    text("[!,@,#] Flip Sign", x+10, y+=15);
    text("[4,5,6] Adjust Base", x+10, y+=15);
    
    // Panel 5: MAPPING
    y += 30;
    drawPanel(x, y, "AXIS MAPPING");
    y += 25;
    fill(200, 200, 255);
    String[] names = {"X", "Y", "Z"};
    text("Map X -> " + names[axisMap[0]] + " (" + (axisSigns[0]>0?"+":"-") + ")", x+10, y+=15);
    text("Map Y -> " + names[axisMap[1]] + " (" + (axisSigns[1]>0?"+":"-") + ")", x+10, y+=15);
    text("Map Z -> " + names[axisMap[2]] + " (" + (axisSigns[2]>0?"+":"-") + ")", x+10, y+=15);
    text("Base: " + int(baseRot[0]) + "°, " + int(baseRot[1]) + "°, " + int(baseRot[2]) + "°", x+10, y+=15);
    
    // Status
    fill(255);
    text("FPS: " + int(frameRate), x+10, height-20);
    text("Status: " + sh.getStatus(), x+10, height-5);
  }
  
  void drawPanel(int x, int y, String title) {
    fill(0, 150);
    noStroke();
    rect(x, y, 650, 20);
    fill(255);
    text(title, x+10, y+15);
  }
  
  void toggleTare(float[] q) {
    if (isTared) {
      isTared = false;
    } else {
      normalize(q);
      tareQ[0] = q[0];
      tareQ[1] = -q[1];
      tareQ[2] = -q[2];
      tareQ[3] = -q[3];
      isTared = true;
    }
  }
  
  void resetTare() { isTared = false; }
  
  void cycleAxis(int i) { axisMap[i] = (axisMap[i] + 1) % 3; }
  void flipSign(int i) { axisSigns[i] *= -1; }
  void adjustBase(int i, float amt) { baseRot[i] += amt; }
  
  void normalize(float[] q) {
    float m = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (m > 0) { q[0]/=m; q[1]/=m; q[2]/=m; q[3]/=m; }
  }
  
  float[] multiply(float[] a, float[] b) {
    return new float[] {
      a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
      a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
      a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
      a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    };
  }
  
  float[] toMatrix(float[] q) {
    float w=q[0], x=q[1], y=q[2], z=q[3];
    return new float[] {
      1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y,
      2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x,
      2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y
    };
  }
  
  float[] remap(float[] m) {
    float[] r = new float[9];
    for (int row = 0; row < 3; row++) {
      r[0 + row*3] = m[axisMap[0] + row*3] * axisSigns[0];
      r[1 + row*3] = m[axisMap[1] + row*3] * axisSigns[1];
      r[2 + row*3] = m[axisMap[2] + row*3] * axisSigns[2];
    }
    return r;
  }
}
