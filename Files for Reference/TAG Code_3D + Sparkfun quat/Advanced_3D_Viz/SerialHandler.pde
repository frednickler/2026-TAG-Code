class SerialHandler {
  PApplet parent;
  Serial port;
  boolean connected = false;
  String lastError = "";
  
  // All sensor data
  float[] q = {1, 0, 0, 0};
  float[] rawAcc = {0,0,0};
  float[] rawGyro = {0,0,0};
  float[] rawMag = {0,0,0};
  float[] acc = {0,0,0};
  float[] gyro = {0,0,0};
  float[] mag = {0,0,0};
  
  String buffer = "";
  
  SerialHandler(PApplet p) {
    parent = p;
    scanPorts();
  }
  
  void scanPorts() {
    String[] portList = Serial.list();
    if (port != null) { port.stop(); port = null; }
    
    if (portList.length > 0) {
      String portName = portList[portList.length - 1];
      for (String s : portList) {
        if (s.toLowerCase().contains("usb") || s.toLowerCase().contains("slab")) {
          portName = s;
          break;
        }
      }
      
      try {
        port = new Serial(parent, portName, BAUD_RATE);
        port.bufferUntil('\n');
        connected = true;
        println("Connected: " + portName);
      } catch (Exception e) {
        lastError = e.getMessage();
        connected = false;
      }
    } else {
      lastError = "No ports";
      connected = false;
    }
  }
  
  void update() {
    if (!connected && frameCount % 120 == 0) scanPorts();
  }
  
  void onSerialEvent(Serial p) {
    try {
      String s = p.readString();
      if (s == null) return;
      
      buffer += s;
      int idx = buffer.lastIndexOf('\n');
      if (idx > 0) {
        String block = buffer.substring(0, idx);
        buffer = buffer.substring(idx + 1);
        String[] lines = split(block, '\n');
        
        for (int i = lines.length - 1; i >= 0; i--) {
          if (trim(lines[i]).startsWith("DATA,")) {
            parse(trim(lines[i]));
            break;
          }
        }
      }
    } catch (Exception e) {}
  }
  
  void parse(String line) {
    try {
      String[] items = split(line.substring(5), ',');
      if (items.length >= 23) {
        // Raw (0-8)
        rawAcc[0] = float(items[0]); rawAcc[1] = float(items[1]); rawAcc[2] = float(items[2]);
        rawGyro[0] = float(items[3]); rawGyro[1] = float(items[4]); rawGyro[2] = float(items[5]);
        rawMag[0] = float(items[6]); rawMag[1] = float(items[7]); rawMag[2] = float(items[8]);
        
        // Calibrated (9-17)
        acc[0] = float(items[9]); acc[1] = float(items[10]); acc[2] = float(items[11]);
        gyro[0] = float(items[12]); gyro[1] = float(items[13]); gyro[2] = float(items[14]);
        mag[0] = float(items[15]); mag[1] = float(items[16]); mag[2] = float(items[17]);
        
        // Quaternion (19-22)
        float w = float(items[19]);
        if (!Float.isNaN(w)) {
          q[0] = w;
          q[1] = float(items[20]);
          q[2] = float(items[21]);
          q[3] = float(items[22]);
        }
      }
    } catch (Exception e) {}
  }
  
  String getStatus() { return connected ? "Connected" : "Disconnected"; }
}
