import serial
import time

def test_serial():
    port = '/dev/cu.usbserial-110'
    print(f"Opening {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
    except Exception as e:
        print(f"Failed: {e}")
        return
    
    print("Connected. Waiting 2s for boot...")
    time.sleep(2)
    
    # Drain any existing data
    while ser.in_waiting:
        print(f"Drained: {ser.read(ser.in_waiting)}")
    
    # Send newline to wake up
    print("Sending newline...")
    ser.write(b'\n')
    time.sleep(1)
    
    # Read whatever comes back
    print("Reading for 5 seconds...")
    start = time.time()
    total_bytes = 0
    while time.time() - start < 5:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            total_bytes += len(data)
            print(f"Received ({len(data)} bytes): {data[:100]}")
        else:
            time.sleep(0.1)
    
    print(f"Total bytes received: {total_bytes}")
    
    # Now send menu commands
    print("\nSending '6' (Config Menu)...")
    ser.write(b'6')
    time.sleep(1)
    
    print("Reading response...")
    start = time.time()
    while time.time() - start < 3:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"Response: {data}")
        else:
            time.sleep(0.1)
    
    # Send 'x' to enter tracking
    print("\nSending 'x' (Start Tracking)...")
    ser.write(b'x')
    time.sleep(1)
    
    print("Reading DATA for 5 seconds...")
    start = time.time()
    lines = 0
    while time.time() - start < 5:
        line = ser.readline()
        if line:
            lines += 1
            if b'DATA' in line:
                print(f"DATA LINE #{lines}: {line[:80]}")
        else:
            time.sleep(0.05)
    
    print(f"Total lines: {lines}")
    ser.close()

if __name__ == "__main__":
    test_serial()
