import serial
import time
import sys

def capture_verification(port='/dev/cu.usbserial-110', baud=115200, duration=20):
    print(f"Opening {port} at {baud}...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    # Wait for connection
    time.sleep(2)
    
    # Send a newline to refresh menu if stuck
    ser.write(b'\n')
    time.sleep(0.5)

    print("Navigating Menu...")
    
    # Reset menu state by sending 'x' multiple times to reach Main Menu or Tracking Mode
    # If in Tracking, 'x' goes to Main Menu.
    # If in Sub-menu, 'x' goes back.
    ser.write(b'x')
    time.sleep(0.5)
    ser.write(b'x')
    time.sleep(0.5)
    ser.write(b'x')
    time.sleep(0.5)

    # Now we should be in Main Menu.
    # Send '6' to enter Config Menu (confirmed)
    ser.write(b'6')
    time.sleep(1)
    
    # Send 'x' to Exit to Tracking Mode (which starts streaming)
    # The output in 'capture_verification.csv' showed: "[x] Exit to Tracking Mode"
    # We send 'x' to trigger sysState = SYS_RUNNING, which streams DATA.
    ser.write(b'x')
    time.sleep(1)
    
    print(f"Capturing data for {duration} seconds...")
    start_time = time.time()
    
    with open('calibration_tool/verification_capture.csv', 'w') as f:
        while time.time() - start_time < duration:
            try:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(line) # Echo to console
                    f.write(line + '\n')
            except Exception as e:
                print(f"Read error: {e}")
                break
                
    print("Capture complete.")
    ser.close()

if __name__ == "__main__":
    capture_verification()
