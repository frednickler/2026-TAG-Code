import serial
import sys

port = '/dev/cu.usbserial-110'
baud = 115200

try:
    ser = serial.Serial(port, baud, timeout=0.1)
    print(f"Connected to {port} at {baud} baud. Monitoring serial output...")
    print("=" * 60)
    
    while True:
        if ser.in_waiting:
            try:
                data = ser.read(ser.in_waiting)
                print(data.decode('utf-8', errors='replace'), end='', flush=True)
            except KeyboardInterrupt:
                print("\n\nMonitoring stopped.")
                break
            except Exception as e:
                print(f"\nError: {e}")
                
except Exception as e:
    print(f"Failed to open port: {e}")
    sys.exit(1)
finally:
    if 'ser' in locals():
        ser.close()
