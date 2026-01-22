import serial
import time
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        ser.dtr = False
        time.sleep(0.1)
        ser.dtr = True # Pulse DTR to reset ESP32
        
        print(f"Connected to {args.port} (Resetting board...)")
        
        start = time.time()
        while time.time() - start < 15:
            if ser.in_waiting:
                try:
                    data = ser.read(ser.in_waiting)
                    print(data.decode('utf-8', errors='ignore'), end='', flush=True)
                except Exception as e:
                    print(f"Error reading: {e}")
            time.sleep(0.01)
            
    except Exception as e:
        print(f"Failed to open port: {e}")

if __name__ == '__main__':
    main()
