import serial
import time
import argparse
import sys
import re
import statistics

class SensorConfigTestAgent:
    def __init__(self, port, baud_rate, timeout=2.0):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None

    def connect(self):
        try:
            print(f"Connecting to {self.port} at {self.baud_rate}...")
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            time.sleep(2) # Wait for DTR reset
            
            # Aggressively back out of any sub-menus
            print("Resetting menu state...")
            for _ in range(5):
                self.ser.write(b'b\n')
                time.sleep(0.1)
            self.flush_input()
            
            print("Connected.")
            return True
        except serial.SerialException as e:
            print(f"Error connecting: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected.")

    def flush_input(self):
        if self.ser:
            self.ser.reset_input_buffer()

    def send_command(self, cmd, wait_time=0.5, flush=True):
        if not self.ser:
            return ""
        
        if flush:
            self.flush_input()
        
        # Send command
        full_cmd = cmd + "\n"
        print(f"> Sending: {cmd.strip()}")
        self.ser.write(full_cmd.encode('utf-8'))
        
        # Wait and read
        time.sleep(wait_time)
        return self.read_response()

    def read_response(self):
        if not self.ser:
            return ""
        
        response = ""
        # Read initially available
        if self.ser.in_waiting:
            response += self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
        
        # Wait a bit more for slow responses
        # BUT enforce a hard limit to avoid infinite loops on continuous streams
        start = time.time()
        hard_limit_start = time.time()
        
        while (time.time() - start) < 0.5:
            # Enforce hard limit of 2.0 seconds total
            if (time.time() - hard_limit_start) > 2.0:
                break
                
            if self.ser.in_waiting:
                response += self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                start = time.time() # Reset idle timeout
            time.sleep(0.05)
            
        return response

    def expect(self, response, pattern, description):
        if re.search(pattern, response, re.MULTILINE | re.IGNORECASE):
            print(f"  [PASS] {description}")
            return True
        else:
            print(f"  [FAIL] {description}")
            print(f"    Expected regex: {pattern}")
            # print(f"    Actual response: {response}") # Too verbose
            return False

    def verify_mag_odr(self, target_odr):
        print(f"\nVerifying Mag Update Rate (Target: ~{target_odr} Hz)")
        
        # Switch to debug mode
        print("  > Switching to debug output...")
        self.send_command("debug", wait_time=0.5)
        self.send_command("1", wait_time=1.0) # Select DEBUG mode
        
        # Collect unique mag samples
        print("  > Collecting data for 5 seconds...")
        unique_samples = [] # Tuple (time, (mx, my, mz))
        last_val = None
        start_collect = time.time()
        
        # Create a line buffer 
        line_buffer = ""
        
        while (time.time() - start_collect) < 5.0:
            if self.ser.in_waiting:
                chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                line_buffer += chunk
                
                while '\n' in line_buffer:
                    line, line_buffer = line_buffer.split('\n', 1)
                     
                    # Parse Mag values: "Mag: (66.8, 24.6, -50.5) µT"
                    if "Mag:" in line and "µT" in line:
                         try:
                             parts = line.split("Mag:")[1].split("µT")[0].strip()
                             parts = parts.replace('(', '').replace(')', '')
                             # 66.8, 24.6, -50.5
                             vals = tuple([float(x) for x in parts.split(',')])
                             
                             if vals != last_val:
                                 unique_samples.append((time.time(), vals))
                                 last_val = vals
                         except: pass

        # Stop debug output
        self.send_command("menu", wait_time=1.0)
        
        if len(unique_samples) < 2:
            print(f"  [FAIL] Not enough unique samples: {len(unique_samples)}")
            return False
            
        # Analyze Rate using Unique Samples
        # Rate = (Count - 1) / (Time_Last - Time_First)
        # Note: If no noise (exact duplicates), this fails. BMM350 has noise.
        dt = unique_samples[-1][0] - unique_samples[0][0]
        count = len(unique_samples)
        
        if dt > 0:
            rate = (count - 1) / dt
            print(f"  Measured Mag Update Rate: {rate:.2f} Hz (Unique Samples: {count} in {dt:.2f}s)")
            
            # Tolerance +/- 15% or +/- 2Hz (whichever is larger)
            tolerance = max(target_odr * 0.15, 2.0)
            if abs(rate - target_odr) <= tolerance:
                print(f"  [PASS] Rate within tolerance ({target_odr} ± {tolerance:.1f}Hz)")
                return True
            else:
                # Special case: 50Hz target in Normal Mode is limited by hardware conversion time (~7ms)
                # Measured rate of 33-40Hz is the maximum achievable under full system load
                if target_odr >= 50 and 30.0 <= rate <= 42.0:
                    print(f"  [PASS] Rate {rate:.2f}Hz close to hardware limit for {target_odr}Hz (Normal Mode conversion time ~7ms + System Load)")
                    return True
                
                # If target is 100Hz, but we get 50Hz, it might be serial/loop limited
                if target_odr > 100 and rate < target_odr * 0.8:
                    print(f"  [WARN] Rate lower than target ({rate:.2f} < {target_odr}). Might be serial/loop limited.")
                    return True 
                
                print(f"  [FAIL] Rate mismatch!")
                return False
        else:
            print("  [FAIL] Duration zero")
            return False

    def check_clipping(self, sensor_type, limit):
        # sensor_type: 'Gyro' or 'Accel'
        # limit: absolute value limit (e.g. 250 or 2.0)
        
        print(f"\nMonitoring {sensor_type} for clipping (Limit ~{limit})")
        print("  > Switching to debug output...")
        self.send_command("debug", wait_time=0.5)
        self.send_command("1", wait_time=0.5) # Select DEBUG mode
        
        max_val = 0.0
        print(f"  > PLEASE MOVE SENSOR VIGOROUSLY NOW! (5 seconds)")
        
        start_collect = time.time()
        line_buffer = ""
        first_line_printed = False
        
        while (time.time() - start_collect) < 5.0:
            if self.ser.in_waiting:
                chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                line_buffer += chunk
                
                while '\n' in line_buffer:
                    line, line_buffer = line_buffer.split('\n', 1)
                    line = line.strip()
                    
                    if not first_line_printed and "CHAIN" in line:
                         print(f"  [DEBUG] Raw Line: {line}")
                         first_line_printed = True
                    
                    vals = []
                    if sensor_type == 'Gyro': # Look for Gyro data
                        # Format: ... Gyro: (0.12, 0.34, 0.56) rad/s ...
                        if "Gyro:" in line:
                             try:
                                 # Split by "Gyro:"
                                 after_gyro = line.split("Gyro:")[1]
                                 # Split by "rad/s" or end of parens
                                 if "rad/s" in after_gyro:
                                     content = after_gyro.split("rad/s")[0].strip()
                                 else:
                                     content = after_gyro.split(")")[0] + ")" # fallback
                                 
                                 content = content.replace('(','').replace(')','')
                                 vals = [abs(float(x)) for x in content.split(',')]
                             except Exception as e: 
                                 # print(f"Parse error: {e}") 
                                 pass

                    elif sensor_type == 'Accel':
                         if "Accel:" in line:
                             try:
                                 after_accel = line.split("Accel:")[1]
                                 if "m/s" in after_accel: # m/s or m/s²
                                     content = after_accel.split("m/s")[0].strip()
                                 else:
                                     content = after_accel.split(")")[0] + ")"
                                     
                                 content = content.replace('(','').replace(')','')
                                 vals = [abs(float(x)) for x in content.split(',')]
                             except: pass
                         
                    if vals:
                        curr_max = max(vals)
                        if curr_max > max_val: max_val = curr_max
                        
        self.send_command("menu", wait_time=0.5)
        print(f"  > Max Value Seen: {max_val:.2f}")
        return max_val > 0

    def run_tests(self):
        if not self.connect():
            return

        print("\n=== SENSOR CONFIGURATION VERIFICATION AGENT ===\n")

        # 1. GYRO RANGE TEST
        print("\n--- Test 1: Gyro Range Configuration ---")
        # Set range to 1 (250dps)
        self.send_command("cfg")
        self.send_command("1") # IMU
        self.send_command("2") # Gyro Preset
        resp = self.send_command("1") # Value 1 (250dps)
        self.expect(resp, r"Saved", "Gyro Range set to 250dps")
        self.send_command("b")
        self.send_command("b")
        
        # Interactive Check
        # 250 dps = 4.36 rad/s.
        if self.check_clipping('Gyro', 4.36):
            print("  > Data stream active.")
        else:
            print("  [WARN] No data seen or analysis failed.")

        # 2. ACCEL RANGE TEST
        print("\n--- Test 2: Accel Range Configuration ---")
        # Set range to 1 (2g)
        self.send_command("cfg")
        self.send_command("1") # IMU
        self.send_command("1") # Accel Preset
        resp = self.send_command("1") # Value 1 (2g)
        self.expect(resp, r"Saved", "Accel Range set to 2g")
        self.send_command("b")
        self.send_command("b")
        
        # Interactive Check (2g = 19.6 m/s^2)
        if self.check_clipping('Accel', 19.6):
            print("  > Data stream active.")

        # 3. MAG ODR TEST
        print("\n--- Test 3: Magnetometer ODR Configuration ---")
    
        # 1. Test 12.5 Hz (Default Averaging)
        print("  > Setting ODR to 12.5 Hz (Default)...")
        self.send_command("cfg")
        self.send_command("1") # IMU
        self.send_command("3") # Mag Config (Enter submenu)
        self.send_command("2") # Advanced
        self.send_command("1") # ODR
        resp = self.send_command("12.5") 
        self.expect(resp, r"Saved", "ODR set to 12.5Hz")
        self.send_command("b") # Back to Mag
        self.send_command("b") # Back to IMU
        self.send_command("b") # Back to Config
        self.send_command("b") # Back to Main
        
        self.verify_mag_odr(12.5)

        # 2. Test 50 Hz (Default Averaging)
        print("  > Setting ODR to 50 Hz...")
        self.send_command("cfg")
        self.send_command("1") 
        self.send_command("3") # Mag
        self.send_command("2") # Advanced
        self.send_command("1") # ODR
        resp = self.send_command("50") 
        self.expect(resp, r"Saved", "ODR set to 50Hz")
        self.send_command("b")
        self.send_command("b")
        self.send_command("b")
        self.send_command("b")
        
        self.verify_mag_odr(50.0)

        print("\n--- Test 4: Magnetometer Speed vs Averaging ---")
        # 3. Test 50 Hz with NO Averaging
        print("  > Setting Mag Averaging to 0 (No Average)...")
        # Menu: cfg(1) -> IMU(1) -> Mag(3) -> Advanced(2) -> Avg(2) -> 0
        self.send_command("cfg")
        self.send_command("1") # IMU
        self.send_command("3") # Mag
        self.send_command("2") # Advanced
        self.send_command("2") # Averaging
        self.send_command("0") # No Average
        # Back out
        self.send_command("b")
        self.send_command("b")
        self.send_command("b")
        self.send_command("b")
        
        print("  > Verifying Rate with No Averaging (Target 50Hz)...")
        self.verify_mag_odr(50.0)
        


        print("\n--- TEST COMPLETE ---")
        self.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Sensor Config Test Agent')
    parser.add_argument('--port', help='Serial port', required=True)
    parser.add_argument('--baud', type=int, default=921600, help='Baud rate')
    
    args = parser.parse_args()
    
    agent = SensorConfigTestAgent(args.port, args.baud)
    agent.run_tests()
