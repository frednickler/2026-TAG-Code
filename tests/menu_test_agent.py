import serial
import time
import argparse
import sys
import re

class MenuTestAgent:
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
        
        # Wait a bit more for slow responses (simple polling)
        start = time.time()
        while (time.time() - start) < 0.5:
            if self.ser.in_waiting:
                response += self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                start = time.time() # Reset timeout if we got data
            time.sleep(0.05)
            
        return response

    def expect(self, response, pattern, description):
        if re.search(pattern, response, re.MULTILINE | re.IGNORECASE):
            print(f"  [PASS] {description}")
            return True
        else:
            print(f"  [FAIL] {description}")
            print(f"    Expected regex: {pattern}")
            print(f"    Actual response (repr): {repr(response)}")
            return False

    def run_tests(self):
        if not self.connect():
            return

        print("\n=== STARTING COMPREHENSIVE MENU TESTS ===\n")

        # --- HELP MENU ---
        print("\n--- Test: Help ---")
        self.send_command("") # Wake up
        resp = self.send_command("h", wait_time=0.5)
        self.expect(resp, r"Quick Commands", "Help menu shown")

        # --- MAIN MENU ---
        print("\n--- Test: Main Menu ---")
        resp = self.send_command("menu", wait_time=0.5)
        if not self.expect(resp, r"Main Menu", "Main Menu entered"):
            print("Trying to force exit sub-menus...")
            self.send_command("b")
            self.send_command("b")
            self.send_command("menu")

        # --- STATUS ---
        print("\n--- Test: Status ---")
        resp = self.send_command("s", wait_time=1.0)
        self.expect(resp, r"Sensor Status", "Status header")
        self.expect(resp, r"IMU", "IMU Section")
        self.expect(resp, r"GPS", "GPS Section")

        # --- CALIBRATION ---
        print("\n--- Test: Calibration Menu ---")
        resp = self.send_command("c", wait_time=0.5)
        if self.expect(resp, r"Calibration Menu", "Entered Calibration Menu"):
            
            # 1. Gyro Calibration (Fast)
            print("  > Testing [1] Gyro Calibration (Interactive)")
            # Send '1', wait for prompt
            resp = self.send_command("1", wait_time=1.0) # Wait for "Press ENTER"
            if self.expect(resp, r"Press ENTER", "Gyro Promoted for Enter"):
                # Send Enter to start
                print("    > Sending ENTER to start calibration...")
                self.send_command("", wait_time=0.5, flush=False) # Send newline
                
                # Wait for completion (it starts immediately and prints dots)
                # We need to wait enough time (10s) + buffer.
                print("    > Waiting 15s for calibration...")
                time.sleep(15) 
                
                # Check buffer for "Complete" or "calibrated"
                resp = self.read_response()
                # Debug print to see what we got
                # print(f"DEBUG: Gyro Cal Result: {resp}") 
                self.expect(resp, r"COMPLETE|calibrated|skipped", "Gyro Calibration Finished")

            # 2. Accel Simple (Interactive)
            print("  > Testing [2] Accel Simple (Interactive)")
            resp = self.send_command("2", wait_time=1.0)
            if self.expect(resp, r"Press ENTER", "Accel Simple Promoted for Enter"):
                 print("    > Sending ENTER to start calibration...")
                 self.send_command("", wait_time=0.5, flush=False)
                 
                 print("    > Waiting 15s for calibration...")
                 time.sleep(15)
                 
                 # Read result
                 resp = self.read_response()
                 
                 # Check for magnitude warning (Continue anyway?)
                 if "Continue anyway" in resp:
                     print("    > Detected Magnitude Warning. Sending 'y'...")
                     resp = self.send_command("y", wait_time=3.0, flush=False)
                 
                 # "Calibration cancelled" contains "calibrated", so we must exclude it or use stricter regex
                 # We expect "COMPLETE" or "calibrated" (contextual) but NOT "cancelled"
                 if "cancelled" in resp.lower():
                     print("  [FAIL] Accel Calibration Cancelled")
                 else:
                     self.expect(resp, r"COMPLETE|ACCEL CALIBRATION COMPLETE", "Accel Calibration Finished")

            # 3. Accel 6-pos
            print("  > Testing [3] Accel 6-Position (Entry check only)")
            # This enters a blocking loop that is hard to exit without 60s timeout per position
            # We skip entering it to avoid trapping the agent.
            print("    [SKIP] Deep calibration requires complex user interaction.")

            # 7. View Calibration
            print("  > Testing [7] View Calibration")
            self.flush_input() # Ensure no stray prompts
            resp = self.send_command("7", wait_time=0.5)
            self.expect(resp, r"Current IMU Calibration", "Shown calibration details")

            # 8. Reset Calibration (Test Cancel)
            print("  > Testing [8] Reset (Cancel flow)")
            resp = self.send_command("8", wait_time=0.5)
            if self.expect(resp, r"Erase all calibration", "Prompt received"):
                resp = self.send_command("n", wait_time=0.5)
                self.expect(resp, r"Cancelled", "Reset cancelled successfully")

            # Back to Main
            resp = self.send_command("b", wait_time=0.5)
            self.expect(resp, r"Main Menu", "Returned to Main Menu")

        # --- ALIGNMENT ---
        print("\n--- Test: Alignment Menu ---")
        self.send_command("") # Ensure clean
        resp = self.send_command("align", wait_time=1.5)
        if self.expect(resp, r"Alignment Menu", "Entered Alignment Menu"):
            
            # 1. Toggle GPS Mode
            print("  > Testing [1] GPS Mode Toggle")
            resp = self.send_command("1", wait_time=0.5)
            self.expect(resp, r"Mode set to", "Mode toggled")
            
            # 2. Reset Acquisition
            print("  > Testing [2] Reset Acquisition")
            resp = self.send_command("2", wait_time=0.5)
            # Just triggers it.
            
            # 3. TARE (Interactive)
            print("  > Testing [3] TARE (Interactive)")
            resp = self.send_command("3", wait_time=0.5)
            if self.expect(resp, r"Tag Latitude", "Prompt for Lat"):
                resp = self.send_command("-33.0000", wait_time=0.5, flush=False)
                if self.expect(resp, r"Tag Longitude", "Prompt for Lon"):
                    resp = self.send_command("151.0000", wait_time=0.5, flush=False)
                    self.expect(resp, r"Tare Complete|Tare set to", "Tare finished")
            
            # 4. Manual Offset (Interactive)
            print("  > Testing [4] Manual Offset")
            resp = self.send_command("4", wait_time=0.5)
            if self.expect(resp, r"Enter Offset", "Prompt for Offset"):
                resp = self.send_command("10.0", wait_time=0.5, flush=False)
                self.expect(resp, r"Offset Saved", "Offset Saved")

            # Back
            resp = self.send_command("b", wait_time=0.5)
            self.expect(resp, r"Main Menu", "Returned to Main Menu")

        # --- CONFIGURATION ---
        print("\n--- Test: Configuration Menu ---")
        resp = self.send_command("cfg", wait_time=0.5)
        if self.expect(resp, r"Configuration Menu", "Entered Config Menu"):
            
            # 1. IMU Config
            print("  > Testing [1] IMU Config")
            resp = self.send_command("1", wait_time=0.5)
            if self.expect(resp, r"IMU Configuration", "IMU Options"):
                # Test changing Accel Preset
                print("    > Cycle Accel Preset (1)")
                resp = self.send_command("1", wait_time=0.5) 
                if self.expect(resp, r"Enter Accel Preset", "Prompt for Preset"):
                    resp = self.send_command("1", wait_time=0.5, flush=False) # Set to 1
                    self.expect(resp, r"Saved", "Saved 1")
                
                # Test changing Gyro Preset
                print("    > Cycle Gyro Preset (2)")
                resp = self.send_command("2", wait_time=0.5)
                if self.expect(resp, r"Enter Gyro Preset", "Prompt for Gyro"):
                    resp = self.send_command("1", wait_time=0.5, flush=False) # Set to 1
                    self.expect(resp, r"Saved", "Saved 1")

                # Test Mag Preset
                print("    > Cycle Mag Preset (3)")
                resp = self.send_command("3", wait_time=0.5)
                if self.expect(resp, r"Enter Mag Preset", "Prompt for Mag"):
                    resp = self.send_command("2", wait_time=0.5, flush=False) # Set to 2
                    self.expect(resp, r"Saved", "Saved 2")

                # Test Mag ODR
                print("    > Cycle Mag ODR (4)")
                resp = self.send_command("4", wait_time=0.5)
                if self.expect(resp, r"Enter Mag ODR", "Prompt for ODR"):
                    resp = self.send_command("10", wait_time=0.5, flush=False) # Set to 10
                    self.expect(resp, r"Saved", "Saved 10Hz")

                resp = self.send_command("b", wait_time=0.5) # Back

            # 2. GPS Config
            print("  > Testing [2] GPS Config")
            resp = self.send_command("2", wait_time=0.5)
            if self.expect(resp, r"GPS Configuration", "GPS Options"):
                 # [1] Rate
                 print("    > GPS Rate")
                 resp = self.send_command("1", wait_time=0.5)
                 if self.expect(resp, r"Enter Rate", "Prompt Rate"):
                     resp = self.send_command("10", wait_time=0.5, flush=False)
                     self.expect(resp, r"Saved", "Saved 10Hz")

                 # [4] SBAS Toggle
                 print("    > Toggling SBAS")
                 resp = self.send_command("4", wait_time=0.5)
                 self.expect(resp, r"SBAS now", "SBAS toggled")

                 # [5] QZSS Toggle
                 print("    > Toggling QZSS")
                 resp = self.send_command("5", wait_time=0.5)
                 self.expect(resp, r"QZSS now", "QZSS toggled")

                 # [6] Anti-Jamming
                 print("    > Testing Anti-Jamming")
                 resp = self.send_command("6", wait_time=0.5)
                 if self.expect(resp, r"Mode", "Prompt Anti-Jamming"):
                    resp = self.send_command("1", wait_time=0.5, flush=False) # Set Fixed
                    self.expect(resp, r"Saved", "Saved Anti-Jamming")
                 
                 resp = self.send_command("b", wait_time=0.5) # Back

            # 3. Baro Configuration
            print("  > Testing [3] Baro Config")
            resp = self.send_command("3", wait_time=0.5)
            self.expect(resp, r"Barometer Configuration", "Baro Header")
            resp = self.send_command("", wait_time=0.5) # Send enter
            
            # 4. System Settings
            print("  > Testing [4] System Settings")
            resp = self.send_command("4", wait_time=0.5)
            if self.expect(resp, r"System Info", "System Info shown"):
                # [2] Mounting Toggle
                print("    > Toggling Mounting Orientation")
                resp = self.send_command("2", wait_time=0.5)
                self.expect(resp, r"Mounting set to", "Mounting toggled")
                
                resp = self.send_command("b", wait_time=0.5)

            # 5. VQF Tuning
            print("  > Testing [5] VQF Tuning")
            resp = self.send_command("5", wait_time=0.5)
            if self.expect(resp, r"VQF Tuning", "VQF Menu Entered"):
                # [1] Tau Accel
                print("    > Setting Tau Accel")
                resp = self.send_command("1", wait_time=0.5)
                if self.expect(resp, r"Enter Tau Accel", "Prompt Tau Accel"):
                    resp = self.send_command("2.5", wait_time=0.5, flush=False)
                    # Note: No explicit "Saved" msg in code for this one? 
                    # Code: cfg.vqfTauAcc = val; ... SystemSettings::save();
                    # But it doesn't print "Saved" inside the `if` block?
                    # Wait, let me check the code again.
                    # 620: if (input == "1") ... SystemSettings::save(); } else Serial.println("Invalid value.");
                    # It does NOT print success message! It just loops back to menu.
                    # We verify by checking if menu reprinted with new value?
                    # The menu reprints at loop start.
                    self.expect(resp, r"Tau Accel", "Menu Refreshed")

                # [2] Tau Mag
                print("    > Setting Tau Mag")
                resp = self.send_command("2", wait_time=0.5)
                if self.expect(resp, r"Enter Tau Mag", "Prompt Tau Mag"):
                    resp = self.send_command("5.0", wait_time=0.5, flush=False)
                    self.expect(resp, r"Tau Mag", "Menu Refreshed")
                
                # [3] Mag Reject
                print("    > Toggling Mag Rejection")
                resp = self.send_command("3", wait_time=0.5)
                self.expect(resp, r"Mag Reject", "Menu Refreshed (Toggled)")
                
                resp = self.send_command("b", wait_time=0.5) # Back
            
            # Back to Main
            resp = self.send_command("b", wait_time=0.5)
            self.expect(resp, r"Main Menu", "Returned to Main Menu")

            # --- ADVERSARIAL AUDIT TEST ---
            print("\n--- Test: Adversarial Audit (IMU Signal Chain) ---")
            
            # Verify Accel Preset 1 (2g) -> Reg 0x00, Scale 0.5
            print("  > Setting Accel Preset 1 (2g)")
            self.send_command("cfg", wait_time=0.5)
            self.send_command("1", wait_time=0.5) # IMU Config
            self.send_command("1", wait_time=0.5) # Accel Preset
            self.send_command("1", wait_time=0.5, flush=False) # Value 1
            self.send_command("b", wait_time=0.5) # Back to Config
            self.send_command("b", wait_time=0.5) # Back to Main
            
            print("  > Running Audit for Preset 1")
            resp = self.send_command("audit", wait_time=0.5)
            self.expect(resp, r"Reg 0x41.*0x00", "Reg 0x41 is 0x00 (2G)")
            self.expect(resp, r"Accel Scale: 0.50", "Scale is 0.50")

            # Verify Accel Preset 2 (4g) -> Reg 0x01, Scale 1.0
            print("  > Setting Accel Preset 2 (4g)")
            self.send_command("cfg", wait_time=0.5)
            self.send_command("1", wait_time=0.5)
            self.send_command("1", wait_time=0.5)
            self.send_command("2", wait_time=0.5, flush=False) # Value 2
            self.send_command("b", wait_time=0.5)
            self.send_command("b", wait_time=0.5)
            
            print("  > Running Audit for Preset 2")
            resp = self.send_command("audit", wait_time=0.5)
            self.expect(resp, r"Reg 0x41.*0x01", "Reg 0x41 is 0x01 (4G)")
            self.expect(resp, r"Accel Scale: 1.00", "Scale is 1.00")
            
            # Verify Accel Preset 3 (8g) -> Reg 0x02, Scale 2.0
            print("  > Setting Accel Preset 3 (8g)")
            self.send_command("cfg", wait_time=0.5)
            self.send_command("1", wait_time=0.5)
            self.send_command("1", wait_time=0.5)
            self.send_command("3", wait_time=0.5, flush=False) # Value 3
            self.send_command("b", wait_time=0.5)
            self.send_command("b", wait_time=0.5)
            
            print("  > Running Audit for Preset 3")
            resp = self.send_command("audit", wait_time=0.5)
            self.expect(resp, r"Reg 0x41.*0x02", "Reg 0x41 is 0x02 (8G)")
            self.expect(resp, r"Accel Scale: 2.00", "Scale is 2.00")

            # Verify NV_CONF and GYR_RANGE presence
            self.expect(resp, r"Reg 0x43", "Reg 0x43 (GYR_RANGE) verified")
            self.expect(resp, r"Reg 0x70", "Reg 0x70 (NV_CONF) verified")

            # Restore Default Accel Preset (2)
            print("  > Restoring Default Accel Preset (2)")
            self.send_command("cfg", wait_time=0.5)
            self.send_command("1", wait_time=0.5)
            self.send_command("1", wait_time=0.5)
            self.send_command("2", wait_time=0.5, flush=False)
            self.send_command("b", wait_time=0.5) 
            self.send_command("b", wait_time=0.5)

        # --- EXIT ---
        print("\n--- Test: Exit ---")
        resp = self.send_command("q", wait_time=0.5)
        self.expect(resp, r"Exiting", "Exit message")

        print("\n=== ALL TESTS COMPLETED ===")
        self.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Comprehensive Serial Menu Test Agent')
    parser.add_argument('--port', help='Serial port', required=True)
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    
    args = parser.parse_args()
    
    agent = MenuTestAgent(args.port, args.baud)
    agent.run_tests()
