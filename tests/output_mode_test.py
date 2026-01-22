import serial
import time
import argparse
import sys
import re

class OutputModeTestAgent:
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

    def expect_not(self, response, pattern, description):
        """Verify pattern does NOT appear in response"""
        if not re.search(pattern, response, re.MULTILINE | re.IGNORECASE):
            print(f"  [PASS] {description}")
            return True
        else:
            print(f"  [FAIL] {description}")
            print(f"    Should NOT contain: {pattern}")
            print(f"    Actual response (repr): {repr(response)}")
            return False

    def wait_and_read(self, duration):
        """Wait for specified duration and collect all output"""
        print(f"  > Waiting {duration}s and collecting output...")
        time.sleep(duration)
        return self.read_response()

    def run_tests(self):
        if not self.connect():
            return

        print("\n=== STARTING OUTPUT MODE TESTS ===\n")

        # --- Test 1: Build Verification (Already Passed) ---
        print("\n--- Test 1: Build Verification ---")
        print("  [INFO] Code should compile without errors")
        print("  [PASS] Build verification (assumed if code is running)")

        # --- Test 2: Startup Mode Selection ---
        # Note: This test is challenging to automate as it requires 
        # catching the boot prompt within 5 seconds. Manual testing recommended.
        print("\n--- Test 2: Startup Mode Selection ---")
        print("  [INFO] This test requires manual verification during startup")
        print("  [INFO] Expected: OUTPUT MODE SELECTION prompt appears after boot")
        print("  [SKIP] Automated testing of startup prompt not implemented")

        # --- Test 3: DEBUG Mode Output ---
        print("\n--- Test 3: DEBUG Mode Output ---")
        resp = self.send_command("d", wait_time=0.5)
        if self.expect(resp, r"Output Mode", "Output mode menu displayed"):
            resp = self.send_command("1", wait_time=0.5, flush=False)
            self.expect(resp, r"DEBUG", "Switched to DEBUG mode")
            
            # Wait and capture output to verify DEBUG mode content
            print("  > Collecting 3s of DEBUG output...")
            time.sleep(3)
            resp = self.read_response()
            
            # Verify DEBUG mode shows detailed chain/euler/vqf data
            self.expect(resp, r"CHAIN\[", "CHAIN debug present")
            self.expect(resp, r"EULER:", "EULER debug present")
            self.expect(resp, r"VQF_INPUT:", "VQF_INPUT debug present")
            # Note: GPS might not have fix, so we check for GPS related output
            has_gps = re.search(r"GPS:", resp, re.IGNORECASE)
            if has_gps:
                print("  [PASS] GPS data present")
            else:
                print("  [INFO] GPS data not visible (may not have fix yet)")

        # --- Test 4: COMPACT Mode Output ---
        print("\n--- Test 4: COMPACT Mode Output ---")
        resp = self.send_command("d", wait_time=0.5)
        if self.expect(resp, r"Output Mode", "Output mode menu displayed"):
            resp = self.send_command("2", wait_time=0.5, flush=False)
            self.expect(resp, r"COMPACT", "Switched to COMPACT mode")
            
            # Wait and capture output
            print("  > Collecting 3s of COMPACT output...")
            time.sleep(3)
            resp = self.read_response()
            
            # Verify COMPACT shows GPS + Heading only, not IMU/Baro debug
            self.expect_not(resp, r"CHAIN\[", "No CHAIN debug in COMPACT")
            self.expect_not(resp, r"EULER:", "No EULER debug in COMPACT")
            self.expect_not(resp, r"VQF_INPUT:", "No VQF_INPUT debug in COMPACT")
            
            # Should have GPS (or "No fix")
            has_gps_output = re.search(r"GPS:|No fix", resp, re.IGNORECASE)
            if has_gps_output:
                print("  [PASS] GPS output present (single line format)")
            else:
                print("  [INFO] GPS output not visible")

        # --- Test 5: SILENT Mode Output ---
        print("\n--- Test 5: SILENT Mode Output ---")
        resp = self.send_command("d", wait_time=0.5)
        if self.expect(resp, r"Output Mode", "Output mode menu displayed"):
            resp = self.send_command("3", wait_time=0.5, flush=False)
            self.expect(resp, r"SILENT", "Switched to SILENT mode")
            
            # Wait and verify no periodic output
            print("  > Collecting 5s of SILENT output...")
            time.sleep(5)
            resp = self.read_response()
            
            # Should have NO sensor data (allowing for ERROR/WARN)
            self.expect_not(resp, r"CHAIN\[", "No CHAIN debug in SILENT")
            self.expect_not(resp, r"GPS:", "No GPS periodic output in SILENT")
            
            # Verify system is still responsive
            resp = self.send_command("menu", wait_time=0.5)
            self.expect(resp, r"Main Menu", "System responsive in SILENT mode")
            self.send_command("b", wait_time=0.3) # Exit menu

        # --- Test 6: ERROR/WARN Always Display ---
        print("\n--- Test 6: ERROR/WARN Always Display ---")
        print("  [INFO] Set mode to SILENT")
        resp = self.send_command("d", wait_time=0.5)
        self.send_command("3", wait_time=0.5, flush=False) # SILENT
        print("  [INFO] To verify ERROR/WARN display, trigger a warning condition")
        print("  [INFO] (e.g., unplug GPS, trigger calibration error, etc.)")
        print("  [SKIP] Automated ERROR/WARN triggering not implemented")
        print("  [INFO] Manual verification: Confirm ERROR/WARN messages appear in SILENT mode")

        # --- Test 7: Runtime Mode Toggle via 'd' Command ---
        print("\n--- Test 7: Runtime Mode Toggle via 'd' Command ---")
        
        # Test 'd' command
        resp = self.send_command("d", wait_time=0.5)
        self.expect(resp, r"Output Mode", "d command opens mode menu")
        self.expect(resp, r"Current Mode:", "Shows current mode")
        self.expect(resp, r"1\. DEBUG", "Option 1: DEBUG listed")
        self.expect(resp, r"2\. COMPACT", "Option 2: COMPACT listed")
        self.expect(resp, r"3\. SILENT", "Option 3: SILENT listed")
        self.expect(resp, r"ERROR/WARN.*always display", "Note about ERROR/WARN shown")
        
        # Test mode change
        resp = self.send_command("1", wait_time=0.5, flush=False)
        self.expect(resp, r"DEBUG", "Mode changed to DEBUG")
        
        # Verify output changes immediately
        time.sleep(2)
        resp = self.read_response()
        has_debug = re.search(r"CHAIN\[|EULER:", resp, re.IGNORECASE)
        if has_debug:
            print("  [PASS] Output updated immediately after mode change")
        else:
            print("  [INFO] Debug output not yet visible (may need more time)")

        # Test 'output' alias
        print("\n  > Testing 'output' command alias")
        resp = self.send_command("output", wait_time=0.5)
        self.expect(resp, r"Output Mode", "'output' command works")
        self.send_command("b", wait_time=0.3) # Back

        # Test 'debug' alias
        print("\n  > Testing 'debug' command alias")
        resp = self.send_command("debug", wait_time=0.5)
        self.expect(resp, r"Output Mode", "'debug' command works")
        self.send_command("b", wait_time=0.3) # Back

        # --- Test 8: Mode Persistence Across Reboots ---
        print("\n--- Test 8: Mode Persistence Across Reboots ---")
        print("  [INFO] Set mode to COMPACT")
        resp = self.send_command("d", wait_time=0.5)
        self.send_command("2", wait_time=0.5, flush=False) # COMPACT
        print("  [INFO] Mode set to COMPACT and saved to NVS")
        print("  [MANUAL] Press ESP32 reset button and verify COMPACT mode persists")
        print("  [SKIP] Automated reboot testing not implemented")

        # --- Test 9: Startup Mode Override ---
        print("\n--- Test 9: Startup Mode Override ---")
        print("  [INFO] This test requires manual verification during startup")
        print("  [MANUAL] Reboot ESP32, press different mode number within 5s")
        print("  [SKIP] Automated startup override testing not implemented")

        # --- Test 10: Serial Menu Works in All Modes ---
        print("\n--- Test 10: Serial Menu Works in All Modes ---")
        
        modes = [
            ("1", "DEBUG"),
            ("2", "COMPACT"),
            ("3", "SILENT")
        ]
        
        for mode_num, mode_name in modes:
            print(f"\n  > Testing menu in {mode_name} mode")
            
            # Set mode
            resp = self.send_command("d", wait_time=0.5)
            self.send_command(mode_num, wait_time=0.5, flush=False)
            
            # Test menu command
            resp = self.send_command("menu", wait_time=0.5)
            self.expect(resp, r"Main Menu", f"Menu works in {mode_name}")
            self.send_command("b", wait_time=0.3)
            
            # Test status command
            resp = self.send_command("s", wait_time=1.0)
            self.expect(resp, r"Sensor Status", f"Status works in {mode_name}")
            
            # Test help command
            resp = self.send_command("h", wait_time=0.5)
            self.expect(resp, r"Help", f"Help works in {mode_name}")

        # --- Test 11: Help Command Lists All Commands ---
        print("\n--- Test 11: Help Command Lists All Commands ---")
        resp = self.send_command("h", wait_time=0.5)
        self.expect(resp, r"Help", "Help header present")
        self.expect(resp, r"d.*output.*Toggle output mode", "'d' command listed in help")
        self.expect(resp, r"audit.*IMU.*diagnostic", "'audit' command listed in help")
        self.expect(resp, r"menu", "'menu' command listed")
        self.expect(resp, r"status", "'status' command listed")
        self.expect(resp, r"cal", "'cal' command listed")

        # --- FINAL: Reset to DEBUG mode for normal operation ---
        print("\n--- Final Cleanup: Set mode to DEBUG ---")
        resp = self.send_command("d", wait_time=0.5)
        self.send_command("1", wait_time=0.5, flush=False)
        print("  [INFO] Mode reset to DEBUG for easier monitoring")

        print("\n=== ALL OUTPUT MODE TESTS COMPLETED ===")
        print("\nNOTE: Some tests require manual verification:")
        print("  - Test 2: Startup mode selection prompt")
        print("  - Test 6: ERROR/WARN display in SILENT mode")
        print("  - Test 8: Mode persistence across reboots")
        print("  - Test 9: Startup mode override")
        
        self.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Output Mode Selection Test Agent')
    parser.add_argument('--port', help='Serial port', required=True)
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    
    args = parser.parse_args()
    
    agent = OutputModeTestAgent(args.port, args.baud)
    agent.run_tests()
