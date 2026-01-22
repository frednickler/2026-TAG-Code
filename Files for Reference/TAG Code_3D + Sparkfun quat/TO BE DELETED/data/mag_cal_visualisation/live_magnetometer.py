#!/usr/bin/env python3
"""live_magnetometer.py

Real-time visualisation of 3-axis magnetometer data streamed from an Arduino (or any
microcontroller) over a serial port.

The script opens a 3-D window using PyQtGraph's OpenGL module and draws three colour-
encoded "ribbons" representing projections on the XY (blue), ZY (red) and XZ (black)
planes, as requested:
    • XY → blue     (x, y, 0)
    • ZY → red      (0, y, z)
    • XZ → black    (x, 0, z)

Usage:
    python live_magnetometer.py [PORT] [BAUD]

Arguments are optional.  If no port is supplied the script will pick the first USB/
TTY device it finds.  BAUD defaults to 115200.

The microcontroller must send ASCII lines in the form:
    <x>,<y>,<z>\n
For example:
    -123.4,98.7,432.1\n
Before streaming, the script transmits a single byte 's' to the MCU.  On exit it
sends 'x'.  Modify these start/stop characters to match your firmware.
"""
from __future__ import annotations
import sys, signal, argparse, time, numpy as np, serial, serial.tools.list_ports as lp
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

# Set global style
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')


# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

def auto_port() -> str | None:
    """Return the first likely serial device or None if nothing found."""
    for p in lp.comports():
        if 'usb' in p.device.lower() or 'tty' in p.device.lower():
            return p.device
    return None


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Live magnetometer visualiser")
    ap.add_argument('port', nargs='?', default=auto_port(),
                    help='Serial port (default: first detected USB/TTY)')
    ap.add_argument('baud', nargs='?', type=int, default=115200,
                    help='Baud rate (default: 115200)')
    return ap.parse_args()


# -----------------------------------------------------------------------------
# Main visualiser class
# -----------------------------------------------------------------------------

class MagVisualiser:
    START_BYTE = b's'
    STOP_BYTE  = b'x'  # Explicit stop command on Arduino via 'x'

    def __init__(self, port: str, baud: int):
        if port is None:
            raise SystemExit("No serial ports found. Connect your device or specify a port.")
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # give MCU time to reset on new connection
        self.ser.reset_input_buffer()
        self.ser.write(self.START_BYTE)
        print(f"[INFO] Connected to {port} @ {baud} baud. Streaming started.")

        # Data buffers
        self.buf_x_raw: list[float] = []
        self.buf_y_raw: list[float] = []
        self.buf_z_raw: list[float] = []
        self.buf_x_cal: list[float] = []
        self.buf_y_cal: list[float] = []
        self.buf_z_cal: list[float] = []
        
        # Calibration parameters
        self.hard_iron = np.zeros(3)  # [x, y, z] bias
        self.soft_iron = np.ones(3)   # [x, y, z] scale factors
        self.is_calibrated = False
        self.calibration_start_time = None
        self.calibration_duration = 30  # seconds
        self.collecting_calibration = False

        # --------- PyQtGraph 2D setup ---------
        self.app = QtWidgets.QApplication([])
        
        # Create main window with a single plot
        self.win = pg.GraphicsLayoutWidget(show=True)
        self.win.setWindowTitle("Magnetometer Data (All Planes)")
        self.win.resize(1000, 1000)  # Square window
        # Ensure window can receive keyboard shortcuts
        self.win.setFocusPolicy(QtCore.Qt.StrongFocus)
        # Add shortcuts: 'C' to open calibrated window, 'Q' and 'Esc' to quit
        QtWidgets.QShortcut(QtGui.QKeySequence('C'), self.win, activated=self.create_new_calibrated_window)
        QtWidgets.QShortcut(QtGui.QKeySequence('Q'), self.win, activated=self.close)
        QtWidgets.QShortcut(QtGui.QKeySequence('Escape'), self.win, activated=self.close)
        
        # Single plot with all planes
        self.plot = self.win.addPlot(title="Magnetometer Data - All Planes")
        self.plot.showGrid(x=True, y=True)
        self.plot.setAspectLocked(True)
        
        # Create scatter plots for each plane in the same view
        self.scatter_xy_raw = self.plot.plot(pen=None, symbol='o', symbolSize=6, symbolBrush='b', name="XY Raw (Blue)")
        self.scatter_yz_raw = self.plot.plot(pen=None, symbol='s', symbolSize=6, symbolBrush='r', name="YZ Raw (Red)")
        self.scatter_xz_raw = self.plot.plot(pen=None, symbol='d', symbolSize=6, symbolBrush='g', name="XZ Raw (Green)")
        
        self.scatter_xy_cal = self.plot.plot(pen=None, symbol='o', symbolSize=4, symbolBrush='c', name="XY Calibrated (Cyan)")
        self.scatter_yz_cal = self.plot.plot(pen=None, symbol='s', symbolSize=4, symbolBrush='m', name="YZ Calibrated (Magenta)")
        self.scatter_xz_cal = self.plot.plot(pen=None, symbol='d', symbolSize=4, symbolBrush='y', name="XZ Calibrated (Yellow)")
        
        # Add legend
        self.plot.addLegend()
        
        # Store plot items for later updates
        self.plots = [self.scatter_xy_raw, self.scatter_yz_raw, self.scatter_xz_raw, self.scatter_xy_cal, self.scatter_yz_cal, self.scatter_xz_cal]

        # timer for periodic updates - faster refresh
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10)  # 100 Hz ~ 10 ms for smoother updates

        # clean exit on Ctrl-C / close
        signal.signal(signal.SIGINT, self.close)

        # enable key-press events
        self.view = self.win

    # ---------------------------------------------------------------------
    # Qt slots
    # ---------------------------------------------------------------------

    def update(self):
        """Read all waiting serial lines and refresh the scatter plots."""
        current_time = time.time()
        
        # Start calibration data collection if needed
        if not self.is_calibrated and not self.collecting_calibration and len(self.buf_x_raw) > 10:
            self.start_calibration()
        
        # Check if calibration collection is complete
        if self.collecting_calibration:
            elapsed = current_time - self.calibration_start_time
            if elapsed >= self.calibration_duration:
                self.finish_calibration()
        
        # Read and process incoming data
        try:
            while self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Debug: Print raw line for inspection
                    print(f"[RAW] {line}")
                    
                    # Skip empty lines and debug/status messages
                    if not line or any(s in line for s in ['rate=', 'time=', 'samples=', 'Auto-calibrating', 'Current calibration', 'Calibration complete', 'mag_bias', 'mag_scale']):
                        print(f"[INFO] {line}")
                        continue
                        
                    # Try to parse as x,y,z data - handle both RAW: and CAL: formats
                    if 'RAW:' in line or 'CAL:' in line:
                        try:
                            # Extract the data part (after RAW: or CAL:)
                            if 'RAW:' in line:
                                data_part = line.split('RAW:')[1].split(',')[:3]
                                x, y, z = map(int, data_part)
                                self.process_data_point(x, y, z, calibrated=False)
                            elif 'CAL:' in line:
                                data_part = line.split('CAL:')[1].split(',')[:3]
                                x, y, z = map(float, data_part)
                                self.process_data_point(x, y, z, calibrated=True)
                        except (ValueError, IndexError) as e:
                            print(f"[WARN] Could not parse line: {line} - {e}")
                    else:
                        print(f"[DEBUG] Unexpected format: {line}")
                        
                except Exception as e:
                    print(f"[ERROR] Error processing line: {e}")
                    continue
                    
        except Exception as e:
            print(f"[ERROR] Serial read error: {e}")
            return

        if not self.buf_x_raw:
            print("No valid data points received yet...")
            return

        try:
            # Update main window plots
            self.update_plots()
            # Update calibrated window plots if open
            if hasattr(self, 'calibrated_window'):
                self.calibrated_window.update_plots()
        except Exception as e:
            print(f"[ERROR] Plot update failed: {e}")
    
    def process_data_point(self, x: float, y: float, z: float, calibrated: bool):
        """Process a single data point, applying calibration if available."""
        # Add to buffers
        if calibrated:
          self.buf_x_cal.append(x)
          self.buf_y_cal.append(y)
          self.buf_z_cal.append(z)
        else:
          self.buf_x_raw.append(x)
          self.buf_y_raw.append(y)
          self.buf_z_raw.append(z)
        
        # Print first few points for debugging
        if len(self.buf_x_raw) <= 5:
            print(f"Parsed: x={x:.1f}, y={y:.1f}, z={z:.1f}")
    
    def start_calibration(self):
        """Start the calibration data collection process."""
        print("\n--- Starting calibration data collection ---")
        print(f"Rotate the sensor in all directions for {self.calibration_duration} seconds...")
        self.collecting_calibration = True
        self.calibration_start_time = time.time()
        # Clear any previous data for a fresh calibration
        self.buf_x_raw.clear()
        self.buf_y_raw.clear()
        self.buf_z_raw.clear()
    
    def finish_calibration(self):
        """Finish calibration and calculate calibration parameters."""
        self.collecting_calibration = False
        print("\n--- Processing calibration data... ---")
        print(f"Collected {len(self.buf_x_raw)} data points for calibration")
        self.calibrate()
    
    def update_plots(self):
        """Update all plots with current data."""
        if not self.buf_x_raw:
            return
            
        # Convert to numpy arrays for easier manipulation
        x_raw = np.array(self.buf_x_raw)
        y_raw = np.array(self.buf_y_raw)
        z_raw = np.array(self.buf_z_raw)
        x_cal = np.array(self.buf_x_cal)
        y_cal = np.array(self.buf_y_cal)
        z_cal = np.array(self.buf_z_cal)
        
        # Update raw data plots
        self.scatter_xy_raw.setData(x=x_raw, y=y_raw)
        self.scatter_yz_raw.setData(x=z_raw, y=y_raw)  # YZ plane (Z vs Y)
        self.scatter_xz_raw.setData(x=x_raw, y=z_raw)  # XZ plane (X vs Z)
        
        # Update calibrated data plots if calibrated
        if self.is_calibrated:
          self.scatter_xy_cal.setData(x=x_cal, y=y_cal)
          self.scatter_yz_cal.setData(x=z_cal, y=y_cal)
          self.scatter_xz_cal.setData(x=x_cal, y=z_cal)
        
        # Auto-range to fit all data
        self.plot.autoRange()

    # ------------------------------------------------------------------
    def calibrate(self):
        """Compute hard-iron offsets and soft-iron scale factors from collected data."""
        if len(self.buf_x_raw) < 100:
            print("[WARN] Need at least 100 points for calibration")
            return False
            
        print("\n--- Calculating calibration parameters... ---")
        
        # Convert to numpy arrays
        x, y, z = np.array(self.buf_x_raw), np.array(self.buf_y_raw), np.array(self.buf_z_raw)
        
        # Hard-iron offsets (bias) are the average of min and max
        self.hard_iron = np.array([
            (x.max() + x.min()) / 2,
            (y.max() + y.min()) / 2,
            (z.max() + z.min()) / 2
        ])
        
        # Soft-iron scaling (stretch to make sphere)
        avg_delta = np.array([
            (x.max() - x.min()) / 2,
            (y.max() - y.min()) / 2,
            (z.max() - z.min()) / 2
        ])
        
        # Calculate scale factors to normalize all axes
        avg_radius = np.mean(avg_delta)
        self.soft_iron = avg_radius / avg_delta
        
        # Calculate calibrated points for visualization
        x_cal = (x - self.hard_iron[0]) * self.soft_iron[0]
        y_cal = (y - self.hard_iron[1]) * self.soft_iron[1]
        z_cal = (z - self.hard_iron[2]) * self.soft_iron[2]
        
        # Calculate calibration quality metrics
        radius_before = np.sqrt(x**2 + y**2 + z**2)
        radius_after = np.sqrt(x_cal**2 + y_cal**2 + z_cal**2)
        
        # Calculate mean and standard deviation of radii
        mean_radius_before = np.mean(radius_before)
        std_radius_before = np.std(radius_before)
        mean_radius_after = np.mean(radius_after)
        std_radius_after = np.std(radius_after)
        
        # Mark as calibrated
        self.is_calibrated = True
        
        # Print calibration results
        print("\n--- Calibration Complete ---")
        print(f"Hard-iron offsets (bias): X={self.hard_iron[0]:.1f}, Y={self.hard_iron[1]:.1f}, Z={self.hard_iron[2]:.1f}")
        print(f"Soft-iron scales: X={self.soft_iron[0]:.3f}, Y={self.soft_iron[1]:.3f}, Z={self.soft_iron[2]:.3f}")
        
        print("\n--- Calibration Quality ---")
        print(f"Before calibration: mean radius = {mean_radius_before:.1f} ± {std_radius_before:.1f}")
        print(f"After calibration:  mean radius = {mean_radius_after:.1f} ± {std_radius_after:.1f}")
        
        print("\n--- Arduino Code ---")
        print("Apply these values in your Arduino code:")
        print(f"float mag_bias[3]  = {{ {self.hard_iron[0]:.1f}, {self.hard_iron[1]:.1f}, {self.hard_iron[2]:.1f} }}; // Hard-iron offsets")
        print(f"float mag_scale[3] = {{ {self.soft_iron[0]:.3f}, {self.soft_iron[1]:.3f}, {self.soft_iron[2]:.3f} }}; // Soft-iron scale factors")
        
        print("\nPress 'C' to recalibrate, 'Q' or ESC to quit.")
        
        return True

    # ------------------------------------------------------------------
    def handle_key_press(self, event):
        if event.key() == QtCore.Qt.Key_C:
            self.create_new_calibrated_window()
        elif event.key() in (QtCore.Qt.Key_Q, QtCore.Qt.Key_Escape):
            self.close()

    def create_new_calibrated_window(self):
        """Stop raw, enable calibration, and start calibrated stream; open window."""
        try:
            # First stop any existing data stream
            print("[DEBUG] Stopping raw data stream.")
            self.ser.write(self.STOP_BYTE)  # 'x' command
            time.sleep(0.2)  # Give more time for Arduino to process
            
            # Clear the serial input buffer to remove any pending data
            self.ser.reset_input_buffer()
            
            # Toggle calibration mode on Arduino
            print("[DEBUG] Enabling calibration on Arduino.")
            self.ser.write(b'c')
            time.sleep(0.2)  # Give more time for Arduino to process
            
            # Instead of sending 's' again, let's check if we need to restart
            # We'll check if collecting is already true on Arduino by looking at recent data
            # If no data is coming in, then we need to start collection
            if not self.ser.in_waiting:
                print("[DEBUG] No data flowing, starting calibrated data stream.")
                self.ser.write(self.START_BYTE)  # 's' command
            else:
                print("[DEBUG] Data already flowing, not sending start command.")
                
        except Exception as e:
            print(f"[ERROR] Calibration sequence failed: {e}")
            
        # Reset calibrated buffers for fresh data
        self.buf_x_cal.clear()
        self.buf_y_cal.clear()
        self.buf_z_cal.clear()
        
        # Create or show window
        if not hasattr(self, 'calibrated_window'):
            print("[DEBUG] Initializing new CalibratedDataWindow.")
            self.calibrated_window = CalibratedDataWindow(self)
            pos = self.win.geometry().topLeft()
            self.calibrated_window.move(pos.x()+50, pos.y()+50)
            self.calibrated_window.plot.autoRange()
        self.calibrated_window.show()
        print("[DEBUG] CalibratedDataWindow shown; awaiting calibrated data.")

    # ------------------------------------------------------------------
    def close(self, *_) -> None:
        """Stop streaming and close resources."""
        print("\n[INFO] Closing…")
        # Stop timer before closing serial
        try:
            self.timer.stop()
        except Exception:
            pass
        # Stop streaming and close serial port
        try:
            self.ser.write(self.STOP_BYTE)
            self.ser.close()
        except Exception:
            pass
        QtWidgets.QApplication.quit()

    # ------------------------------------------------------------------
    def run(self) -> None:
        self.app.exec_()
        self.close()


class CalibratedDataWindow(pg.GraphicsLayoutWidget):
    """Dedicated window for showing calibrated data"""
    
    def __init__(self, vis):
        super().__init__()
        # Keep reference to main visualiser for calibrated buffers
        self.vis = vis
        
        # Standalone window
        self.setWindowFlags(QtCore.Qt.Window)
        self.setWindowTitle("Calibrated Magnetometer Data (All Planes)")
        self.resize(1000, 1000)  # Match main window size
        
        # Create plot and legend - match main window style
        self.plot = self.addPlot(title="Calibrated Magnetometer Data - All Planes")
        self.plot.showGrid(x=True, y=True)
        self.plot.setAspectLocked(True)
        
        # Create scatter plots for each plane with same styling as main window
        self.scatter_xy = self.plot.plot(pen=None, symbol='o', symbolSize=6, symbolBrush='c', name="XY Calibrated (Cyan)")
        self.scatter_yz = self.plot.plot(pen=None, symbol='s', symbolSize=6, symbolBrush='m', name="YZ Calibrated (Magenta)")
        self.scatter_xz = self.plot.plot(pen=None, symbol='d', symbolSize=6, symbolBrush='y', name="XZ Calibrated (Yellow)")
        
        # Add legend
        self.plot.addLegend()
        
        # Auto-range upon window creation to prevent zoomed-in view
        self.plot.enableAutoRange()
        
        # Add keyboard shortcuts for consistency
        QtWidgets.QShortcut(QtGui.QKeySequence('Q'), self, activated=self.close)
        QtWidgets.QShortcut(QtGui.QKeySequence('Escape'), self, activated=self.close)
        
    def update_plots(self):
        """Update plots with latest calibrated data from main visualiser"""
        # Skip until there is calibrated data
        if not self.vis.buf_x_cal:
            return
        x = np.array(self.vis.buf_x_cal)
        y = np.array(self.vis.buf_y_cal)
        z = np.array(self.vis.buf_z_cal)
        # Update the scatter plots
        self.scatter_xy.setData(x=x, y=y)
        self.scatter_yz.setData(x=z, y=y)
        self.scatter_xz.setData(x=x, y=z)
        # Auto-range to show all points
        self.plot.autoRange()


# -----------------------------------------------------------------------------
# Script entry point
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    args = parse_args()
    vis  = MagVisualiser(args.port, args.baud)
    vis.run()
