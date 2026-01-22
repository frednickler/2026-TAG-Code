#!/usr/bin/env python3
"""Pure-Matplotlib version of IMU visualiser.

• Two time-series windows: raw & calibrated accel/gyro
• One 3-D cube window rendered with Matplotlib's mplot3d
  (avoids macOS Cocoa/GLUT threading issues)
"""
from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
from collections import deque
from typing import Deque, List, Tuple, Optional

import matplotlib.pyplot as plt
import numpy as np
import serial
import serial.tools.list_ports as list_ports
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------------------------------------------------------------------
# Parameters & globals
# ---------------------------------------------------------------------------
BAUD_RATE = 115200
BUFFER_SIZE = 500
MAX_CAL_SAMPLES = 500

START_BYTE = b"s"
STOP_BYTE = b"x"
CAL_BYTE = b"c"
BIAS_BYTE = b"b"

# Global variables
# ---------------------------------------------------------------------------
data_queue = queue.Queue()
exit_event = threading.Event()

# Data buffers
raw_accel: Deque[Tuple[float, float, float]] = deque(maxlen=BUFFER_SIZE)
raw_gyro: Deque[Tuple[float, float, float]] = deque(maxlen=BUFFER_SIZE)
cal_accel: Deque[Tuple[float, float, float]] = deque(maxlen=BUFFER_SIZE)
cal_gyro: Deque[Tuple[float, float, float]] = deque(maxlen=BUFFER_SIZE)

# Calibration state
cal_mode = False
cal_samples: List[Tuple[np.ndarray, np.ndarray]] = []
acc_bias = np.zeros(3)
acc_scale = np.ones(3)
gyr_bias = np.zeros(3)
gyr_scale = np.ones(3)

# Orientation tracking (simple complementary filter)
quaternion = np.array([1.0, 0.0, 0.0, 0.0])
last_ts = time.time()

# Serial handle
ser_handle: Optional[serial.Serial] = None

# Plot handles
fig_raw = None
fig_cal = None
fig_cube = None
ax_raw = None
ax_cal = None
ax_cube = None
lns_raw = None
lns_cal = None
cube_poly = None
cube_verts = None
faces = None

# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

def find_arduino_port() -> str | None:
    """Find and return the first Arduino port."""
    for p in list_ports.comports():
        if "Arduino" in p.description or "usbserial" in p.device or "usbmodem" in p.device:
            return p.device
    return None

def serial_reader(port: str, baud: int) -> None:
    """Read data from serial port and put it in the queue."""
    global ser_handle
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            ser_handle = ser
            print(f"Serial connected: {port} @ {baud}")
            # Give MCU time then start stream
            time.sleep(1)
            ser.write(START_BYTE)
            print("START command sent")
            while not exit_event.is_set():
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    data_queue.put(line)
    except serial.SerialException as exc:
        print(f"Serial open error ({exc}). Will retry in 2s...")
        time.sleep(2)
        serial_reader(port, baud)

# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def quat_mult(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Multiply two quaternions."""
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = r
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])

def axis_angle_to_quat(axis: np.ndarray, angle: float) -> np.ndarray:
    """Convert axis-angle to quaternion."""
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2)
    return np.array([math.cos(angle / 2), *(axis * s)])

def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """Convert quaternion to rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w),     2 * (x*z + y*w)],
        [2 * (x*y + z*w),       1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
        [2 * (x*z - y*w),       2 * (y*z + x*w),       1 - 2 * (x**2 + y**2)],
    ])

def update_orientation(acc: np.ndarray, gyr: np.ndarray, dt: float) -> None:
    """Update orientation quaternion using acc and gyro data."""
    global quaternion, last_ts
    
    # Gyro integration
    angle = np.linalg.norm(gyr) * dt
    if angle > 0:
        dq = axis_angle_to_quat(gyr, angle)
        quaternion = quat_mult(quaternion, dq)
    
    # Accelerometer correction (simple complementary filter)
    g = acc / (np.linalg.norm(acc) + 1e-6)
    pitch = math.atan2(-g[0], math.sqrt(g[1]**2 + g[2]**2))
    roll = math.atan2(g[1], g[2])
    q_acc = axis_angle_to_quat(np.array([1, 0, 0]), pitch)  # pitch
    q_acc = quat_mult(axis_angle_to_quat(np.array([0, 1, 0]), roll), q_acc)
    
    alpha = 0.98
    quaternion = alpha * quaternion + (1 - alpha) * q_acc
    quaternion /= np.linalg.norm(quaternion)

# ---------------------------------------------------------------------------
# Data parsing
# ---------------------------------------------------------------------------

def parse_line(line: str) -> None:
    """Parse a line of serial data and update buffers."""
    global last_ts
    parts = line.split(",")
    if len(parts) < 16 or parts[0] != "A":
        return
    
    try:
        # Parse raw accel/gyro
        ax, ay, az = map(float, parts[1:4])
        gx, gy, gz = map(float, parts[5:8])
        
        # Parse calibrated accel/gyro
        ax_c, ay_c, az_c = map(float, parts[9:12])
        gx_c, gy_c, gz_c = map(float, parts[13:16])
        
        # Update buffers
        raw_accel.append((ax, ay, az))
        raw_gyro.append((gx, gy, gz))
        cal_accel.append((ax_c, ay_c, az_c))
        cal_gyro.append((gx_c, gy_c, gz_c))
        
        # Update orientation
        now = time.time()
        update_orientation(
            np.array([ax_c, ay_c, az_c]),
            np.array([gx_c, gy_c, gz_c]),
            now - last_ts
        )
        last_ts = now
        
    except (ValueError, IndexError) as e:
        print(f"Parse error: {e} in line: {line}")

# ---------------------------------------------------------------------------
# Plot updates
# ---------------------------------------------------------------------------

def update_plots(frame):
    """Update time-series plots."""
    global raw_accel, raw_gyro, cal_accel, cal_gyro, data_queue
    
    # Process any queued data
    while not data_queue.empty():
        try:
            line = data_queue.get_nowait()
            parse_line(line)
        except Exception as e:
            print(f"Queue processing error: {e}")
            continue
    
    # Update plot data
    if not raw_accel:
        return lns_raw
    
    x = np.arange(len(raw_accel))
    
    # Update raw data plots
    for i, ln in enumerate(lns_raw[:3]):
        ln.set_data(x, [a[i] for a in raw_accel])
    for i, ln in enumerate(lns_raw[3:]):
        ln.set_data(x, [g[i] for g in raw_gyro])
    
    # Update calibrated data plots
    for i, ln in enumerate(lns_cal[:3]):
        ln.set_data(x, [a[i] for a in cal_accel])
    for i, ln in enumerate(lns_cal[3:]):
        ln.set_data(x, [g[i] for g in cal_gyro])
    
    # Rescale axes
    for ax in list(ax_raw) + list(ax_cal):
        ax.relim()
        ax.autoscale_view()
    
    return lns_raw + lns_cal

def update_cube(frame):
    """Update 3D cube orientation."""
    global quaternion, cube_poly, cube_verts, faces
    
    if cube_poly is None or cube_verts is None or faces is None:
        return []
    
    # Rotate cube vertices by current quaternion
    rot = quat_to_rot(quaternion)
    cube_verts_rot = np.dot(cube_verts, rot.T)
    
    # Update cube faces
    cube_poly.set_verts([cube_verts_rot[face] for face in faces])
    
    return [cube_poly]

# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

import serial
import time
import struct
import sys

# Global serial port handle
ser_handle = None

# Calibration biases (example values)
acc_bias = [-3509.648, -1846.416, -15625.312]
gyr_bias = [-128.586, 40.816, -9.594]

# Serial commands
START_BYTE = b's'
STOP_BYTE = b'x' # Using 'x' for explicit stop
DEBUG_TOGGLE_BYTE = b'd'
CALIBRATE_TOGGLE_BYTE = b'c'
DLPF_CYCLE_BYTE = b'f'
BIAS_BYTE = b'b' # Command to send bias values to Arduino

# --- New Helper Function for Debug Reading ---
def read_all_available_serial(timeout_ms=500, label=""):
    """Reads all available bytes from serial within a timeout and prints them."""
    global ser_handle
    if ser_handle is None or not ser_handle.is_open:
        return ""
    
    start_time = time.time()
    received_data = bytearray()
    
    while (time.time() - start_time) * 1000 < timeout_ms:
        if ser_handle.in_waiting > 0:
            data = ser_handle.read(ser_handle.in_waiting)
            received_data.extend(data)
            start_time = time.time() # Reset timer if new data comes in
        time.sleep(0.005) # Poll every 5ms
        
    decoded_data = received_data.decode('ascii', errors='ignore').strip()
    if decoded_data:
        print(f"PYTHON_DEBUG_READ ({label}): ---START---")
        print(decoded_data)
        print(f"PYTHON_DEBUG_READ ({label}): ---END---")
    return decoded_data

# --- Modified send_calibration_to_arduino ---
def send_calibration_to_arduino() -> bool:
    """Send calibration values to Arduino with improved reliability and delays."""
    global ser_handle, acc_bias, gyr_bias
    
    if ser_handle is None or not ser_handle.is_open:
        print("ERROR: Serial not connected, cannot send calibration")
        return False
    
    try:
        # 0. Initial Read - check for any lingering data before we start
        print("\nPYTHON_CALIBRATION_FLOW: Starting calibration sequence.")
        read_all_available_serial(timeout_ms=500, label="Pre-Command Clear")
        
        # 1. Stop any current data streaming
        print("PYTHON_CALIBRATION_FLOW: Stopping data stream with 'x'...")
        ser_handle.write(STOP_BYTE)
        ser_handle.flush()
        time.sleep(0.2)  # Give MCU time to process the stop command and clear its output buffer
        
        # 2. Aggressively clear any pending input in Python's buffer
        print("PYTHON_CALIBRATION_FLOW: Clearing Python serial input buffer aggressively...")
        read_all_available_serial(timeout_ms=1000, label="Post-Stop Clear") # Read for 1 second
        
        # 3. Send the 'b' command to Arduino to initiate calibration bias reception
        print("PYTHON_CALIBRATION_FLOW: Sending 'b' command to Arduino to initiate bias reception...")
        ser_handle.write(BIAS_BYTE) # This is 'b'
        ser_handle.flush()
        time.sleep(0.1) # Give Arduino a moment to receive 'b' and start processing it
        
        # 4. Wait for Arduino's explicit "READY" acknowledgment
        print("PYTHON_CALIBRATION_FLOW: Waiting for Arduino to send 'ARDUINO_READY_FOR_BIAS_DATA'...")
        timeout_start = time.time()
        timeout_duration = 10.0 # Increased timeout even more for debugging
        ready_ack_received = False
        
        # Read available bytes in chunks, looking for the ACK
        all_received_during_ack_wait = bytearray()
        
        while time.time() - timeout_start < timeout_duration:
            if ser_handle.in_waiting > 0:
                chunk = ser_handle.read(ser_handle.in_waiting)
                all_received_during_ack_wait.extend(chunk)
                
                # Try to decode and check for ACK in the accumulated data
                current_decoded_buffer = all_received_during_ack_wait.decode('ascii', errors='ignore')
                
                # Look for specific markers, or just print everything
                if "--- ARDUINO RESET / POWER-ON ---" in current_decoded_buffer:
                    print("\nPYTHON_CALIBRATION_FLOW: WARNING: Arduino appears to have reset mid-sequence!")
                    print(f"PYTHON_CALIBRATION_FLOW: Full buffer at reset detection: \n{current_decoded_buffer}")
                    return False
                
                if "ARDUINO_READY_FOR_BIAS_DATA" in current_decoded_buffer:
                    ready_ack_received = True
                    print("\nPYTHON_CALIBRATION_FLOW: Arduino acknowledged readiness for bias data.")
                    print(f"PYTHON_CALIBRATION_FLOW: Full buffer at ACK: \n{current_decoded_buffer}")
                    break
                
                # If we've received something but not the ACK, print it out for debug
                if len(chunk) > 0:
                    sys.stdout.write(f"PYTHON_DEBUG_RECEIVING ({len(chunk)} bytes): {chunk.decode('ascii', errors='ignore')}")
                    sys.stdout.flush() # Print immediately

            time.sleep(0.01) # Poll every 10ms
        
        if not ready_ack_received:
            print("\nERROR: Timeout waiting for Arduino's 'ARDUINO_READY_FOR_BIAS_DATA' acknowledgement.")
            print(f"PYTHON_CALIBRATION_FLOW: Final buffer state after timeout: \n{all_received_during_ack_wait.decode('ascii', errors='ignore')}")
            return False

        # 5. Arduino is now ready. Prepare and send the actual calibration data string.
        # Ensure floats are formatted with enough precision
        cal_data_string = f"B,{acc_bias[0]:.6f},{acc_bias[1]:.6f},{acc_bias[2]:.6f}," \
                          f"{gyr_bias[0]:.6f},{gyr_bias[1]:.6f},{gyr_bias[2]:.6f}\n"
        
        print(f"\nPYTHON_CALIBRATION_FLOW: Sending calibration data string: {cal_data_string.strip()}")
        ser_handle.write(cal_data_string.encode('ascii'))
        ser_handle.flush()
        time.sleep(0.15) # Slightly increased delay after sending data string
        
        # 6. Wait for Arduino's processing confirmation/error messages
        print("PYTHON_CALIBRATION_FLOW: Waiting for Arduino's processing feedback...")
        feedback_timeout_start = time.time()
        feedback_timeout_duration = 5.0 # 5 seconds for feedback
        feedback_received = False
        
        all_received_during_feedback_wait = bytearray()

        while time.time() - feedback_timeout_start < feedback_timeout_duration:
            if ser_handle.in_waiting > 0:
                chunk = ser_handle.read(ser_handle.in_waiting)
                all_received_during_feedback_wait.extend(chunk)
                
                current_decoded_buffer = all_received_during_feedback_wait.decode('ascii', errors='ignore')
                
                if "Calibration biases received and applied" in current_decoded_buffer:
                    feedback_received = True
                    print("\nPYTHON_CALIBRATION_FLOW: Arduino successfully applied calibration biases.")
                    print(f"PYTHON_CALIBRATION_FLOW: Full feedback buffer: \n{current_decoded_buffer}")
                    break 
                elif "ERROR" in current_decoded_buffer or "Invalid calibration data format" in current_decoded_buffer:
                    print("\nPYTHON_CALIBRATION_FLOW: Arduino reported an error during parsing.")
                    print(f"PYTHON_CALIBRATION_FLOW: Full feedback buffer (error): \n{current_decoded_buffer}")
                    feedback_received = True 
                    break
                
                # Print any incoming data during feedback wait
                if len(chunk) > 0:
                    sys.stdout.write(f"PYTHON_DEBUG_FEEDBACK ({len(chunk)} bytes): {chunk.decode('ascii', errors='ignore')}")
                    sys.stdout.flush()

            time.sleep(0.01)
        
        if not feedback_received:
            print("PYTHON_CALIBRATION_FLOW: WARNING: No explicit confirmation/error feedback received from Arduino for calibration data.")
            print(f"PYTHON_CALIBRATION_FLOW: Final feedback buffer state: \n{all_received_during_feedback_wait.decode('ascii', errors='ignore')}")

        # 7. Restart data streaming (important after stopping for calibration)
        print("PYTHON_CALIBRATION_FLOW: Restarting data stream...")
        ser_handle.write(START_BYTE)
        ser_handle.flush()
        time.sleep(0.2) # Give Arduino time to restart streaming
        
        return True 
        
    except Exception as e:
        print(f"PYTHON_CALIBRATION_FLOW: CRITICAL ERROR in send_calibration_to_arduino: {e}")
        # Attempt to restart the stream as a last resort
        try:
            if ser_handle and ser_handle.is_open:
                print("PYTHON_CALIBRATION_FLOW: Attempting emergency stream restart...")
                ser_handle.write(START_BYTE)
                ser_handle.flush()
        except Exception as e2:
            print(f"PYTHON_CALIBRATION_FLOW: ERROR during emergency stream restart: {e2}")
        return False

# You will need to put this back into your main Python script, 
# replacing your existing send_calibration_to_arduino function.
# Ensure 'ser_handle' is properly initialized and opened before calling this.# ---------------------------------------------------------------------------
# Main function
# ---------------------------------------------------------------------------

def main():
    global fig_raw, fig_cal, fig_cube, ax_raw, ax_cal, ax_cube, lns_raw, lns_cal, cube_poly, cube_verts, faces
    
    parser = argparse.ArgumentParser(description="IMU Visualizer with Matplotlib")
    parser.add_argument("--port", help="Serial port (auto-detected if not specified)")
    args = parser.parse_args()

    # Set up plots
    fig_raw, ax_raw = plt.subplots(2, 1, figsize=(9, 6), num="Raw IMU Data")
    fig_raw.suptitle("Raw IMU Data")
    fig_cal, ax_cal = plt.subplots(2, 1, figsize=(9, 6), num="Calibrated IMU Data")
    fig_cal.suptitle("Calibrated IMU Data")

    # Configure raw data plots
    for i, label in enumerate(["X", "Y", "Z"]):
        ax_raw[0].plot([], [], label=f"A{label}")[0]  # Accel
        ax_raw[1].plot([], [], label=f"G{label}")[0]  # Gyro
    
    ax_raw[0].set_ylabel("Acceleration (m/s²)")
    ax_raw[1].set_ylabel("Angular velocity (rad/s)")
    ax_raw[1].set_xlabel("Sample")
    
    for ax in ax_raw:
        ax.legend()
        ax.grid(True)

    # Configure calibrated data plots
    for i, label in enumerate(["X", "Y", "Z"]):
        ax_cal[0].plot([], [], label=f"A{label}")[0]  # Accel
        ax_cal[1].plot([], [], label=f"G{label}")[0]  # Gyro
    
    ax_cal[0].set_ylabel("Acceleration (m/s²)")
    ax_cal[1].set_ylabel("Angular velocity (rad/s)")
    ax_cal[1].set_xlabel("Sample")
    
    for ax in ax_cal:
        ax.legend()
        ax.grid(True)

    # Set up 3D cube
    fig_cube = plt.figure(figsize=(6, 6), num="3D Orientation")
    ax_cube = fig_cube.add_subplot(111, projection="3d")
    ax_cube.set_xlim([-1.5, 1.5])
    ax_cube.set_ylim([-1.5, 1.5])
    ax_cube.set_zlim([-1.5, 1.5])
    ax_cube.set_box_aspect([1, 1, 1])
    ax_cube.set_title("IMU 3D Orientation")
    ax_cube.set_xlabel("X")
    ax_cube.set_ylabel("Y")
    ax_cube.set_zlabel("Z")

    # Define cube vertices and faces
    cube_verts = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],  # Bottom face
        [-1, -1,  1], [1, -1,  1], [1, 1,  1], [-1, 1,  1]   # Top face
    ]) * 0.5  # Scale down the cube
    
    faces = [
        [0, 1, 2, 3], [4, 5, 6, 7],  # front, back
        [0, 1, 5, 4], [2, 3, 7, 6],  # bottom, top
        [1, 2, 6, 5], [0, 3, 7, 4]   # right, left
    ]
    
    # Create cube faces
    cube_poly = Poly3DCollection([], linewidths=1, edgecolors="k", alpha=0.6)
    ax_cube.add_collection3d(cube_poly)
    init_polys = [[cube_verts[idx] for idx in face] for face in faces]
    cube_poly.set_verts(init_polys)
    cube_poly.set_facecolor([
        [1, 0, 0, 0.8], [0, 1, 0, 0.8], [0, 0, 1, 0.8],  # RGB faces
        [1, 1, 0, 0.8], [0, 1, 1, 0.8], [1, 0, 1, 0.8]   # CMY faces
    ])

    # Get line objects for updates
    lns_raw = ax_raw[0].get_lines() + ax_raw[1].get_lines()
    lns_cal = ax_cal[0].get_lines() + ax_cal[1].get_lines()

    # Auto-detect port if not specified
    port = args.port or find_arduino_port()
    if not port:
        print("No Arduino found. Please specify port with --port")
        return

    # Start serial thread
    print(f"Connecting to {port}...")
    serial_thread = threading.Thread(
        target=serial_reader, 
        args=(port, BAUD_RATE),
        daemon=True
    )
    serial_thread.start()

    # Set up key press handler
    def on_key(event):
        global cal_mode, cal_samples, ser_handle, acc_bias, gyr_bias # Ensure acc_bias, gyr_bias are global here

        print(f"Key pressed: {event.key}") # This will confirm if other keys are even detected by Matplotlib

        if ser_handle is None or not ser_handle.is_open:
            print("Serial not connected or not open. Cannot send command.")
            return

        if event.key == "c":
            print(f"{'Starting' if not cal_mode else 'Stopping'} calibration (Arduino command 'c')...")
            ser_handle.write(STOP_BYTE) # Stop streaming before changing mode
            ser_handle.flush()
            time.sleep(0.1)
            ser_handle.write(CAL_BYTE) # Send the 'c' command
            ser_handle.flush()
            time.sleep(0.1)
            
            if cal_mode:  # Ending calibration, process and send biases
                if raw_accel and raw_gyro: # Check if data was collected during calibration
                    # Calculate raw means (LSBs) to send to Arduino for bias calculation
                    # The Arduino code will then apply its specific 1g offset (e.g., -8192 for Z-axis)
                    acc_bias = np.mean(np.array(raw_accel), axis=0) # These are raw LSB averages
                    gyr_bias = np.mean(np.array(raw_gyro), axis=0) # These are raw LSB averages (should be near zero)

                    print(f"Calibration complete. Calculated biases (raw means to send) - ")
                    print(f"  Accel (LSB): {acc_bias}")
                    print(f"  Gyro (LSB): {gyr_bias}")

                    if send_calibration_to_arduino():
                        print("Calibration values sent to Arduino")
                    else:
                        print("Failed to send calibration to Arduino")
                else:
                    print("No data collected during calibration to calculate biases.")
            
            cal_mode = not cal_mode
            cal_samples = [] # Clear collected samples regardless
            ser_handle.write(START_BYTE) # Restart streaming after calibration toggle

        elif event.key == "d": # <--- THIS IS THE MISSING BLOCK FOR 'd'
            print("Toggling debug mode (Arduino command 'd')...")
            ser_handle.write(b'd') # Send 'd' byte
            ser_handle.flush()
            time.sleep(0.1) # Give Arduino time to process
            # Arduino will print its debug status. Python doesn't need to read it here.
            
        elif event.key == "b": # <--- THIS IS THE MISSING BLOCK FOR 'b'
            # This 'b' command is meant for sending *existing* calibration biases,
            # not for triggering calibration collection.
            print("Sending current calibration biases to Arduino (Arduino command 'b')...")
            if send_calibration_to_arduino():
                print("Current calibration values re-sent to Arduino.")
            else:
                print("Failed to re-send current calibration to Arduino.")

        # Optional: Add 's' for start and 'x' for stop as direct controls
        elif event.key == "s":
            print("Sending START command to Arduino (command 's')...")
            ser_handle.write(START_BYTE)
            ser_handle.flush()
            time.sleep(0.1)

        elif event.key == "x":
            print("Sending STOP command to Arduino (command 'x')...")
            ser_handle.write(STOP_BYTE)
            ser_handle.flush()
            time.sleep(0.1)

        # You can add more elif blocks for other commands (e.g., 'f' for DLPF) if your Arduino supports them.
    # Connect key handlers
    for fig in [fig_raw, fig_cal, fig_cube]:
        fig.canvas.mpl_connect("key_press_event", on_key)

    # Start animations
    print("Starting animations...")
    ani_raw = FuncAnimation(
        fig_raw, update_plots, 
        interval=50, blit=True, 
        cache_frame_data=False
    )
    ani_cube = FuncAnimation(
        fig_cube, update_cube,
        interval=50, blit=False,
        cache_frame_data=False
    )

    # Show plots (this blocks until windows are closed)
    print("Visualization running. Press 'c' to calibrate.")
    plt.show()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
        exit_event.set()
        if ser_handle and ser_handle.is_open:
            ser_handle.write(STOP_BYTE)
            ser_handle.close()
        sys.exit(0)