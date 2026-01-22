#!/usr/bin/env python3
"""
DMP Drift Analysis Tool
Captures serial data and calculates heading drift over time.
"""

import serial
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import argparse

def parse_data_line(line):
    """Parse DATA line from serial output."""
    try:
        parts = line.split(',')
        if len(parts) < 8:
            return None
        return {
            'timestamp': float(parts[0]),
            'qw': float(parts[1]),
            'qx': float(parts[2]),
            'qy': float(parts[3]),
            'qz': float(parts[4]),
            'ax': float(parts[5]),
            'ay': float(parts[6]),
            'az': float(parts[7])
        }
    except:
        return None

def quaternion_to_euler(qw, qx, qy, qz):
    """Convert quaternion to Euler angles (degrees)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation) - HEADING
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

def analyze_drift(csv_file):
    """Analyze heading drift from captured data."""
    timestamps = []
    headings = []
    pitches = []
    rolls = []
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamps.append(float(row['timestamp']))
            headings.append(float(row['heading']))
            pitches.append(float(row['pitch']))
            rolls.append(float(row['roll']))
    
    timestamps = np.array(timestamps)
    headings = np.array(headings)
    
    # Normalize time to start at 0
    timestamps = timestamps - timestamps[0]
    
    # Unwrap heading to handle 360° -> 0° transitions
    headings_unwrapped = np.unwrap(np.radians(headings))
    headings_unwrapped = np.degrees(headings_unwrapped)
    
    # Calculate drift
    initial_heading = headings[0]
    final_heading = headings[-1]
    total_drift = abs(final_heading - initial_heading)
    duration_min = timestamps[-1] / 60.0
    drift_per_min = total_drift / duration_min if duration_min > 0 else 0
    
    # Calculate heading stability (std dev)
    heading_std = np.std(headings)
    
    # Identify large movements (pitch/roll > 10°)
    large_movements = (np.abs(pitches) > 10) | (np.abs(rolls) > 10)
    if np.any(large_movements):
        drift_during_movement = np.max(np.abs(np.diff(headings_unwrapped[large_movements])))
    else:
        drift_during_movement = 0.0
    
    return {
        'total_drift': total_drift,
        'drift_per_min': drift_per_min,
        'heading_std': heading_std,
        'drift_during_movement': drift_during_movement,
        'duration': timestamps[-1],
        'timestamps': timestamps,
        'headings': headings
    }

def main():
    parser = argparse.ArgumentParser(description='Analyze IMU heading drift')
    parser.add_argument('--port', default='/dev/cu.usbserial-0001', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--duration', type=int, default=60, help='Capture duration (seconds)')
    parser.add_argument('--output', default='drift_test.csv', help='Output CSV file')
    args = parser.parse_args()
    
    print(f"🔌 Connecting to {args.port} at {args.baud} baud...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)  # Wait for connection
    
    print("📤 Sending 'x' to exit menu/start tracking...")
    ser.write(b'x')
    time.sleep(1)
    
    print(f"📊 Capturing data for {args.duration} seconds...")
    print("   Keep device STILL for static test, or perform tilt movements for dynamic test\n")
    
    data = []
    start_time = time.time()
    
    try:
        while (time.time() - start_time) < args.duration:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('DATA'):
                parsed = parse_data_line(line[5:])  # Skip "DATA,"
                if parsed:
                    roll, pitch, yaw = quaternion_to_euler(
                        parsed['qw'], parsed['qx'], parsed['qy'], parsed['qz']
                    )
                    data.append({
                        'timestamp': parsed['timestamp'],
                        'qw': parsed['qw'],
                        'qx': parsed['qx'],
                        'qy': parsed['qy'],
                        'qz': parsed['qz'],
                        'roll': roll,
                        'pitch': pitch,
                        'heading': yaw % 360
                    })
                    
                    # Progress indicator
                    elapsed = time.time() - start_time
                    print(f"\r   Progress: {int(elapsed)}/{args.duration}s | Samples: {len(data)}", end='')
    
    except KeyboardInterrupt:
        print("\n⚠️  Capture interrupted by user")
    
    ser.close()
    print(f"\n\n💾 Saving {len(data)} samples to {args.output}...")
    
    with open(args.output, 'w', newline='') as f:
        if data:
            writer = csv.DictWriter(f, fieldnames=data[0].keys())
            writer.writeheader()
            writer.writerows(data)
    
    print("✅ Data saved!\n")
    print("📈 Analyzing drift...\n")
    
    results = analyze_drift(args.output)
    
    print("=" * 60)
    print("DRIFT ANALYSIS RESULTS")
    print("=" * 60)
    print(f"Duration:                {results['duration']:.1f} seconds")
    print(f"Total Heading Drift:     {results['total_drift']:.2f}°")
    print(f"Drift Rate:              {results['drift_per_min']:.2f}°/min")
    print(f"Heading Stability (σ):   {results['heading_std']:.2f}°")
    print(f"Max Dynamic Drift:       {results['drift_during_movement']:.2f}°")
    print("=" * 60)
    
    # Pass/fail
    if results['drift_per_min'] < 1.0 and results['drift_during_movement'] < 2.0:
        print("✅ PASS: Drift within acceptable limits")
    else:
        print("❌ FAIL: Drift exceeds target (<1°/min static, <2° dynamic)")
    print()
    
    # Generate plot
    plt.figure(figsize=(12, 6))
    plt.plot(results['timestamps'], results['headings'], 'b-', linewidth=1.5)
    plt.xlabel('Time (seconds)')
    plt.ylabel('Heading (degrees)')
    plt.title(f'DMP Heading Stability Test - Drift: {results["drift_per_min"]:.2f}°/min')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    plot_file = args.output.replace('.csv', '_plot.png')
    plt.savefig(plot_file, dpi=150)
    print(f"📊 Plot saved to {plot_file}")

if __name__ == '__main__':
    main()
