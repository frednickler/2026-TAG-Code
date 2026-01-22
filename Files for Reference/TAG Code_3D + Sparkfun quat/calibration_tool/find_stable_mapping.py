import numpy as np
import sys
import math

def calculate_tilt_compensated_heading(ax, ay, az, mx, my, mz):
    # Normalize Accel
    norm_a = math.sqrt(ax*ax + ay*ay + az*az)
    if norm_a == 0: return 0
    ax /= norm_a
    ay /= norm_a
    az /= norm_a
    
    # Calculate Roll/Pitch (Standard NED Equations)
    # Pitch (Theta) = asin(-ax) ... wait, depends on definition.
    # Using Madgwick Paper / Standard Aerospace:
    # Roll (Phi) = atan2(ay, az)
    # Pitch (Theta) = atan2(-ax, sqrt(ay*ay + az*az))
    
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    
    # Tilt Compensated Mag
    # Xh = mx*cos(theta) + my*sin(theta)*sin(phi) + mz*sin(theta)*cos(phi)
    # Yh = my*cos(phi) - mz*sin(phi)
    
    cos_roll = math.cos(roll)
    sin_roll = math.sin(roll)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    
    Xh = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll
    Yh = my * cos_roll - mz * sin_roll
    
    heading = math.atan2(Yh, Xh) * 180.0 / math.pi
    if heading < 0: heading += 360.0
    
    return heading

def load_data(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f:
            if "DATA," in line:
                try:
                    parts = line.split(',')
                    # Index Mapping:
                    # 1,2,3: Raw Acc
                    # 7,8,9: Raw Mag
                    # Need to parse and store
                    if len(parts) >= 10:
                        ax = float(parts[1])
                        ay = float(parts[2])
                        az = float(parts[3])
                        mx = float(parts[7])
                        my = float(parts[8])
                        mz = float(parts[9])
                        data.append({'a': [ax, ay, az], 'm': [mx, my, mz]})
                except ValueError:
                    continue
    return data

def find_best_mapping(filename):
    data = load_data(filename)
    if not data:
        print("No data found")
        return

    # Signs to test: [sx, sy, sz] where s is +/- 1
    signs = [-1, 1]
    best_std = 9999.0
    best_config = None
    
    print(f"Analyzing {len(data)} samples...")
    print("Config | Std Dev (deg) | Range (deg)")
    print("-" * 40)
    
    # We assume Accel is correct (Identity). We flip Mag signs.
    # Also consider Accel Z flip? 
    # Current Code: Accel Z is flipped (-1.0) in applyAxisMapping.
    # Raw Accel Z is usually +1g (flat). So Mapped Accel Z is -1g (Up).
    # Wait, NED frame requires Down +1g.
    # So if Raw=+1 (Up), Mapped should be +1 (Down)?
    # Standard: Gravity=Down. Accel measures Up force.
    # stationary table: z = +1g.
    # NED: Gravity vector g = [0,0,1]. Accel = [0,0,-1] (reaction).
    # So we want [0,0,-1].
    # So if input is +1, we want output -1.
    # So Accel Z MUST be inverted.
    # So we fix Accel Mapping: [1, 1, -1]
    
    acc_map = [1, 1, -1] # X, Y, -Z
    
    for sx in signs:
        for sy in signs:
            for sz in signs:
                headings = []
                for sample in data:
                    # Apply Mapping
                    ax = sample['a'][0] * acc_map[0]
                    ay = sample['a'][1] * acc_map[1]
                    az = sample['a'][2] * acc_map[2]
                    
                    mx = sample['m'][0] * sx
                    my = sample['m'][1] * sy
                    mz = sample['m'][2] * sz
                    
                    h = calculate_tilt_compensated_heading(ax, ay, az, mx, my, mz)
                    headings.append(h)
                
                # Unwrap
                headings_rad = np.radians(headings)
                headings_unwrap = np.degrees(np.unwrap(headings_rad))
                
                std_dev = np.std(headings_unwrap)
                r = np.ptp(headings_unwrap)
                
                config_str = f"[{sx:+d}, {sy:+d}, {sz:+d}]"
                print(f"{config_str} | {std_dev:6.2f} | {r:6.2f}")
                
                if std_dev < best_std:
                    best_std = std_dev
                    best_config = [sx, sy, sz]

    print("-" * 40)
    print(f"BEST MAG MAPPING: {best_config} (Std: {best_std:.2f})")
    print("Apply this to applyAxisMapping() for Mag.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python find_stable_mapping.py <filename>")
    else:
        find_best_mapping(sys.argv[1])
