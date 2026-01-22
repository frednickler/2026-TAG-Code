import csv
import sys
import numpy as np

def verify_stability(filename):
    data = []
    print(f"Reading {filename}...")
    try:
        with open(filename, 'r') as f:
            for line in f:
                if "DATA," in line:
                    try:
                        parts = line.split(',')
                        # Index Mapping from main.cpp
                        # 24: Roll, 25: Pitch, 29: AlgHeading
                        if len(parts) >= 30:
                            roll = float(parts[24])
                            pitch = float(parts[25])
                            yaw = float(parts[26])
                            alg_head = float(parts[29])
                            data.append([roll, pitch, yaw, alg_head])
                    except ValueError:
                        continue
    except FileNotFoundError:
        print(f"File {filename} not found.")
        return

    if not data:
        print("No Valid 'DATA' lines found.")
        return

    arr = np.array(data)
    
    # Unwrap Heading to handle 0/360 crossover
    # We use a simple unwrap: if jump > 180, adjust.
    heading_rad = np.radians(arr[:, 3])
    heading_unwrapped = np.degrees(np.unwrap(heading_rad))
    
    # Identify Static vs Motion Segments can be complex, 
    # so we just analyze global ranges for this quick test.
    
    roll_range = np.ptp(arr[:, 0])
    pitch_range = np.ptp(arr[:, 1])
    heading_range = np.ptp(heading_unwrapped)
    
    print("\n=== TEST RESULTS ===")
    print(f"Total Samples: {len(arr)}")
    print(f"Pitch Change:  {pitch_range:.2f}°")
    print(f"Roll Change:   {roll_range:.2f}°")
    print(f"Heading Drift: {heading_range:.2f}°")
    
    # Pass/Fail Criteria
    # If Pitch OR Roll changed significantly (>20deg)
    # AND Heading drift is LOW (<10deg), then PASS.
    
    is_pitching = pitch_range > 15.0
    is_rolling = roll_range > 15.0
    is_stable = heading_range < 12.0 # Allow slightly more margin for noise
    
    print("\n=== VERDICT ===")
    if not is_pitching and not is_rolling:
        print("INVALID TEST: Not enough movement detected.")
        print("Please Tilt/Pitch the sensor > 15°.")
    elif is_stable:
        print("PASS: Heading remained stable during tilt.")
    else:
        print("FAIL: Heading drifted significantly during tilt.")
        print("Possible Causes: Axis Mapping, Soft Iron, or Interference.")
        
    # Analyze correlation just in case
    # If Heading correlates strongly with Pitch/Roll, it's a mapping issue.
    # Norm calc...
    if len(arr) > 10 and (is_pitching or is_rolling):
        # Normalize
        p_std = np.std(arr[:, 1])
        r_std = np.std(arr[:, 0])
        h_std = np.std(heading_unwrapped)
        
        if h_std > 0.1:
            if p_std > 0.1:
                corr_ph = np.corrcoef(arr[:, 1], heading_unwrapped)[0, 1]
                print(f"Pitch vs Heading Correlation: {corr_ph:.2f}")
            if r_std > 0.1:
                corr_rh = np.corrcoef(arr[:, 0], heading_unwrapped)[0, 1]
                print(f"Roll vs Heading Correlation:  {corr_rh:.2f}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python verify_stability.py <filename>")
    else:
        verify_stability(sys.argv[1])
