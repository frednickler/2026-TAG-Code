import csv
import numpy as np
import matplotlib.pyplot as plt

def analyze_correlations(filename):
    acc_data = []
    mag_data = []
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("DATA,"):
                try:
                    parts = line.split(',')
                    # Index mapping based on main.cpp:
                    # 0-2: Accel (X, Y, Z) - Raw? No, let's check headers or assume standard
                    # "DATA, ax, ay, az, gx, gy, gz, mx, my, mz, ..."
                    # The parts list starts after "DATA," so index 0 is first number.
                    # 0: ax, 1: ay, 2: az
                    # 6: mx, 7: my, 8: mz
                    if len(parts) >= 9:
                        ax = float(parts[1])
                        ay = float(parts[2])
                        az = float(parts[3])
                        
                        mx = float(parts[7])
                        my = float(parts[8])
                        mz = float(parts[9])
                        
                        acc_data.append([ax, ay, az])
                        mag_data.append([mx, my, mz])
                except ValueError:
                    continue

    if not acc_data:
        print("No valid DATA lines found.")
        return

    acc = np.array(acc_data) # N x 3
    mag = np.array(mag_data) # N x 3
    
    # Calculate variances to see which axes were active
    acc_var = np.var(acc, axis=0)
    mag_var = np.var(mag, axis=0)
    
    print("=== AXIS VARIANCE (Activity) ===")
    print(f"Accel Var: X={acc_var[0]:.2f}, Y={acc_var[1]:.2f}, Z={acc_var[2]:.2f}")
    print(f"Mag Var:   X={mag_var[0]:.2f}, Y={mag_var[1]:.2f}, Z={mag_var[2]:.2f}")
    
    dom_acc_idx = np.argmax(acc_var)
    axes = ['X', 'Y', 'Z']
    print(f"Dominant Accel Axis: {axes[dom_acc_idx]}")
    
    print("\n=== CORRELATIONS ===")
    # Calculate correlation matrix between Accel and Mag columns
    # We want to see which Mag axis correlates with the Active Accel axis
    
    # Normalize data for correlation
    acc_norm = (acc - np.mean(acc, axis=0)) / np.std(acc, axis=0)
    mag_norm = (mag - np.mean(mag, axis=0)) / np.std(mag, axis=0)
    
    correlation = np.dot(acc_norm.T, mag_norm) / len(acc_norm)
    
    print("      Mag X   Mag Y   Mag Z")
    for i, row in enumerate(correlation):
        print(f"Acc {axes[i]}: {row[0]:.2f}    {row[1]:.2f}    {row[2]:.2f}")
        
    print("\n=== INTERPRETATION ===")
    if dom_acc_idx == 1: # Y-axis roll
        print("Detailed check for Roll (Acc Y active):")
        print("Expectation: If rolling, Mag Y and Mag Z should change (if heading is N/S) or Mag Z and Mag X (if heading E/W).")
        print("CRITICAL: Mag Axis corresponding to Accel Axis should ONLY correlate if there is a rotation component aligned with it.")
        
    print("\nCheck which Mag axis tracks the GRAVITY vector.")
    # Gravity is seen on Accel Y during roll?
    # If device is flat (Z=1G), and we roll right (Y axis rotation), Accel Y increases?
    # No, Accel Y is sideways. 
    # If we roll, Gravity vector moves from Z to Y.
    # So Accel Z and Accel Y should be anti-correlated or correlated (quadrature).
    
    # If we roll, Mag vector moves relative to sensor similarly.
    # So Mag Y should correlate with Accel Y?
    # Or Mag Z with Accel Z?
    
    print(f"Correlation Acc Y vs Mag Y: {correlation[1,1]:.2f}")
    print(f"Correlation Acc Z vs Mag Z: {correlation[2,2]:.2f}")

if __name__ == "__main__":
    analyze_correlations("calibration_capture.csv")
