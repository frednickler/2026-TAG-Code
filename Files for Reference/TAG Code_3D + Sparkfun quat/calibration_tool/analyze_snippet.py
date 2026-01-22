import csv
import numpy as np

def analyze_snippet(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("DATA,"):
                try:
                    parts = line.split(',')
                    # Index mapping based on main.cpp:
                    # 24: Roll, 25: Pitch, 26: Yaw
                    # 27: MagHeading, 28: TrueHeading, 29: AlgHeading
                    if len(parts) >= 30:
                        roll = float(parts[24])
                        pitch = float(parts[25])
                        yaw = float(parts[26])
                        mag_head = float(parts[27])
                        alg_head = float(parts[29])
                        data.append([roll, pitch, yaw, mag_head, alg_head])
                except ValueError:
                    continue

    if not data:
        print("No valid DATA lines found.")
        return

    arr = np.array(data)
    
    # Calculate Range (Max - Min) for each axis
    ranges = np.ptp(arr, axis=0)
    std_devs = np.std(arr, axis=0)
    
    print("=== AXIS ANALYSIS ===")
    print(f"Roll Range:  {ranges[0]:.2f} (Std: {std_devs[0]:.2f})")
    print(f"Pitch Range: {ranges[1]:.2f} (Std: {std_devs[1]:.2f})")
    print(f"Yaw Range:   {ranges[2]:.2f} (Std: {std_devs[2]:.2f})")
    
    print("\n=== DOMINANT AXIS ===")
    axes = ["Roll (X)", "Pitch (Y)", "Yaw (Z)"]
    dominant_idx = np.argmax(ranges[:3])
    print(f"Most active axis: {axes[dominant_idx]}")

    print("\n=== HYPOTHESIS CHECK ===")
    if dominant_idx == 0:
        print("CONFIRMED: Rotation was primarily around X-axis (Roll).")
    elif dominant_idx == 1:
        print("DISPROVED: Rotation was primarily around Y-axis (Pitch).")
    else:
        print("DISPROVED: Rotation was primarily around Z-axis (Yaw) - Wait, Yaw also changes with Heading!")

    print("\n=== HEADINGS ===")
    print(f"MagHeading Range: {ranges[3]:.2f}")
    print(f"AlgHeading Range: {ranges[4]:.2f}")
    
    # Heading Drift Check
    # If rotating around X or Y, Heading should be relatively stable 
    # (unless there is significant tilt error or magnetic interference).
    print(f"MagHeading Std Dev: {std_devs[3]:.2f}")
    print(f"AlgHeading Std Dev: {std_devs[4]:.2f}")

if __name__ == "__main__":
    analyze_snippet("calibration_tool/analysis_snippet.csv")
