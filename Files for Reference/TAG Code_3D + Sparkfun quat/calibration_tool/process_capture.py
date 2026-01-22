import csv
import sys

def process_file(input_file, output_file):
    unique_points = set()
    
    # Read existing data if file exists
    try:
        with open(output_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("DATA"):
                    unique_points.add(line)
        print(f"Loaded {len(unique_points)} existing points from {output_file}")
    except FileNotFoundError:
        print(f"No existing data in {output_file}, starting fresh.")

    # Read new data
    new_count = 0
    with open(input_file, 'r', errors='ignore') as fin:
        for line in fin:
            line = line.strip()
            if line.startswith("DATA,"):
                parts = line.split(',')
                # Expecting at least 10 columns (DATA + 3acc + 3gyro + 3mag)
                if len(parts) >= 10:
                    try:
                        # Extract mx, my, mz at indices 7, 8, 9
                        mx = float(parts[7])
                        my = float(parts[8])
                        mz = float(parts[9])
                        point_str = f"{mx},{my},{mz}"
                        if point_str not in unique_points:
                            unique_points.add(point_str)
                            new_count += 1
                    except ValueError:
                        continue
                        
    # Write back all unique points
    with open(output_file, 'w') as fout:
        for point in unique_points:
            fout.write(f"{point}\n")
            
    print(f"Added {new_count} new points. Total unique points: {len(unique_points)}")

if __name__ == "__main__":
    process_file("calibration_capture.csv", "calibration_tool/mag_test.csv")
