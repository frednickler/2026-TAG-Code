#!/usr/bin/env python3
"""
Validate DMP quaternion output quality.
"""

import csv
import numpy as np
import argparse

def validate_quaternions(csv_file):
    """Check quaternion normalization and continuity."""
    quaternions = []
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            quaternions.append([
                float(row['qw']),
                float(row['qx']),
                float(row['qy']),
                float(row['qz'])
            ])
    
    quaternions = np.array(quaternions)
    
    # Check normalization
    magnitudes = np.linalg.norm(quaternions, axis=1)
    mag_mean = np.mean(magnitudes)
    mag_std = np.std(magnitudes)
    mag_max_error = np.max(np.abs(magnitudes - 1.0))
    
    # Check for discontinuities (large jumps)
    diffs = np.linalg.norm(np.diff(quaternions, axis=0), axis=1)
    max_jump = np.max(diffs)
    
    print("=" * 60)
    print("QUATERNION VALIDATION RESULTS")
    print("=" * 60)
    print(f"Samples analyzed:        {len(quaternions)}")
    print(f"Magnitude mean:          {mag_mean:.6f} (target: 1.0)")
    print(f"Magnitude std dev:       {mag_std:.6f}")
    print(f"Max normalization error: {mag_max_error:.6f} (target: <0.01)")
    print(f"Max quaternion jump:     {max_jump:.6f}")
    print("=" * 60)
    
    if mag_max_error < 0.01 and max_jump < 0.5:
        print("✅ PASS: Quaternions are well-normalized and continuous")
    else:
        print("❌ FAIL: Quaternion quality issues detected")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file', help='CSV file from drift_analysis.py')
    args = parser.parse_args()
    
    validate_quaternions(args.csv_file)
