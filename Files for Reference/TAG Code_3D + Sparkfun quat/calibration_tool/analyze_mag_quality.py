import numpy as np
import sys

def analyze_mag_quality(filename):
    """
    Analyze magnetometer calibration quality.
    
    A well-calibrated magnetometer should:
    1. Have constant magnitude (~55 uT for Earth's field in Australia)
    2. Raw magnitude should vary when tilting (incorrect calibration)
    3. Calibrated magnitude should stay constant when tilting
    """
    raw_data = []
    cal_data = []
    
    with open(filename, 'r') as f:
        for line in f:
            if "DATA," in line:
                try:
                    parts = line.split(',')
                    if len(parts) >= 18:
                        # Index mapping:
                        # 7,8,9: Raw Mag (mx, my, mz)
                        # 16,17,18: Calibrated Mag (mx, my, mz)
                        raw_mx = float(parts[7])
                        raw_my = float(parts[8])
                        raw_mz = float(parts[9])
                        
                        cal_mx = float(parts[16])
                        cal_my = float(parts[17])
                        cal_mz = float(parts[18])
                        
                        raw_mag = np.sqrt(raw_mx**2 + raw_my**2 + raw_mz**2)
                        cal_mag = np.sqrt(cal_mx**2 + cal_my**2 + cal_mz**2)
                        
                        raw_data.append([raw_mx, raw_my, raw_mz, raw_mag])
                        cal_data.append([cal_mx, cal_my, cal_mz, cal_mag])
                except (ValueError, IndexError):
                    continue
    
    if not raw_data:
        print("No valid data found")
        return
    
    raw_arr = np.array(raw_data)
    cal_arr = np.array(cal_data)
    
    print("=== MAGNETOMETER CALIBRATION QUALITY ===\n")
    
    print(f"Total Samples: {len(raw_arr)}\n")
    
    print("RAW MAGNETOMETER:")
    print(f"  Magnitude: {raw_arr[:,3].mean():.2f} ± {raw_arr[:,3].std():.2f} uT")
    print(f"  Range:     {raw_arr[:,3].min():.2f} - {raw_arr[:,3].max():.2f} uT")
    print(f"  Variance:  {raw_arr[:,3].var():.2f}")
    
    print("\nCALIBRATED MAGNETOMETER:")
    print(f"  Magnitude: {cal_arr[:,3].mean():.2f} ± {cal_arr[:,3].std():.2f} uT")
    print(f"  Range:     {cal_arr[:,3].min():.2f} - {cal_arr[:,3].max():.2f} uT")
    print(f"  Variance:  {cal_arr[:,3].var():.2f}")
    
    print("\nEXPECTED (Sydney, Australia): ~55 uT\n")
    
    # Quality assessment
    cal_std = cal_arr[:,3].std()
    cal_mean = cal_arr[:,3].mean()
    
    print("=== DIAGNOSIS ===")
    
    if cal_std > 5.0:
        print("❌ POOR CALIBRATION: Magnitude varies too much during movement")
        print("   → Soft Iron matrix OR Hard Iron bias is incorrect")
        print("   → Need to recalibrate magnetometer")
    elif abs(cal_mean - 55) > 10:
        print("⚠️  WARNING: Average magnitude is off from expected field strength")
        print(f"   Expected: ~55 uT, Got: {cal_mean:.1f} uT")
        print("   → Scale factors might be wrong")
    else:
        print("✅ GOOD: Calibration appears correct")
        print("   → Heading drift must be from other sources (interference, etc.)")
    
    # Check if raw varies more than cal (it should!)
    improvement = raw_arr[:,3].std() / cal_arr[:,3].std()
    print(f"\nCalibration Improvement Factor: {improvement:.2f}x")
    if improvement < 2.0:
        print("   → Calibration is not fixing much (might be bad calibration data)")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_mag_quality.py <filename>")
    else:
        analyze_mag_quality(sys.argv[1])
