import numpy as np
import sys
import os

def load_data(filename):
    """
    Load CSV data. Expects mx,my,mz columns.
    Skip header if present.
    Handles both quoted ("x,y,z") and unquoted (x,y,z) formats.
    """
    data = []
    with open(filename, 'r') as f:
        for line in f:
            # Skip headers and markers
            if "mx" in line or "RAW" in line or "END" in line:
                continue
            
            # Strip whitespace and quotes
            line = line.strip().strip('"')
            
            # Skip empty lines
            if not line:
                continue
                
            parts = line.split(',')
            if len(parts) >= 3:
                try:
                    mx = float(parts[0].strip())
                    my = float(parts[1].strip())
                    mz = float(parts[2].strip())
                    data.append([mx, my, mz])
                except ValueError:
                    continue
    return np.array(data)

def ellipsoid_fit(data):
    """
    Fit an ellipsoid to the data points.
    Returns the center (bias) and transformation matrix (soft iron).
    Based on Yury Petrov's Ellipsoid Fit.
    """
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]
    
    # Check 3D coverage - data should span all axes
    x_range = np.max(x) - np.min(x)
    y_range = np.max(y) - np.min(y)
    z_range = np.max(z) - np.min(z)
    
    if x_range < 10 or y_range < 10 or z_range < 10:
        raise ValueError(
            f"Insufficient 3D coverage!\n"
            f"Data span: X={x_range:.1f}, Y={y_range:.1f}, Z={z_range:.1f} µT\n"
            f"You need to rotate the sensor in ALL THREE axes.\n"
            f"Try figure-8 patterns and flip the sensor upside down."
        )
    
    # Simplified approach using least squares
    D = np.array([x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z]).T
    
    # Solve D * v = 1
    ones = np.ones(len(x))
    v = np.linalg.lstsq(D, ones, rcond=None)[0]
    
    # Algebraic form: Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
    A, B, C, D, E, F, G, H, I = v
    
    # Form Matrix A_mat
    A_mat = np.array([[A, D, E],
                      [D, B, F],
                      [E, F, C]])
    
    # Check if matrix is singular (determinant near zero)
    det = np.linalg.det(A_mat)
    if abs(det) < 1e-10:
        raise ValueError(
            f"Singular matrix (determinant = {det:.2e})!\n"
            f"This usually means data is planar (rotated in only 2 dimensions).\n"
            f"Make sure to rotate sensor in ALL directions, not just flat on a table."
        )
    
    # Center = - A_mat^-1 * [G, H, I]^T
    try:
        center = np.linalg.solve(-A_mat, np.array([G, H, I]))
    except np.linalg.LinAlgError as e:
        raise ValueError(f"Failed to compute center: {e}\nData may be degenerate.")
    
    # Decompose A_mat
    evals, evecs = np.linalg.eigh(A_mat)
    
    # Check for negative eigenvalues (indicates bad fit)
    if np.any(evals <= 0):
        neg_evals = evals[evals <= 0]
        raise ValueError(
            f"Negative eigenvalues detected: {neg_evals}\n"
            f"This indicates severe outliers or systematic errors in data.\n"
            f"Check that sensor is rotating smoothly and readings are stable."
        )
    
    # Radii
    radii = 1.0 / np.sqrt(evals)
    
    # Sanity check: radii should be similar (within 3x of each other)
    rad_ratio = np.max(radii) / np.min(radii)
    if rad_ratio > 3.0:
        print(f"WARNING: Large distortion detected (ratio={rad_ratio:.2f})")
        print(f"Radii: {radii}")
        print("This may indicate poor calibration quality or extreme magnetic distortion.")
    
    # Scale correction
    scale = np.zeros((3,3))
    avg_rad = np.mean(radii)
    scale[0,0] = avg_rad / radii[0]
    scale[1,1] = avg_rad / radii[1]
    scale[2,2] = avg_rad / radii[2]
    
    # Rotation
    transform = evecs @ scale @ evecs.T
    
    return center, transform, avg_rad

def main():
    if len(sys.argv) < 2:
        print("Usage: python calibrate_mag.py <log_file.csv>")
        print("Example: python calibrate_mag.py mag_log.csv")
        return

    filename = sys.argv[1]
    
    try:
        print(f"Loading {filename}...")
        
        data = load_data(filename)
        if len(data) < 100:
            print("Error: Not enough data points! Need at least 100, preferably 300+.")
            return
            
        print(f"Found {len(data)} points.")
        print("Fitting Ellipsoid...")
        
        bias, soft_iron, field_strength = ellipsoid_fit(data)
        
        print("\n" + "="*40)
        print("CALIBRATION RESULTS")
        print("="*40)
        print(f"Bias (Hard Iron): {bias[0]:.4f}, {bias[1]:.4f}, {bias[2]:.4f}")
        print(f"Field Strength: {field_strength:.4f} uT")
        print("\nSoft Iron Matrix:")
        print(f"{soft_iron[0,0]:.4f}, {soft_iron[0,1]:.4f}, {soft_iron[0,2]:.4f}")
        print(f"{soft_iron[1,0]:.4f}, {soft_iron[1,1]:.4f}, {soft_iron[1,2]:.4f}")
        print(f"{soft_iron[2,0]:.4f}, {soft_iron[2,1]:.4f}, {soft_iron[2,2]:.4f}")
        
        print("\n" + "="*40)
        print("C++ CODE (Paste into config_store.cpp / Console)")
        print("="*40)
        print("// HARD IRON BIAS")
        print(f"float mag_bias[3] = {{ {bias[0]:.4f}f, {bias[1]:.4f}f, {bias[2]:.4f}f }};")
        print("\n// SOFT IRON MATRIX")
        print(f"float mag_soft[3][3] = {{")
        print(f"  {{ {soft_iron[0,0]:.4f}f, {soft_iron[0,1]:.4f}f, {soft_iron[0,2]:.4f}f }},")
        print(f"  {{ {soft_iron[1,0]:.4f}f, {soft_iron[1,1]:.4f}f, {soft_iron[1,2]:.4f}f }},")
        print(f"  {{ {soft_iron[2,0]:.4f}f, {soft_iron[2,1]:.4f}f, {soft_iron[2,2]:.4f}f }}")
        print("};")
        print("="*40)
        
    except FileNotFoundError:
        print(f"\n❌ ERROR: File '{filename}' not found!")
        print("Make sure you saved the calibration data to this file.")
        
    except ValueError as e:
        print(f"\n❌ CALIBRATION FAILED:")
        print(str(e))
        print("\nTroubleshooting:")
        print("1. Make sure you rotated the sensor in ALL three axes")
        print("2. Don't just spin it flat on a table")
        print("3. Do figure-8 patterns and flip upside down")
        print("4. Collect at least 300 points (60 seconds at 50Hz)")
        
    except Exception as e:
        print(f"\n❌ UNEXPECTED ERROR: {type(e).__name__}")
        print(str(e))
        print("\nPlease report this error with your data file.")

if __name__ == "__main__":
    main()
