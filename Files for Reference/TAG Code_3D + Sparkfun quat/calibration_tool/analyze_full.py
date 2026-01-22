import numpy as np
import sys

def analyze_full_data(text_data):
    """Parse the full user-provided data and analyze heading stability."""
    lines = text_data.strip().split('\n')
    
    algheadings = []
    pitches = []
    rolls = []
    
    for line in lines:
        if line.startswith('DATA,'):
            try:
                parts = line.split(',')
                if len(parts) >= 30:
                    # Based on DATA format:
                    # Index 24: Pitch (Euler)
                    # Index 25: Roll (Euler)  
                    # Index 29: AlgHeading (last column)
                    pitch = float(parts[24])
                    roll = float(parts[25])
                    alghead = float(parts[29])
                    
                    # Skip invalid readings
                    if alghead != -1.0:
                        pitches.append(pitch)
                        rolls.append(roll)
                        algheadings.append(alghead)
            except (ValueError, IndexError):
                continue
    
    if not algheadings:
        print("No valid data found")
        return
    
    # Convert to numpy
    pitches = np.array(pitches)
    rolls = np.array(rolls)
    headings = np.array(algheadings)
    
    # Unwrap headings for proper drift calculation
    headings_rad = np.radians(headings)
    headings_unwrap = np.degrees(np.unwrap(headings_rad))
    
    print(f"=== ANALYSIS RESULTS ===")
    print(f"Total Valid Samples: {len(headings)}")
    print(f"")
    print(f"MOVEMENT RANGE:")
    print(f"  Pitch: {pitches.min():.1f}° to {pitches.max():.1f}° (Range: {np.ptp(pitches):.1f}°)")
    print(f"  Roll:  {rolls.min():.1f}° to {rolls.max():.1f}° (Range: {np.ptp(rolls):.1f}°)")
    print(f"")
    print(f"HEADING STABILITY:")
    print(f"  AlgHeading Range: {headings.min():.1f}° to {headings.max():.1f}°")
    print(f"  AlgHeading Drift (unwrapped): {np.ptp(headings_unwrap):.1f}°")
    print(f"  AlgHeading Std Dev: {np.std(headings_unwrap):.1f}°")
    print(f"")
    print(f"CORRELATIONS (should be near 0):")
    print(f"  Pitch vs Heading: {np.corrcoef(pitches, headings_unwrap)[0,1]:.3f}")
    print(f"  Roll vs Heading:  {np.corrcoef(rolls, headings_unwrap)[0,1]:.3f}")
    print(f"")
    
    drift = np.ptp(headings_unwrap)
    if drift < 15:
        print("=== VERDICT: PASS ===")
        print("Heading is stable during tilt! Drift < 15°")
    elif drift < 50:
        print("=== VERDICT: IMPROVED ===")
        print("Much better than before, but some drift remains.")
    else:
        print("=== VERDICT: NEEDS MORE WORK ===")
        print(f"Drift of {drift:.1f}° is still too high.")

if __name__ == "__main__":
    # Read from stdin or file
    if len(sys.argv) > 1:
        with open(sys.argv[1], 'r') as f:
            data = f.read()
    else:
        data = sys.stdin.read()
    
    analyze_full_data(data)
