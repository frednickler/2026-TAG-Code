# Magnetometer Live Visualisation & Calibration

This folder contains a standalone Python utility that streams magnetometer data from
a micro-controller over a serial port, shows it live in a 3-D PyQtGraph window, and
computes **hard-iron offsets** and **soft-iron scale factors** on demand.

## Files

| File | Purpose |
|------|---------|
| `live_magnetometer.py` | Main application. Shows live data and lets you press **`C`** to run a quick hard-/soft-iron calibration which is then over-plotted in green and printed to the console. |
| `requirements.txt` | Minimal Python dependencies. |

## Quick-start

1. **Install dependencies (once):**
   ```bash
   python -m pip install -r requirements.txt
   ```

2. **Connect your Arduino / MCU** and upload firmware that streams one line per
   sample in the format:
   ```
   <mx>,<my>,<mz>\n      e.g.   -243.0,87.5,-432.8\n
   ```
   It should begin streaming when it receives the single byte `s` and stop when it
   receives `x` (these characters can be changed in the script).

3. **Run the visualiser:**
   ```bash
   # auto-detect first USB/TTY port @ 115200
   python live_magnetometer.py

   # OR explicitly specify
   python live_magnetometer.py /dev/ttyUSB0 115200
   ```

4. **Rotate the sensor** slowly through all orientations.  Blue/red/black ribbons
   representing the XY, ZY, and XZ plane projections will appear.

5. **Press `C`** once you have a good sphere of points.
   • The script prints hard-iron offsets and soft-iron scale factors.
   • A new green point-cloud (calibrated) is over-laid.

6. **Quit** with `Q`, `Esc`, or by closing the window (Ctrl-C in the terminal also works).

## Notes
* The quick calibration uses the min/max algorithm – good enough for hobby use.
* For higher accuracy you can export the raw CSV (`python live_magnetometer.py > data.csv`) and run a least-squares ellipsoid fit in MATLAB, NumPy, etc.
