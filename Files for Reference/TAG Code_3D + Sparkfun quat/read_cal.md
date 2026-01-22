# IMU Calibration and Validation Guide

This document provides a detailed explanation of the available calibration methods for the Accelerometer, Gyroscope, Magnetometer, and Temperature sensors in this project. It covers how each method works, the differences between them, and how the results are validated to ensure accuracy.

---

## 1. Accelerometer Calibration

Accelerometer calibration corrects for manufacturing imperfections that cause bias (offset) and scale factor errors. Advanced methods also correct for cross-axis misalignment and temperature-induced drift.

### Available Methods

1.  **Simple Bias-Only (`ACCEL_CAL_SIMPLE`)**
2.  **6-Position (`ACCEL_CAL_SIX_POSITION`)**
3.  **Temperature Compensation**

### How Each Method Works

#### a. Simple Bias-Only Calibration

*   **Concept**: This is the most basic calibration. It assumes the only significant error is a constant offset (bias) on each axis. It calculates this bias when the sensor is stationary and level.
*   **Process**:
    1.  You place the device on a flat, level surface.
    2.  The system collects several hundred accelerometer samples.
    3.  It calculates the average reading for each axis (`avg_x`, `avg_y`, `avg_z`).
    4.  The bias is calculated as:
        *   `bias_x = avg_x`
        *   `bias_y = avg_y`
        *   `bias_z = avg_z - 1.0g` (since the Z-axis should read 1g due to gravity).
*   **Use Case**: Quick and simple, but only effective if scale factor and cross-axis errors are negligible.

#### b. 6-Position Calibration

*   **Concept**: This method is more robust as it calculates both **bias** and **scale factor** for each axis. It requires placing the sensor in six distinct orientations, with each axis pointing directly up and down relative to gravity.
*   **Process**:
    1.  You are prompted to place the device in 6 static positions (+X up, -X up, +Y up, -Y up, +Z up, -Z up).
    2.  In each position, the system records the average accelerometer reading.
    3.  The bias and scale are calculated using the pairs of readings:
        *   `bias_x = (reading_at_+X + reading_at_-X) / 2`
        *   `scale_x = 2.0g / (reading_at_+X - reading_at_-X)`
        *   (The same logic is applied for the Y and Z axes).
*   **Differences**: Superior to the simple method as it corrects for sensitivity differences between axes. It does not correct for cross-axis misalignment.

#### c. Temperature Compensation

*   **Concept**: Sensor bias can drift with temperature. This routine measures the bias at two different temperatures to calculate a thermal coefficient, allowing the system to compensate for drift in real-time.
*   **Process**:
    1.  A baseline bias is measured at a known starting temperature (e.g., room temp).
    2.  You are asked to change the device's temperature (e.g., by warming it in your hand).
    3.  A second bias measurement is taken at the new temperature.
    4.  The temperature coefficient is calculated as: `coeff = (bias_2 - bias_1) / (temp_2 - temp_1)`.
*   **Application**: This coefficient is then used during normal operation to adjust the bias based on the current temperature, improving accuracy across different thermal conditions.

### Accelerometer Validation

*   **How it Works**: After any calibration, `validateAccelCalibration()` is run. You place the device still on a flat surface. The system applies the new calibration and collects data.
*   **Metrics**:
    1.  **Mean Magnitude**: It calculates the average vector magnitude (`sqrt(x² + y² + z²)`). This should be very close to **1.0g**.
    2.  **Standard Deviation**: It measures the noise/jitter in the magnitude readings. This should be very low.
*   **Pass/Fail**: The calibration is considered good if the mean is within a tight tolerance of 1.0g (e.g., 0.98-1.02g) and the standard deviation is below a noise threshold.

---

## 2. Gyroscope Calibration

Gyroscope calibration primarily aims to remove the zero-rate offset or bias, which is the output from the sensor when it is perfectly stationary.

### How it Works

*   **Concept**: This is a simple bias calibration. It assumes the main source of error is a constant offset on each axis when the device is not rotating.
*   **Process**:
    1.  You are instructed to keep the device perfectly still.
    2.  The system collects several hundred gyroscope samples.
    3.  The average reading for each axis (`avg_gx`, `avg_gy`, `avg_gz`) is calculated. This average *is* the bias.
*   **Temperature Compensation**: A similar two-temperature process as the accelerometer can be performed to calculate temperature coefficients for the gyroscope bias, making it more stable over temperature changes.

### Gyroscope Validation

*   **How it Works**: After calibration, `validateGyroCalibration()` is run. The device is kept stationary, and the system analyzes the newly calibrated output.
*   **Metrics**:
    1.  **Residual Bias (Mean)**: The average of the calibrated output for each axis. This should be extremely close to **0 dps** (degrees per second).
    2.  **Standard Deviation**: Measures the noise level on each axis.
*   **Pass/Fail**: The calibration is successful if the residual bias and standard deviation are below predefined low thresholds, confirming the bias was removed and the output is stable.

---

## 3. Magnetometer Calibration

Magnetometer calibration is complex because it must correct for two types of magnetic distortion: hard iron (additive offsets) and soft iron (ellipsoidal distortion).

### Available Methods

1.  **Simple (Hard Iron Only)**
2.  **Advanced Ellipsoid Fit (Hard & Soft Iron)**

### How Each Method Works

#### a. Simple (Hard Iron)

*   **Concept**: Corrects for magnetic fields originating from the device itself (e.g., magnets, speakers). This is treated as a constant additive bias.
*   **Process**:
    1.  You slowly rotate the device to collect magnetometer readings from all orientations.
    2.  The system finds the minimum and maximum reading on each axis (`max_x`, `min_x`, etc.).
    3.  The hard iron offset (bias) is the center of this range: `bias_x = (max_x + min_x) / 2`.

#### b. Advanced Ellipsoid Fit (Hard & Soft Iron)

*   **Concept**: This is a much more robust method. It corrects for both hard iron offsets and soft iron distortions, which are caused by nearby ferrous materials that warp the Earth's magnetic field into an ellipsoid shape.
*   **Process**:
    1.  You rotate the device to collect a large set of data points, forming a distorted ellipsoid.
    2.  A mathematical algorithm (e.g., least-squares fitting) is used to find the best-fit ellipsoid for the data.
    3.  The algorithm calculates:
        *   **Hard Iron Offset (Bias)**: The center of the ellipsoid.
        *   **Soft Iron Correction (Scale/Matrix)**: A transformation matrix that reshapes the ellipsoid back into a perfect sphere.

### Magnetometer Validation

*   **How it Works**: The `validateMagCalibrationEnhanced()` function is run after calibration. You rotate the device again to collect a new set of calibrated data points.
*   **Metrics**:
    1.  **Sphere Fit Error**: After calibration, the data should form a perfect sphere. This metric calculates how much the corrected data deviates from a true sphere, reported as a percentage.
    2.  **Field Magnitude & Consistency**: It calculates the average radius (magnitude) of the data sphere and its standard deviation. The standard deviation should be very low, indicating all points are at a consistent distance from the center.
    3.  **Residual Bias**: It checks how close the center of the corrected data sphere is to (0,0,0).
*   **Pass/Fail**: The calibration is considered high-quality if the sphere fit error is low, the magnitude standard deviation is low, and the residual bias is minimal.

---

## 4. Temperature Sensor Calibration

*   **Concept**: The onboard temperature sensor itself can have an offset. This routine allows you to calibrate it against a known, accurate reference thermometer.
*   **Process**:
    1.  You measure the ambient temperature with a reference thermometer.
    2.  You enter this reference temperature into the system.
    3.  The system reads its own internal temperature sensor.
    4.  The difference between the reference and the internal reading is stored as the `tempOffset`.
*   **Application**: This offset is then applied to all future temperature readings to provide a more accurate measurement, which in turn improves the quality of temperature compensation for the other sensors.
