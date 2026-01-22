# IMU Visualization System

This project provides a comprehensive system for visualizing accelerometer and gyroscope data from an ICM-20948 IMU sensor in real-time. The system consists of an Arduino sketch that reads sensor data and a Python application that visualizes the data in multiple ways.

## Features

- Real-time streaming of accelerometer and gyroscope data from ICM-20948 IMU
- Configurable Digital Low-Pass Filter (DLPF) settings directly from the serial interface
- Sensor calibration for both accelerometer and gyroscope
- Three separate visualization windows:
  1. Raw accelerometer and gyroscope data plots
  2. Calibrated accelerometer and gyroscope data plots
  3. 3D cube visualization that responds to the IMU orientation in real-time
- Quaternion-based orientation tracking

## Hardware Requirements

- Arduino-compatible board (ESP32 recommended)
- ICM-20948 IMU sensor
- USB cable for connecting Arduino to computer

## Software Requirements

- Arduino IDE (or compatible environment)
- Python 3.6 or higher
- Required Python packages:
  - pyserial
  - numpy
  - matplotlib
  - PyOpenGL
  - PyOpenGL_accelerate

## Installation

### Arduino Setup

1. Connect the ICM-20948 IMU to your Arduino board:
   - Connect SDA to pin 4 (configurable in code)
   - Connect SCL to pin 5 (configurable in code)
   - Connect VCC to 3.3V
   - Connect GND to GND

2. Open the Arduino IDE and upload the `IMU_data_stream.ino` sketch to your board.

### Python Setup

1. Install the required Python packages:

```bash
pip install pyserial numpy matplotlib PyOpenGL PyOpenGL_accelerate
```

2. If you're on macOS, you might need to install GLUT:

```bash
brew install freeglut
```

## Usage

### Arduino Commands

The Arduino sketch responds to the following serial commands:

- `s` - Start/stop data streaming
- `d` - Toggle debug mode
- `c` - Toggle calibration mode
- `f` - Cycle through DLPF settings

### Running the Visualization

1. Connect your Arduino board to your computer via USB.

2. Run the Python visualization script:

```bash
python imu_visualizer.py
```

Or specify a serial port manually:

```bash
python imu_visualizer.py --port /dev/ttyUSB0
```

3. The script will automatically detect the Arduino and open three visualization windows:
   - Raw IMU Data window showing accelerometer and gyroscope readings
   - Calibrated IMU Data window showing processed sensor data
   - 3D Orientation window showing a cube that moves with the IMU

### Calibration Procedure

1. Place the IMU on a flat, stable surface.
2. Send the `c` command to enter calibration mode.
3. Slowly rotate the IMU through all possible orientations to collect calibration data.
4. Send the `c` command again to exit calibration mode and apply the calibration.

## How It Works

### Arduino Code

The Arduino sketch initializes the ICM-20948 IMU and continuously reads accelerometer and gyroscope data. It provides configurable DLPF settings to filter the sensor data and implements a calibration procedure to improve accuracy.

Data is streamed over the serial port in the following format:
```
A,ax,ay,az,G,gx,gy,gz,C,ax_cal,ay_cal,az_cal,D,gx_cal,gy_cal,gz_cal
```

Where:
- `A` - Accelerometer raw data marker
- `ax,ay,az` - Raw accelerometer values
- `G` - Gyroscope raw data marker
- `gx,gy,gz` - Raw gyroscope values
- `C` - Calibrated accelerometer data marker
- `ax_cal,ay_cal,az_cal` - Calibrated accelerometer values
- `D` - Calibrated gyroscope data marker
- `gx_cal,gy_cal,gz_cal` - Calibrated gyroscope values

### Python Visualization

The Python script reads the serial data stream and processes it for visualization:

1. **Raw Data Plots**: Display the raw sensor readings directly from the IMU.
2. **Calibrated Data Plots**: Show the sensor data after applying calibration parameters.
3. **3D Orientation**: Uses a complementary filter to combine accelerometer and gyroscope data into a quaternion representation of the IMU's orientation, which is then used to rotate a 3D cube.

## Troubleshooting

- **Serial Connection Issues**: Make sure the correct port is selected. The script attempts to auto-detect the Arduino, but you can specify the port manually with the `--port` argument.
- **3D Visualization Not Working**: Ensure PyOpenGL and GLUT are properly installed for your operating system.
- **Noisy Data**: Try adjusting the DLPF settings using the `f` command to reduce noise.

## License

This project is open-source and available under the MIT License.

## Acknowledgments

- Based on the ICM-20948 IMU sensor
- Uses Matplotlib for 2D plotting
- Uses PyOpenGL for 3D visualization
