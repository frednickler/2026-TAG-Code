#!/bin/bash

# Navigate to tests directory
cd "$(dirname "$0")"

# Create venv if it doesn't exist
if [ ! -d ".venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv .venv
fi

# Activate venv
source .venv/bin/activate

# Install requirements
echo "Installing requirements..."
pip install -r requirements.txt

# Run the test agent
echo "Running Sensor Config Test Agent..."
# Use default port /dev/cu.usbserial-110 if argument not provided
if [ $# -eq 0 ]; then
    python3 sensor_config_test_agent.py --port /dev/cu.usbserial-110
else
    # Allow passing arguments
    python3 sensor_config_test_agent.py "$@"
fi
