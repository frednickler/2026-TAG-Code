#!/bin/bash
# Quick setup and run script for test agents

cd "$(dirname "$0")/.."

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate venv
echo "Activating virtual environment..."
source venv/bin/activate

# Install dependencies
echo "Installing dependencies..."
pip install -q -r tests/requirements.txt

# Run test agent
echo ""
echo "Running menu test agent..."
echo "================================"
python3 tests/menu_test_agent.py

# Deactivate
deactivate
