# Test Agent Setup & Usage

## 🔧 **Setup Virtual Environment**

### 1. Create Virtual Environment
```bash
cd "/Users/xande/Documents/Arduino/2026 TAG Code"
python3 -m venv venv
```

### 2. Activate Virtual Environment
```bash
source venv/bin/activate
```

### 3. Install Dependencies
```bash
pip install pyserial
```

## 🧪 **Running Tests**

### Menu Test Agent
Tests all interactive menus and commands:
```bash
python3 tests/menu_test_agent.py
```

### Output Mode Test
Tests DEBUG/COMPACT/SILENT output modes:
```bash
python3 tests/output_mode_test.py
```

## 📋 **What Gets Tested**

### Menu Test Agent:
- ✅ Main menu navigation
- ✅ Calibration menu access
- ✅ Configuration menus (IMU, GPS, VQF, System)
- ✅ Alignment menu
- ✅ Radio menu
- ✅ Data logger menu
- ✅ Command responses
- ✅ Sensor status output

### Output Mode Test:
- ✅ DEBUG mode (all sensor data)
- ✅ COMPACT mode (GPS + heading only)
- ✅ SILENT mode (no periodic output)

## 🔌 **Requirements**

- Device connected via USB
- Serial port: `/dev/cu.usbserial-110` (auto-detected)
- Baud rate: 115200
- Python 3.x with pyserial

## 🚀 **Quick Start**

```bash
# One-time setup
cd "/Users/xande/Documents/Arduino/2026 TAG Code"
python3 -m venv venv
source venv/bin/activate
pip install pyserial

# Run tests
python3 tests/menu_test_agent.py
```

## 🛑 **Deactivate Virtual Environment**

When done testing:
```bash
deactivate
```
