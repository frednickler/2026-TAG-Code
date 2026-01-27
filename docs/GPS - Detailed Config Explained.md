# GPS Configuration - Detailed Explanation
**Document Version:** 1.0  
**Date:** 2026-01-27  
**Module:** u-blox NEO-M9N GPS  

---

## Table of Contents
1. [I2C Clock Speed](#1-i2c-clock-speed)
2. [Update Rate](#2-update-rate)
3. [Protocol Mode](#3-protocol-mode)
4. [Constellation](#4-constellation)
5. [Dynamic Model](#5-dynamic-model)
6. [SBAS](#6-sbas)
7. [QZSS](#7-qzss)
8. [Anti-Jamming](#8-anti-jamming)

---

## 1. I2C Clock Speed

### Overview
Controls the speed of I2C communication between ESP32 and GPS module on the dedicated GPS bus (Wire1, GPIO 8/9).

### Available Options
| Option | Speed | Description | Use Case |
|--------|-------|-------------|----------|
| 1 | 100 kHz | Standard Mode | Maximum compatibility, long wires |
| 2 | 200 kHz | Low Speed | Conservative setting |
| 3 | 400 kHz | **Fast Mode (DEFAULT)** | Recommended for most uses |
| 4 | 600 kHz | Fast Mode+ | Higher performance |
| 5 | 800 kHz | High Speed | Short wires only |
| 6 | 1000 kHz | Maximum | Best performance, <10cm wires |

### Performance Impact
Based on testing at different GPS update rates:

**At 1 Hz GPS:**
- 400 kHz: 145 µs GPS processing time
- Impact: Minimal (already fast)

**At 15 Hz GPS:**
- 400 kHz: 959 µs GPS processing time
- Higher speeds can reduce this by 20-30%

**At 25 Hz GPS:**
- 400 kHz: 1776 µs GPS processing time
- 1000 kHz: ~1200 µs (significant improvement)

### Recommendations
| GPS Update Rate | Recommended I2C Clock | Reasoning |
|-----------------|----------------------|-----------|
| 1-5 Hz | 400 kHz | Adequate, safe default |
| 10-15 Hz | 600-800 kHz | Better balance |
| 20-25 Hz | 1000 kHz | Maximize throughput |

### Technical Details
- **Communication:** I2C over Wire1 bus (separate from sensors)
- **Pins:** SDA=GPIO8, SCL=GPIO9
- **Data Format:** UBX binary protocol (default)
- **Saved:** Yes, persists across power cycles

---

## 2. Update Rate

### Overview
Sets how often the GPS calculates and outputs new position/velocity data.

### Available Options
| Rate | Description | Loop Impact | Best For |
|------|-------------|-------------|----------|
| 1 Hz | Once per second | ~298 Hz system | Stationary/slow movement |
| 2 Hz | 2 times/second | ~290 Hz system | Walking |
| 5 Hz | 5 times/second | ~280 Hz system | Running/cycling |
| 10 Hz | 10 times/second | ~260 Hz system | **Recommended for TAG** |
| 15 Hz | 15 times/second | ~242 Hz system | Fast vehicles |
| 20 Hz | 20 times/second | ~220 Hz system | Racing applications |
| 25 Hz | 25 times/second | ~204 Hz system | Maximum responsiveness |

### Performance Analysis
```
GPS Rate | GPS Time | System Loop Rate | Notes
---------|----------|------------------|-------
1 Hz     | 145 µs   | 298 Hz          | GPS nearly free
10 Hz    | ~600 µs  | ~260 Hz         | Good balance
15 Hz    | 959 µs   | 242 Hz          | High update
25 Hz    | 1776 µs  | 204 Hz          | GPS dominates
```

### Trade-offs

**Higher Update Rate (20-25 Hz):**
- ✅ More responsive position tracking
- ✅ Smoother movement visualization
- ✅ Better for fast-moving targets
- ❌ Slower overall system loop rate
- ❌ Higher I2C bus utilization
- ❌ More CPU time consumed

**Lower Update Rate (1-5 Hz):**
- ✅ Faster overall system performance
- ✅ More CPU time for IMU/heading
- ✅ Lower power consumption
- ❌ Choppy position updates
- ❌ May miss rapid movements

### Recommendations
- **TAG Application:** 10-15 Hz (smooth tracking, good system performance)
- **Base Station:** 1-5 Hz (position doesn't change)
- **Racing/Drones:** 20-25 Hz (maximum responsiveness)

### Technical Details
- **Unit:** Hertz (Hz) = updates per second
- **Range:** 1-25 Hz (NEO-M9N limit)
- **Saved:** Yes, persists in GPS NVM
- **Command:** `setNavigationFrequency(rate)`

---

## 3. Protocol Mode

### Overview
Selects which data format the GPS outputs over I2C.

### Available Modes

#### Mode 0: UBX (Binary) - **DEFAULT**
- **Format:** Binary protocol, compact and efficient
- **Output:** `COM_TYPE_UBX` only
- **Data Rate:** ~50-200 bytes per position update
- **Parsing:** Fast, efficient (uses SparkFun library autoPVT)
- **Use Case:** Production, normal operation
- **I2C Traffic:** Minimal, optimized

**Advantages:**
- ✅ Fastest processing
- ✅ Most efficient
- ✅ Lower I2C bandwidth
- ✅ Clean, no NMEA "noise"

**Disadvantages:**
- ❌ Not human-readable
- ❌ Requires library support

#### Mode 1: NMEA Debug (UBX + NMEA)
- **Format:** Both binary AND text sentences
- **Output:** `COM_TYPE_UBX | COM_TYPE_NMEA`
- **Data Rate:** ~300-500 bytes per position update
- **Parsing:** UBX for data, NMEA visible in serial
- **Use Case:** Debugging, development, verification
- **I2C Traffic:** Higher (both protocols active)

**Advantages:**
- ✅ Human-readable NMEA in serial monitor
- ✅ Can verify GPS data manually
- ✅ UBX still works for position parsing
- ✅ Useful for troubleshooting

**Disadvantages:**
- ❌ Slower I2C communication
- ❌ Higher CPU usage
- ❌ Cluttered serial output

### NMEA Sample Output (Debug Mode)
```
$GNGGA,123456.00,3300.12345,S,15140.67890,E,1,12,0.8,45.2,M,12.3,M,,*5A
$GNRMC,123456.00,A,3300.12345,S,15140.67890,E,0.1,346.2,270126,,,A*7F
```

### Technical Details
- **Switching:** Requires GPS restart (~3 seconds)
- **Saved:** Yes, persists in GPS flash
- **Performance Impact:** NMEA debug mode increases GPS processing time by ~30-40%

---

## 4. Constellation

### Overview
Selects which satellite systems the GPS uses for positioning.

### Available Presets

#### Preset 0: All (4 Systems) - **DEFAULT**
- **Systems:** GPS + GLONASS + Galileo + BeiDou
- **Satellites:** Typically 15-25 visible
- **Fix Time:** Fastest (more satellites = faster fix)
- **Accuracy:** Best (multi-constellation reduces errors)
- **Power:** Highest consumption

#### Preset 1: GPS + GLONASS + Galileo (3 Systems)
- **Systems:** US + Russian + European
- **Satellites:** Typically 12-20 visible
- **Fix Time:** Fast
- **Accuracy:** Excellent
- **Power:** Medium-high
- **Use Case:** BeiDou restricted regions

#### Preset 2: GPS + GLONASS (2 Systems)
- **Systems:** US + Russian only
- **Satellites:** Typically 10-15 visible
- **Fix Time:** Moderate
- **Accuracy:** Good
- **Power:** Medium
- **Use Case:** Legacy compatibility

### Satellite System Details

| System | Owner | Satellites | Coverage | Frequency |
|--------|-------|------------|----------|-----------|
| **GPS** | USA | 31 active | Global | L1/L2/L5 |
| **GLONASS** | Russia | 24 active | Global | L1/L2 |
| **Galileo** | EU | 24+ active | Global | E1/E5 |
| **BeiDou** | China | 35+ active | Global | B1/B2 |

### Performance Comparison
```
Constellation | Avg Sats | Time to Fix | Accuracy
--------------|----------|-------------|----------
All (4)       | 18-22    | 5-15 sec   | 1-2 meters
GPS+GLO+GAL   | 14-18    | 10-20 sec  | 1-3 meters
GPS+GLO       | 10-14    | 15-30 sec  | 2-4 meters
```

### Recommendations
- **General Use:** All (4) - Best performance
- **Urban/Obstructed:** All (4) - Need maximum satellites
- **Power Critical:** GPS+GLO - Lower consumption
- **Regulatory:** May need to disable BeiDou in some regions

---

## 5. Dynamic Model

### Overview
Tells the GPS what type of movement to expect, optimizing Kalman filter parameters.

### Available Models

#### Portable - **DEFAULT**
- **Max Altitude:** 12,000 m
- **Max Velocity:** 310 m/s (1116 km/h)
- **Max Vertical Velocity:** 50 m/s
- **Acceleration:** Medium-high
- **Best For:** Walking, hiking, general handheld use
- **Your TAG Application:** ✅ **RECOMMENDED**

**Characteristics:**
- Moderate position smoothing
- Accepts human movement patterns
- Filters out unrealistic jumps
- Good for pedestrian speeds

#### Automotive
- **Max Altitude:** 6,000 m (ground level)
- **Max Velocity:** 515 m/s (limited by road speeds)
- **Acceleration:** High lateral, low vertical
- **Best For:** Cars, trucks, ground vehicles

**Characteristics:**
- Strong vertical position filtering (assumes roads)
- Accepts rapid horizontal acceleration
- Rejects altitude changes
- Optimized for vehicle dynamics

#### Airborne <1g
- **Max Altitude:** 50,000 m
- **Max Velocity:** 515 m/s
- **Acceleration:** <1g (light aircraft)
- **Best For:** Drones, planes, light aircraft

**Characteristics:**
- High altitude tolerance
- Three-dimensional movement
- Accepts rapid altitude changes
- Suitable for aerial platforms

#### Stationary
- **Movement:** Assumes ZERO movement
- **Filtering:** Maximum smoothing
- **Best For:** Survey equipment, base stations

**Characteristics:**
- Treats ALL movement as noise
- Heavily averages position
- Best accuracy when truly stationary
- **DO NOT USE** for moving TAG!

### How It Works

The Dynamic Model adjusts internal GPS algorithms:

**Kalman Filter Tuning:**
- **Portable:** Medium process noise, trusts movement
- **Automotive:** Low vertical noise, high horizontal
- **Airborne:** High process noise, 3D freedom
- **Stationary:** Minimal process noise, maximum averaging

**Velocity Rejection:**
```
Model      | Speed Limit | What Happens if Exceeded
-----------|-------------|-------------------------
Portable   | 310 m/s     | Position rejected as error
Automotive | 515 m/s     | Position rejected (unlikely on roads)
Airborne   | 515 m/s     | Position accepted
Stationary | 0.5 m/s     | Movement rejected as noise!
```

### Recommendations
- **TAG (Walking/Running):** Portable ✅
- **TAG (Vehicle-mounted):** Automotive
- **Base Station:** Stationary
- **Drone TAG:** Airborne <1g

---

## 6. SBAS (Satellite-Based Augmentation System)

### Overview
Regional correction systems that improve GPS accuracy using geostationary satellites.

### What is SBAS?
SBAS broadcasts correction data to reduce GPS errors caused by:
- Atmospheric delays
- Satellite clock drift
- Orbital errors

### Regional Systems

| System | Region | Coverage | Satellites |
|--------|--------|----------|------------|
| **WAAS** | North America | USA, Canada, Mexico | INMARSAT |
| **EGNOS** | Europe | EU + nearby | INMARSAT |
| **MSAS** | Asia | Japan region | MTSAT |
| **GAGAN** | Asia | India region | GSAT |

### Benefits

**Enabled (Default):**
- ✅ Improved accuracy (1-2 meters → 0.5-1 meter)
- ✅ Reduced position drift
- ✅ Better integrity monitoring
- ✅ Faster time-to-fix
- ⚠️ Only works if in coverage area
- ⚠️ Requires clear sky view

**Disabled:**
- ❌ Standard GPS accuracy
- ✅ Works everywhere
- ✅ Slightly lower power consumption

### Performance Impact
```
Location     | SBAS Benefit | Recommendation
-------------|--------------|---------------
Australia    | Minimal      | Can disable
North America| High         | Enable
Europe       | High         | Enable
Asia (Japan) | High         | Enable
Open Ocean   | None         | Disable
```

### Recommendation for Your TAG
- **Location: Australia:** SBAS provides minimal benefit
- **Setting:** Can be disabled to save minor power
- **Default:** Enabled (no harm, auto-selects if available)

---

## 7. QZSS (Quasi-Zenith Satellite System)

### Overview
Japanese regional satellite navigation system, compatible with GPS.

### What is QZSS?
- **Owner:** Japan
- **Satellites:** 4-7 satellites
- **Orbit:** Highly inclined, figure-8 ground track
- **Coverage:** Asia-Pacific region, especially Japan
- **Compatibility:** Uses GPS-compatible signals

### Coverage Map
```
Region          | QZSS Visibility | Benefit
----------------|-----------------|----------
Japan           | Excellent       | High
Australia       | Good            | Medium
Asia-Pacific    | Good            | Medium
Europe/Americas | None            | None
```

### Benefits

**In Coverage Area (Australia):**
- ✅ Additional satellites (1-3 more)
- ✅ Better urban canyon performance
- ✅ Improved availability
- ✅ Faster fixes

**Outside Coverage:**
- No impact (satellites not visible)

### Recommendation for Your TAG
- **Location: Australia:** ✅ Enable (benefits from QZSS)
- **Performance:** Adds 1-2 satellites typically
- **Default:** Enabled

---

## 8. Anti-Jamming

### Overview
Interference detection and mitigation features.

### ⚠️ Current Status
**NOT IMPLEMENTED** - Placeholder for future feature

### Planned Functionality
- Jamming detection
- Interference monitoring
- Automatic gain control
- Requires NEO-M9N specific configuration

### Configuration
Currently does nothing (stub function).

### Recommendation
- **Status:** Unavailable
- **Impact:** None
- **Future:** May be implemented for RF-critical applications

---

## Quick Reference Table

| Setting | Your TAG Recommendation | Reasoning |
|---------|------------------------|-----------|
| **I2C Clock** | 400-600 kHz | Good balance for 10-15 Hz GPS |
| **Update Rate** | 10-15 Hz | Smooth tracking, good system performance |
| **Protocol** | UBX (Binary) | Production mode, efficient |
| **Constellation** | All (4 systems) | Maximum satellites, best accuracy |
| **Dynamic Model** | Portable | Perfect for pedestrian movement |
| **SBAS** | Enabled | No harm, minor benefit in Australia |
| **QZSS** | Enabled | Good coverage in Australia |
| **Anti-Jamming** | N/A | Not implemented |

---

## Performance Summary

**Current Configuration (Default):**
```
GPS Update Rate: 15 Hz
I2C Clock: 400 kHz
Constellation: All (4)
Dynamic Model: Portable

Result:
- GPS Processing: ~960 µs
- System Loop Rate: ~242 Hz
- Satellites Visible: 17-20
- Position Accuracy: 1-2 meters
```

**Optimized for TAG:**
```
GPS Update Rate: 10 Hz → 15 Hz
I2C Clock: 400 kHz → 600 kHz
Constellation: All (4)
Dynamic Model: Portable

Expected Result:
- GPS Processing: ~600-700 µs
- System Loop Rate: ~260-270 Hz
- Excellent balance for TAG tracking
```

---

**End of Document**
