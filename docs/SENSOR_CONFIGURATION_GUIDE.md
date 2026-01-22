# Sensor Configuration Guide
## ICM-20948 + BMM350 Configuration Options & Defaults

---

## 🎯 **Project Context**

**Application**: TAG device for tracking/following
- Mounted on moving object (person, vehicle, animal)
- Tracks GPS position and orientation
- Transmits data to BASE station via ESP-NOW radio
- Needs accurate heading for directional tracking
- Battery-powered (power efficiency matters)

---

## 📊 **Default Configuration**

### **Gyroscope: 500dps, DLPF 24Hz**
### **Accelerometer: 4g, DLPF 24Hz**
### **Magnetometer: 12.5Hz, avg=1 (REGULAR preset)**

---

## 1️⃣ **GYROSCOPE CONFIGURATION**

### **Range Options**

| Range | Use Case | Saturation Risk |
|-------|----------|-----------------|
| **250 dps** | Slow, precise movements | High - fast turns will saturate |
| **500 dps** ✅ | Normal human/vehicle movement | Low - covers most scenarios |
| **1000 dps** | Fast vehicles, sports | Very low - only extreme movements |
| **2000 dps** | Extreme sports, aerobatics | Minimal - maximum range |

**Movement Analysis**:
- Walking: ~50-100 dps
- Running: ~100-200 dps
- Vehicle turning: ~100-300 dps
- Fast head movement: ~200-400 dps
- Extreme sports: 500-1000+ dps

**Default Choice: 500dps**
- ✅ Covers all normal human/vehicle movement
- ✅ Won't saturate during fast turns
- ✅ Better resolution than 2000dps
- ✅ Headroom for unexpected movements

### **DLPF (Digital Low-Pass Filter) Options**

| Level | Bandwidth | Response | Noise | Use Case |
|-------|-----------|----------|-------|----------|
| 0 | 361 Hz | Fastest | Highest | Raw data, high-speed |
| 1 | 197 Hz | Very fast | High | Fast response needed |
| 2 | 152 Hz | Fast | Medium-high | Sports tracking |
| 3 | 120 Hz | Medium-fast | Medium | Active movement |
| 4 | 51 Hz | Medium | Medium-low | General use |
| **5** ✅ | **24 Hz** | **Balanced** | **Low** | **Datasheet default** |
| 6 | 12 Hz | Slow | Very low | Smooth, slow movement |
| 7 | 6 Hz | Slowest | Minimal | Maximum smoothing |

**What is DLPF?**
- Filters out high-frequency noise and vibration
- Higher level = Lower bandwidth = Smoother but slower response
- Trade-off: Noise reduction vs. response time

**Default Choice: Level 5 (24Hz)**
- ✅ Datasheet recommended default
- ✅ Filters vibration (walking, vehicle engine)
- ✅ Responsive enough for direction changes
- ✅ Matches typical human movement frequency (~1-10 Hz)
- ✅ Reduces VQF heading jitter

---

## 2️⃣ **ACCELEROMETER CONFIGURATION**

### **Range Options**

| Range | Use Case | Saturation Risk |
|-------|----------|-----------------|
| **2g** | Stationary, very gentle movement | High - running will saturate |
| **4g** ✅ | Walking, running, jumping | Low - covers normal activities |
| **8g** | High-impact sports, vehicles | Very low - extreme activities only |
| **16g** | Crash detection, extreme impacts | Minimal - maximum range |

**Acceleration Analysis**:
- Gravity: 1g (always present)
- Walking: ~1.2-1.5g peak
- Running: ~2-3g peak
- Jumping: ~3-4g peak
- Vehicle braking: ~1-2g
- Impact/crash: 8-16g+

**Default Choice: 4g**
- ✅ Covers walking, running, jumping
- ✅ Vehicle tracking
- ✅ Good resolution (16-bit ADC over ±4g)
- ✅ Sufficient headroom (4x gravity)
- ✅ Won't saturate unless extreme impact

### **DLPF Options**

Same as gyroscope (see above table).

**Default Choice: Level 4 (24Hz)**
- ✅ Datasheet recommended
- ✅ Filters footstep vibration
- ✅ Preserves actual movement
- ✅ Reduces VQF noise

---

## 3️⃣ **MAGNETOMETER CONFIGURATION**

### **Output Data Rate (ODR) Options**

| ODR | Period | Use Case | Power |
|-----|--------|----------|-------|
| 1.5625 Hz | 640 ms | Very slow monitoring | Minimal |
| 3.125 Hz | 320 ms | Slow compass updates | Very low |
| 6.25 Hz | 160 ms | Basic compass | Low |
| **12.5 Hz** ✅ | **80 ms** | **Default - good balance** | **Low** |
| 25 Hz | 40 ms | Smooth compass | Medium |
| 50 Hz | 20 ms | Fast motion tracking | Medium-high |
| 100 Hz | 10 ms | Very fast tracking | High |
| 200 Hz | 5 ms | Ultra-fast (limited avg) | Very high |
| 400 Hz | 2.5 ms | Maximum (avg 0-1 only) | Maximum |

**Update Rate Analysis**:
- VQF fusion: Runs at ~100Hz (IMU rate)
- Mag updates: Can be slower (heading changes slowly)
- Human turning: ~1-5 Hz typical
- GPS updates: 10Hz (project setting)

**Default Choice: 12.5Hz**
- ✅ Faster than typical heading changes
- ✅ Matches GPS update rate (10Hz)
- ✅ Low power consumption
- ✅ Smooth heading without lag
- ✅ Good balance for VQF fusion

### **Averaging (Power Mode) Options**

| Level | Samples | Noise (X/Y) | Power @ 100Hz | Use Case |
|-------|---------|-------------|---------------|----------|
| **0** | 1 | ±190 nT | 200 µA | LOW POWER - Battery saving |
| **1** ✅ | **2** | **~135 nT** | **200 µA** | **REGULAR - Good balance** |
| **2** | 4 | ~95 nT | 335 µA | LOW NOISE - Better accuracy |
| **3** | 8 | ~67 nT | 325 µA @ 50Hz | ULTRA LOW NOISE - Best accuracy |

**Noise vs Power Trade-off**:
- More averaging = Lower noise but higher power consumption
- Each sample takes time, so higher averaging limits max ODR

**Default Choice: avg=1 (REGULAR)**
- ✅ 135 nT noise (~0.5° heading error)
- ✅ Low power (important for battery)
- ✅ 2x noise reduction over no averaging
- ✅ Fast enough (no lag)

### **⚠️ CRITICAL: Valid Combinations**

**Not all ODR + averaging combinations work!**

| ODR | Max Averaging | Reason |
|-----|---------------|--------|
| 400 Hz | avg 0-1 only | Not enough time for 4 or 8 samples |
| 200 Hz | avg 0-2 only | Not enough time for 8 samples |
| ≤100 Hz | avg 0-3 (all) | Sufficient time for all averaging levels |

**The menu will validate and reject invalid combinations!**

---

## 🔄 **Alternative Configurations**

### **Scenario A: Maximum Accuracy** (Stationary/Slow Movement)
```
Gyro: 250dps, DLPF 12Hz
Accel: 2g, DLPF 12Hz
Mag: 12.5Hz, avg=3 (ULTRA LOW NOISE)
```
**Best for**: Precision orientation, slow movement
**Trade-off**: Might saturate during fast movement

### **Scenario B: High-Speed Tracking** (Fast Vehicles, Sports)
```
Gyro: 1000dps, DLPF 51Hz
Accel: 8g, DLPF 51Hz
Mag: 25Hz, avg=1 (REGULAR)
```
**Best for**: Fast vehicles, extreme sports
**Trade-off**: More noise, less resolution

### **Scenario C: Ultra Low Power** (Long Battery Life)
```
Gyro: 500dps, DLPF 12Hz
Accel: 4g, DLPF 12Hz
Mag: 6.25Hz, avg=0 (LOW POWER)
```
**Best for**: Weeks/months of battery life
**Trade-off**: Noisier data, slower response

### **Scenario D: High Precision Heading** (Navigation)
```
Gyro: 500dps, DLPF 24Hz
Accel: 4g, DLPF 24Hz
Mag: 25Hz, avg=2 (LOW NOISE)
```
**Best for**: Precision compass/navigation
**Trade-off**: Higher power for mag

---

## 📋 **Configuration Comparison Table**

| Use Case | Gyro | Accel | Mag ODR | Mag Avg | Best For |
|----------|------|-------|---------|---------|----------|
| **Default** ✅ | 500dps | 4g | 12.5Hz | avg=1 | **General tracking** |
| Max Accuracy | 250dps | 2g | 12.5Hz | avg=3 | Stationary/slow |
| High Speed | 1000dps | 8g | 25Hz | avg=1 | Fast vehicles |
| Ultra Low Power | 500dps | 4g | 6.25Hz | avg=0 | Long battery |
| Precision Heading | 500dps | 4g | 25Hz | avg=2 | Navigation |

---

## ✅ **Why the Defaults Are Optimal**

For a **TAG tracking device**, the defaults provide:

1. **500dps gyro**: Covers all human/vehicle movement without saturation
2. **4g accel**: Handles running, jumping, vehicle movement
3. **24Hz DLPF**: Datasheet default, filters vibration, preserves movement
4. **12.5Hz mag**: Matches GPS rate, fast enough for heading changes
5. **avg=1 mag**: Good noise reduction with low power

**These defaults**:
- ✅ Work for 95% of tracking scenarios
- ✅ Balance accuracy, responsiveness, and power
- ✅ Won't saturate during normal use
- ✅ Provide smooth, accurate heading
- ✅ Match datasheet recommendations

---

## 🎛️ **How to Change Configuration**

### **Via Serial Menu**:
```
1. Enter menu: 'm'
2. Config: 'cfg'
3. IMU: '1'
4. Select sensor to configure:
   [1] Accel Preset (1-4)
   [2] Gyro Preset (1-4)
   [3] Magnetometer Configuration
       [1] Simple (Presets)
       [2] Advanced (Manual ODR + Averaging)
```

### **Presets vs Manual**:

**Simple (Presets)**: One choice, automatic configuration
- Best for: Quick setup, don't want to think about details
- Options: 4 presets per sensor

**Advanced (Manual)**: Full control with validation
- Best for: Specific use cases, fine-tuning
- Options: All ODR and averaging combinations (validated)

---

## 🎯 **Recommendations**

### **Keep Defaults If**:
- General tracking application
- Normal human/vehicle movement
- Battery life is important
- Don't have specific requirements

### **Change To High Speed If**:
- Tracking fast vehicles (>60 mph)
- Extreme sports (skateboarding, skiing)
- Rapid direction changes

### **Change To Low Power If**:
- Need weeks/months of battery
- Slow-moving applications
- Can tolerate noisier data

### **Change To High Accuracy If**:
- Precision navigation required
- Stationary or slow movement
- Power consumption not critical

---

## 📚 **Technical References**

- **ICM-20948 Datasheet**: TDK InvenSense
- **BMM350 Datasheet**: Bosch Sensortec
- **VQF Algorithm**: Sensor fusion for orientation

**Last Updated**: 2026-01-17
**Project**: 2026 TAG Code
**Hardware**: ESP32-S3 + ICM-20948 + BMM350
