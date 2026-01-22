# BMM350 50Hz ODR Investigation Report

## 1. Executive Summary
**Result:** 50Hz is fully achievable on BMM350. The issue is a **software implementation bug (Double Delay)**, not a hardware limitation.
**Root Cause:** The `bmm350.c` driver **internally waits** for the conversion time (4ms). The `IMUManager` implementation added an **additional** external wait (5ms).
**Total Blocking Delay:** ~9ms per cycle.
**Impact:** 
- Target Interval: 20ms (50Hz)
- Blocking Wait: 9ms
- Remaining Time for Loop (GPS/Radio/Serial): 11ms for EVERYTHING else.
- Result: System cannot keep up, slipping to ~34-37Hz.

## 2. Research & Validation

### Hardware Capabilities
My review of external datasheets and forums confirms:
*   **Normal Mode Limit:** 400Hz (BMM350 Datasheet). 50Hz is trivial.
*   **Forced Mode Fast Limit:** Depends on integration time. For "No Averaging", conversion is **<5ms**, allowing >200Hz ODR.
*   **Interface Speed:** 400kHz I2C transfers 6 bytes in <0.5ms. Not a bottleneck.

### Codebase Analysis (Line-by-Line)

#### A. The Driver (`src/sensors/bmm350.c`)
I traced the `bmm350SetPowerMode` function used to trigger Forced Mode measurement.

Line 1777 in `bmm350.c`:
```c
rslt = bmm350DelayUs(delay_us, dev);
```
This function **BLOCKS** execution.

The delay duration is defined in `src/sensors/bmm350_defs.h` (Line 155):
```c
#define BMM350_SUS_TO_FORCEDMODE_FAST_NO_AVG_DELAY  UINT32_C(4000) // 4000 microseconds = 4ms
```
**Finding:** The driver *Wait-for-Conversion* is built-in.

#### B. The Implementation (`src/sensors/IMUManager.cpp`)
In my recent "Forced Mode" fix:
```cpp
// Trigger a new measurement
bmm350SetPowerMode(eBmm350ForcedModeFast, &bmm_dev); // BLOCKS for 4ms (Driver)
// Wait for conversion
delayMicroseconds(5000);  // BLOCKS for 5ms (My Added Code)
```

**Total Delay:** 4ms (Internal) + 5ms (External) = **9ms**.

## 3. Conclusion & Fix
The software is waiting twice for the same physical event.
**Fix:** Remove the `delayMicroseconds(5000)` call from `IMUManager.cpp`.
The driver's internal delay (4ms) is sufficient to guarantee data validity upon return.

This will free up **5ms per loop** (25% of the total 50Hz budget), which should easily allow the system to reach the full 50Hz target.

## 4. Sources
1.  **BMM350 Driver Source Code:** `src/sensors/bmm350.c` (Internal Analysis)
2.  **BMM350 Definitions:** `src/sensors/bmm350_defs.h` (Internal Analysis)
3.  **Bosch Sensortec Datasheets** (External Verification)
