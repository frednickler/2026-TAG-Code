//================================================================================================ 
// MadgwickAHRS.h
//================================================================================================ 
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
//================================================================================================ 
//
//  Extended documentation added 2025-07-17
//
//  OVERVIEW
//  --------
//  This header provides a complete *header-only* implementation of Sebastian
//  Madgwick’s IMU/AHRS filter converted from his original C/Matlab release.
//  The algorithm fuses gyroscope, accelerometer, and (optionally) magnetometer
//  data to estimate device orientation as a quaternion.  Compared to a simple
//  complementary filter it offers faster convergence, automatic gyroscope bias
//  rejection, and a tunable trade-off between latency and noise.
//
//  KEY CONCEPTS / PARAMETERS
//  -------------------------
//  • beta  – Gradient-descent gain (sometimes noted as 2ζ).  Larger *beta*
//            increases responsiveness but lets gyro noise leak into the
//            solution.  Smaller values give smoother output at the cost of
//            longer convergence after fast manoeuvres.  A good starting point
//            is 0.1–0.4 for 200 Hz updates, then tune empirically.
//
//  • sampleFrequency – Call begin(fHz) once (or set *invSampleFreq* directly)
//            so the integrator knows the true Δt between updates.
//
//  • update() vs updateIMU() – Call *update()* when you have gyro+accel+mag
//            readings; it automatically falls back to *updateIMU()* if the
//            magnetometer vector is zero.  If your hardware has no magneto-
//            meter, call *updateIMU()* directly for best speed.
//
//  • getQuaternion() – Lightweight getter; call as fast as you need.  Yaw/pitch/
//            roll helpers can be added if desired but beware of gimbal lock.
//
//  USAGE SUMMARY
//  -------------
//      Madgwick ahrs;
//      ahrs.begin(200);               // 200 Hz update rate
//      // inside your sensor loop:
//      ahrs.update(gx,gy,gz, ax,ay,az, mx,my,mz); // rad/s, g, µT
//      float w,x,y,z; ahrs.getQuaternion(w,x,y,z);
//
//  TUNING TIPS
//  -----------
//  1. Units – Gyro [rad/s]; Accel normalised to *g* (do this before calling);
//             Magnetometer in any consistent unit (the algorithm rescales).
//  2. Beta   – Scale roughly with (gyroNoiseRMS / gyroBandwidth).  For MEMS
//             IMUs at ≤500 dps FS a value ≈ 0.041 ≡ 2 * 0.0205 is suggested by
//             Madgwick; higher bandwidth apps may need ≈0.1–0.25.
//  3. Dynamic beta – Advanced users can adapt *beta* based on accel error.
//
//  This block is purely informational; the executable code begins below.
//
//================================================================================================ 
/*
================================================================================
 MADGWICK FILTER - QUICK START & TUNING GUIDE
================================================================================
This section provides a practical guide for tuning the Madgwick filter parameters.

1. SET SAMPLE RATE
------------------
Call begin() with your actual sensor update rate in Hz:

  Madgwick filter;
  void setup() {
      filter.begin(200);   // Set to match your sensor loop rate (e.g., 200 Hz)
  }

2. TUNING BETA (β) PARAMETER
---------------------------
Beta controls the trade-off between noise and responsiveness:

  • 0.02-0.05  Very smooth, high latency (good for VR headsets)
  • 0.05-0.15  Good default for most IMUs @100-200 Hz
  • 0.15-0.30  More responsive but noisier (good for drones)
  • >0.30      Usually too noisy, not recommended

Set beta at compile-time (before including this header):
  #define betaDef 0.08f

Or at runtime:
  filter.setBeta(0.08f);  // Adjust as needed

3. OPTIONAL: ADAPTIVE BETA
------------------------
For automatic adjustment of beta based on motion:

  filter.setMode(2);  // Enable adaptive mode
  
  // Optional: Set aggressiveness (default=0.5, range ~0.05-1.0)
  // Higher = more aggressive response to motion
  filter.setMode(2, 0.5f);

4. READING ORIENTATION
---------------------
Get the quaternion at any time:

  float w, x, y, z;
  filter.getQuaternion(w, x, y, z);

5. TROUBLESHOOTING
-----------------
- Drifting when still? Try lowering beta
- Response too slow? Increase beta or sample rate
- Yaw jumps 180°? Check magnetometer calibration
- Unstable output? Verify sensor units:
  • Gyro: rad/s
  • Accel: g (1g = 9.81 m/s²)
  • Mag: µT (microtesla)

For more details, see Madgwick's original paper:
"An efficient orientation filter for inertial and inertial/magnetic
sensor arrays" (2010)
================================================================================
*/
#ifndef Madgwick_h
#define Madgwick_h
#include <math.h>

//----------------------------------------------------------------------------------------------------
// Definitions
//
// betaDef – default β (2·ζ).
//
// • High β  (≈0.2–0.4)  → faster convergence, noisier output
// • Low β   (≈0.02–0.08)→ smoother output, slower convergence
//
// For a 200 Hz loop 0.08–0.15 is a good starting point.
//
// This is only the *initial* value: you can override it at runtime with
//   setBeta(newBeta)               // manual
//   updateBetaAdaptive(...)        // adaptive mode
// or by selecting a different Mode via setMode() / begin(cfg).
//
// #define betaDef  0.1f              // default β

//----------------------------------------------------------------------------------------------------
// Definitions

#define betaDef         0.1f            // 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable declarations

class Madgwick {
private:
    float beta;                   // Algorithm gain (2ζ)
    float q0, q1, q2, q3;         // Quaternion components
    float invSampleFreq;          // 1.0 / sample rate (Hz)
    char anglesComputed;
    uint8_t currentMode = 1;      // Default to BASIC mode
    float adaptiveScale = 0.5f;   // Default aggressiveness for adaptive mode

    // Mode-specific parameters
    struct ModeParams {
        const char* name;
        float defaultBeta;
        bool usesAdaptiveBeta;
    };

    static constexpr ModeParams MODES[3] = {
        {"BASIC",    0.1f,  false},  // Mode 1: Fixed beta
        {"ADAPTIVE", 0.1f,  true},   // Mode 2: Auto-tuning , Let the filter adjust beta based on motion. adaptive mode is particularly useful for handling both slow movements (needing low beta) and fast motions (needing higher beta)
        {"MANUAL",   0.1f,  false}   // Mode 3: User controls beta, Change beta dynamically with setBeta()
    };

public:
    Madgwick(void) {
        beta = betaDef;
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        anglesComputed = 0;
    }

    void begin(float sampleFrequency) {
        invSampleFreq = 1.0f / sampleFrequency;
    }

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
            updateIMU(gx, gy, gz, ax, ay, az);
            return;
        }

        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            _2q0mx = 2.0f * q0 * mx;
            _2q0my = 2.0f * q0 * my;
            _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q0q2 = 2.0f * q0 * q2;
            _2q2q3 = 2.0f * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        q0 += qDot1 * invSampleFreq;
        q1 += qDot2 * invSampleFreq;
        q2 += qDot3 * invSampleFreq;
        q3 += qDot4 * invSampleFreq;

        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        anglesComputed = 0;
    }

    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        q0 += qDot1 * invSampleFreq;
        q1 += qDot2 * invSampleFreq;
        q2 += qDot3 * invSampleFreq;
        q3 += qDot4 * invSampleFreq;

        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        anglesComputed = 0;
    }
    
    // Get/set the beta parameter at runtime (useful for adaptive filtering)
    void setBeta(float newBeta) {
        beta = newBeta;
    }
    
    float getBeta() const {
        return beta;
    }

    // Convert quaternion to Euler angles (yaw, pitch, roll) in radians
    // WARNING: Subject to gimbal lock when pitch approaches ±90°
    void getEulerAngles(float& yaw, float& pitch, float& roll) {
        if (!anglesComputed) {
            // Roll (x-axis rotation)
            float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
            float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
            roll = atan2f(sinr_cosp, cosr_cosp);

            // Pitch (y-axis rotation)
            float sinp = 2.0f * (q0 * q2 - q3 * q1);
            if (fabs(sinp) >= 1.0f) {
                pitch = copysignf(M_PI / 2.0f, sinp); // Use 90° if out of range
            } else {
                pitch = asinf(sinp);
            }

            // Yaw (z-axis rotation)
            float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
            float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
            yaw = atan2f(siny_cosp, cosy_cosp);
            
            anglesComputed = 1;
        }
    }

    // Example adaptive beta: Adjust based on acceleration error
    // Call this after update() to automatically tune beta
    void updateBetaAdaptive(float ax, float ay, float az, float errorScale = 1.0f) {
        // Expected gravity vector (in world frame)
        float gx = 2.0f * (q1*q3 - q0*q2);
        float gy = 2.0f * (q0*q1 + q2*q3);
        float gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        
        // Error is |1 - ||accel||| (should be 1g for static conditions)
        float error = fabs(sqrtf(ax*ax + ay*ay + az*az) - 1.0f);
        
        // Simple P-controller to adjust beta
        // errorScale tunes aggressiveness (0.1 = slow, 1.0 = fast adaptation)
        beta = betaDef * (1.0f + error * errorScale);
        beta = constrain(beta, 0.01f, 2.0f);  // Keep within sane limits
    }

    void getQuaternion(float &w, float &x, float &y, float &z) {
        w = q0;
        x = q1;
        y = q2;
        z = q3;
    }

    // ============================================
    //  MODE SELECTION & CONFIGURATION
    // ============================================
    
    /**
     * Set the filter operating mode
     * @param mode 1=BASIC, 2=ADAPTIVE, 3=MANUAL
     * @param param Optional parameter (e.g., adaptive scale factor)
     * @return True if mode was set successfully
     */
    bool setMode(uint8_t mode, float param = 0.5f) {
        if (mode < 1 || mode > 3) return false;
        currentMode = mode;
        beta = MODES[mode-1].defaultBeta;
        adaptiveScale = (param > 0) ? param : 0.5f;
        return true;
    }

    /**
     * Get current mode information
     * @param mode Output: Current mode number (1-3)
     * @param name Output: Mode name string
     * @param isAdaptive Output: Whether mode uses adaptive beta
     */
    void getModeInfo(uint8_t &mode, const char* &name, bool &isAdaptive) const {
        mode = currentMode;
        name = MODES[currentMode-1].name;
        isAdaptive = MODES[currentMode-1].usesAdaptiveBeta;
    }

    // ============================================
    //  UPDATED PROCESSING LOOP EXAMPLE
    // ============================================
    /**
     * Example processing loop with mode support
     * Call this from your main loop after reading sensors
     */
    void processIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        // Update filter with current sensor data
        updateIMU(gx, gy, gz, ax, ay, az);

        // Handle mode-specific processing
        switch (currentMode) {
            case 2: // ADAPTIVE
                updateBetaAdaptive(ax, ay, az, adaptiveScale);
                break;
            // MANUAL mode requires external setBeta() calls
        }
    }

    static float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
};
#endif
