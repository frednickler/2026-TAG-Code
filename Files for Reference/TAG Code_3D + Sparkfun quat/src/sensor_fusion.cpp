#include "sensor_fusion.h"
#include <math.h>

//===================================================================================================
// Definitions

#define sampleFreqDef   25.0f          // sample frequency in Hz
#define betaDef         0.1f           // 2 * proportional gain

//===================================================================================================
// Variable definitions

// Madgwick filter parameters
static float beta = betaDef;                               // 2 * proportional gain (Kp)
static float sampleFreq = sampleFreqDef;

// Quaternion of orientation from sensor frame to auxiliary frame
// (q9 for 9-DOF, q6 for 6-DOF)
static float q9[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float q6[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// Euler angles (calculated from q9)
static float roll, pitch, yaw;

//===================================================================================================
// Function prototypes (internal)
static void MadgwickAHRSupdate(float* q, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
static void MadgwickAHRSupdateIMU(float* q, float gx, float gy, float gz, float ax, float ay, float az, float dt);
static float invSqrt(float x);

//===================================================================================================
// Public Functions

void initSensorFusion(float sample_freq) {
    sampleFreq = sample_freq;
    // Reset quaternions
    q9[0] = 1.0f; q9[1] = 0.0f; q9[2] = 0.0f; q9[3] = 0.0f;
    q6[0] = 1.0f; q6[1] = 0.0f; q6[2] = 0.0f; q6[3] = 0.0f;
}

void updateSensorFusion9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    MadgwickAHRSupdate(q9, gx, gy, gz, ax, ay, az, mx, my, mz, dt);
}

void updateSensorFusion6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    MadgwickAHRSupdateIMU(q6, gx, gy, gz, ax, ay, az, dt);
}

void getQuaternion9DOF(float &w, float &x, float &y, float &z) {
    w = q9[0];
    x = q9[1];
    y = q9[2];
    z = q9[3];
}

void getQuaternion6DOF(float &w, float &x, float &y, float &z) {
    w = q6[0];
    x = q6[1];
    y = q6[2];
    z = q6[3];
}

// These functions calculate and return the Euler angles from the 9-DOF quaternion
float getRoll() {
    roll = atan2f(q9[0]*q9[1] + q9[2]*q9[3], 0.5f - q9[1]*q9[1] - q9[2]*q9[2]);
    return roll * 180.0f / M_PI;
}

float getPitch() {
    pitch = asinf(-2.0f * (q9[1]*q9[3] - q9[0]*q9[2]));
    return pitch * 180.0f / M_PI;
}

float getYaw() {
    yaw = atan2f(q9[1]*q9[2] + q9[0]*q9[3], 0.5f - q9[2]*q9[2] - q9[3]*q9[3]);
    return yaw * 180.0f / M_PI;
}

float getHeading() {
    float heading = atan2f(2.0f * (q9[1] * q9[2] + q9[0] * q9[3]), q9[0] * q9[0] + q9[1] * q9[1] - q9[2] * q9[2] - q9[3] * q9[3]);
    heading *= 180.0f / M_PI;
    if (heading < 0) {
        heading += 360.0f;
    }
    return heading;
}

// Forward prototype
static void dcmToQuaternion(const float dcm[3][3], float &w, float &x, float &y, float &z);

void computeAccelMagQuaternion(float ax, float ay, float az,
                               float mx, float my, float mz,
                               float &w, float &x, float &y, float &z) {
    // Normalize vectors
    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm; ay *= norm; az *= norm;
    norm = invSqrt(mx*mx + my*my + mz*mz);
    mx *= norm; my *= norm; mz *= norm;

    // East = gravity x magnetic
    float ex = ay * mz - az * my;
    float ey = az * mx - ax * mz;
    float ez = ax * my - ay * mx;
    norm = invSqrt(ex*ex + ey*ey + ez*ez);
    ex *= norm; ey *= norm; ez *= norm;

    // North = East x gravity
    float nx = ey * az - ez * ay;
    float ny = ez * ax - ex * az;
    float nz = ex * ay - ey * ax;

    // Build DCM (body→world): columns = East, North, -Gravity
    float dcm[3][3] = {
        {ex, nx, -ax},
        {ey, ny, -ay},
        {ez, nz, -az}
    };
    dcmToQuaternion(dcm, w, x, y, z);
}

void setQuaternion6DOF(float w, float x, float y, float z) {
    q6[0] = w; q6[1] = x; q6[2] = y; q6[3] = z;
}

static void dcmToQuaternion(const float dcm[3][3], float &w, float &x, float &y, float &z) {
    float trace = dcm[0][0] + dcm[1][1] + dcm[2][2];
    if (trace > 0.0f) {
        float s = sqrtf(trace + 1.0f) * 2.0f;
        w = 0.25f * s;
        x = (dcm[2][1] - dcm[1][2]) / s;
        y = (dcm[0][2] - dcm[2][0]) / s;
        z = (dcm[1][0] - dcm[0][1]) / s;
    } else if ((dcm[0][0] > dcm[1][1]) && (dcm[0][0] > dcm[2][2])) {
        float s = sqrtf(1.0f + dcm[0][0] - dcm[1][1] - dcm[2][2]) * 2.0f;
        w = (dcm[2][1] - dcm[1][2]) / s;
        x = 0.25f * s;
        y = (dcm[0][1] + dcm[1][0]) / s;
        z = (dcm[0][2] + dcm[2][0]) / s;
    } else if (dcm[1][1] > dcm[2][2]) {
        float s = sqrtf(1.0f + dcm[1][1] - dcm[0][0] - dcm[2][2]) * 2.0f;
        w = (dcm[0][2] - dcm[2][0]) / s;
        x = (dcm[0][1] + dcm[1][0]) / s;
        y = 0.25f * s;
        z = (dcm[1][2] + dcm[2][1]) / s;
    } else {
        float s = sqrtf(1.0f + dcm[2][2] - dcm[0][0] - dcm[1][1]) * 2.0f;
        w = (dcm[1][0] - dcm[0][1]) / s;
        x = (dcm[0][2] + dcm[2][0]) / s;
        y = (dcm[1][2] + dcm[2][1]) / s;
        z = 0.25f * s;
    }
}

//===================================================================================================
// Internal Madgwick Implementation

static void MadgwickAHRSupdate(float* q, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(q, gx, gy, gz, ax, ay, az, dt);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
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

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient descent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
}

static void MadgwickAHRSupdateIMU(float* q, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
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

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
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
