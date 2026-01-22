// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT

#ifndef VQF_H
#define VQF_H

#include <stddef.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cassert>

// Epsilon for numeric comparisons (renamed to avoid conflict with ESP32 SDK)
#define VQF_EPS 1e-9

// #define VQF_SINGLE_PRECISION
// #define VQF_NO_MOTION_BIAS_ESTIMATION

#ifndef VQF_SINGLE_PRECISION
typedef double vqf_real_t;
#else
typedef float vqf_real_t;
#endif

struct VQFParams {
    VQFParams();
    
    vqf_real_t tauAcc;
    vqf_real_t tauMag;
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    bool motionBiasEstEnabled;
#endif
    bool restBiasEstEnabled;
    bool magDistRejectionEnabled;
    
    vqf_real_t biasSigmaInit;
    vqf_real_t biasForgettingTime;
    vqf_real_t biasClip;
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t biasSigmaMotion;
    vqf_real_t biasVerticalForgettingFactor;
#endif
    vqf_real_t biasSigmaRest;
    
    vqf_real_t restMinT;
    vqf_real_t restFilterTau;
    vqf_real_t restThGyr;
    vqf_real_t restThAcc;
    
    vqf_real_t magCurrentTau;
    vqf_real_t magRefTau;
    vqf_real_t magNormTh;
    vqf_real_t magDipTh;
    vqf_real_t magNewTime;
    vqf_real_t magNewFirstTime;
    vqf_real_t magNewMinGyr;
    vqf_real_t magMinUndisturbedTime;
    vqf_real_t magMaxRejectionTime;
    vqf_real_t magRejectionFactor;
};

struct VQFState {
    vqf_real_t gyrQuat[4];
    vqf_real_t accQuat[4];
    vqf_real_t delta;
    bool restDetected;
    bool magDistDetected;
    
    vqf_real_t lastAccLp[3];
    double accLpState[3*2];
    vqf_real_t lastAccCorrAngularRate;
    
    vqf_real_t kMagInit;
    vqf_real_t lastMagDisAngle;
    vqf_real_t lastMagCorrAngularRate;
    
    vqf_real_t bias[3];
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t biasP[9];
    double motionBiasEstRLpState[9*2];
    double motionBiasEstBiasLpState[2*2];
#else
    vqf_real_t biasP;
#endif
    
    vqf_real_t restLastSquaredDeviations[2];
    vqf_real_t restT;
    vqf_real_t restLastGyrLp[3];
    double restGyrLpState[3*2];
    vqf_real_t restLastAccLp[3];
    double restAccLpState[3*2];
    
    vqf_real_t magRefNorm;
    vqf_real_t magRefDip;
    vqf_real_t magUndisturbedT;
    vqf_real_t magRejectT;
    vqf_real_t magCandidateNorm;
    vqf_real_t magCandidateDip;
    vqf_real_t magCandidateT;
    vqf_real_t magNormDip[2];
    double magNormDipLpState[2*2];
};

struct VQFCoefficients {
    vqf_real_t gyrTs;
    vqf_real_t accTs;
    vqf_real_t magTs;
    
    double accLpB[3];
    double accLpA[2];
    
    vqf_real_t kMag;
    
    vqf_real_t biasP0;
    vqf_real_t biasV;
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t biasMotionW;
    vqf_real_t biasVerticalW;
#endif
    vqf_real_t biasRestW;
    
    double restGyrLpB[3];
    double restGyrLpA[2];
    double restAccLpB[3];
    double restAccLpA[2];
    
    vqf_real_t kMagRef;
    double magNormDipLpB[3];
    double magNormDipLpA[2];
};

class VQF {
public:
    VQF(vqf_real_t gyrTs, vqf_real_t accTs=-1.0, vqf_real_t magTs=-1.0);
    VQF(const VQFParams& params, vqf_real_t gyrTs, vqf_real_t accTs=-1.0, vqf_real_t magTs=-1.0);
    
    void updateGyr(const vqf_real_t gyr[3]);
    void updateAcc(const vqf_real_t acc[3]);
    void updateMag(const vqf_real_t mag[3]);
    void update(const vqf_real_t gyr[3], const vqf_real_t acc[3]);
    void update(const vqf_real_t gyr[3], const vqf_real_t acc[3], const vqf_real_t mag[3]);
    
    void updateBatch(const vqf_real_t gyr[], const vqf_real_t acc[], const vqf_real_t mag[], size_t N,
                     vqf_real_t out6D[], vqf_real_t out9D[], vqf_real_t outDelta[], vqf_real_t outBias[],
                     vqf_real_t outBiasSigma[], bool outRest[], bool outMagDist[]);
    
    void getQuat3D(vqf_real_t out[4]) const;
    void getQuat6D(vqf_real_t out[4]) const;
    void getQuat9D(vqf_real_t out[4]) const;
    vqf_real_t getDelta() const;
    
    vqf_real_t getBiasEstimate(vqf_real_t out[3]=nullptr) const;
    void setBiasEstimate(vqf_real_t bias[3], vqf_real_t sigma=-1.0);
    bool getRestDetected() const;
    bool getMagDistDetected() const;
    void getRelativeRestDeviations(vqf_real_t out[2]) const;
    vqf_real_t getMagRefNorm() const;
    vqf_real_t getMagRefDip() const;
    void setMagRef(vqf_real_t norm, vqf_real_t dip);
    
    void setTauAcc(vqf_real_t tauAcc);
    void setTauMag(vqf_real_t tauMag);
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    void setMotionBiasEstEnabled(bool enabled);
#endif
    void setRestBiasEstEnabled(bool enabled);
    void setMagDistRejectionEnabled(bool enabled);
    void setRestDetectionThresholds(vqf_real_t thGyr, vqf_real_t thAcc);
    
    const VQFParams& getParams() const;
    const VQFCoefficients& getCoeffs() const;
    const VQFState& getState() const;
    void setState(const VQFState& state);
    void resetState();
    
    static void quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4]);
    static void quatConj(const vqf_real_t q[4], vqf_real_t out[4]);
    static void quatSetToIdentity(vqf_real_t out[4]);
    static void quatApplyDelta(vqf_real_t q[4], vqf_real_t delta, vqf_real_t out[4]);
    static void quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3]);
    static vqf_real_t norm(const vqf_real_t vec[], size_t N);
    static void normalize(vqf_real_t vec[], size_t N);
    static void clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max);
    static vqf_real_t gainFromTau(vqf_real_t tau, vqf_real_t Ts);
    static void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, double outB[3], double outA[2]);
    static void filterInitialState(vqf_real_t x0, const double b[], const double a[], double out[2]);
    static void filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, const double b_old[3],
                                               const double a_old[2], const double b_new[3],
                                               const double a_new[2], double state[]);
    static vqf_real_t filterStep(vqf_real_t x, const double b[3], const double a[2], double state[2]);
    static void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const double b[3],
                          const double a[2], double state[], vqf_real_t out[]);
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    static void matrix3SetToScaledIdentity(vqf_real_t scale, vqf_real_t out[9]);
    static void matrix3Multiply(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]);
    static void matrix3MultiplyTpsFirst(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]);
    static void matrix3MultiplyTpsSecond(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]);
    static bool matrix3Inv(const vqf_real_t in[9], vqf_real_t out[9]);
#endif

protected:
    void setup();
    
    VQFParams params;
    VQFState state;
    VQFCoefficients coeffs;
};

#endif // VQF_H
