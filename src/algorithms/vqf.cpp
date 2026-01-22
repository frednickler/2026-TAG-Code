// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT

#include "vqf.h"

#ifdef ARDUINO
#include <Arduino.h>  // For millis() and Serial in debug logging
#endif

# define NaN std::numeric_limits<vqf_real_t>::quiet_NaN()

inline vqf_real_t square(vqf_real_t x) { return x*x; }

VQFParams::VQFParams()
    : tauAcc(3.0)
    , tauMag(9.0)
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    , motionBiasEstEnabled(true)
#endif
    , restBiasEstEnabled(true)
    , magDistRejectionEnabled(true)
    , biasSigmaInit(0.5)
    , biasForgettingTime(100.0)
    , biasClip(2.0)
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    , biasSigmaMotion(0.1)
    , biasVerticalForgettingFactor(0.0001)
#endif
    , biasSigmaRest(0.03)
    , restMinT(1.5)
    , restFilterTau(0.5)
    , restThGyr(2.0)
    , restThAcc(0.5)
    , magCurrentTau(0.05)
    , magRefTau(20.0)
    , magNormTh(0.1)
    , magDipTh(10.0)
    , magNewTime(20.0)
    , magNewFirstTime(2.0)
    , magNewMinGyr(1.0)
    , magMinUndisturbedTime(0.5)
    , magMaxRejectionTime(60.0)
    , magRejectionFactor(2.0)
{
}

VQF::VQF(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    coeffs.magTs = magTs > 0 ? magTs : gyrTs;
    setup();
}

VQF::VQF(const VQFParams &params, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    this->params = params;
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    coeffs.magTs = magTs > 0 ? magTs : gyrTs;
    setup();
}

void VQF::updateGyr(const vqf_real_t gyr[3])
{
    if (params.restBiasEstEnabled || params.magDistRejectionEnabled) {
        filterVec(gyr, 3, params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA,
                  state.restGyrLpState, state.restLastGyrLp);
        
        state.restLastSquaredDeviations[0] = square(gyr[0] - state.restLastGyrLp[0])
                + square(gyr[1] - state.restLastGyrLp[1]) + square(gyr[2] - state.restLastGyrLp[2]);
        
        vqf_real_t biasClip = params.biasClip*vqf_real_t(M_PI/180.0);
        if (state.restLastSquaredDeviations[0] >= square(params.restThGyr*vqf_real_t(M_PI/180.0))
                || fabs(state.restLastGyrLp[0]) > biasClip || fabs(state.restLastGyrLp[1]) > biasClip
                || fabs(state.restLastGyrLp[2]) > biasClip) {
            state.restT = 0.0;
            state.restDetected = false;
        }
    }
    
    vqf_real_t gyrNoBias[3] = {gyr[0]-state.bias[0], gyr[1]-state.bias[1], gyr[2]-state.bias[2]};
    
    vqf_real_t gyrNorm = norm(gyrNoBias, 3);
    vqf_real_t angle = gyrNorm * coeffs.gyrTs;
    if (gyrNorm > VQF_EPS) {
        vqf_real_t c = cos(angle/2);
        vqf_real_t s = sin(angle/2)/gyrNorm;
        vqf_real_t gyrStepQuat[4] = {c, s*gyrNoBias[0], s*gyrNoBias[1], s*gyrNoBias[2]};
        quatMultiply(state.gyrQuat, gyrStepQuat, state.gyrQuat);
        normalize(state.gyrQuat, 4);
        
        /*
        // DEBUG: VQF INTERNAL STATE (Throttled)
        #ifdef ARDUINO
        extern uint32_t g_imu_debug_log_id; // Correlation ID from IMUManager
        static uint32_t last_vqf_print = 0;
        if (millis() - last_vqf_print > 500) {
            // Using Serial directly since we are deep in library
            Serial.printf("[%03d] [VQF_INT] GyrIn=[%.2f, %.2f, %.2f] -> Quat=[%.2f, %.2f, %.2f, %.2f]\n", 
                g_imu_debug_log_id,
                gyrNoBias[0], gyrNoBias[1], gyrNoBias[2],
                state.gyrQuat[0], state.gyrQuat[1], state.gyrQuat[2], state.gyrQuat[3]);
            last_vqf_print = millis();
        }
        #endif
        */
    }
}

void VQF::updateAcc(const vqf_real_t acc[3])
{
    if (acc[0] == vqf_real_t(0.0) && acc[1] == vqf_real_t(0.0) && acc[2] == vqf_real_t(0.0)) {
        return;
    }
    
    if (params.restBiasEstEnabled) {
        filterVec(acc, 3, params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA,
                  state.restAccLpState, state.restLastAccLp);
        
        state.restLastSquaredDeviations[1] = square(acc[0] - state.restLastAccLp[0])
                + square(acc[1] - state.restLastAccLp[1]) + square(acc[2] - state.restLastAccLp[2]);
        
        if (state.restLastSquaredDeviations[1] >= square(params.restThAcc)) {
            state.restT = 0.0;
            state.restDetected = false;
        } else {
            state.restT += coeffs.accTs;
            if (state.restT >= params.restMinT) {
                state.restDetected = true;
            }
        }
    }
    
    vqf_real_t accEarth[3];
    
    quatRotate(state.gyrQuat, acc, accEarth);
    filterVec(accEarth, 3, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.accLpState, state.lastAccLp);
    
    quatRotate(state.accQuat, state.lastAccLp, accEarth);
    normalize(accEarth, 3);
    
    vqf_real_t accCorrQuat[4];
    vqf_real_t q_w = sqrt((accEarth[2]+1)/2);
    if (q_w > 1e-6) {
        accCorrQuat[0] = q_w;
        accCorrQuat[1] = 0.5*accEarth[1]/q_w;
        accCorrQuat[2] = -0.5*accEarth[0]/q_w;
        accCorrQuat[3] = 0;
    } else {
        accCorrQuat[0] = 0;
        accCorrQuat[1] = 1;
        accCorrQuat[2] = 0;
        accCorrQuat[3] = 0;
    }
    quatMultiply(accCorrQuat, state.accQuat, state.accQuat);
    normalize(state.accQuat, 4);
    
    state.lastAccCorrAngularRate = acos(accEarth[2])/coeffs.accTs;
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    if (params.motionBiasEstEnabled || params.restBiasEstEnabled) {
        vqf_real_t biasClip = params.biasClip*vqf_real_t(M_PI/180.0);
        
        vqf_real_t accGyrQuat[4];
        vqf_real_t R[9];
        vqf_real_t biasLp[2];
        
        getQuat6D(accGyrQuat);
        R[0] = 1 - 2*square(accGyrQuat[2]) - 2*square(accGyrQuat[3]);
        R[1] = 2*(accGyrQuat[2]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[3]);
        R[2] = 2*(accGyrQuat[0]*accGyrQuat[2] + accGyrQuat[3]*accGyrQuat[1]);
        R[3] = 2*(accGyrQuat[0]*accGyrQuat[3] + accGyrQuat[2]*accGyrQuat[1]);
        R[4] = 1 - 2*square(accGyrQuat[1]) - 2*square(accGyrQuat[3]);
        R[5] = 2*(accGyrQuat[2]*accGyrQuat[3] - accGyrQuat[1]*accGyrQuat[0]);
        R[6] = 2*(accGyrQuat[3]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[2]);
        R[7] = 2*(accGyrQuat[0]*accGyrQuat[1] + accGyrQuat[3]*accGyrQuat[2]);
        R[8] = 1 - 2*square(accGyrQuat[1]) - 2*square(accGyrQuat[2]);
        
        biasLp[0] = R[0]*state.bias[0] + R[1]*state.bias[1] + R[2]*state.bias[2];
        biasLp[1] = R[3]*state.bias[0] + R[4]*state.bias[1] + R[5]*state.bias[2];
        
        filterVec(R, 9, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstRLpState, R);
        filterVec(biasLp, 2, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstBiasLpState,
                  biasLp);
        
        vqf_real_t w[3];
        vqf_real_t e[3];
        if (state.restDetected && params.restBiasEstEnabled) {
            e[0] = state.restLastGyrLp[0] - state.bias[0];
            e[1] = state.restLastGyrLp[1] - state.bias[1];
            e[2] = state.restLastGyrLp[2] - state.bias[2];
            matrix3SetToScaledIdentity(1.0, R);
            std::fill(w, w+3, coeffs.biasRestW);
        } else if (params.motionBiasEstEnabled) {
            e[0] = -accEarth[1]/coeffs.accTs + biasLp[0] - R[0]*state.bias[0] - R[1]*state.bias[1] - R[2]*state.bias[2];
            e[1] = accEarth[0]/coeffs.accTs + biasLp[1] - R[3]*state.bias[0] - R[4]*state.bias[1] - R[5]*state.bias[2];
            e[2] = - R[6]*state.bias[0] - R[7]*state.bias[1] - R[8]*state.bias[2];
            w[0] = coeffs.biasMotionW;
            w[1] = coeffs.biasMotionW;
            w[2] = coeffs.biasVerticalW;
        } else {
            std::fill(w, w+3, -1);
        }
        
        if (state.biasP[0] < coeffs.biasP0) {
            state.biasP[0] += coeffs.biasV;
        }
        if (state.biasP[4] < coeffs.biasP0) {
            state.biasP[4] += coeffs.biasV;
        }
        if (state.biasP[8] < coeffs.biasP0) {
            state.biasP[8] += coeffs.biasV;
        }
        if (w[0] >= 0) {
            clip(e, 3, -biasClip, biasClip);
            
            vqf_real_t K[9];
            matrix3MultiplyTpsSecond(state.biasP, R, K);
            matrix3Multiply(R, K, K);
            K[0] += w[0];
            K[4] += w[1];
            K[8] += w[2];
            matrix3Inv(K, K);
            matrix3MultiplyTpsFirst(R, K, K);
            matrix3Multiply(state.biasP, K, K);
            
            state.bias[0] += K[0]*e[0] + K[1]*e[1] + K[2]*e[2];
            state.bias[1] += K[3]*e[0] + K[4]*e[1] + K[5]*e[2];
            state.bias[2] += K[6]*e[0] + K[7]*e[1] + K[8]*e[2];
            
            matrix3Multiply(K, R, K);
            matrix3Multiply(K, state.biasP, K);
            for(size_t i = 0; i < 9; i++) {
                state.biasP[i] -= K[i];
            }
            
            clip(state.bias, 3, -biasClip, biasClip);
        }
    }
#else
    if (params.restBiasEstEnabled) {
        vqf_real_t biasClip = params.biasClip*vqf_real_t(M_PI/180.0);
        if (state.biasP < coeffs.biasP0) {
            state.biasP += coeffs.biasV;
        }
        if (state.restDetected) {
            vqf_real_t e[3];
            e[0] = state.restLastGyrLp[0] - state.bias[0];
            e[1] = state.restLastGyrLp[1] - state.bias[1];
            e[2] = state.restLastGyrLp[2] - state.bias[2];
            clip(e, 3, -biasClip, biasClip);
            
            vqf_real_t k = state.biasP/(coeffs.biasRestW + state.biasP);
            state.bias[0] += k*e[0];
            state.bias[1] += k*e[1];
            state.bias[2] += k*e[2];
            state.biasP -= k*state.biasP;
            clip(state.bias, 3, -biasClip, biasClip);
        }
    }
#endif
}

void VQF::updateMag(const vqf_real_t mag[3])
{
    if (mag[0] == vqf_real_t(0.0) && mag[1] == vqf_real_t(0.0) && mag[2] == vqf_real_t(0.0)) {
        return;
    }
    
    vqf_real_t magEarth[3];
    
    vqf_real_t accGyrQuat[4];
    getQuat6D(accGyrQuat);
    quatRotate(accGyrQuat, mag, magEarth);
    
    if (params.magDistRejectionEnabled) {
        state.magNormDip[0] = norm(magEarth, 3);
        state.magNormDip[1] = -asin(magEarth[2]/state.magNormDip[0]);
        
        if (params.magCurrentTau > 0) {
            filterVec(state.magNormDip, 2, params.magCurrentTau, coeffs.magTs, coeffs.magNormDipLpB,
                      coeffs.magNormDipLpA, state.magNormDipLpState, state.magNormDip);
        }
        
        if (fabs(state.magNormDip[0] - state.magRefNorm) < params.magNormTh*state.magRefNorm
                && fabs(state.magNormDip[1] - state.magRefDip) < params.magDipTh*vqf_real_t(M_PI/180.0)) {
            state.magUndisturbedT += coeffs.magTs;
            if (state.magUndisturbedT >= params.magMinUndisturbedTime) {
                state.magDistDetected = false;
                state.magRefNorm += coeffs.kMagRef*(state.magNormDip[0] - state.magRefNorm);
                state.magRefDip += coeffs.kMagRef*(state.magNormDip[1] - state.magRefDip);
            }
        } else {
            state.magUndisturbedT = 0.0;
            state.magDistDetected = true;
        }
        
        if (fabs(state.magNormDip[0] - state.magCandidateNorm) < params.magNormTh*state.magCandidateNorm
                && fabs(state.magNormDip[1] - state.magCandidateDip) < params.magDipTh*vqf_real_t(M_PI/180.0)) {
            if (norm(state.restLastGyrLp, 3) >= params.magNewMinGyr*M_PI/180.0) {
                state.magCandidateT += coeffs.magTs;
            }
            state.magCandidateNorm += coeffs.kMagRef*(state.magNormDip[0] - state.magCandidateNorm);
            state.magCandidateDip += coeffs.kMagRef*(state.magNormDip[1] - state.magCandidateDip);
            
            if (state.magDistDetected && (state.magCandidateT >= params.magNewTime || (
                    state.magRefNorm == 0.0 && state.magCandidateT >= params.magNewFirstTime))) {
                state.magRefNorm = state.magCandidateNorm;
                state.magRefDip = state.magCandidateDip;
                state.magDistDetected = false;
                state.magUndisturbedT = params.magMinUndisturbedTime;
            }
        } else {
            state.magCandidateT = 0.0;
            state.magCandidateNorm = state.magNormDip[0];
            state.magCandidateDip = state.magNormDip[1];
        }
    }
    
    state.lastMagDisAngle = atan2(magEarth[0], magEarth[1]) - state.delta;
    
    if (state.lastMagDisAngle > vqf_real_t(M_PI)) {
        state.lastMagDisAngle -= vqf_real_t(2*M_PI);
    } else if (state.lastMagDisAngle < vqf_real_t(-M_PI)) {
        state.lastMagDisAngle += vqf_real_t(2*M_PI);
    }
    
    vqf_real_t k = coeffs.kMag;
    
    if (params.magDistRejectionEnabled) {
        if (state.magDistDetected) {
            if (state.magRejectT <= params.magMaxRejectionTime) {
                state.magRejectT += coeffs.magTs;
                k = 0;
            } else {
                k /= params.magRejectionFactor;
            }
        } else {
            state.magRejectT = std::max(state.magRejectT - params.magRejectionFactor*coeffs.magTs, vqf_real_t(0.0));
        }
    }
    
    if (state.kMagInit != vqf_real_t(0.0)) {
        if (k < state.kMagInit) {
            k = state.kMagInit;
        }
        
        state.kMagInit = state.kMagInit/(state.kMagInit+1);
        
        if (state.kMagInit*params.tauMag < coeffs.magTs) {
            state.kMagInit = 0.0;
        }
    }
    
    state.delta += k*state.lastMagDisAngle;
    state.lastMagCorrAngularRate = k*state.lastMagDisAngle/coeffs.magTs;
    
    if (state.delta > vqf_real_t(M_PI)) {
        state.delta -= vqf_real_t(2*M_PI);
    } else if (state.delta < vqf_real_t(-M_PI)) {
        state.delta += vqf_real_t(2*M_PI);
    }
}

void VQF::update(const vqf_real_t gyr[3], const vqf_real_t acc[3])
{
    updateGyr(gyr);
    updateAcc(acc);
}

void VQF::update(const vqf_real_t gyr[3], const vqf_real_t acc[3], const vqf_real_t mag[3])
{
    updateGyr(gyr);
    updateAcc(acc);
    updateMag(mag);
}

void VQF::updateBatch(const vqf_real_t gyr[], const vqf_real_t acc[], const vqf_real_t mag[], size_t N,
                      vqf_real_t out6D[], vqf_real_t out9D[], vqf_real_t outDelta[], vqf_real_t outBias[],
                      vqf_real_t outBiasSigma[], bool outRest[], bool outMagDist[])
{
    for (size_t i = 0; i < N; i++) {
        if (mag) {
            update(gyr+3*i, acc+3*i, mag+3*i);
        } else {
            update(gyr+3*i, acc+3*i);
        }
        if (out6D) {
            getQuat6D(out6D+4*i);
        }
        if (out9D) {
            getQuat9D(out9D+4*i);
        }
        if (outDelta) {
            outDelta[i] = state.delta;
        }
        if (outBias) {
            std::copy(state.bias, state.bias+3, outBias+3*i);
        }
        if (outBiasSigma) {
            outBiasSigma[i] = getBiasEstimate(0);
        }
        if (outRest) {
            outRest[i] = state.restDetected;
        }
        if (outMagDist) {
            outMagDist[i] = state.magDistDetected;
        }
    }
}

void VQF::getQuat3D(vqf_real_t out[4]) const
{
    std::copy(state.gyrQuat, state.gyrQuat+4, out);
}

void VQF::getQuat6D(vqf_real_t out[4]) const
{
    quatMultiply(state.accQuat, state.gyrQuat, out);
}

void VQF::getQuat9D(vqf_real_t out[4]) const
{
    quatMultiply(state.accQuat, state.gyrQuat, out);
    quatApplyDelta(out, state.delta, out);
}

vqf_real_t VQF::getDelta() const
{
    return state.delta;
}

vqf_real_t VQF::getBiasEstimate(vqf_real_t out[3]) const
{
    if (out) {
        std::copy(state.bias, state.bias+3, out);
    }
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t sum1 = fabs(state.biasP[0]) + fabs(state.biasP[1]) + fabs(state.biasP[2]);
    vqf_real_t sum2 = fabs(state.biasP[3]) + fabs(state.biasP[4]) + fabs(state.biasP[5]);
    vqf_real_t sum3 = fabs(state.biasP[6]) + fabs(state.biasP[7]) + fabs(state.biasP[8]);
    vqf_real_t P = std::min(std::max(std::max(sum1, sum2), sum3), coeffs.biasP0);
#else
    vqf_real_t P = state.biasP;
#endif
    return sqrt(P)*vqf_real_t(M_PI/100.0/180.0);
}

void VQF::setBiasEstimate(vqf_real_t bias[3], vqf_real_t sigma)
{
    std::copy(bias, bias+3, state.bias);
    if (sigma > 0) {
        vqf_real_t P = square(sigma*vqf_real_t(180.0*100.0/M_PI));
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
        matrix3SetToScaledIdentity(P, state.biasP);
#else
        state.biasP = P;
#endif
    }
}

bool VQF::getRestDetected() const
{
    return state.restDetected;
}

bool VQF::getMagDistDetected() const
{
    return state.magDistDetected;
}

void VQF::getRelativeRestDeviations(vqf_real_t out[2]) const
{
    out[0] = sqrt(state.restLastSquaredDeviations[0]) / (params.restThGyr*vqf_real_t(M_PI/180.0));
    out[1] = sqrt(state.restLastSquaredDeviations[1]) / params.restThAcc;
}

vqf_real_t VQF::getMagRefNorm() const
{
    return state.magRefNorm;
}

vqf_real_t VQF::getMagRefDip() const
{
    return state.magRefDip;
}

void VQF::setMagRef(vqf_real_t norm, vqf_real_t dip)
{
    state.magRefNorm = norm;
    state.magRefDip = dip;
}

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void VQF::setMotionBiasEstEnabled(bool enabled)
{
    if (params.motionBiasEstEnabled == enabled) {
        return;
    }
    params.motionBiasEstEnabled = enabled;
    std::fill(state.motionBiasEstRLpState, state.motionBiasEstRLpState + 9*2, NaN);
    std::fill(state.motionBiasEstBiasLpState, state.motionBiasEstBiasLpState + 2*2, NaN);
}
#endif

void VQF::setRestBiasEstEnabled(bool enabled)
{
    if (params.restBiasEstEnabled == enabled) {
        return;
    }
    params.restBiasEstEnabled = enabled;
    state.restDetected = false;
    std::fill(state.restLastSquaredDeviations, state.restLastSquaredDeviations + 3, 0.0);
    state.restT = 0.0;
    std::fill(state.restLastGyrLp, state.restLastGyrLp + 3, 0.0);
    std::fill(state.restGyrLpState, state.restGyrLpState + 3*2, NaN);
    std::fill(state.restLastAccLp, state.restLastAccLp + 3, 0.0);
    std::fill(state.restAccLpState, state.restAccLpState + 3*2, NaN);
}

void VQF::setMagDistRejectionEnabled(bool enabled)
{
    if (params.magDistRejectionEnabled == enabled) {
        return;
    }
    params.magDistRejectionEnabled = enabled;
    state.magDistDetected = true;
    state.magRefNorm = 0.0;
    state.magRefDip = 0.0;
    state.magUndisturbedT = 0.0;
    state.magRejectT = params.magMaxRejectionTime;
    state.magCandidateNorm = -1.0;
    state.magCandidateDip = 0.0;
    state.magCandidateT = 0.0;
    std::fill(state.magNormDipLpState, state.magNormDipLpState + 2*2, NaN);
}

void VQF::setTauAcc(vqf_real_t tauAcc)
{
    if (params.tauAcc == tauAcc) {
        return;
    }
    params.tauAcc = tauAcc;
    double newB[3];
    double newA[2];
    
    filterCoeffs(params.tauAcc, coeffs.accTs, newB, newA);
    filterAdaptStateForCoeffChange(state.lastAccLp, 3, coeffs.accLpB, coeffs.accLpA, newB, newA, state.accLpState);
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t R[9];
    for (size_t i = 0; i < 9; i++) {
        R[i] = state.motionBiasEstRLpState[2*i];
    }
    filterAdaptStateForCoeffChange(R, 9, coeffs.accLpB, coeffs.accLpA, newB, newA, state.motionBiasEstRLpState);
    vqf_real_t biasLp[2];
    for (size_t i = 0; i < 2; i++) {
        biasLp[i] = state.motionBiasEstBiasLpState[2*i];
    }
    filterAdaptStateForCoeffChange(biasLp, 2, coeffs.accLpB, coeffs.accLpA, newB, newA, state.motionBiasEstBiasLpState);
#endif
    std::copy(newB, newB+3, coeffs.accLpB);
    std::copy(newA, newA+2, coeffs.accLpA);
}

void VQF::setTauMag(vqf_real_t tauMag)
{
    params.tauMag = tauMag;
    coeffs.kMag = gainFromTau(params.tauMag, coeffs.magTs);
}

void VQF::setRestDetectionThresholds(vqf_real_t thGyr, vqf_real_t thAcc)
{
    params.restThGyr = thGyr;
    params.restThAcc = thAcc;
}

const VQFParams& VQF::getParams() const
{
    return params;
}

const VQFCoefficients& VQF::getCoeffs() const
{
    return coeffs;
}

const VQFState& VQF::getState() const
{
    return state;
}

void VQF::setState(const VQFState& state)
{
    this->state = state;
}

void VQF::resetState()
{
    quatSetToIdentity(state.gyrQuat);
    quatSetToIdentity(state.accQuat);
    state.delta = 0.0;
    
    state.restDetected = false;
    state.magDistDetected = true;
    
    std::fill(state.lastAccLp, state.lastAccLp+3, 0);
    std::fill(state.accLpState, state.accLpState + 3*2, NaN);
    state.lastAccCorrAngularRate = 0.0;
    
    state.kMagInit = 1.0;
    state.lastMagDisAngle = 0.0;
    state.lastMagCorrAngularRate = 0.0;
    
    std::fill(state.bias, state.bias+3, 0);
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    matrix3SetToScaledIdentity(coeffs.biasP0, state.biasP);
    std::fill(state.motionBiasEstRLpState, state.motionBiasEstRLpState + 9*2, NaN);
    std::fill(state.motionBiasEstBiasLpState, state.motionBiasEstBiasLpState + 2*2, NaN);
#else
    state.biasP = coeffs.biasP0;
#endif
    
    std::fill(state.restLastSquaredDeviations, state.restLastSquaredDeviations + 3, 0.0);
    state.restT = 0.0;
    std::fill(state.restLastGyrLp, state.restLastGyrLp + 3, 0.0);
    std::fill(state.restGyrLpState, state.restGyrLpState + 3*2, NaN);
    std::fill(state.restLastAccLp, state.restLastAccLp + 3, 0.0);
    std::fill(state.restAccLpState, state.restAccLpState + 3*2, NaN);
    
    state.magRefNorm = 0.0;
    state.magRefDip = 0.0;
    state.magUndisturbedT = 0.0;
    state.magRejectT = params.magMaxRejectionTime;
    state.magCandidateNorm = -1.0;
    state.magCandidateDip = 0.0;
    state.magCandidateT = 0.0;
    std::fill(state.magNormDip, state.magNormDip + 2, 0);
    std::fill(state.magNormDipLpState, state.magNormDipLpState + 2*2, NaN);
}

void VQF::quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4])
{
    vqf_real_t w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    vqf_real_t x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    vqf_real_t y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    vqf_real_t z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void VQF::quatConj(const vqf_real_t q[4], vqf_real_t out[4])
{
    vqf_real_t w = q[0];
    vqf_real_t x = -q[1];
    vqf_real_t y = -q[2];
    vqf_real_t z = -q[3];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void VQF::quatSetToIdentity(vqf_real_t out[4])
{
    out[0] = 1;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
}

void VQF::quatApplyDelta(vqf_real_t q[4], vqf_real_t delta, vqf_real_t out[4])
{
    vqf_real_t c = cos(delta/2);
    vqf_real_t s = sin(delta/2);
    vqf_real_t w = c * q[0] - s * q[3];
    vqf_real_t x = c * q[1] - s * q[2];
    vqf_real_t y = c * q[2] + s * q[1];
    vqf_real_t z = c * q[3] + s * q[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void VQF::quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3])
{
    vqf_real_t x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0] + 2*v[1]*(q[2]*q[1] - q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
    vqf_real_t y = 2*v[0]*(q[0]*q[3] + q[2]*q[1]) + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3]) + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
    vqf_real_t z = 2*v[0]*(q[3]*q[1] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[3]*q[2]) + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
    out[0] = x; out[1] = y; out[2] = z;
}

vqf_real_t VQF::norm(const vqf_real_t vec[], size_t N)
{
    vqf_real_t s = 0;
    for(size_t i = 0; i < N; i++) {
        s += vec[i]*vec[i];
    }
    return sqrt(s);
}

void VQF::normalize(vqf_real_t vec[], size_t N)
{
    vqf_real_t n = norm(vec, N);
    if (n < VQF_EPS) {
        return;
    }
    for(size_t i = 0; i < N; i++) {
        vec[i] /= n;
    }
}

void VQF::clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max)
{
    for(size_t i = 0; i < N; i++) {
        if (vec[i] < min) {
            vec[i] = min;
        } else if (vec[i] > max) {
            vec[i] = max;
        }
    }
}

vqf_real_t VQF::gainFromTau(vqf_real_t tau, vqf_real_t Ts)
{
    assert(Ts > 0);
    if (tau < vqf_real_t(0.0)) {
        return 0;
    } else if (tau == vqf_real_t(0.0)) {
        return 1;
    } else {
        return 1 - exp(-Ts/tau);
    }
}

void VQF::filterCoeffs(vqf_real_t tau, vqf_real_t Ts, double outB[3], double outA[2])
{
    assert(tau > 0);
    assert(Ts > 0);
    double fc = (M_SQRT2 / (2.0*M_PI))/double(tau);
    double C = tan(M_PI*fc*double(Ts));
    double D = C*C + sqrt(2)*C + 1;
    double b0 = C*C/D;
    outB[0] = b0;
    outB[1] = 2*b0;
    outB[2] = b0;
    outA[0] = 2*(C*C-1)/D;
    outA[1] = (1-sqrt(2)*C+C*C)/D;
}

void VQF::filterInitialState(vqf_real_t x0, const double b[3], const double a[2], double out[2])
{
    out[0] = x0*(1 - b[0]);
    out[1] = x0*(b[2] - a[1]);
}

void VQF::filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, const double b_old[3],
                                         const double a_old[2], const double b_new[3],
                                         const double a_new[2], double state[])
{
    if (std::isnan(state[0])) {
        return;
    }
    for (size_t i = 0; i < N; i++) {
        state[0+2*i] = state[0+2*i] + (b_old[0] - b_new[0])*last_y[i];
        state[1+2*i] = state[1+2*i] + (b_old[1] - b_new[1] - a_old[0] + a_new[0])*last_y[i];
    }
}

vqf_real_t VQF::filterStep(vqf_real_t x, const double b[3], const double a[2], double state[2])
{
    double y = b[0]*x + state[0];
    state[0] = b[1]*x - a[0]*y + state[1];
    state[1] = b[2]*x - a[1]*y;
    return y;
}

void VQF::filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const double b[3],
                    const double a[2], double state[], vqf_real_t out[])
{
    assert(N>=2);
    
    if (std::isnan(state[0])) {
        if (std::isnan(state[1])) {
            state[1] = 0;
            for(size_t i = 0; i < N; i++) {
                state[2+i] = 0;
            }
        }
        state[1]++;
        for (size_t i = 0; i < N; i++) {
            state[2+i] += x[i];
            out[i] = state[2+i]/state[1];
        }
        if (state[1]*Ts >= tau) {
            for(size_t i = 0; i < N; i++) {
               filterInitialState(out[i], b, a, state+2*i);
            }
        }
        return;
    }
    
    for (size_t i = 0; i < N; i++) {
        out[i] = filterStep(x[i], b, a, state+2*i);
    }
}

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void VQF::matrix3SetToScaledIdentity(vqf_real_t scale, vqf_real_t out[9])
{
    out[0] = scale;
    out[1] = 0.0;
    out[2] = 0.0;
    out[3] = 0.0;
    out[4] = scale;
    out[5] = 0.0;
    out[6] = 0.0;
    out[7] = 0.0;
    out[8] = scale;
}

void VQF::matrix3Multiply(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[1]*in2[3] + in1[2]*in2[6];
    tmp[1] = in1[0]*in2[1] + in1[1]*in2[4] + in1[2]*in2[7];
    tmp[2] = in1[0]*in2[2] + in1[1]*in2[5] + in1[2]*in2[8];
    tmp[3] = in1[3]*in2[0] + in1[4]*in2[3] + in1[5]*in2[6];
    tmp[4] = in1[3]*in2[1] + in1[4]*in2[4] + in1[5]*in2[7];
    tmp[5] = in1[3]*in2[2] + in1[4]*in2[5] + in1[5]*in2[8];
    tmp[6] = in1[6]*in2[0] + in1[7]*in2[3] + in1[8]*in2[6];
    tmp[7] = in1[6]*in2[1] + in1[7]*in2[4] + in1[8]*in2[7];
    tmp[8] = in1[6]*in2[2] + in1[7]*in2[5] + in1[8]*in2[8];
    std::copy(tmp, tmp+9, out);
}

void VQF::matrix3MultiplyTpsFirst(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[3]*in2[3] + in1[6]*in2[6];
    tmp[1] = in1[0]*in2[1] + in1[3]*in2[4] + in1[6]*in2[7];
    tmp[2] = in1[0]*in2[2] + in1[3]*in2[5] + in1[6]*in2[8];
    tmp[3] = in1[1]*in2[0] + in1[4]*in2[3] + in1[7]*in2[6];
    tmp[4] = in1[1]*in2[1] + in1[4]*in2[4] + in1[7]*in2[7];
    tmp[5] = in1[1]*in2[2] + in1[4]*in2[5] + in1[7]*in2[8];
    tmp[6] = in1[2]*in2[0] + in1[5]*in2[3] + in1[8]*in2[6];
    tmp[7] = in1[2]*in2[1] + in1[5]*in2[4] + in1[8]*in2[7];
    tmp[8] = in1[2]*in2[2] + in1[5]*in2[5] + in1[8]*in2[8];
    std::copy(tmp, tmp+9, out);
}

void VQF::matrix3MultiplyTpsSecond(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[1]*in2[1] + in1[2]*in2[2];
    tmp[1] = in1[0]*in2[3] + in1[1]*in2[4] + in1[2]*in2[5];
    tmp[2] = in1[0]*in2[6] + in1[1]*in2[7] + in1[2]*in2[8];
    tmp[3] = in1[3]*in2[0] + in1[4]*in2[1] + in1[5]*in2[2];
    tmp[4] = in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5];
    tmp[5] = in1[3]*in2[6] + in1[4]*in2[7] + in1[5]*in2[8];
    tmp[6] = in1[6]*in2[0] + in1[7]*in2[1] + in1[8]*in2[2];
    tmp[7] = in1[6]*in2[3] + in1[7]*in2[4] + in1[8]*in2[5];
    tmp[8] = in1[6]*in2[6] + in1[7]*in2[7] + in1[8]*in2[8];
    std::copy(tmp, tmp+9, out);
}

bool VQF::matrix3Inv(const vqf_real_t in[9], vqf_real_t out[9])
{
    double A = in[4]*in[8] - in[5]*in[7];
    double D = in[2]*in[7] - in[1]*in[8];
    double G = in[1]*in[5] - in[2]*in[4];
    double B = in[5]*in[6] - in[3]*in[8];
    double E = in[0]*in[8] - in[2]*in[6];
    double H = in[2]*in[3] - in[0]*in[5];
    double C = in[3]*in[7] - in[4]*in[6];
    double F = in[1]*in[6] - in[0]*in[7];
    double I = in[0]*in[4] - in[1]*in[3];
    
    double det = in[0]*A + in[1]*B + in[2]*C;
    
    if (det >= -VQF_EPS && det <= VQF_EPS) {
        std::fill(out, out+9, 0);
        return false;
    }
    
    out[0] = A/det;
    out[1] = D/det;
    out[2] = G/det;
    out[3] = B/det;
    out[4] = E/det;
    out[5] = H/det;
    out[6] = C/det;
    out[7] = F/det;
    out[8] = I/det;
    
    return true;
}
#endif

void VQF::setup()
{
    assert(coeffs.gyrTs > 0);
    assert(coeffs.accTs > 0);
    assert(coeffs.magTs > 0);
    
    filterCoeffs(params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA);
    
    coeffs.kMag = gainFromTau(params.tauMag, coeffs.magTs);
    
    coeffs.biasP0 = square(params.biasSigmaInit*100.0);
    coeffs.biasV = square(0.1*100.0)*coeffs.accTs/params.biasForgettingTime;
    
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t pMotion = square(params.biasSigmaMotion*100.0);
    coeffs.biasMotionW = square(pMotion) / coeffs.biasV + pMotion;
    coeffs.biasVerticalW = coeffs.biasMotionW / std::max(params.biasVerticalForgettingFactor, vqf_real_t(1e-10));
#endif
    
    vqf_real_t pRest = square(params.biasSigmaRest*100.0);
    coeffs.biasRestW = square(pRest) / coeffs.biasV + pRest;
    
    filterCoeffs(params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA);
    filterCoeffs(params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA);
    
    coeffs.kMagRef = gainFromTau(params.magRefTau, coeffs.magTs);
    if (params.magCurrentTau > 0) {
        filterCoeffs(params.magCurrentTau, coeffs.magTs, coeffs.magNormDipLpB, coeffs.magNormDipLpA);
    } else {
        std::fill(coeffs.magNormDipLpB, coeffs.magNormDipLpB + 3, NaN);
        std::fill(coeffs.magNormDipLpA, coeffs.magNormDipLpA + 2, NaN);
    }
    
    resetState();
}
