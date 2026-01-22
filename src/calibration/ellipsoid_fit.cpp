#include "ellipsoid_fit.h"
#include <algorithm> // for std::swap
#include <Arduino.h>

// --- Internal small helper for 9×9 system ------------------------------
static bool solveNormal9(float M[9][9], float v[9], float out[9]) {
    // Gauss-Jordan elimination with partial pivoting on 9×10 augmented matrix
    for (int col = 0; col < 9; ++col) {
        // Find pivot
        int pivot = col;
        float maxA = fabsf(M[col][col]);
        for (int row = col + 1; row < 9; ++row) {
            float a = fabsf(M[row][col]);
            if (a > maxA) { pivot = row; maxA = a; }
        }
        if (maxA < 1e-6f) {
            return false; // Singular
        }
        // Swap rows in both matrix and vector
        if (pivot != col) {
            for (int k = col; k < 9; ++k) std::swap(M[col][k], M[pivot][k]);
            std::swap(v[col], v[pivot]);
        }
        // Normalize pivot row
        float invPivot = 1.0f / M[col][col];
        for (int k = col; k < 9; ++k) M[col][k] *= invPivot;
        v[col] *= invPivot;
        // Eliminate other rows
        for (int row = 0; row < 9; ++row) {
            if (row == col) continue;
            float factor = M[row][col];
            if (fabsf(factor) < 1e-6f) continue;
            for (int k = col; k < 9; ++k) M[row][k] -= factor * M[col][k];
            v[row] -= factor * v[col];
        }
    }
    // Solution now in v
    for (int i = 0; i < 9; ++i) out[i] = v[i];
    return true;
}

bool fitEllipsoidLS(const float (*data)[3], int n, EllipsoidFitResult &out) {
    if (n < 9) {
        Serial.printf("[ELLIPSOID] FAIL: Only %d samples, need at least 9\n", n);
        return false;
    }
    
    Serial.printf("[ELLIPSOID] Fitting %d samples...\n", n);
    
    // STEP 1: Calculate data scale for normalization
    // This prevents numerical underflow when computing determinants
    float maxVal = 0.0f;
    for (int i = 0; i < n; ++i) {
        float ax = fabsf(data[i][0]);
        float ay = fabsf(data[i][1]);
        float az = fabsf(data[i][2]);
        if (ax > maxVal) maxVal = ax;
        if (ay > maxVal) maxVal = ay;
        if (az > maxVal) maxVal = az;
    }
    
    if (maxVal < 1e-6f) {
        Serial.println("[ELLIPSOID] FAIL: Data magnitude too small");
        return false;
    }
    
    float scale = 1.0f / maxVal;  // Normalize to range [-1, 1]
    Serial.printf("[ELLIPSOID] Data scale: %.2f (max mag: %.2f uT)\n", scale, maxVal);

    // Build normal equations A^T A and A^T b, but with b=1 (vector of ones)
    // Using NORMALIZED data for numerical stability
    float ATA[9][9] = {0};
    float ATb[9] = {0};

    for (int i = 0; i < n; ++i) {
        // Normalize each sample
        float x = data[i][0] * scale;
        float y = data[i][1] * scale;
        float z = data[i][2] * scale;
        float row[9] = { x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z };
        // Fill ATA
        for (int r = 0; r < 9; ++r) {
            ATb[r] += row[r]; // b is 1
            for (int c = r; c < 9; ++c) {
                ATA[r][c] += row[r] * row[c];
            }
        }
    }
    // Symmetrically fill lower triangle
    for (int r = 1; r < 9; ++r) {
        for (int c = 0; c < r; ++c) ATA[r][c] = ATA[c][r];
    }

    float params[9];
    if (!solveNormal9(ATA, ATb, params)) {
        Serial.println("[ELLIPSOID] FAIL: Matrix singular (not enough rotation coverage)");
        return false;
    }
    
    Serial.println("[ELLIPSOID] Matrix solved, computing geometry...");

    // Algebraic to geometric parameters conversion (see literature)
    float A = params[0];
    float B = params[1];
    float C = params[2];
    float D = params[3];
    float E = params[4];
    float F = params[5];
    float G = params[6];
    float H = params[7];
    float I = params[8];
    
    Serial.printf("[ELLIPSOID] Params: A=%.4f B=%.4f C=%.4f\n", A, B, C);

    // Build symmetric matrix form
    float T[4][4] = {
        { A,     D,     E,     G },
        { D,     B,     F,     H },
        { E,     F,     C,     I },
        { G,     H,     I,    -1 }
    };

    // Invert upper-left 3×3 to get center: c = -0.5 * inv(Q) * u  where Q is 3×3, u is G,H,I
    float Q[3][3] = { {A, D, E}, {D, B, F}, {E, F, C} };
    // Determinant
    float detQ = A*(B*C - F*F) - D*(D*C - E*F) + E*(D*F - B*E);
    Serial.printf("[ELLIPSOID] detQ = %.6f\n", detQ);
    if (fabsf(detQ) < 1e-6f) {
        Serial.println("[ELLIPSOID] FAIL: Determinant near zero (degenerate ellipsoid)");
        return false;
    }
    float invQ[3][3];
    // Adjugate / determinant
    invQ[0][0] =  (B*C - F*F)/detQ;
    invQ[0][1] = -(D*C - E*F)/detQ;
    invQ[0][2] =  (D*F - B*E)/detQ;
    invQ[1][0] = invQ[0][1];
    invQ[1][1] =  (A*C - E*E)/detQ;
    invQ[1][2] = -(A*F - D*E)/detQ;
    invQ[2][0] = invQ[0][2];
    invQ[2][1] = invQ[1][2];
    invQ[2][2] =  (A*B - D*D)/detQ;

    float u[3] = { G, H, I };
    // Compute center in NORMALIZED coordinates
    float centerNorm[3];
    for (int r = 0; r < 3; ++r) {
        centerNorm[r] = -0.5f * (invQ[r][0]*u[0] + invQ[r][1]*u[1] + invQ[r][2]*u[2]);
    }
    
    // Convert center back to ORIGINAL coordinates (divide by scale = multiply by maxVal)
    for (int r = 0; r < 3; ++r) {
        out.center[r] = centerNorm[r] / scale;
    }
    
    Serial.printf("[ELLIPSOID] Center (uT): X=%.2f Y=%.2f Z=%.2f\n", 
                  out.center[0], out.center[1], out.center[2]);

    // Compute correction matrix via Cholesky of translated form.
    // For embedded simplicity we approximate by scaling along principal axes only.
    // Compute radii squared: r^2 = (x-c)^T Q (x-c) for any sample, choose average.
    // ---------------------------------------------------------
    // UNIT CORRECTION EXTENSION
    // ---------------------------------------------------------
    // The standard algorithm normalizes the output to a unit sphere (R=1).
    // However, our system expects units of micro-Tesla (µT).
    // We calculate the average "radius" (field strength) of the RAW samples
    // centered on the calculated hard-iron offset, and scale the transform
    // to target that magnitude instead of 1.0.
    
    // Calculate RMS field strength of raw data (centered) - in ORIGINAL units
    float sumSqDist = 0;
    for (int i = 0; i < n; ++i) {
        float x = data[i][0] - out.center[0];
        float y = data[i][1] - out.center[1];
        float z = data[i][2] - out.center[2];
        sumSqDist += x*x + y*y + z*z;
    }
    float avgFieldStrength = sqrtf(sumSqDist / n);
    Serial.printf("[ELLIPSOID] Avg field strength: %.2f uT\n", avgFieldStrength);
    
    // The A, B, C parameters are in normalized space
    // For soft-iron correction, we need to scale them appropriately
    // Since data was scaled by 'scale', quadratic terms were scaled by scale^2
    // So A, B, C represent the normalized ellipsoid
    
    // Soft-iron matrix: FULL matrix correction
    // The quadric form is x^T Q x = const. We want to find Transform M such that (Mx)^T (Mx) = const.
    // This implies M^T M = Q. So M = sqrt(Q).
    // Since Q is symmetric positive definite (for an ellipsoid), we can use Eigen decomposition:
    // Q = V D V^T -> sqrt(Q) = V sqrt(D) V^T.
    
    // We need to invert this to map Ellipsoid -> Sphere:
    // Correction = inv(sqrt(Q)) = V inv(sqrt(D)) V^T.
    
    // 1. Construct Q matrix from params (normalized)
    float Q33[3][3] = { {A, D, E}, {D, B, F}, {E, F, C} };
    
    // 2. Compute Eigenvalues and Eigenvectors of Q (Jacobi method for 3x3)
    // Initialize V as identity
    float V[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} };
    float d[3] = { Q33[0][0], Q33[1][1], Q33[2][2] }; // Eigenvalues will be stored here
    float bw[3], zw[3]; // workspace
    
    for (int i=0; i<3; i++) {
        V[i][i] = 1.0f;
        d[i] = Q33[i][i];
        bw[i] = d[i];
        zw[i] = 0.0f;
    }
    
    // Jacobi Iterations (usually converges in 3-4 sweeps)
    for (int i=0; i<50; i++) {
        float sm = fabsf(Q33[0][1]) + fabsf(Q33[0][2]) + fabsf(Q33[1][2]);
        if (sm < 1e-9f) break; // Converged
        
        float tresh = (i < 3) ? 0.2f * sm / 9.0f : 0.0f;
        
        for (int ip=0; ip<2; ip++) {
            for (int iq=ip+1; iq<3; iq++) {
                float g = 100.0f * fabsf(Q33[ip][iq]);
                if (i > 3 && (fabsf(d[ip]) + g == fabsf(d[ip])) 
                          && (fabsf(d[iq]) + g == fabsf(d[iq]))) {
                    Q33[ip][iq] = 0.0f;
                } else if (fabsf(Q33[ip][iq]) > tresh) {
                    float h = d[iq] - d[ip];
                    float t;
                    if (fabsf(h) + g == fabsf(h)) {
                        t = (Q33[ip][iq]) / h;
                    } else {
                        float theta = 0.5f * h / (Q33[ip][iq]);
                        t = 1.0f / (fabsf(theta) + sqrtf(1.0f + theta*theta));
                        if (theta < 0.0f) t = -t;
                    }
                    float c = 1.0f / sqrtf(1.0f + t*t);
                    float s = t * c;
                    float tau = s / (1.0f + c);
                    float h_val = t * Q33[ip][iq];
                    zw[ip] -= h_val;
                    zw[iq] += h_val;
                    d[ip] -= h_val;
                    d[iq] += h_val;
                    Q33[ip][iq] = 0.0f;
                    
                    for (int j=0; j<ip; j++) {
                        g = Q33[j][ip]; float h = Q33[j][iq];
                        Q33[j][ip] = g - s*(h + g*tau);
                        Q33[j][iq] = h + s*(g - h*tau);
                    }
                    for (int j=ip+1; j<iq; j++) {
                        g = Q33[ip][j]; float h = Q33[j][iq];
                        Q33[ip][j] = g - s*(h + g*tau);
                        Q33[j][iq] = h + s*(g - h*tau);
                    }
                    for (int j=iq+1; j<3; j++) {
                        g = Q33[ip][j]; float h = Q33[iq][j];
                        Q33[ip][j] = g - s*(h + g*tau);
                        Q33[iq][j] = h + s*(g - h*tau);
                    }
                    for (int j=0; j<3; j++) {
                        g = V[j][ip]; float h = V[j][iq];
                        V[j][ip] = g - s*(h + g*tau);
                        V[j][iq] = h + s*(g - h*tau);
                    }
                }
            }
        }
        for (int ip=0; ip<3; ip++) {
            bw[ip] += zw[ip];
            d[ip] = bw[ip];
            zw[ip] = 0.0f;
        }
    }
    
    // 3. Compute Scaling Factors from Eigenvalues
    // Q = V D V^T.  Quadric: x^T Q x = 1.
    // Principal radii: r_i = 1 / sqrt(lambda_i).
    // We want to scale r_i to 1. So S_i * r_i = 1.
    // S_i = sqrt(lambda_i).
    // To preserve average scale: S_i = sqrt(lambda_i / avgLambda).
    
    float avgLambda = (fabsf(d[0]) + fabsf(d[1]) + fabsf(d[2])) / 3.0f;
    float scales[3];
    for (int k=0; k<3; k++) {
        // Correct formula: sqrt(lambda_i / avg)
        scales[k] = sqrtf(fabsf(d[k]) / avgLambda);
    }
    
    // 4. Reconstruct Soft Iron Matrix M = V * S * V^T
    memset(out.transform, 0, sizeof(out.transform));
    for (int r=0; r<3; r++) {
        for (int c=0; c<3; c++) {
            for (int k=0; k<3; k++) {
                out.transform[r][c] += V[r][k] * scales[k] * V[c][k];
            }
        }
    }
    
    Serial.printf("[ELLIPSOID] Soft iron: [%.4f, %.4f, %.4f]\n", 
                  out.transform[0][0], out.transform[1][1], out.transform[2][2]);

    // RMS error (in µT)
    float err = 0;
    for (int i = 0; i < n; ++i) {
        // Apply calibration: subtract center, then apply soft iron
        float x = (data[i][0] - out.center[0]) * out.transform[0][0];
        float y = (data[i][1] - out.center[1]) * out.transform[1][1];
        float z = (data[i][2] - out.center[2]) * out.transform[2][2];
        float mag = sqrtf(x*x + y*y + z*z);
        // Error relative to the target field strength
        err += (mag - avgFieldStrength)*(mag - avgFieldStrength);
    }
    out.rms = sqrtf(err / n);
    Serial.printf("[ELLIPSOID] RMS error: %.2f uT\n", out.rms);
    
    return true;
}
