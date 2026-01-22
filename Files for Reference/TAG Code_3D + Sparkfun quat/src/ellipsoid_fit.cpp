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
    if (n < 9) return false;

    // Build normal equations A^T A and A^T b, but with b=1 (vector of ones)
    float ATA[9][9] = {0};
    float ATb[9] = {0};

    for (int i = 0; i < n; ++i) {
        float x = data[i][0];
        float y = data[i][1];
        float z = data[i][2];
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
    if (!solveNormal9(ATA, ATb, params)) return false;

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
    if (fabsf(detQ) < 1e-6f) return false;
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
    for (int r = 0; r < 3; ++r) {
        out.center[r] = -0.5f * (invQ[r][0]*u[0] + invQ[r][1]*u[1] + invQ[r][2]*u[2]);
    }

    // Compute correction matrix via Cholesky of translated form.
    // For embedded simplicity we approximate by scaling along principal axes only.
    // Compute radii squared: r^2 = (x-c)^T Q (x-c) for any sample, choose average.
    float sumR2 = 0;
    for (int i = 0; i < n; ++i) {
        float x = data[i][0] - out.center[0];
        float y = data[i][1] - out.center[1];
        float z = data[i][2] - out.center[2];
        sumR2 += A*x*x + B*y*y + C*z*z + D*2*x*y + E*2*x*z + F*2*y*z;
    }
    float r2 = (sumR2 / n);
    float k = 1.0f / sqrtf(r2);

    // Soft-iron matrix: scale only (off-diagonal ignored here for numerical stability)
    memset(out.transform, 0, sizeof(out.transform));
    out.transform[0][0] = k * sqrtf(A);
    out.transform[1][1] = k * sqrtf(B);
    out.transform[2][2] = k * sqrtf(C);

    // RMS error
    float err = 0;
    for (int i = 0; i < n; ++i) {
        float x = (data[i][0] - out.center[0]) * out.transform[0][0];
        float y = (data[i][1] - out.center[1]) * out.transform[1][1];
        float z = (data[i][2] - out.center[2]) * out.transform[2][2];
        float mag = sqrtf(x*x + y*y + z*z);
        err += (mag - 1.0f)*(mag - 1.0f);
    }
    out.rms = sqrtf(err / n);
    return true;
}
