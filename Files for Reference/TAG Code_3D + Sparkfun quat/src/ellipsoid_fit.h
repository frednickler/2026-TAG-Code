#pragma once
/**
 * Lightweight ellipsoid-fitting helper for magnetometer calibration.
 *
 * The objective is to find the affine transform that maps the raw sample
 * space   x_raw  →  x_corrected  such that the corrected samples lie on a
 * unit sphere:
 *      ||x_corrected|| = 1             (Earth-field magnitude normalised)
 *
 * We model       x_corrected =  T (x_raw – c) ,
 * with 3-vector bias c (hard-iron) and 3×3 matrix T (soft-iron).
 * The ‘full’ algebra would require solving a 9-parameter quadratic form.
 * Here we implement the well-known linear least-squares approach described in
 *  "Least-Squares Ellipsoid Specific Fitting" (Li/Rao, 2004)  but simplified
 *  to return   c   and a diagonal-dominant T  that captures cross-axis terms.
 *
 * The implementation is intentionally compact – it avoids dynamic allocation
 * and external libraries so it can run on the ESP32 without heap pressure.
 * It is NOT as numerically robust as offline desktop solvers, but provides a
 * good embedded compromise.
 *
 * Usage:
 *   const int N = ...;                     // number of samples
 *   float samples[N][3];                   // raw magnetometer samples (µT)
 *   EllipsoidFitResult fit;
 *   bool ok = fitEllipsoidLS(samples, N, fit);
 *   if (ok) { ... use fit.center / fit.transform ... }
 */

struct EllipsoidFitResult {
    float center[3];      // c_x, c_y, c_z (hard-iron)
    float transform[3][3];// 3×3 soft-iron correction matrix (to unit sphere)
    float rms;            // root-mean-square residual in µT (quality metric)
};

/**
 * Perform least-squares ellipsoid fitting.
 *
 * @param data    Array of 3-D samples [N][3]
 * @param n       Number of samples (>= 9 recommended)
 * @param out     Result structure
 * @return true on success, false on numeric failure or insufficient data.
 */
bool fitEllipsoidLS(const float (*data)[3], int n, EllipsoidFitResult &out);
