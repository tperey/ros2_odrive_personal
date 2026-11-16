#include "FurataUKF.hpp"
#include <math.h>

/*
 * ======================= UNSCENTED KALMAN FILTER CHEAT SHEET =======================
 *
 *  Model:   x[k+1] = f(x[k], u[k]) + w,   w ~ N(0, Q)       (additive noise form)
 *           z[k]   = h(x[k]) + v,         v ~ N(0, R)
 *
 *  L = NX = 4,  lambda = alpha^2 (L + kappa) - L,  2L+1 = 9 sigma points
 *
 *  WEIGHTS (once, at construction)
 *      Wm_0 = lambda / (L + lambda)
 *      Wc_0 = lambda / (L + lambda) + (1 - alpha^2 + beta)
 *      Wm_i = Wc_i = 1 / (2 (L + lambda)),        i = 1 .. 2L
 *      (check: sum of Wm = 1)
 *
 *  PREDICT (time update)                                   -> this file
 *      S = chol( (L + lambda) P[k-1|k-1] ),  S S^T = (L + lambda) P,  s_i = i-th COLUMN of S
 *      X_0     = x_hat[k-1|k-1]
 *      X_i     = x_hat[k-1|k-1] + s_i          i = 1 .. L               (U1)
 *      X_{i+L} = x_hat[k-1|k-1] - s_i
 *
 *      X_i^-        = f( X_i, u[k-1] )          for all 2L+1 points       (U2)
 *      x_hat[k|k-1] = sum_i Wm_i X_i^-                                     (U3)
 *      P[k|k-1]     = sum_i Wc_i (X_i^- - x_hat)(X_i^- - x_hat)^T + Q     (U4)
 *
 *  UPDATE (measurement update)                             -> inherited from FurataKFBase
 *      With h(x) = H x linear, this is EXACTLY Eq (3)-(7).                 (U5)
 *
 *      (For reference only, the general sigma-point update if h were nonlinear:
 *          Z_i    = h(X_i^-)
 *          z_hat  = sum Wm_i Z_i
 *          S      = sum Wc_i (Z_i - z_hat)(Z_i - z_hat)^T + R
 *          Pxz    = sum Wc_i (X_i^- - x_hat)(Z_i - z_hat)^T
 *          K      = Pxz S^-1
 *          x_hat += K (z - z_hat)
 *          P     -= K S K^T
 *       Plug in h = H x and you get back the standard KF update.)
 * ===================================================================================
 */

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------
FurataUKF::FurataUKF(const FurataParams& params, float dt,
                     const float x0[NX], const float P0[NX][NX],
                     const float Q[NX][NX], const float R[NZ][NZ],
                     float alpha, float beta, float kappa)
    : FurataKFBase(params, dt, x0, P0, Q, R)
{
    // TODO: store alpha, beta, kappa
    // TODO: _computeWeights();
}

void FurataUKF::_computeWeights() {
    // TODO: _lambda, then _Wm[0], _Wc[0], and _Wm[i] = _Wc[i] for i = 1..2NX
    // Tip: sanity-check sum(_Wm) == 1 once, e.g. Serial print during bring-up
}

// ----------------------------------------------------------------------------
// Predict
// ----------------------------------------------------------------------------
void FurataUKF::predict(float tau) {
    // TODO: (U1) _generateSigmaPoints(); if it fails, decide what to do:
    //            e.g. _symmetrize(_P), add a small diagonal jitter, retry,
    //            or fall back to just propagating the mean
    // TODO: (U2) _propagateSigmaPoints(tau);
    // TODO: (U3) _computePredictedMean();
    // TODO: (U4) _computePredictedCovariance();
}

// ----------------------------------------------------------------------------
// Predict helpers
// ----------------------------------------------------------------------------
bool FurataUKF::_generateSigmaPoints() {
    // (U1)
    // TODO: build M = (NX + _lambda) * _P
    // TODO: _cholesky(M, S); return false on failure
    // TODO: _sigma[0] = _x
    //       _sigma[1+i]    = _x + column i of S     (i = 0..NX-1)
    //       _sigma[1+NX+i] = _x - column i of S
    //       COLUMN, not row: S is lower triangular, so it matters!
    // TODO: wrap the dt2 component of each sigma point with _wrapPi()
    return false;
}

void FurataUKF::_propagateSigmaPoints(float tau) {
    // (U2)
    // TODO: for each i: _propagate(_sigma[i], tau, _sigmaProp[i]);   (inherited)
    // Cost: NSIG RK4 steps = 36 _dynamics() calls per predict. Teensy 4 should
    //       handle that at 1 kHz, but time it.
}

void FurataUKF::_computePredictedMean() {
    // (U3) _x = sum_i _Wm[i] * _sigmaProp[i]
    // TODO
    //
    // Think about: the dt2 component is an ANGLE. A plain weighted sum fails if
    // sigma points straddle +/-pi (e.g. +3.1 and -3.1 average to ~0, not pi).
    // Near upright dt2 ~ 0 so it's fine, but a robust option is:
    //   mean = X_0[1] + sum_i Wm_i * wrap(X_i[1] - X_0[1])   (average the offsets)
    // or a circular mean via atan2(sum Wm sin, sum Wm cos).
}

void FurataUKF::_computePredictedCovariance() {
    // (U4) _P = sum_i _Wc[i] * d_i d_i^T + _Q,   d_i = _sigmaProp[i] - _x
    // TODO
    // TODO: wrap d_i[1] (the dt2 difference) with _wrapPi()
    // TODO: _symmetrize(_P)  -- Wc_0 can be negative, so float error matters here
}

// ----------------------------------------------------------------------------
// Math helper
// ----------------------------------------------------------------------------
bool FurataUKF::_cholesky(const float M[NX][NX], float L[NX][NX]) {
    /*
     * Cholesky-Banachiewicz, M = L L^T, L lower triangular:
     *
     *   for i in 0..n-1:
     *     for j in 0..i:
     *       s = sum_{k<j} L[i][k] * L[j][k]
     *       if i == j:  L[i][i] = sqrt( M[i][i] - s )      <- fail if arg <= 0
     *       else:       L[i][j] = ( M[i][j] - s ) / L[j][j]
     *
     *   Zero the upper triangle.
     */
    // TODO
    return false;
}
