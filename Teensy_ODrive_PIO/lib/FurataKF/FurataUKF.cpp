#include "FurataUKF.hpp"
#include <math.h>
#include <string.h>

/*
 * ======================= UNSCENTED KALMAN FILTER CHEAT SHEET =======================
 *
 *  Model:   x[k+1] = f(x[k], u[k]) + w,   w ~ N(0, Q)       (additive noise form)
 *           z[k]   = h(x[k]) + v,         v ~ N(0, R)
 *
 *  L = NX = 4,  lambda = alpha^2 (L + kappa) - L,  2L+1 = 9 sigma points
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
    // Store alpha, beta, kappa
    _alpha = alpha, _beta = beta, _kappa = kappa;
    _computeWeights();
}

void FurataUKF::_computeWeights() {
    /* Computes _lambda, then _Wm[0], _Wc[0], and _Wm[i]
    *  Recall L = NX
    *  WEIGHTS (once, at construction)
    *      Wm_0 = lambda / (L + lambda)
    *      Wc_0 = lambda / (L + lambda) + (1 - alpha^2 + beta)
    *      Wm_i = Wc_i = 1 / (2 (L + lambda)),        i = 1 .. 2L
    *      (check: sum of Wm = 1) = _Wc[i] for i = 1..2NX */

    _lambda = _alpha*_alpha * (NX + _kappa) - NX;
    _Wm[0] = _lambda / (NX + _lambda);
    _Wc[0] = _Wm[0] + (1.0f - _alpha*_alpha + _beta);

    for (int i = 1; i < N_SIGMA; i++) {
        _Wm[i] = 1.0f / (2.0f * (NX + _lambda));
        _Wc[i] = _Wm[i];
    }
    // Tip: sanity-check sum(_Wm) == 1 once, e.g. Serial print during bring-up
}

// ----------------------------------------------------------------------------
// Predict
// ----------------------------------------------------------------------------
void FurataUKF::predict(float tau) {
    // (U1) _generateSigmaPoints();
    if (_generateSigmaPoints()) {
        _propagateSigmaPoints(tau);  // (U2)
        _computePredictedMean();  // (U3)
        _computePredictedCovariance(); // (U4)
    } else {
        // Fall back to just propagating the mean and adding Q to P
        float x_next[NX];
        _propagate(_x, tau, x_next);
        memcpy(_x, x_next, sizeof(x_next));

        float Pnew[NX][NX];
        _matAdd(_P, _Q, Pnew);
        memcpy(_P, Pnew, sizeof(Pnew));
    }
  
}

// ----------------------------------------------------------------------------
// Predict helpers
// ----------------------------------------------------------------------------
bool FurataUKF::_generateSigmaPoints() {
    // (U1)
    // Build M = (NX + _lambda) * _P
    float M[NX][NX];
    for (int i = 0; i < NX; i++) {
        for (int j = 0; j < NX; j++) {
            M[i][j] = (NX + _lambda) * _P[i][j];
        }
    }

    // _cholesky(M, S); returns false on failure
    float S[NX][NX]{};
    if (_cholesky(M, S)) {
        // _sigma[0] = _x
        memcpy(_sigma[0], _x, sizeof(_sigma[0]));
        _sigma[0][1] = _wrapPi(_sigma[0][1]); // Wrap the dt2 component

        // _sigma[1+i]    = _x + column i of S     (i = 0..NX-1)
        // _sigma[1+NX+i] = _x - column i of S
        // COLUMN, not row: S is lower triangular, so it matters!
        for (int i = 0; i < NX; i++) {
            for (int k = 0; k < NX; k++){
                _sigma[1+i][k] = _x[k] + S[k][i];
                _sigma[1+NX+i][k] = _x[k] - S[k][i];
            }
            // Wrap the dt2 component of each sigma point with _wrapPi()
            _sigma[1+i][1] = _wrapPi(_sigma[1+i][1]);
            _sigma[1+i+NX][1] = _wrapPi(_sigma[1+i+NX][1]);
        }
        return true;
    }
    return false;
}

void FurataUKF::_propagateSigmaPoints(float tau) {
    // (U2)
    // For each i: _propagate(_sigma[i], tau, _sigmaProp[i]);   (inherited)
    for (int i = 0; i < N_SIGMA; i++) {
        _propagate(_sigma[i], tau, _sigmaProp[i]);
    }
    // Cost: N_SIGMA RK4 steps = 36 _dynamics() calls per predict. Teensy 4 should
    //       handle that at 1 kHz, but consider timing it.
}

void FurataUKF::_computePredictedMean() {
    // (U3) _x = sum_i _Wm[i] * _sigmaProp[i]

    for (int i = 0; i < NX; i++) {
        float cur = 0.0f;

        // Think about: the dt2 component is an ANGLE. A plain weighted sum fails if
        // sigma points straddle +/-pi (e.g. +3.1 and -3.1 average to ~0, not pi).
        // Near upright dt2 ~ 0 so it's fine, but a robust option is:
        //   mean = X_0[1] + sum_i Wm_i * wrap(X_i[1] - X_0[1])
        // (average the offsets about the center sigma point)

        if (i == 1) {
            float center = _sigmaProp[0][1];
            cur = center;
            for (int j = 1; j < N_SIGMA; j++) {
                cur += _Wm[j] * _wrapPi(_sigmaProp[j][1] - center);
            }
        } else {
            for (int j = 0; j < N_SIGMA; j++) {
                cur += _Wm[j] *_sigmaProp[j][i];
            }
        }
        _x[i] = cur;
        _x[1] = _wrapPi(_x[1]); // Wrap final, since averaged offsets can push past pi
    }
    
}

void FurataUKF::_computePredictedCovariance() {
    // (U4) _P = sum_i _Wc[i] * d_i d_i^T + _Q,   d_i = _sigmaProp[i] - _x
    float sum_matrix[NX][NX]{};
    for (int i = 0; i < N_SIGMA; i++) {

        // Compute di, e.g. delta for this sigma point
        float di[NX];
        for (int j = 0; j < NX; j++) {
            di[j] = _sigmaProp[i][j] - _x[j];
        }

        // wrap d_i[1] (the dt2 difference) with _wrapPi()
        di[1] = _wrapPi(di[1]);

        // Add in Wc[i] * d_i d_i ^T, (NX x NX) matrix for this sigma point
        for (int j = 0; j < NX; j++) {
            for (int k = 0; k < NX; k++) {
                sum_matrix[j][k] += _Wc[i]*di[j]*di[k];
            }
        }
    }

    // Add _Q and update _P
    float Pnew[NX][NX];
    _matAdd(sum_matrix, _Q, Pnew);
    memcpy(_P, Pnew, sizeof(Pnew));
    _symmetrize(_P); // Wc_0 can be negative, so float error matters here
}

// ----------------------------------------------------------------------------
// Math helper
// ----------------------------------------------------------------------------
bool FurataUKF::_cholesky(const float M[NX][NX], float L[NX][NX]) {
    /*
     * Given M, find L
     * Cholesky-Banachiewicz, M = L L^T, L lower triangular:
     *
     *   for i in 0..n-1:
     *     for j in 0..i:
     *       s = sum_{k<=j} L[i][k] * L[j][k]
     *       if i == j:  L[i][i] = sqrt( M[i][i] - s )      <- fail if arg <= 0
     *       else:       L[i][j] = ( M[i][j] - s ) / L[j][j]
     *
     *   Zero the upper triangle.
     */
    // Assumes L starts as zeros

    for (int i = 0; i < NX; i++) {
        for (int j = 0; j <= i; j++) {
            float s = 0.0f;
            for (int k = 0; k < j; k++) s+= L[i][k] * L[j][k];
            if (i==j) {
                float arg = M[i][i] - s;
                if (arg <= 0.0f) { return false; } // Fail safely if sqrt arg <= 0.0
                L[i][i] = sqrtf(arg);
            } else {
                L[i][j] = ( M[i][j] - s ) / L[j][j];
            }
        }
    }
    return true;
}
