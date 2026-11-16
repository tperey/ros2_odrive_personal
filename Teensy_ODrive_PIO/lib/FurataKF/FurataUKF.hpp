#ifndef FURATA_UKF
#define FURATA_UKF

#include "FurataKFBase.hpp"

/*
 * FurataUKF -- Unscented Kalman Filter using the full NONLINEAR Furata model
 * -------------------------------------------------------------------------
 * TEMPLATE: function bodies are TODOs to fill in (see FurataUKF.cpp).
 *
 * Same state / measurement / getters / update as every FurataKFBase filter.
 * The ONLY difference is predict():
 *
 *   EKF: linearize f (Jacobian F), push the covariance through F P F^T.
 *   UKF: pick 2*NX+1 "sigma points" spread around x_hat according to P,
 *        push EACH through the true nonlinear f, then recompute the mean
 *        and covariance from where they land. No Jacobians at all.
 *
 * DISCRETE MODEL:
 *      x[k+1] = f(x[k], u[k]) + w,    w ~ N(0, Q)     f = _propagate() (RK4 of _dynamics())
 *      z[k]   = H x[k] + v,           v ~ N(0, R)     H linear -> update is inherited as-is
 *
 * Why the update can be inherited: with a LINEAR h(x) = H x, the UKF's
 * sigma-point measurement update gives exactly the standard KF update
 * (Eq (3)-(7)). If you ever add a nonlinear measurement, override update().
 * (Formulas for that are in FurataUKF.cpp for reference.)
 *
 * TUNING PARAMS (scaled unscented transform):
 *   alpha : spread of sigma points around the mean. Typical 1e-3 .. 1.
 *   beta  : prior knowledge of the distribution. 2 is optimal for Gaussian.
 *   kappa : secondary scaling. Usually 0 or 3 - NX.
 *   NOTE: textbook alpha = 1e-3 makes lambda ~ -NX and gives a huge negative
 *         Wc0. In float32 on the Teensy that can wreck P (lose positive
 *         definiteness). Starting with alpha ~ 0.5-1 is safer.
 */
class FurataUKF : public FurataKFBase {
public:
    static constexpr int NSIG = 2 * NX + 1;  // # sigma points (9)

    FurataUKF(const FurataParams& params, float dt,
              const float x0[NX], const float P0[NX][NX],
              const float Q[NX][NX], const float R[NZ][NZ],
              float alpha = 1.0f, float beta = 2.0f, float kappa = 0.0f);

    // UKF Eq (U1)-(U5) -- see FurataUKF.cpp
    void predict(float tau) override;

private:
    // ---- Unscented transform params ----
    float _alpha, _beta, _kappa;
    float _lambda;       // = alpha^2 (NX + kappa) - NX
    float _Wm[NSIG];     // weights for the MEAN
    float _Wc[NSIG];     // weights for the COVARIANCE

    // ---- Sigma points (members to keep them off the stack; also handy to inspect) ----
    float _sigma[NSIG][NX];      // X_i     : sigma points around x_hat[k-1|k-1]
    float _sigmaProp[NSIG][NX];  // X_i^-   : after f(X_i, u)

    // ---- Setup (called once from constructor) ----
    void _computeWeights();      // fills _lambda, _Wm, _Wc

    // ---- Predict helpers ----
    bool _generateSigmaPoints();             // (U1) from _x, _P. false if Cholesky fails
    void _propagateSigmaPoints(float tau);   // (U2) _sigmaProp[i] = f(_sigma[i], tau)
    void _computePredictedMean();            // (U3) _x = sum Wm_i X_i^-
    void _computePredictedCovariance();      // (U4) _P = sum Wc_i (X_i^- - x)(X_i^- - x)^T + Q

    // ---- Math helper ----
    // Lower-triangular L with L L^T = M. false if M isn't positive definite.
    static bool _cholesky(const float M[NX][NX], float L[NX][NX]);
};

#endif
