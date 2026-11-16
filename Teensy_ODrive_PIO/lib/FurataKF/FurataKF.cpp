#include "FurataKF.hpp"

// (Kalman filter equations (1)-(7) are listed at the top of FurataKFBase.cpp)

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------
FurataKF::FurataKF(const FurataParams& params, float dt,
                   const float x0[NX], const float P0[NX][NX],
                   const float Q[NX][NX], const float R[NZ][NZ])
    : FurataKFBase(params, dt, x0, P0, Q, R)  // base handles params, dt, Q, R, H, x0, P0
{
    // By the time we get here the base is fully constructed (_p, _dt are set).
    // TODO: build the linear model in order:
    //         _buildContinuousModel();  -> A, B
    //         _discretize();            -> F, G   (needs A, B, dt)
}

// ----------------------------------------------------------------------------
// Predict
// ----------------------------------------------------------------------------
void FurataKF::predict(float tau) {
    // Eq (1) and (2)
    // TODO: _predictState(tau);
    // TODO: _propagateCovariance(_F);   (inherited from base)
}

void FurataKF::_predictState(float tau) {
    // Eq (1):  x = F x + G u
    // TODO: compute into a temp array (don't overwrite _x while still reading it!), then copy back
}

// ----------------------------------------------------------------------------
// Model building
// ----------------------------------------------------------------------------
void FurataKF::_buildContinuousModel() {
    /*
     * Linearized about upright (t2 = pi), state [t1, dt2, t1d, t2d].
     * Mirrors FurataAnalyzer._obtain_linear_statespace_matrices():
     *
     *   Jstar   = J1zz_hat + m2*L_1^2 + J2xx
     *   delstar = J2zz_hat*Jstar - m2^2 * l2^2 * L_1^2
     *
     *   A = [ 0   0    1    0  ]       B = [  0  ]
     *       [ 0   0    0    1  ]           [  0  ]
     *       [ 0  a32  a33  a34 ]           [ b31 ]
     *       [ 0  a42  a43  a44 ]           [ b41 ]
     *
     *   a32 =  g*m2^2*l2^2*L_1 / delstar     a33 = -b1*J2zz_hat   / delstar
     *   a34 = -b2*m2*l2*L_1    / delstar     a42 =  g*m2*l2*Jstar / delstar
     *   a43 = -b1*m2*l2*L_1    / delstar     a44 = -b2*Jstar      / delstar
     *   b31 =  J2zz_hat        / delstar     b41 =  m2*l2*L_1     / delstar
     *
     * Sanity check: compare against "A" and "B" in config/m8325s_furata/lqr.json.
     * (Also: this A is exactly the Jacobian df_c/dx of _dynamics() evaluated at
     *  x = 0, tau = 0 -- a nice cross-check once the EKF exists.)
     */
    // TODO: compute Jstar, delstar
    // TODO: zero _A and _B, then fill in nonzero entries
}

void FurataKF::_discretize() {
    /*
     * Need F (= Ad) and G (= Bd) from A, B, dt.
     *
     * Exact (ZOH):   F = expm(A dt),   G = (integral_0^dt expm(A s) ds) B
     *
     * No expm on the Teensy, so truncate the series (dt = 1 ms is small):
     *   1st order (Euler):  F = I + A dt,                    G = B dt
     *   2nd order:          F = I + A dt + (A dt)^2 / 2,     G = (I dt + A dt^2 / 2) B
     *
     * NOTE: Euler gives G[0] = G[1] = 0 (torque doesn't hit the angles within
     *       one step). 2nd order captures the 0.5*dt^2 coupling. Compare with
     *       "Ad"/"Bd" in lqr.json to see how close each gets.
     *
     * Alternative: skip this and pass in Ad/Bd computed offline in Python
     *              (like how LQRConfig::K is generated).
     */
    // TODO: pick an order and compute _F, _G
}
