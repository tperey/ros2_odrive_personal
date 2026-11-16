#include "FurataEKF.hpp"

/*
 * ======================= EXTENDED KALMAN FILTER CHEAT SHEET =======================
 *
 *  Model:   x[k+1] = f(x[k], u[k]) + w,   w ~ N(0, Q)
 *           z[k]   = h(x[k]) + v,         v ~ N(0, R)
 *
 *  PREDICT (time update)                          -> this file
 *      F[k]         = df/dx |_( x_hat[k-1|k-1], u[k-1] )          (1')
 *                       ^ evaluate at the OLD estimate, BEFORE propagating it
 *      x_hat[k|k-1] = f( x_hat[k-1|k-1], u[k-1] )                 (1'')
 *                       ^ the full nonlinear model, NOT F x
 *      P[k|k-1]     = F[k] P[k-1|k-1] F[k]^T + Q                  (2)
 *
 *  UPDATE (measurement update)                    -> inherited from FurataKFBase
 *      Identical to the linear KF with H[k] = dh/dx. Our h(x) = H x is linear,
 *      so H[k] = H (constant) and nothing changes. Eq (3)-(7) in FurataKFBase.cpp.
 *
 *  Key idea: the STATE goes through the true nonlinear f; only the COVARIANCE
 *  uses the linearization F.
 * ==================================================================================
 */

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------
FurataEKF::FurataEKF(const FurataParams& params, float dt,
                     const float x0[NX], const float P0[NX][NX],
                     const float Q[NX][NX], const float R[NZ][NZ],
                     JacobianMethod method, float fdEps)
    : FurataKFBase(params, dt, x0, P0, Q, R)
{
    // TODO: store method and fdEps
    // TODO: initialize _F to something sane (identity) so getLastJacobian()
    //       isn't garbage before the first predict
}

// ----------------------------------------------------------------------------
// Predict
// ----------------------------------------------------------------------------
void FurataEKF::predict(float tau) {
    // TODO: Eq (1')  -> _computeJacobian(_x, tau);     ORDER MATTERS: do this first
    // TODO: Eq (1'') -> _propagate(_x, tau, x_next) (inherited), then copy x_next into _x
    //                   (_propagate takes separate in/out arrays, so no aliasing issue)
    // TODO: Eq (2)   -> _propagateCovariance(_F);   (inherited)
}

// ----------------------------------------------------------------------------
// Jacobian
// ----------------------------------------------------------------------------
void FurataEKF::_computeJacobian(const float x[NX], float tau) {
    // TODO: switch on _method:
    //   Numerical  -> _computeJacobianNumerical(x, tau, _F)
    //   Analytical -> _computeContinuousJacobian(x, tau, A); _discretizeJacobian(A, _F)
}

void FurataEKF::_computeJacobianNumerical(const float x[NX], float tau, float F[NX][NX]) const {
    /*
     * Column j of F = how x_next changes when you nudge state j:
     *
     *   forward difference:  F[:, j] = ( f(x + eps e_j) - f(x) ) / eps            (NX+1 propagations)
     *   central difference:  F[:, j] = ( f(x + eps e_j) - f(x - eps e_j) ) / 2eps (2*NX propagations, more accurate)
     *
     *   where f = _propagate() and e_j is the j-th unit vector.
     *
     * Notes:
     *  - This gives the DISCRETE F directly (no separate discretization step).
     *  - float32 on the Teensy: eps too small -> round-off noise, too big -> truncation
     *    error. ~1e-4 to 1e-3 rad is a reasonable start. Sanity check: at x = 0 this
     *    should be close to "Ad" in lqr.json.
     *  - Watch the dt2 row: if a perturbation pushes dt2 across +/-pi, _propagate's
     *    re-wrap will make the difference blow up. Wrap the dt2 difference with _wrapPi().
     */
    // TODO
}

void FurataEKF::_computeContinuousJacobian(const float x[NX], float tau, float A[NX][NX]) const {
    /*
     * A(x,u) = df_c/dx, where f_c = _dynamics():
     *
     *   A = [ 0              0              1              0             ]
     *       [ 0              0              0              1             ]
     *       [ d t1dd/dt1     d t1dd/ddt2    d t1dd/dt1d    d t1dd/dt2d   ]
     *       [ d t2dd/dt1     d t2dd/ddt2    d t2dd/dt1d    d t2dd/dt2d   ]
     *
     * The accelerations come from  Gamma(t2) * qdd = Phi(x, tau),  qdd = [t1dd; t2dd].
     * Differentiating both sides w.r.t. a state s gives a compact formula:
     *
     *     d qdd / ds = Gamma^-1 ( dPhi/ds - (dGamma/ds) qdd )
     *
     *   - Gamma depends only on t2  ->  dGamma/ds = 0 for s in {t1, t1d, t2d}
     *   - Nothing depends on t1     ->  the whole t1 column is 0
     *   - Remember t2 = dt2 + pi, and d/d(dt2) = d/d(t2)
     *
     * Partials you'll need (with D = J2yy_hat - J2xx):
     *   dgamma_1/dt2 = D sin(2 t2)        dgamma_2/dt2 = -m2 L_1 l2 sin(t2)
     *   dPhi_1/dt2   = m2 L_1 l2 cos(t2) t2d^2 - 2 D t1d t2d cos(2 t2)
     *   dPhi_1/dt1d  = -D t2d sin(2 t2) - b1
     *   dPhi_1/dt2d  = 2 m2 L_1 l2 sin(t2) t2d - D t1d sin(2 t2)
     *   dPhi_2/dt2   = D t1d^2 cos(2 t2) - g m2 l2 cos(t2)
     *   dPhi_2/dt1d  = D t1d sin(2 t2)
     *   dPhi_2/dt2d  = -b2
     *   (double-check these against _dynamics() yourself!)
     *
     * Sanity check: at x = 0 (upright), tau = 0 this must equal FurataKF's _A.
     */
    // TODO
}

void FurataEKF::_discretizeJacobian(const float A[NX][NX], float F[NX][NX]) const {
    /*
     * Same truncated series as FurataKF::_discretize():
     *   1st order:  F = I + A dt
     *   2nd order:  F = I + A dt + (A dt)^2 / 2
     */
    // TODO
}

// ----------------------------------------------------------------------------
// Debug
// ----------------------------------------------------------------------------
void FurataEKF::getLastJacobian(float out[NX][NX]) const {
    // TODO: copy _F into out
}
