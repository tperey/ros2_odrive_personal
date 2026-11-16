#ifndef FURATA_EKF
#define FURATA_EKF

#include "FurataKFBase.hpp"

/*
 * FurataEKF -- Extended Kalman Filter using the full NONLINEAR Furata model
 * ------------------------------------------------------------------------
 * TEMPLATE: function bodies are TODOs to fill in (see FurataEKF.cpp).
 *
 * Same state / measurement / getters / update as every FurataKFBase filter.
 * The ONLY difference from the linear FurataKF is predict():
 *
 *   Linear KF:  x = F x + G u,      F constant (Ad about upright)
 *   EKF:        x = f(x, u),        F = df/dx re-evaluated EVERY step at x_hat
 *
 * Since the model is re-linearized about the current estimate (not about
 * upright), the EKF stays reasonable far from upright -- e.g. during swing-up.
 *
 * DISCRETE MODEL:
 *      x[k+1] = f(x[k], u[k]) + w,    w ~ N(0, Q)     f = _propagate() (RK4 of _dynamics())
 *      z[k]   = H x[k] + v,           v ~ N(0, R)     H linear -> update is inherited as-is
 *
 * Relation to state space: the linear A, B in FurataKF are exactly
 *      A = df_c/dx,  B = df_c/du   evaluated at x = 0 (upright), tau = 0.
 * The EKF's F is the discrete version of df_c/dx evaluated at x_hat instead.
 */
class FurataEKF : public FurataKFBase {
public:
    // How to get the Jacobian F each step.
    enum class JacobianMethod {
        Numerical,   // finite differences of _propagate(). Easy, slower. Start here.
        Analytical   // hand-derived df_c/dx, then discretize. Faster, more work.
    };

    FurataEKF(const FurataParams& params, float dt,
              const float x0[NX], const float P0[NX][NX],
              const float Q[NX][NX], const float R[NZ][NZ],
              JacobianMethod method = JacobianMethod::Numerical,
              float fdEps = 1e-4f);   // finite-difference step (Numerical only)

    // EKF Eq (1'), (1''), (2) -- see FurataEKF.cpp
    void predict(float tau) override;

    // Debug: the most recent F. Compare against FurataKF's Ad near upright --
    // they should match closely at x = 0, tau = 0.
    void getLastJacobian(float out[NX][NX]) const;

private:
    JacobianMethod _method;
    float _fdEps;
    float _F[NX][NX];   // Jacobian from the most recent predict

    // Fill _F = df/dx at (x, tau)  -- dispatches on _method
    void _computeJacobian(const float x[NX], float tau);

    // Option A: perturb each state, difference the propagated results
    void _computeJacobianNumerical(const float x[NX], float tau, float F[NX][NX]) const;

    // Option B: continuous Jacobian A(x,u) = df_c/dx analytically ...
    void _computeContinuousJacobian(const float x[NX], float tau, float A[NX][NX]) const;
    // ... then discretize it (same idea as FurataKF::_discretize)
    void _discretizeJacobian(const float A[NX][NX], float F[NX][NX]) const;
};

#endif
