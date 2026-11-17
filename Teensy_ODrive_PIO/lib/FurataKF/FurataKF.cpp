#include "FurataKF.hpp"
#include <string.h>

// (Kalman filter equations (1)-(7) are listed at the top of FurataKFBase.cpp)

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------
FurataKF::FurataKF(const FurataParams& params, float dt,
                   const float x0[NX], const float P0[NX][NX],
                   const float Q[NX][NX], const float R[NZ][NZ])
    : FurataKFBase(params, dt, x0, P0, Q, R)  // base construction handles params, dt, Q, R, H, x0, P0
{
    // By the time we get here the base is fully constructed (_p, _dt are set).
    // Now, build the linear model in order:
    _buildContinuousModel(); // -> A, B
    _discretize(); // -> F = Ad, G = Bd
}

// ----------------------------------------------------------------------------
// Predict
// ----------------------------------------------------------------------------
void FurataKF::predict(float tau) {
    // Eq (1) and (2)
    _predictState(tau);
    _propagateCovariance(_F);   // (inherited from base)
}

void FurataKF::_predictState(float tau) {
    // Eq (1):  x = F x + G u
    float x_new[NX], Fx[NX];

    _vecmatMul(_F, _x, Fx);
    for (int i = 0; i < NX; i++) x_new[i] = Fx[i] + tau*_G[i];

    memcpy(_x, x_new, sizeof(x_new));
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
    // Compute Jstar, delstar
    float Jstar = _p.J1zz_hat + _p.m2*_p.L_1*_p.L_1 + _p.J2xx;
    float delstar = _p.J2zz_hat*Jstar - _p.m2*_p.m2 * _p.l2*_p.l2 * _p.L_1*_p.L_1;


    // For _A and _Bc, then fill in nonzero entries
    _A[0][2] = 1.0f;
    _A[1][3] = 1.0f;
    _A[2][1] = (_p.g * _p.m2*_p.m2 * _p.l2*_p.l2 * _p.L_1)/delstar;
    _A[2][2] = (-_p.b1 * _p.J2zz_hat)/delstar;
    _A[2][3] = (-_p.b2 * _p.m2 * _p.l2 * _p.L_1)/delstar;
    _A[3][1] = (_p.g * _p.m2 * _p.l2 * Jstar)/delstar;
    _A[3][2] = (-_p.b1 * _p.m2 * _p.l2 * _p.L_1)/delstar;
    _A[3][3] = (-_p.b2 * Jstar)/delstar;

    _Bc[2] = _p.J2zz_hat/delstar;
    _Bc[3] = _p.m2 * _p.l2 * _p.L_1/delstar;
}

void FurataKF::_discretize() {
    /*
     * Need F (= Ad) and G (= Bd) from A, B, dt.
     *
     * Exact (ZOH):   F = expm(A dt),   G = (integral_0^dt expm(A s) ds) B
     *
     * No expm on the Teensy, so truncate the series (dt = 1 ms is small):
     *   2nd order:          F = I + A dt + (A dt)^2 / 2,     G = (I dt + A dt^2 / 2) B
     *
     * NOTE: Euler gives G[0] = G[1] = 0 (torque doesn't hit the angles within
     *       one step). 2nd order captures the 0.5*dt^2 coupling. Compare with
     *       "Ad"/"Bd" in lqr.json to see how close each gets.
     *
     * Alternative: skip this and pass in Ad/Bd computed offline in Python
     *              (like how LQRConfig::K is generated).
     */

    // Discretize _A to get _F
    float Adt[NX][NX], Adt2[NX][NX], Adt2half[NX][NX];
    for (int i = 0; i < NX; i++) {
        for (int j = 0; j < NX; j++) {
            Adt[i][j] = _dt*_A[i][j];
        }
    }
    _matMul(Adt, Adt, Adt2);
    for (int i = 0; i < NX; i++) {
        for (int j = 0; j < NX; j++) {
            Adt2half[i][j] = 0.5f*Adt2[i][j];
        }
    }

    float IAdt[NX][NX];
    float I[NX][NX] = {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0},
    };
    _matAdd(I, Adt, IAdt);
    _matAdd(IAdt, Adt2half, _F);

    // Discretize _Bc and _A to get _G
    float Idt[NX][NX], Adt22[NX][NX];
    for (int i = 0; i < NX; i++) {
        for (int j = 0; j < NX; j++) {
            Idt[i][j] = _dt*I[i][j];
            Adt22[i][j] = 0.5f*_dt*_dt*_A[i][j];
        }
    }
    float G_left[NX][NX];
    _matAdd(Idt, Adt22, G_left);
    _vecmatMul(G_left, _Bc, _G);
}
