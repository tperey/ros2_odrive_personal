#include "FurataKFBase.hpp"
#include <math.h>
#include <string.h>
#include <stdint.h>

/*
 * ===================== DISCRETE KALMAN FILTER CHEAT SHEET =====================
 *
 *  Model:   x[k+1] = F x[k] + G u[k] + w,   w ~ N(0, Q)
 *           z[k]   = H x[k] + v,            v ~ N(0, R)
 *
 *  Notation: x_hat[a|b] = estimate of x at time a using measurements up to b.
 *
 *  PREDICT (time update)                          -> derived classes
 *      x_hat[k|k-1] = F x_hat[k-1|k-1] + G u[k-1]                 (1)
 *      P[k|k-1]     = F P[k-1|k-1] F^T + Q                         (2)
 *
 *  UPDATE (measurement update)                    -> this file
 *      y[k]   = z[k] - H x_hat[k|k-1]          innovation          (3)
 *      S[k]   = H P[k|k-1] H^T + R             innovation cov      (4)
 *      K[k]   = P[k|k-1] H^T S[k]^-1           Kalman gain         (5)
 *      x_hat[k|k] = x_hat[k|k-1] + K[k] y[k]                       (6)
 *      P[k|k]     = (I - K[k] H) P[k|k-1]                          (7)
 *        (or Joseph form, more numerically robust:
 *         P = (I - K H) P (I - K H)^T + K R K^T)                   (7')
 *
 *  Relation to state space:  F = Ad,  G = Bd,  H = C.
 *  Dimensions here: x 4x1, u 1x1, z 2x1, F 4x4, G 4x1, H 2x4,
 *                   P,Q 4x4, R,S 2x2, K 4x2, y 2x1.
 * ==============================================================================
 */

// ----------------------------------------------------------------------------
// Constructor (only the stuff common to every filter type)
// ----------------------------------------------------------------------------
FurataKFBase::FurataKFBase(const FurataParams& params, float dt,
                           const float x0[NX], const float P0[NX][NX],
                           const float Q[NX][NX], const float R[NZ][NZ]) {
    // Copy into members
    _p = params;
    _dt = dt;
    memcpy(_Q, Q, sizeof(_Q));
    memcpy(_R, R, sizeof(_R));

    _buildMeasurementModel();

    // Initialize estimate
    reset(x0, P0);
}

// ----------------------------------------------------------------------------
// Main interface
// ----------------------------------------------------------------------------
void FurataKFBase::update(float t1_meas, float t2_meas) {
    // Build z[NZ] in FILTER coordinates
    float z[NZ];
    z[0] = t1_meas;
    z[1] = t2_meas - kPi;

    // Eq (3)-(7) via the helpers:
    //     y = innovation, S = innovation cov, K = gain (bail if it fails),
    //     then update state and covariance
    float y[NZ];
    _computeInnovation(z, y); // (3)
    float S[NZ][NZ];
    _computeInnovationCov(S); // (4)
    float K[NX][NZ];
    if (_computeGain(S, K)) { // (5)
        // Stop if S inversion fails.
        // State and covariance will remain at prior value
        _updateState(K, y); // (6)
        _updateCovariance(K); // (7)
    }
}

void FurataKFBase::step(float tau, float t1_meas, float t2_meas) {
    predict(tau);
    update(t1_meas, t2_meas);
}

void FurataKFBase::reset(const float x0[NX], const float P0[NX][NX]) {
    memcpy(_x, x0, sizeof(_x));
    memcpy(_Pcov, P0, sizeof(_Pcov));
}

// ----------------------------------------------------------------------------
// Shared predict helper
// ----------------------------------------------------------------------------
void FurataKFBase::_propagateCovariance(const float F[NX][NX]) {
    float Floc[NX][NX]; // Copy F pointer to local object for passing to functions
    memcpy(Floc, F, sizeof(Floc));
    
    // Eq (2):  P = F P F^T + Q
    // Symemtrize to prevent float drift
    // Left term
    float Ft[NX][NX];
    _transpose(Floc, Ft);
    float PFt[NX][NX];
    _matMul(_Pcov, Ft, PFt);
    float FPFt[NX][NX];
    _matMul(Floc, PFt, FPFt);

    // Generate new result, and store
    float Pnew[NX][NX];
    _matAdd(FPFt, _Q, Pnew);
    memcpy(_Pcov, Pnew, sizeof(_Pcov));
    _symmetrize(_Pcov);
}

// ----------------------------------------------------------------------------
// Nonlinear model (for EKF / UKF; the linear KF never calls these)
// ----------------------------------------------------------------------------
void FurataKFBase::_dynamics(const float x[NX], float tau, float xdot[NX]) const {
    /*
     *   Get state deriv
     *   Upper half is simply the state velocities. Lower found with...
     *
     *   Gamma * [t1dd; t2dd] = Phi
     *
     *   Gamma = [ gamma_1   gamma_2  ]
     *           [ gamma_2   J2zz_hat ]
     *     gamma_1 = J1zz_hat + m2 L_1^2 + J2yy_hat sin^2(t2) + J2xx cos^2(t2)
     *     gamma_2 = m2 L_1 l2 cos(t2)
     *
     *   Phi = [ tau + m2 L_1 l2 sin(t2) t2d^2 - t1d t2d sin(2 t2)(J2yy_hat - J2xx) - b1 t1d   ]
     *         [ 0.5 t1d^2 sin(2 t2)(J2yy_hat - J2xx) - b2 t2d - g m2 l2 sin(t2)               ]
     *
     *   xdot = [ t1d, t2d, t1dd, t2dd ]
     *
     * Solve the 2x2 with _invert2x2 (or Cramer's rule directly).
     */

    // Initialization
    // NOTE: Must convert form LQR upright linearization.
    // However, t2 = dt2 + pi  =>  sin(t2) = -sin(dt2),  cos(t2) = -cos(dt2)
    xdot[0] = x[2];
    xdot[1] = x[3];
    float st2 = -sinf(x[1]), s2t2 = st2*st2;
    float ct2 = -cosf(x[1]), c2t2 = ct2*ct2, s_2t2 = 2.0f*st2*ct2;
    float Ra = _p.m2*_p.L_1*_p.l2, J2tilde = _p.J2yy_hat-_p.J2xx, Ga = _p.g*_p.m2*_p.l2;
    float J1tilde = _p.J1zz_hat + _p.m2*(_p.L_1*_p.L_1);

    // Build up matrices
    float Gamma[2][2]; // Mass matrix for 2 dof arm (NZ = 2);
    Gamma[0][0] = J1tilde + _p.J2yy_hat*s2t2 + _p.J2xx*c2t2;
    Gamma[0][1] = Ra*ct2;
    Gamma[1][0] = Ra*ct2;
    Gamma[1][1] = _p.J2zz_hat;
    float GammaInv[2][2];
    _invert2x2(Gamma, GammaInv); // Mass matrix is always positive definite and thus invertible

    float phi[2];
    phi[0] = tau + Ra*st2*x[3]*x[3] - J2tilde*x[2]*x[3]*s_2t2 - _p.b1*x[2];
    phi[1] = 0.5f*J2tilde*x[2]*x[2]*s_2t2 - Ga*st2 - _p.b2*x[3] - _p.fs2*tanhf(_p.fs2_k*x[3]);

    // Solve
    float xlower[2]; 
    _vecmatMul(GammaInv, phi, xlower);
    xdot[2] = xlower[0];
    xdot[3] = xlower[1];
}

void FurataKFBase::_propagate(const float x[NX], float tau, float x_next[NX]) const {
    /*
     * One step of RK4 with tau held constant (ZOH) over _dt:
     *   k1 = f_c(x)
     *   k2 = f_c(x + dt/2 k1)
     *   k3 = f_c(x + dt/2 k2)
     *   k4 = f_c(x + dt   k3)
     *   x_next = x + dt/6 (k1 + 2 k2 + 2 k3 + k4)
     * (Euler, x_next = x + dt f_c(x), is fine to start with at 1 ms.)
     */
    // Constants
    float xcur[NX];  // For _dynamics call arguments

    float k1[NX]; // k1
    _dynamics(x, tau, k1);

    float k2[NX]; // k2
    for (int i = 0; i < NX; i++) xcur[i] = x[i] + 0.5f*_dt*k1[i];
    _dynamics(xcur, tau, k2);

    float k3[NX]; // k3
    for (int i = 0; i < NX; i++) xcur[i] = x[i] + 0.5f*_dt*k2[i];
    _dynamics(xcur, tau, k3);

    float k4[NX]; // k4
    for (int i = 0; i < NX; i++) xcur[i] = x[i] + _dt*k3[i];
    _dynamics(xcur, tau, k4);

    // Next state estimate
    for (int i = 0; i < NX; i++) x_next[i] = x[i] + (_dt/6.0f)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    x_next[1] = _wrapPi(x_next[1]); // re-wrap t2
}

// ----------------------------------------------------------------------------
// Update helpers
// ----------------------------------------------------------------------------
void FurataKFBase::_buildMeasurementModel() {
    /*
     * z = H x picks out the two angles:
     *   H = C = [ 1 0 0 0 ]
     *           [ 0 1 0 0 ]
     */
    _H[0][0] = 1.0f;
    _H[1][1] = 1.0f;
}

void FurataKFBase::_computeInnovation(const float z[NZ], float y[NZ]) {
    // Eq (3):  y = z - H x
    // NOTE: should wrap pendulum angle error values near peak 
    y[0] = (z[0] - _x[0]);
    y[1] = _wrapPi(z[1] - _x[1]);
}

void FurataKFBase::_computeInnovationCov(float S[NZ][NZ]) {
    // Eq (4):  S = H P H^T + R
    // Note: with H = [I2 0], H P H^T is just the top-left 2x2 block of P.
    for (uint8_t i = 0; i < NZ; i++) {
        for (uint8_t j = 0; j < NZ; j++) {
            S[i][j] = _Pcov[i][j] + _R[i][j];
        }
    }    
}

bool FurataKFBase::_computeGain(const float S[NZ][NZ], float K[NX][NZ]) {
    // Eq (5):  K = P H^T S^-1
    // Invert S with _invert2x2; return false if it fails
    float Sinv[NZ][NZ];
    if (_invert2x2(S, Sinv)) {
        // Compute P H^T (4x2)
        // NOTE: with H = [I2 0], P H^T is just the first NZ columns of P.
        float PHt[NX][NZ];
        for (int i = 0; i < NX; i++) {
            for (int j = 0; j < NZ; j++) {
                PHt[i][j] = _Pcov[i][j];
            }
        }

        // Then multiply by S^-1
        float Kloc[NX][NZ];
        _matMul(PHt, Sinv, Kloc);
        memcpy(K, Kloc, sizeof(Kloc));

        return true;
    }
    return false;
}

void FurataKFBase::_updateState(const float K[NX][NZ], const float y[NZ]) {
    // Eq (6):  x = x + K y
    for (int i = 0; i < NX; i++) {
        for (int j = 0; j < NZ; j++) {
            _x[i] += K[i][j]*y[j];
        }
    }
}

void FurataKFBase::_updateCovariance(const float K[NX][NZ]) {

    float Kloc[NX][NZ]; // Copy K pointer to local object for passing to _matMul
    memcpy(Kloc, K, sizeof(Kloc));
    // Eq (7) Joseph form: P = (I - K H) P (I - K H)^T + K R K^T
    // More numerically stable, esp with _symmetrize 

    // K R K^T
    float Kt[NZ][NX]; // K^T
    _transpose(Kloc, Kt);
    float RKt[NZ][NX]; // R K^T
    _matMul(_R, Kt, RKt);
    float KRKt[NX][NX]; // K R K ^T
    _matMul(Kloc, RKt, KRKt);

    // I - K H
    // Note: With H = [I2 0], K H is just K padded with two zero cols
    float ImKH[NX][NX];
    for (int i = 0; i < NX; i++) {
        for (int j = 0; j < NX; j++) {
            if (j < NZ) {
                ImKH[i][j] = -K[i][j]; // Flip sign
            } else {
                ImKH[i][j] = 0.0f;
            }
            if (i==j) {
                ImKH[i][j] += 1.0f; // Ad one on diag
            }
        }
    }

    float ImKHt[NX][NX]; // (I - K H) ^ T
    _transpose(ImKH, ImKHt);

    // Left term
    float Pit[NX][NX];
    _matMul(_Pcov, ImKHt, Pit);
    float iPit[NX][NX];
    _matMul(ImKH, Pit, iPit);

    // Generate new result, copy over, and clean
    float Pnew[NX][NX];
    _matAdd(iPit, KRKt, Pnew);
    memcpy(_Pcov, Pnew, sizeof(_Pcov));
    _symmetrize(_Pcov);
}

// ----------------------------------------------------------------------------
// Math helpers
// ----------------------------------------------------------------------------
float FurataKFBase::_wrapPi(float a) {
    // Wrap into (-pi, pi]. Uses atan2 of sin/cos, which maps angle into this range
    return atan2f(sinf(a), cosf(a));
}

bool FurataKFBase::_invert2x2(const float M[NZ][NZ], float Minv[NZ][NZ]) {
    // [a b; c d]^-1 = 1/(ad - bc) * [d -b; -c a]
    // Return false if |det| is ~0
    float det = M[0][0]*M[1][1] - M[0][1]*M[1][0];
    if (det <= 0.0f) {
        return false;
    }

    Minv[0][0] = M[1][1]/det;
    Minv[0][1] = -M[0][1]/det;
    Minv[1][0] = -M[1][0]/det;
    Minv[1][1] = M[0][0]/det;
    return true;
}

void FurataKFBase::_symmetrize(float M[NX][NX]) {
    // Maintain symmetric matrix entries following rounding
    // by using average off diagonal
    // i.e.: M = 0.5 (M + M^T)
    for (int i = 0; i < NX; i++) {
        for (int j = i+ 1; j < NX; j++) {
            float avg = 0.5f*(M[i][j] + M[j][i]);
            M[i][j] = avg;
            M[j][i] = avg;
        }
    }
}

// ----------------------------------------------------------------------------
// Getters
// ----------------------------------------------------------------------------
float FurataKFBase::getMotorAngle() const               { return _x[0]; }
float FurataKFBase::getPendulumAngle() const            { return (_x[1] + kPi); } // raw t2 = dt2
float FurataKFBase::getPendulumAngleFromUpright() const { return _x[1]; }
float FurataKFBase::getMotorVelocity() const            { return _x[2]; }
float FurataKFBase::getPendulumVelocity() const         { return _x[3]; }

void FurataKFBase::getState(float out[NX]) const {
    memcpy(out, _x, sizeof(_x));
}

void FurataKFBase::getCovariance(float out[NX][NX]) const {
    memcpy(out, _Pcov, sizeof(_Pcov));
}
