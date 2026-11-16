#ifndef FURATA_KF_BASE
#define FURATA_KF_BASE

/*
 * FurataKFBase -- shared machinery for Kalman-type filters on the Furata pendulum
 * -------------------------------------------------------------------------------
 * TEMPLATE: function bodies are TODOs to fill in (see FurataKFBase.cpp).
 *
 * CLASS LAYOUT
 *
 *                       FurataKFBase   (this file, abstract)
 *                     /       |       \
 *              FurataKF   FurataEKF   FurataUKF
 *              (linear)  (Jacobian)  (sigma points)
 *
 *   The three filters differ ONLY in how they PREDICT. The measurement is
 *   linear (z = H x, we read the angles directly), so the UPDATE step is
 *   the same for all three and lives here. (With a linear h(x), the UKF's
 *   sigma-point update collapses to exactly the standard KF update.)
 *
 *   What each derived class supplies for predict(tau):
 *     KF : x = F x + G u                      P = F P F^T + Q   (F, G constant)
 *     EKF: x = f(x, u)  (nonlinear, RK4)      P = F P F^T + Q   (F = df/dx at x[k-1|k-1])
 *     UKF: sigma points -> f(.) -> mean/cov   (doesn't use F at all)
 *
 *   Shared helpers they can lean on:
 *     _propagateCovariance(F)   -> used by KF and EKF
 *     _dynamics / _propagate    -> nonlinear model, used by EKF and UKF
 *
 * STATE (matches LQRConfig ordering, for ALL derived filters):
 *      x = [ t1,  dt2,  t1d,  t2d ]^T          (NX = 4)
 *          t1  = motor/arm angle                               [rad]
 *          dt2 = wrap(t2 - pi), pendulum angle FROM UPRIGHT     [rad]
 *          t1d = motor/arm velocity                            [rad/s]
 *          t2d = pendulum velocity                             [rad/s]
 *      (raw t2: 0 = hanging down, pi = upright)
 *
 * INPUT:        u = tau                        (NU = 1)
 * MEASUREMENT:  z = [ t1, dt2 ]^T = H x + v    (NZ = 2),  H = C = [I2 0]
 *
 * C++ GOTCHA: don't call virtual functions from a constructor -- in the base
 * constructor the object isn't a derived type yet, so the base version runs
 * (or it's undefined if pure virtual). Derived-specific setup (e.g. building
 * F, G) goes in the DERIVED constructor.
 */

// Physical params defining the Furata pendulum.
// Names match lib/config/m8325s_furata/arm1.hpp / arm2.hpp and lqr.json.
struct FurataParams {
    // Arm 1 (motor arm)
    float J1zz_hat;   // [kg m^2]    arm1 inertia about motor axis (incl. parallel axis)
    float b1;         // [N m s/rad] arm1 viscous damping
    float L_1;        // [m]         arm1 length (motor axis -> pendulum pivot)
    // Note static friction is pre-compensated

    // Arm 2 (pendulum)
    float J2zz_hat;   // [kg m^2]    pendulum inertia about its pivot
    float J2yy_hat;   // [kg m^2]    needed by the NONLINEAR model (EKF/UKF) only
    float J2xx;       // [kg m^2]
    float b2;         // [N m s/rad] pendulum viscous damping
    float m2;         // [kg]        pendulum mass
    float l2;         // [m]         pivot -> pendulum COM
    float fs2 = 0.0;  // [N-m]       static friction. Default to zero.
    float fs2_k = 100.0;

    float g = 9.81f;  // [m/s^2]
};

class FurataKFBase {
public:
    static constexpr int NX = 4;  // # states
    static constexpr int NZ = 2;  // # measurements
    static constexpr float kPi = 3.14159265358979f; // # float pi constant

    /*
     * params : physical pendulum params
     * dt     : filter timestep [s] (assumed constant)
     * x0     : initial state estimate  x_hat[0|0]   (x0[1] is dt2, NOT raw t2)
     * P0     : initial estimate covariance  P[0|0]
     * Q      : process noise covariance (NX x NX)
     * R      : measurement noise covariance (NZ x NZ)
     */
    FurataKFBase(const FurataParams& params, float dt,
                 const float x0[NX], const float P0[NX][NX],
                 const float Q[NX][NX], const float R[NZ][NZ]);

    // Virtual destructor: required whenever you might delete a derived object
    // through a base pointer. Costs nothing here.
    // Basically, ensures derived class destructor runs.
    virtual ~FurataKFBase() = default;

    // ---- Main interface ----

    // Time update. PURE VIRTUAL: each filter type implements its own.
    // Produces x_hat[k|k-1], P[k|k-1] from x_hat[k-1|k-1], P[k-1|k-1].
    // = 0 indicates pure virtual dn prevents creation of base classes.
    virtual void predict(float tau) = 0;

    // Measurement update, Eq (3)-(7). Shared by all filters (linear measurement).
    // virtual so a derived class CAN override it if ever needed, but none should have to.
    // t2_meas is the RAW pendulum angle (0 = hanging down).
    virtual void update(float t1_meas, float t2_meas);

    // predict(tau) then update(...). Non-virtual: it calls the virtual predict(),
    // so it automatically dispatches to KF/EKF/UKF. Write it once, here.
    void step(float tau, float t1_meas, float t2_meas);

    // Re-initialize the estimate (e.g. after swing-up, when entering the balance region)
    void reset(const float x0[NX], const float P0[NX][NX]);

    // ---- Getters ----
    float getMotorAngle() const;                // t1
    float getPendulumAngle() const;             // raw t2 (= dt2 + pi)
    float getPendulumAngleFromUpright() const;  // dt2, what LQR wants
    float getMotorVelocity() const;             // t1d
    float getPendulumVelocity() const;          // t2d
    void  getState(float out[NX]) const;          // full x_hat, LQR ordering
    void  getCovariance(float out[NX][NX]) const; // full P (diagonal = per-state variance)

protected:
    // protected (not private) so derived classes can read/write these directly.

    // ---- Model / config ----
    FurataParams _p;
    float _dt;
    float _H[NZ][NX]{};   // = C. Same for every filter.

    // ---- Noise ----
    float _Q[NX][NX];
    float _R[NZ][NZ];

    // ---- Estimate ----
    float _x[NX];       // x_hat
    float _P[NX][NX];   // P

    // ---- Shared predict helper (KF + EKF) ----
    // Eq (2): P = F P F^T + Q, for whatever F the caller hands in
    // (constant Ad for KF, Jacobian for EKF).
    void _propagateCovariance(const float F[NX][NX]);

    // ---- Nonlinear model (EKF + UKF) ----
    // Continuous-time xdot = f_c(x, u), full nonlinear Furata dynamics.
    void _dynamics(const float x[NX], float tau, float xdot[NX]) const;
    // Discrete-time x[k+1] = f(x[k], u[k]): integrate _dynamics over one _dt (RK4).
    void _propagate(const float x[NX], float tau, float x_next[NX]) const;

    // ---- Update helpers, Eq (3)-(7) ----
    void _buildMeasurementModel();                                     // fills _H
    void _computeInnovation(const float z[NZ], float y[NZ]);           // y = z - H x
    void _computeInnovationCov(float S[NZ][NZ]);                       // S = H P H^T + R
    bool _computeGain(const float S[NZ][NZ], float K[NX][NZ]);         // K = P H^T S^-1 (false if S singular)
    void _updateState(const float K[NX][NZ], const float y[NZ]);       // x = x + K y
    void _updateCovariance(const float K[NX][NZ]);                     // P = (I - K H) P

    // ---- Small math helpers ----
    static float _wrapPi(float a);  // wrap angle into (-pi, pi]
    static bool  _invert2x2(const float M[NZ][NZ], float Minv[NZ][NZ]);
    static void  _symmetrize(float M[NX][NX]);  // M = 0.5 (M + M^T)

    // Basic matrix math, For varying sizes
    // static = no this pointer, bc it shouldn't modify object vars (pure math)
    template <int N, int P, int M>
    static void _matMul(const float (&A)[N][M], const float (&B)[M][P], float (&C)[N][P]);

    template <int N, int M>
    static void _vecmatMul(const float (&A)[N][M], const float (&b)[M], float (&c)[N]);

    template<int N, int M>
    static void _transpose(const float (&A)[N][M], float (&At)[M][N]);

    template<int N, int M>
    static void _matAdd(const float (&A)[N][M], const float (&B)[N][M], float (&C)[N][M]);
};

// Compute C [N X P] = A [N x M] @ B [M x P]
template <int N, int P, int M>
void FurataKFBase::_matMul(const float (&A)[N][M], const float (&B)[M][P], float (&C)[N][P]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < P; j++) {
            float cur = 0.0f;
            // Matrix math gives dot of row_i(A) with col_j(B)
            for (int k = 0; k < M; k++) {
                cur += A[i][k]*B[k][j];
            }
            C[i][j] = cur;
        }
    }
}

// Compute c [N] = A [N x M] @ b [M]
template <int N, int M>
void FurataKFBase::_vecmatMul(const float (&A)[N][M], const float (&b)[M], float (&c)[N]) {
    for (int i = 0; i < N; i++) {
        float cur = 0.0f;
        for (int j = 0; j < M; j++) {
            cur += A[i][j]*b[j];
        }
        c[i] = cur;
    }
}

// Compute transpose
template<int N, int M>
void FurataKFBase::_transpose(const float (&A)[N][M], float (&At)[M][N]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < M; j++) {
            At[j][i] = A[i][j];
        }
    }
}

// Compute C = A + B
template <int N, int M>
void FurataKFBase::_matAdd(const float (&A)[N][M], const float (&B)[N][M], float (&C)[N][M]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < M; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

#endif