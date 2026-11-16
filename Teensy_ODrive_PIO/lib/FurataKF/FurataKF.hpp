#ifndef FURATA_KF
#define FURATA_KF

#include "FurataKFBase.hpp"

/*
 * FurataKF -- LINEAR discrete Kalman Filter, linearized about upright
 * -------------------------------------------------------------------
 * TEMPLATE: function bodies are TODOs to fill in (see FurataKF.cpp).
 *
 * Only adds the linear model + its predict(). Update, getters, etc. are
 * inherited from FurataKFBase.
 *
 * CONTINUOUS MODEL (from FurataAnalyzer._obtain_linear_statespace_matrices):
 *      xdot = A x + B u
 * DISCRETE MODEL:
 *      x[k+1] = F x[k] + G u[k]
 *      F <-> Ad,   G <-> Bd
 *
 * NOTE: only trustworthy near upright (dt2 ~ 0). See FurataEKF/FurataUKF
 *       for filters using the full nonlinear model.
 */
class FurataKF : public FurataKFBase {
public:
    FurataKF(const FurataParams& params, float dt,
             const float x0[NX], const float P0[NX][NX],
             const float Q[NX][NX], const float R[NZ][NZ]);

    // Eq (1), (2) with constant F, G.
    // 'override' makes the compiler check that this really overrides a base virtual.
    void predict(float tau) override;

private:
    float _A[NX][NX]{};  // continuous-time A
    float _B[NX]{};      // continuous-time B (column, single input)
    float _F[NX][NX];    // discrete-time state transition  (Ad)
    float _G[NX];        // discrete-time input matrix      (Bd)

    // Called once from the constructor (non-virtual, so safe there)
    void _buildContinuousModel();  // fills _A, _B from _p
    void _discretize();            // fills _F, _G from _A, _B, _dt

    void _predictState(float tau); // x = F x + G u
};

#endif
