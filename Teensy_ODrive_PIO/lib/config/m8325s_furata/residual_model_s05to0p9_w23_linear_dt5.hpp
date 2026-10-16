#pragma once

namespace ResidualModel {

// Feature order: 1, tau, t1d, t2d, t1d*|t1d|, t2d*|t2d|, tanh(50*t1d), tanh(50*t2d), sin(t2), cos(t2), t1d^2*sin(2*t2), t1d*t2d*sin(2*t2), strib_pos_t1d, strib_neg_t1d, strib_pos_t2d, strib_neg_t2d
// accel_corr_motor = sum_j (feat_j / feat_scale[j]) * coef[j]
// accel_corr_pend  = sum_j (feat_j / feat_scale[j]) * coef[16 + j]
// Fit with integration step dt = 5 ms (ZOH torque)

constexpr int NF = 16;

constexpr int FIT_DT_MS = 5;

constexpr float coef[32] = {1.53352663e-01f, -4.33957813e-01f, 5.55268899e-01f, -2.86006323e-02f, -1.94515200e-01f, 3.56734602e-02f, -2.55223993e+00f, 4.29487456e-02f, 3.99204437e-02f, -9.38442900e-03f, -1.05044102e-01f, -3.32902511e-01f, -3.28539567e-01f, -1.85500264e-01f, 1.37111301e-01f, -3.89212497e-01f, 6.33724475e-01f, 1.63139776e-02f, 1.57227823e+00f, -7.92899628e-03f, -1.02154282e+00f, -1.06729062e-01f, -1.31231215e+00f, -2.05546838e-02f, 1.58065930e+00f, -9.10831324e-01f, 1.71357516e-02f, 1.58793235e+00f, -1.77214129e+00f, 9.94069255e-01f, 9.72567089e-01f, 3.65384428e-01f};

constexpr float feat_scale[16] = {1.00000000e+00f, 1.60930019e-01f, 4.07000397e+00f, 1.14455653e+01f, 3.15142780e+01f, 2.38553755e+02f, 9.98049467e-01f, 9.98879288e-01f, 6.04747603e-01f, 6.40831093e-01f, 2.18371875e+01f, 3.43899148e+01f, 6.47866753e-02f, 6.50307666e-02f, 4.38696172e-02f, 4.40521741e-02f};

} // namespace ResidualModel
