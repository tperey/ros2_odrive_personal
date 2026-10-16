#pragma once

namespace ResidualModel {

// Feature order: 1, tau, t1d, t2d, t1d*|t1d|, t2d*|t2d|, tanh(50*t1d), tanh(50*t2d), sin(t2), cos(t2), t1d^2*sin(2*t2), t1d*t2d*sin(2*t2), strib_pos_t1d, strib_neg_t1d, strib_pos_t2d, strib_neg_t2d
// accel_corr_motor = sum_j (feat_j / feat_scale[j]) * coef[j]
// accel_corr_pend  = sum_j (feat_j / feat_scale[j]) * coef[16 + j]
// Fit with integration step dt = 5 ms (ZOH torque)

constexpr int NF = 16;

constexpr int FIT_DT_MS = 5;

constexpr float coef[32] = {4.50013158e-02f, 5.96083453e-01f, -4.49489470e-01f, -4.00693891e-03f, 3.09529039e-01f, -1.10552021e-01f, -1.53136480e+00f, 1.10461162e-01f, -1.51078151e-01f, 1.61953762e-02f, 2.70056093e-02f, 1.07018306e-02f, -6.97466335e-02f, 1.91356551e-02f, 6.76514942e-02f, -1.43069073e-02f, 1.59047425e-01f, 1.69994493e-01f, 4.49947450e-02f, 2.23262431e-01f, -1.61625170e-01f, 3.47605833e-01f, 1.04167734e-01f, -2.12282650e-02f, 3.81737141e-01f, -1.04500800e+00f, -1.81652820e-01f, 4.22338374e-01f, 7.30706868e-02f, -5.46547806e-02f, 6.13689671e-02f, -5.91587736e-02f};

constexpr float feat_scale[16] = {1.00000000e+00f, 1.60779901e-01f, 4.06561200e+00f, 1.14673073e+01f, 3.14367464e+01f, 2.39305963e+02f, 9.98037115e-01f, 9.98891832e-01f, 6.05103340e-01f, 6.41327135e-01f, 2.17891308e+01f, 3.43938556e+01f, 6.48714399e-02f, 6.51965624e-02f, 4.37623291e-02f, 4.39757816e-02f};

} // namespace ResidualModel
