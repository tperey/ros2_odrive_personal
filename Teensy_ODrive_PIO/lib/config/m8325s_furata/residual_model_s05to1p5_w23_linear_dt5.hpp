#pragma once

namespace ResidualModel {

// Feature order: 1, tau, t1d, t2d, t1d*|t1d|, t2d*|t2d|, tanh(50*t1d), tanh(50*t2d), sin(t2), cos(t2), t1d^2*sin(2*t2), t1d*t2d*sin(2*t2), strib_pos_t1d, strib_neg_t1d, strib_pos_t2d, strib_neg_t2d
// accel_corr_motor = sum_j (feat_j / feat_scale[j]) * coef[j]
// accel_corr_pend  = sum_j (feat_j / feat_scale[j]) * coef[16 + j]
// Fit with integration step dt = 5 ms (ZOH torque)

constexpr int NF = 16;

constexpr int FIT_DT_MS = 5;

constexpr float coef[32] = {7.75323301e-02f, -1.06100034e-01f, 7.90922710e-01f, -2.83680756e-01f, -3.61210151e-01f, 2.48204961e-01f, -2.89947601e+00f, 2.08438206e-01f, 1.58613607e-01f, -1.09718051e-02f, -1.77105146e-01f, 1.42498754e-01f, 3.90153048e-01f, -2.09852767e-01f, -5.52617422e-01f, -1.98401874e-01f, 2.27969349e-01f, -7.55859123e-01f, -8.42204527e-01f, -2.18873236e-01f, 9.62784105e-01f, -5.40240165e-02f, -9.56813072e-01f, 3.03467094e-01f, 1.11457611e+00f, -8.25819703e-01f, -3.13598916e-01f, 1.88286471e+00f, 7.93631890e-01f, -1.33723276e+00f, 1.30162467e+00f, 1.95620464e+00f};

constexpr float feat_scale[16] = {1.00000000e+00f, 1.60336406e-01f, 4.08125983e+00f, 1.14646334e+01f, 3.16977852e+01f, 2.38913120e+02f, 9.98049899e-01f, 9.98824947e-01f, 6.07279581e-01f, 6.40664142e-01f, 2.19880152e+01f, 3.45564280e+01f, 6.48371187e-02f, 6.50861406e-02f, 4.39156377e-02f, 4.40391193e-02f};

} // namespace ResidualModel
