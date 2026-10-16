#pragma once

namespace ResidualModel {

// Feature order: 1, tau, t1d, t2d, t1d*|t1d|, t2d*|t2d|, tanh(50*t1d), tanh(50*t2d), sin(t2), cos(t2), t1d^2*sin(2*t2), t1d*t2d*sin(2*t2), strib_pos_t1d, strib_neg_t1d, strib_pos_t2d, strib_neg_t2d
// accel_corr_motor = sum_j (feat_j / feat_scale[j]) * coef[j]
// accel_corr_pend  = sum_j (feat_j / feat_scale[j]) * coef[16 + j]
// Fit with integration step dt = 5 ms (ZOH torque)

constexpr int NF = 16;

constexpr int FIT_DT_MS = 5;

constexpr float coef[32] = {5.39580436e-02f, -1.78331906e-01f, 1.12809816e+00f, -2.74156486e-01f, -4.00545372e-01f, 2.37467077e-01f, -3.17062487e+00f, 1.81333989e-01f, 1.59531956e-01f, 6.96368784e-02f, -1.18926867e-01f, 1.73796845e-01f, 4.51026532e-01f, -2.86598294e-01f, -2.82485167e-01f, -5.52876871e-01f, 2.61549847e-01f, -7.14772421e-01f, -2.06495549e-01f, -4.28914311e-01f, 5.94683104e-01f, 3.79066312e-02f, -1.40228546e+00f, 5.27482471e-01f, 1.10854905e+00f, -1.01717631e+00f, -4.09390011e-01f, 1.84274668e+00f, 1.09794639e+00f, -1.79733037e+00f, 1.37277315e+00f, 2.10602356e+00f};

constexpr float feat_scale[16] = {1.00000000e+00f, 1.60725405e-01f, 4.07000798e+00f, 1.14663559e+01f, 3.15059777e+01f, 2.38938968e+02f, 9.98035351e-01f, 9.98850699e-01f, 6.06183999e-01f, 6.41932647e-01f, 2.18325980e+01f, 3.43287354e+01f, 6.49296351e-02f, 6.52472105e-02f, 4.37838330e-02f, 4.39682364e-02f};

} // namespace ResidualModel
