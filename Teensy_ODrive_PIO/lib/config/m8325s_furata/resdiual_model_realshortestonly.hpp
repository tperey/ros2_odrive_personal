#pragma once

namespace ResidualModel {

// Feature order: 1, tau, t1d, t2d, t1d*|t1d|, t2d*|t2d|, tanh(50*t1d), tanh(50*t2d), sin(t2), cos(t2), t1d^2*sin(2*t2), t1d*t2d*sin(2*t2), strib_pos_t1d, strib_neg_t1d, strib_pos_t2d, strib_neg_t2d
// accel_corr_motor = sum_j (feat_j / feat_scale[j]) * coef[j]
// accel_corr_pend  = sum_j (feat_j / feat_scale[j]) * coef[16 + j]
// Fit with integration step dt = 5 ms (ZOH torque)

constexpr int NF = 16;

constexpr int FIT_DT_MS = 5;

constexpr float coef[32] = {1.25718851e-02f, 1.04522393e+00f, -2.73803292e-01f, 3.30241639e-02f, -1.00039022e-01f, -3.59236345e-02f, 3.41914161e-02f, 6.74602911e-01f, 2.19644826e-01f, 5.38204503e-01f, -3.29313135e-01f, -2.57309657e-01f, -3.94808268e-01f, 3.33445713e-01f, 1.70457123e+00f, -5.24432515e-01f, -1.57303408e+00f, -1.49577723e+00f, -7.97731257e-01f, -2.02647209e-01f, -9.55798307e-01f, -3.13585923e-01f, -3.55223764e-01f, 2.83406044e+00f, -2.11881765e+00f, -3.19891510e+00f, 1.62914223e+00f, 3.30126723e-01f, 4.72045329e+00f, -7.24194273e-01f, 1.65390557e+00f, -1.63128458e+00f};

constexpr float feat_scale[16] = {1.00000000e+00f, 1.54111826e-01f, 3.31355940e+00f, 8.82485716e+00f, 2.19699020e+01f, 1.69645074e+02f, 9.88583979e-01f, 9.92593123e-01f, 5.48914816e-01f, 4.69869517e-01f, 1.53990874e+01f, 2.36915880e+01f, 8.76533688e-02f, 9.87584367e-02f, 9.73057452e-02f, 1.01956361e-01f};

} // namespace ResidualModel
