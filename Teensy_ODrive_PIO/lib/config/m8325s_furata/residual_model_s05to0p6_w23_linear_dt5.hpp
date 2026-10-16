#pragma once

namespace ResidualModel {

// Feature order: 1, tau, t1d, t2d, t1d*|t1d|, t2d*|t2d|, tanh(50*t1d), tanh(50*t2d), sin(t2), cos(t2), t1d^2*sin(2*t2), t1d*t2d*sin(2*t2), strib_pos_t1d, strib_neg_t1d, strib_pos_t2d, strib_neg_t2d
// accel_corr_motor = sum_j (feat_j / feat_scale[j]) * coef[j]
// accel_corr_pend  = sum_j (feat_j / feat_scale[j]) * coef[16 + j]
// Fit with integration step dt = 5 ms (ZOH torque)

constexpr int NF = 16;

constexpr int FIT_DT_MS = 5;

constexpr float coef[32] = {8.04085913e-02f, 3.47169647e-02f, 2.55146339e-01f, 7.08979945e-02f, -9.05652101e-02f, -7.41628264e-02f, -2.17406897e+00f, 1.23637571e-02f, -5.51174030e-02f, -3.25549340e-03f, -2.09558832e-02f, -1.08455028e-01f, 6.54169140e-03f, -1.35389906e-01f, 1.54564639e-01f, -1.55181698e-01f, 4.69902729e-01f, 1.83627205e-01f, -4.16050048e-01f, -5.00514457e-02f, 5.15095011e-02f, 2.67543920e-01f, -7.29460295e-01f, 2.06622606e-02f, 1.01633955e+00f, -1.28324458e+00f, 1.24507827e-01f, 1.27582798e+00f, -6.87113523e-01f, -7.15074139e-01f, 2.45895114e-01f, 6.70520092e-01f};

constexpr float feat_scale[16] = {1.00000000e+00f, 1.60702260e-01f, 4.07438176e+00f, 1.14672367e+01f, 3.15655560e+01f, 2.38994421e+02f, 9.98042166e-01f, 9.98882164e-01f, 6.06077588e-01f, 6.41518278e-01f, 2.18827079e+01f, 3.44547893e+01f, 6.48221897e-02f, 6.51168770e-02f, 4.36629795e-02f, 4.38565493e-02f};

} // namespace ResidualModel
