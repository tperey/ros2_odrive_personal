#pragma once

namespace ResidualModel {

// Feature order: 1, tau, t1d, t2d, t1d*|t1d|, t2d*|t2d|, tanh(50*t1d), tanh(50*t2d), sin(t2), cos(t2), t1d^2*sin(2*t2), t1d*t2d*sin(2*t2), strib_pos_t1d, strib_neg_t1d, strib_pos_t2d, strib_neg_t2d
// accel_corr_motor = sum_j (feat_j / feat_scale[j]) * coef[j]
// accel_corr_pend  = sum_j (feat_j / feat_scale[j]) * coef[16 + j]
// Fit with integration step dt = 5 ms (ZOH torque)

constexpr int NF = 16;

constexpr int FIT_DT_MS = 5;

constexpr float coef[32] = {1.50793906e-01f, -2.17682360e-01f, 7.22516393e-01f, 1.89608328e-01f, -1.76526432e-01f, -1.04603710e-01f, -3.00937145e+00f, -1.57129084e-01f, 2.83801356e-02f, 5.84208679e-03f, -3.48529376e-03f, -8.14552170e-02f, 1.77004041e-01f, -3.80007271e-01f, -3.16694179e-01f, -3.32971854e-01f, 9.60625730e-02f, -6.44308245e-01f, 1.14955214e+00f, 3.68341815e-01f, -5.14351143e-01f, -4.26356668e-01f, -2.07178554e+00f, 5.28406880e-02f, 1.01558100e+00f, -1.00576524e+00f, -2.95250945e-01f, 1.83381322e+00f, 3.10229597e-01f, -4.28644533e-01f, 1.76150936e+00f, 1.91901658e+00f};

constexpr float feat_scale[16] = {1.00000000e+00f, 1.60914345e-01f, 4.06422631e+00f, 1.14604725e+01f, 3.14499542e+01f, 2.39187042e+02f, 9.98047415e-01f, 9.98870411e-01f, 6.04399530e-01f, 6.41390324e-01f, 2.17871338e+01f, 3.43695323e+01f, 6.48356648e-02f, 6.51619743e-02f, 4.37877044e-02f, 4.39792039e-02f};

} // namespace ResidualModel
