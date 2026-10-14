#pragma once

namespace ResidualModel {

// Feature order: 1, tau, t1d, t2d, tanh(50*t1d), tanh(50*t2d), sin(t2), cos(t2), t1d^2*sin(2*t2), t1d*t2d*sin(2*t2), strib_pos_t1d, strib_neg_t1d, strib_pos_t2d, strib_neg_t2d
// accel_corr_motor = sum_j (feat_j / feat_scale[j]) * coef[j]
// accel_corr_pend  = sum_j (feat_j / feat_scale[j]) * coef[14 + j]

constexpr int NF = 14;

constexpr float coef[28] = {8.87959573e-02f, -3.12617657e-01f, 4.63691535e-01f, 1.25923894e-01f, -2.97108685e+00f, -2.09937349e-01f, -5.99230266e-02f, -4.91720780e-02f, -4.75788377e-02f, -2.30091861e-01f, 3.50596470e-02f, -3.54662023e-01f, 7.84942397e-01f, -4.66733385e-01f, 1.18620845e-01f, 1.19917584e-01f, 1.58522918e-01f, 4.64799645e-01f, -1.01966086e+00f, -1.75757847e-02f, 2.70453610e-01f, -8.11164337e-01f, 3.75332886e-01f, 2.60498899e-01f, -2.22561895e-01f, 6.44695140e-01f, 9.38496335e-01f, -9.66899111e-01f};

constexpr float feat_scale[14] = {1.00000000e+00f, 1.60344404e-01f, 4.08156506e+00f, 1.14657759e+01f, 9.98050721e-01f, 9.98918767e-01f, 6.07302833e-01f, 6.40690555e-01f, 2.19889319e+01f, 3.45585269e+01f, 6.47494930e-02f, 6.51383656e-02f, 4.29342769e-02f, 4.30396259e-02f};

} // namespace ResidualModel
