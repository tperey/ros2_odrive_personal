#pragma once

namespace LQRConfig {

// Discrete LQR about upright (theta2 = pi)
// tau = -(K[0]*t1 + K[1]*wrap(t2 - pi) + K[2]*t1d + K[3]*t2d)
// NOTE: state order is [t1, t2, t1d, t2d], wrap(t2 - pi) into (-pi, pi]
// Designed with Q diag = [0.1111, 14.59, 0.01, 0.01], R = 0.4444

// And with DT = 0.001000f;

constexpr float K[4] = {-4.68885569e-01f, 7.11725757e+00f, -3.40572954e-01f, 4.81922649e-01f};

} // namespace LQRConfig
