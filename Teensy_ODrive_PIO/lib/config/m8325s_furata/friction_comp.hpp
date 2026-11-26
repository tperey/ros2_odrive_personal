#pragma once
#include "arm1_fs.hpp"

namespace FrictionCompConfig {
constexpr float FS       = Arm1FsConfig::fs1;  // [N-m] identified Coulomb friction (don't duplicate it)
constexpr float PCT      = 0.7f;               // TODO tune
constexpr float V_TH_HI  = 0.05f;              // TODO [rad/s] enter MOVING above this
constexpr float V_TH_LO  = 0.02f;              // TODO [rad/s] leave MOVING below this
constexpr float TAU_TH   = 0.01f;              // TODO [N-m]  breakaway only if |tau_cmd| > this
}
