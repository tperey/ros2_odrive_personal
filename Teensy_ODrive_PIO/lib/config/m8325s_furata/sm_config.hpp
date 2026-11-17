#pragma once

#include <cstdint>

/*
 * SMConfig -- thresholds / constants for FurataStateMachine
 * ---------------------------------------------------------
 * TEMPLATE: every value marked TODO is a PLACEHOLDER. Tune / verify on hardware.
 *
 * Angle conventions (match FurataKFBase):
 *   t1  : motor/arm angle [rad]
 *   t2  : RAW pendulum angle [rad], 0 = hanging down, pi = upright
 *   dt2 : wrap(t2 - pi), pendulum angle FROM UPRIGHT [rad]
 */

namespace SMConfig {

// ---- Timing ----
constexpr float    DT_S  = 0.001f;   // [s]  must match the KF dt and main's loop period
constexpr uint32_t DT_US = 1000;     // [us]

// ---- Pendulum encoder ----
constexpr float RAD_PER_PULSE = (2.0f * 3.14159265358979f) / 2400.0f;  // empirically 2400 pulses/rev
constexpr float ENC_SIGN      = 1.0f;   // TODO: verify +t2 direction matches the model's +t2

// ---- ODrive ----
constexpr float ANTICOG_PCT = 1.0f;     // same as your SID mains

// ---- Actuation ----
constexpr float TAU_MAX = 0.3f;         // TODO [N-m] hard torque clamp applied every tick

// ---- IDLE -> SWING_UP guard: only accept START if pendulum is hanging & still ----
constexpr float START_MAX_T2  = 0.05f;  // TODO [rad]   |t2|  (raw, from hanging)
constexpr float START_MAX_T2D = 0.10f;  // TODO [rad/s] |t2d|

// ---- SWING_UP ----
constexpr uint32_t SWINGUP_TIMEOUT_MS = 15000;  // TODO [ms] give up -> IDLE
constexpr float    SWINGUP_GAIN       = 0.0f;   // TODO energy-pumping gain (0 = hand catch)

// ---- SWING_UP -> BALANCE (catch) ----
constexpr float CATCH_ANGLE = 0.30f;    // TODO [rad]   |dt2| below this ...
constexpr float CATCH_RATE  = 3.0f;     // TODO [rad/s] ... AND |t2d| below this

// ---- BALANCE -> IDLE (fell) ----
constexpr float FALL_ANGLE = 0.60f;     // TODO [rad] |dt2| above this

// ---- Faults (any active state -> FAULT) ----
constexpr float    T1_LIMIT             = 6.28f;  // TODO [rad] |t1| above this (arm wandered off)
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 300;    // ODrive heartbeat default period is 100 ms

// ---- Filter (re)seeding: P[0|0] used by begin() and BALANCE entry ----
// Diagonal only. Order: [t1, dt2, t1d, t2d]
constexpr float P0_DIAG[4] = {1e-4f, 1e-4f, 1e-2f, 1e-2f};  // TODO

} // namespace SMConfig
