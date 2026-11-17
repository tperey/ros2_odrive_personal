#include <Arduino.h>
#include <Encoder.h>

#include <FurataUKF.hpp>
#include <FurataStateMachine.hpp>

#include <m8325s_furata/arm1.hpp>
#include <m8325s_furata/arm2.hpp>
#include <m8325s_furata/sm_config.hpp>

/*
 * furata_balance -- main for the state-machine-driven Furata pendulum
 * -------------------------------------------------------------------
 * main's only jobs:
 *   1. Build the objects (filter choice + tuning + encoder pins live HERE)
 *   2. setup(): sm.begin()   (all hardware init happens inside, once)
 *   3. loop():  run the clock, call sm.tick() every DT
 *
 * Swap filter: change FurataUKF -> FurataEKF / FurataKF below.
 * The SM only sees FurataKFBase&.
 */

/* ---- Encoder ---- */
constexpr int ENC_PIN_A = 2;
constexpr int ENC_PIN_B = 3;
Encoder pendEncoder(ENC_PIN_A, ENC_PIN_B);

/* ---- Filter params ---- */
FurataParams makeParams() {
    FurataParams p{};
    p.J1zz_hat = Arm1Config::J1zz_hat;
    p.b1       = Arm1Config::b1;
    p.L_1      = Arm1Config::L_1;
    p.J2zz_hat = Arm2Config::J2zz_hat;
    p.J2yy_hat = Arm2Config::J2yy_hat;
    p.J2xx     = Arm2Config::J2xx;
    p.b2       = Arm2Config::b2;
    p.m2       = Arm2Config::m2;
    p.l2       = Arm2Config::l2;
    // TODO: fs2 / fs2_k if you identify pendulum static friction
    return p;
}

/* ---- Filter tuning ----
 * State order: [t1, dt2, t1d, t2d].  x0/P0 only matter until begin() reseeds
 * from measurements (see FurataStateMachine::_seedFilterFromMeasurements).
 */
const float X0[4] = {0.0f, 3.14159265f, 0.0f, 0.0f};  // hanging: dt2 = wrap(0 - pi) = pi

const float P0[4][4] = {
    {1e-4f, 0,     0,     0    },
    {0,     1e-4f, 0,     0    },
    {0,     0,     1e-2f, 0    },
    {0,     0,     0,     1e-2f},
};

// TODO: tune. Process noise (model mismatch). Velocities usually need the most.
const float Q[4][4] = {
    {1e-7f, 0,     0,     0    },
    {0,     1e-7f, 0,     0    },
    {0,     0,     1e-3f, 0    },
    {0,     0,     0,     1e-3f},
};

// TODO: tune. Measurement noise ~ (quantization)^2 / 12 is a good start:
//   t1: ODrive pos (14-bit abs enc?)   t2: (2pi/2400)^2 / 12 ~ 5.7e-7
const float R[2][2] = {
    {1e-6f, 0     },
    {0,     5.7e-7f},
};

FurataUKF kf(makeParams(), SMConfig::DT_S, X0, P0, Q, R);  // alpha/beta/kappa: defaults

/* ---- State machine ---- */
FurataStateMachine sm(pendEncoder, kf);

/* ---- Clock ---- */
uint32_t t_prev = 0;

void setup() {
    sm.begin();          // blocks until ODrive ready + PC handshake, then IDLE
    t_prev = micros();
}

void loop() {
    uint32_t now = micros();
    if (now - t_prev >= SMConfig::DT_US) {
        // TODO: decide overrun policy. Entry actions that call start/stop_motor_single()
        // block ~150 ms. "+= DT_US" would then fire a burst of catch-up ticks.
        //   Normal tick:  t_prev += SMConfig::DT_US;     (no drift)
        //   Overrun:      t_prev = now;                  (skip ahead)
        // e.g. if (now - t_prev > 2 * SMConfig::DT_US) t_prev = now; else t_prev += SMConfig::DT_US;
        t_prev = now;

        sm.tick();
    }
}
