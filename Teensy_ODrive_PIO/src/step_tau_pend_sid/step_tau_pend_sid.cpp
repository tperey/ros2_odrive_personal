#include <OdriveCANWrapper.hpp> // Include necessary libraries
#include <Arduino.h>
#include <AccelKF.hpp>
#include <Encoder.h>

#include <m8325s_furata/cogging_config.hpp>

/* ENCODER SETUP */

// Constants
#define DEG_PER_PULSE 360.0/2400 // Website says 600 P.R?
#define RAD_PER_PULSE (2.0 * 3.1415926535)/2400
// But maybe theres a gear ratio b/c empirically it was 2400
// Also this is (360 deg or 2pi rad/1 rev) * (1 rev / 2400 pulse)
int enc1Pin = 2;
int enc2Pin = 3;
// float current_rad;

Encoder furataEncoder(enc1Pin, enc2Pin);

// ---------- EKF parameters ----------
float dt = 0.001;  // sample time in seconds

// Process noise covariance
float Q[3][3] = {
  {1e-6, 0, 0},
  {0, 1e-2, 0},
  {0, 0, 10.0}
};

// Measurement noise (encoder)
const float R = 5.8e-7;

// Init EKF state 
float xi[3] = {0.0, 0.0, 0.0};       // [theta, theta_dot, theta_ddot]
float Pi[3][3] = { {0.01, 0.0, 0.0}, {0.0, 0.01, 0.0}, {0.0, 0.0, 0.01} };  // covariance

AccelKF thisKF = AccelKF(xi[0], xi[1], xi[2], R, Q, dt, Pi);

/* ODRIVE SETUP */
#define STEP_TORQUE_BASE_AMP 0.08 //0.11
#define ANTICOG_PERCENT_TO_USE 1.0 // No scaling seems to be smoothest - TP 2026-07-29
constexpr uint8_t n_runs = 13;
OdriveStepTorqueSID st_sid_runs[n_runs] = {
    {STEP_TORQUE_BASE_AMP, 1.0, 10},
    {STEP_TORQUE_BASE_AMP, 2.0, 10},
    {2*STEP_TORQUE_BASE_AMP, 2.0, 10},
    {3*STEP_TORQUE_BASE_AMP, 2.0, 10},
    {STEP_TORQUE_BASE_AMP, 3.0, 10},
    {2*STEP_TORQUE_BASE_AMP, 3.0, 15},
    {3*STEP_TORQUE_BASE_AMP, 3.0, 15},
    {STEP_TORQUE_BASE_AMP, 4.0, 20},
    {2*STEP_TORQUE_BASE_AMP, 4.0, 20},
    {3*STEP_TORQUE_BASE_AMP, 4.0, 20},
    {STEP_TORQUE_BASE_AMP, 5.0, 20},
    {2*STEP_TORQUE_BASE_AMP, 5.0, 20},
    {3*STEP_TORQUE_BASE_AMP, 5.0, 20},
};
volatile static bool this_status_ok = true;

/* MAIN */
void setup() {

    // Pendulum setup
    furataEncoder.write(0); // Init to 0

    // Odrive setup
    initialize_singleODCW();
    load_anticogging_config(CoggingConfig::cogging_map, ANTICOG_PERCENT_TO_USE, 0.0, 0.0);
}

void loop() {
    for (uint8_t i = 0; i < n_runs; i++) {
        while (!this_status_ok) {
            // Infinite loop if stop msg received
        }
        this_status_ok = perform_sid_with_pend_steptorque(st_sid_runs[i], &thisKF, &furataEncoder, RAD_PER_PULSE, true);
    }
    this_status_ok = false;
}