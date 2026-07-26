#include <OdriveCANWrapper.hpp> // Include necessary libraries
#include <Arduino.h>
#include <AccelKF.hpp>
#include <Encoder.h>

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
#define SINE_TORQUE_BASE_AMP 0.11
constexpr uint8_t n_runs = 14;
OdriveSinusoidTorqueSID st_sid_runs[n_runs] = {
    { {SINE_TORQUE_BASE_AMP}, {0.05}, 4, 1}, // Get static friciton data
    { {0.4}, {2.0}, 100, 1}, // Firmly achieves full pendulum rotation
    { {SINE_TORQUE_BASE_AMP}, {0.1}, 4, 1}, // Single frequency examples
    { {SINE_TORQUE_BASE_AMP}, {0.2}, 4, 1},
    { {SINE_TORQUE_BASE_AMP}, {0.5}, 10, 1},
    { {SINE_TORQUE_BASE_AMP}, {1.0}, 20, 1},
    { {SINE_TORQUE_BASE_AMP}, {2.0}, 20, 1},
    { {4*SINE_TORQUE_BASE_AMP}, {5.0}, 20, 1}, // High accel, low speed (UNTESTED)
    { {SINE_TORQUE_BASE_AMP}, {10.0}, 20, 1},
    { {SINE_TORQUE_BASE_AMP, SINE_TORQUE_BASE_AMP/2}, {2.0, 5.0}, 10, 2},
    { {SINE_TORQUE_BASE_AMP, SINE_TORQUE_BASE_AMP/2}, {0.5, 2.0}, 10, 2},
    { {SINE_TORQUE_BASE_AMP, SINE_TORQUE_BASE_AMP/2}, {1.0, 2.0}, 10, 2},
    { {0.15, 0.05}, {0.5, 2.5}, 10, 2},
    { {0.2, 0.075}, {1.0, 2.0}, 10, 2}
};
bool status_ok = true;

/* MAIN */
void setup() {

    // Pendulum setup
    furataEncoder.write(0); // Init to 0

    // Odrive setup
    initialize_singleODCW();
}

void loop() {
    for (uint8_t i = 0; i < n_runs; i++) {
        status_ok = perform_sid_with_pend_sinetorque(st_sid_runs[i], &thisKF, &furataEncoder, RAD_PER_PULSE);

        while (!status_ok) {
            // Infinite loop if stop msg received
        }
    }
}