#include <OdriveCANWrapper.hpp> // Include necessary libraries
#include <Arduino.h>
#include <AccelKF.hpp>
#include <Encoder.h>
#include <random>

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
#define ANTICOG_PERCENT_TO_USE 1.0 // No scaling seems to be smoothest - TP 2026-07-29
volatile static bool this_status_ok = true;

// Random run generation
std::random_device rd;
std::mt19937 gen(rd());

std::uniform_int_distribution<int> which_dist(0.0, 1.0);
std::uniform_real_distribution<float> freq_dist(1.0, 5.0);
std::uniform_int_distribution<int> entries_dist(1.0, 3.0);
std::normal_distribution<float> amp_dist(0.2, 0.03);

/* MAIN */
void setup() {

    // Pendulum setup
    furataEncoder.write(0); // Init to 0

    // Odrive setup
    initialize_singleODCW();
    load_anticogging_config(CoggingConfig::cogging_map, ANTICOG_PERCENT_TO_USE, 0.0, 0.0);
}

void loop() {
    while (this_status_ok) {
        bool which = (bool)(which_dist(gen));
        if (which) {
            // Sinusoidal
            uint8_t entries = (uint8_t)(entries_dist(gen));
            OdriveSinusoidTorqueSID cur_st_sid{}; // zero-init
            cur_st_sid.n_components = entries;

            uint32_t max_freq = 0;
            for (uint8_t i = 0; i < entries; i++) {
                cur_st_sid.amps[i]  = amp_dist(gen)/(i+1);
                float f = freq_dist(gen);
                cur_st_sid.freqs[i] = f;
                if (f > max_freq) max_freq = f;
            }
            cur_st_sid.cycles = max_freq * 10;

            this_status_ok = perform_sid_with_pend_sinetorque(
                cur_st_sid, &thisKF, &furataEncoder, RAD_PER_PULSE, true);
        } else {
            // Step
            float freq = freq_dist(gen);
            float amp = amp_dist(gen);
            OdriveStepTorqueSID cur_st_sid = {amp, freq, (uint32_t) (freq*10)};
            this_status_ok = perform_sid_with_pend_steptorque(
                cur_st_sid, &thisKF, &furataEncoder, RAD_PER_PULSE, true);
        }
    }
    this_status_ok = false;
}