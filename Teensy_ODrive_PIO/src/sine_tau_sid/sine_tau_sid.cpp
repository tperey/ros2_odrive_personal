#include <OdriveCANWrapper.hpp>
#include <Arduino.h>


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
    initialize_singleODCW();
}

void loop() {
    while (status_ok) {
        for (uint8_t i = 0; i < n_runs; i++) {
            status_ok = perform_sinusoidal_torque_sid(st_sid_runs[i]);
        }
    }
}
