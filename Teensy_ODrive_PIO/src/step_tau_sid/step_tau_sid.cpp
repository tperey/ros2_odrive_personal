#include <OdriveCANWrapper.hpp>
#include <Arduino.h>


#define SINE_TORQUE_BASE_AMP 0.08
constexpr uint8_t n_runs = 2;
OdriveStepTorqueSID st_sid_runs[n_runs] = {
    {SINE_TORQUE_BASE_AMP, 1.0, 5},
    {SINE_TORQUE_BASE_AMP, 2.0, 5},
};
bool status_ok = true;

/* MAIN */
void setup() {
    initialize_singleODCW();
}

void loop() {
    for (uint8_t i = 0; i < n_runs; i++) {
        status_ok = perform_step_torque_sid(st_sid_runs[i]);

        while (!status_ok) {
            // Infinite loop if stop msg received
        }
    }
}
