#include <OdriveCANWrapper.hpp>
#include <Arduino.h>

#include <m8325s_furata/cogging_config.hpp>

#define SINE_TORQUE_BASE_AMP 0.08
#define ANTICOG_PERCENT_TO_USE 1.0 // No scaling seems to be smoothest - TP 2026-07-29
constexpr uint8_t n_runs = 4;
OdriveStepTorqueSID st_sid_runs[n_runs] = {
    {SINE_TORQUE_BASE_AMP, 0.75, 10},
    {SINE_TORQUE_BASE_AMP, 1.0, 10},
    {SINE_TORQUE_BASE_AMP, 2.0, 10},
    {2*SINE_TORQUE_BASE_AMP, 5.0, 20},
};
volatile static bool status_ok = true;

/* MAIN */
void setup() {
    initialize_singleODCW();
    load_anticogging_config(CoggingConfig::cogging_map, ANTICOG_PERCENT_TO_USE, 0.0, 0.0);
}

void loop() {
    for (uint8_t i = 0; i < n_runs; i++) {
        while (!status_ok) {
            // Infinite loop if stop msg received, or done
        }
        status_ok = perform_step_torque_sid(st_sid_runs[i], true);
    }
    status_ok = false; // Enforce stop
}
