#include <OdriveCANWrapper.hpp>
#include <Arduino.h>

#define BASE_COGGING_VEL 0.01 // Gives ~100 ms at each encoder position
constexpr uint8_t n_runs = 2;
OdriveLinearPositionSID lp_sid_runs[n_runs] = {
    // { 10*BASE_COGGING_VEL, 10*10.0, 1},
    // { 5*BASE_COGGING_VEL, 5*10.0, 1},
    { BASE_COGGING_VEL, 4.0, 1}
};
bool status_ok = true;


/* MAIN */
void setup() {
    initialize_singleODCW();
}

void loop() {
    for (uint8_t i = 0; i < n_runs; i++) {
        status_ok = perform_linear_position_sid(lp_sid_runs[i]);

        while (!status_ok) {
            // Infinite loop if stop msg received
        }
    }
}
