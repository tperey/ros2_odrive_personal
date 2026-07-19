#include <OdriveCANWrapper.hpp>
#include <Arduino.h>

OdriveLinearPositionSID lp_sid_run;
bool status_ok = true;

/* MAIN */
void setup() {
    initialize_singleODCW();
}

void loop() {
    if (status_ok) {
        perform_linear_position_sid(lp_sid_run);
        status_ok = false;
    }
}
