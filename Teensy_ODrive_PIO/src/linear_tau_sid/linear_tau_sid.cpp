#include <OdriveCANWrapper.hpp>
#include <Arduino.h>

OdriveLinearTorqueSID lt_sid_run;
bool status_ok = true;

/* MAIN */
void setup() {
    initialize_singleODCW();
}

void loop() {
    if (status_ok) {
        perform_linear_torque_sid(lt_sid_run);
        status_ok = false;
    }
}
