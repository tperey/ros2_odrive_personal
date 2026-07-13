#include <OdriveCANWrapper.hpp>
#include <Arduino.h>

uint32_t t_prev = 0;

/* MAIN */
void setup() {
    initialize_singleODCW();
    t_prev = micros();
}

void loop() {
    static bool status_ok = true;
    while (status_ok) {
        // Enforce 1 Khz Loop
        uint32_t now = micros();
        if ( (now-t_prev) > 1000) {
            // Feels pointless to code these separately
            //status_ok = simplesine_pos_singleODCW();
            status_ok = simplesine_tau_singleODCW();
            t_prev = now;
        }
    }
}
