#include <OdriveCANWrapper.hpp>
#include <Arduino.h>

uint32_t t_prev = 0;

/* MAIN */
void setup() {
    initialize_singleODCW();
    t_prev = micros();
}

void loop() {
    // Enforce 1 Khz Loop
    uint32_t now = micros();
    if ( (now-t_prev) > 1000) {
        simplesine_singleODCW();
        t_prev = now;
    }
}
