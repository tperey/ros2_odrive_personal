#include <OdriveCANWrapper.hpp>
#include <Arduino.h>

#include <m8325s_furata/cogging_config.hpp>

uint32_t t_prev = 0;
float cur_count = 0;
#define ANTICOG_PERCENT_TO_USE 1.0 // No scaling seems to be smoothest - TP 2026-07-29

/* MAIN */
void setup() {
    initialize_singleODCW();
    load_anticogging_config(CoggingConfig::cogging_map, ANTICOG_PERCENT_TO_USE, 0.0, 0.0);
    t_prev = micros();
}

void loop() {
    static bool status_ok = true;
    while (status_ok) {
        // Enforce 1 Khz Loop
        uint32_t now = micros();
        if ( (now-t_prev) > 1000) {
            cur_count = command_odrv0_torque(0.0);  // Command oNLY feedforward to test smoothness
            sendTelemetry_singleODCW(cur_count); // Send telemetry
            status_ok = check_for_shutdown_msg();
        }
    }
}
