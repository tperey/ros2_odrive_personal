#ifndef ODRIVE_CAN_WRAPPER_HPP
#define ODRIVE_CAN_WRAPPER_HPP

#include <cstdint>
#include <cmath>
#include <AccelKF.hpp>
#include <Encoder.h>

// CONSTANTS
#define DEFAULT_BAUD 1000000
#define ODRV0_NODE_ID 0

#define SINGLE_TEL_PKCT_SIZE 35 // 4 bits * 8 telemetry entries + 2 start bits + 1 checksum bit
#define SING_PEND_TEL_PKCT_SIZE 47 // 4 bits * 11 telemetry entries + 2 start bits + 1 checksum bit

#define M8325S_TORQUE_CONSTANT 0.083  // [N-m/A]

// Arbitrary parameter access
#define NODE_ID_SHIFT 5
#define CMD_ID_SELECTOR 0x1F
#define CMD_RXSDO 0x04
#define CMD_TXSDO 0x05
#define OPCODE_READ 0x00
#define OPCODE_WRITE 0x01
#define ONBOARD_ENCODER0_RAW 598  // ID for CAN msg. Assumes FW version 0.6.11 (on my S1 as of 2026-08-22)

#define HOMED_STARTING_POS_ESTIMATE 0.3632275462150574  // What pos_estimate set to by Teensy on connection. Assumes manually homed.
// TODO: rather than manual, use startup absolute position
#define ENCODER_COUNTS_PER_REV 1024 //4096

/* PUBLIC STRUCTS */
// Linear torque sys id input
struct OdriveLinearTorqueSID {
    float amp = 0.11; // [N-m], pre-cogging
    float ramp_t = 2.0; // [s]
    float fall_t = 2.0; // [s]
    uint32_t cycles = 100.0; // [cycles]
};

// Sinusoidal torque sys id input
#define MAX_N_FREQS 20
struct OdriveSinusoidTorqueSID {
    float amps[MAX_N_FREQS];
    float freqs[MAX_N_FREQS];
    uint32_t cycles;
    uint8_t n_components = 0;
};

// Step torque sys id input
struct OdriveStepTorqueSID {
    float amp;
    float freq;
    uint32_t cycle;
};

// Linear POSITION sys id input
struct OdriveLinearPositionSID {
    float vel = 0.05; // [rev/s];
    float n_revs = 10.0; // Number of revs to go in each direction
    uint32_t cycles = 2; 
};

/* PUBLIC FUNCTIONS */
void initialize_singleODCW(uint8_t n_drives_ = 1, uint32_t baud_rate_ = DEFAULT_BAUD);
bool simplesine_pos_singleODCW();
bool simplesine_tau_singleODCW();
void perform_linear_torque_sid(OdriveLinearTorqueSID cur_lt_sid);
bool perform_sinusoidal_torque_sid(OdriveSinusoidTorqueSID cur_st_sid, bool deCog = false);
bool perform_step_torque_sid(OdriveStepTorqueSID cur_st_sid, bool deCog = false);
bool perform_sid_with_pend_sinetorque(OdriveSinusoidTorqueSID cur_st_sid, AccelKF* thisKF, Encoder* thisEncoder, float rad_per_pulse, bool deCog = false); // With pendulum
bool perform_sid_with_pend_steptorque(OdriveStepTorqueSID cur_st_sid, AccelKF* thisKF, Encoder* thisEncoder, float rad_per_pulse, bool deCog = false);
bool perform_linear_position_sid(OdriveLinearPositionSID cur_lp_sid);  // Likely most useful for personal cogging, so should turn Odrive's off

// Telemetry reads and writes
void sendTelemetry_singleODCW(float cmd);
bool check_for_shutdown_msg();

void load_anticogging_config(const float* cogging_map, float cogging_percentage = 0.0, float friction = 0.0, float friction_percentage = 0.0);
// poll = false when the caller already called poll_telemetry() this tick (state machine does)
float command_odrv0_torque(float tau_setpoint, bool poll = true);

/* ================================================================== */
/* FURATA STATE MACHINE SUPPORT                                       */
/* Plain-float interface: this module never includes a KF header.     */
/* ================================================================== */

// BLOCKING (>=150 ms, wait on heartbeat).
void start_motor_single();   // -> CLOSED_LOOP_CONTROL
void stop_motor_single();    // -> IDLE. Loops forever if the ODrive is dead!
void poll_telemetry();       // request feedback / Iq / torques (3 blocking requests, 10 ms timeout each)

// Fire-and-forget IDLE request. Safe to call when the ODrive may be unresponsive (FAULT entry).
void request_motor_idle_nonblocking();

// Motor feedback in the TELEMETRY frame (sign-flipped vs ODrive), radians.
struct OdriveFeedback {
    float pos_rad = 0.0f;  // t1
    float vel_rad = 0.0f;  // t1d
    bool  fresh   = false; // true if new since last call (else ZOH of last value)
};
// The ONLY consumer of received_feedback. Telemetry reads cached values, never the flag.
// Does NOT update/poll feedback. Simply returns values.
bool convert_current_odrv0_feedback(OdriveFeedback& out);

struct OdriveHealth {
    uint32_t axis_error         = 0;  // from heartbeat, 0 = OK
    uint8_t  axis_state         = 0;  // ODriveAxisState
    uint32_t ms_since_heartbeat = 0;
};
void read_current_odrv0_health(OdriveHealth& out);

// SM-side telemetry. sendTelemetry_furata appends the ODrive-side fields itself.
struct FurataTelemetry {
    uint8_t state    = 0;      // State enum
    uint8_t sm_fault    = 0;      // FaultCode enum
    float   tau_cmd  = 0.0f;   // [N-m] asked for by the active state
    float   x_hat[4] = {};     // [t1, dt2, t1d, t2d]
};
// 2 header + 4 t_ms + 1 state + 1 fault + 9 floats * 4 + 1 checksum
#define FURATA_TEL_PKCT_SIZE 45
void sendTelemetry_furata(const FurataTelemetry& tel);

#endif