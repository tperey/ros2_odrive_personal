#ifndef ODRIVE_CAN_WRAPPER_HPP
#define ODRIVE_CAN_WRAPPER_HPP

#include <cstdint>

// CONSTANTS
#define DEFAULT_BAUD 1000000
#define ODRV0_NODE_ID 0

#define SINGLE_TEL_PKCT_SIZE 35 // 4 bits * 8 telemetry entries + 2 start bits + 1 checksum bit

#define M8325S_TORQUE_CONSTANT 0.083  // [N-m/A]

#define CMD_RXSDO 0x04;
#define CMD_TXSDO 0x05;
#define OPCODE_READ 0x00;
#define OPCODE_WRITE 0x01;

/* PUBLIC FUNCTIONS */
void initialize_singleODCW(uint8_t n_drives_ = 1, uint32_t baud_rate_ = DEFAULT_BAUD);
bool simplesine_pos_singleODCW();
bool simplesine_tau_singleODCW();

/* PUBLIC STRUCTS */
// Linear torque sys id input
struct OdriveLinearTorqueSID {
    float amp = 0.05; // [N-m], pre-cogging
    float ramp_t = 5.0; // [s]
    float fall_t = 2.0; // [s]
    uint32_t cycles = 10.0; // [cycles]
};


#endif