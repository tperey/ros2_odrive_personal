// #ifndef ODRIVE_CAN_WRAPPER_HPP
// #define ODRIVE_CAN_WRAPPER_HPP

// #include <cstdint>
// #include <Arduino.h>
// #include "ODriveCAN.h"
// #include <FlexCAN_T4.h>
// #include "ODriveFlexCAN.hpp"
// struct ODriveStatus; // hack to prevent teensy compile error

// #define DEFAULT_BAUD 250000
// #define ODRV0_NODE_ID 0

// /* COMMON STRUCTS */
// struct ODriveUserData {
//   Heartbeat_msg_t last_heartbeat;
//   bool received_heartbeat = false;
//   Get_Encoder_Estimates_msg_t last_feedback;
//   bool received_feedback = false;
// };

// /* COMMON CALLBACKS */
// // Called every time a Heartbeat message arrives from the ODrive
// void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);

// // Called every time a feedback message arrives from the ODrive
// void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

// // Called for every message that arrives on the CAN bus
// void onCanMessage(const CanMsg& msg);

// /* INIT FUNCS */
// // CAN
// bool setupCan();

// // GENERAL
// void ocw_init(uint8_t n_drives_ = 1, uint32_t baud_rate_ = DEFAULT_BAUD);
// void ocw_basic_sinusoid();

// #endif

// /* GLOBAL VARS */
// static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
// static ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
// static ODriveCAN** odrives; // Make sure all ODriveCAN instances are accounted for here
// static ODriveUserData odrv0_user_data; // Keep some application-specific user data for every ODrive.
// static uint8_t n_drives = 1;
// static uint32_t baud_rate = DEFAULT_BAUD;

// /* COMMON CALLBACKS */
// void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
//   ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
//   odrv_user_data->last_heartbeat = msg;
//   odrv_user_data->received_heartbeat = true;
// }

// void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
//   ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
//   odrv_user_data->last_feedback = msg;
//   odrv_user_data->received_feedback = true;
// }

// void onCanMessage(const CanMsg& msg) {
//   for (uint8_t i = 0; i < n_drives; ++i) {
//     ODriveCAN* odrive = odrives[i];
//     onReceive(msg, *odrive);
//   }
// }

// /* INIT FUNCS */
// // CAN
// bool setupCan() {
//   can_intf.begin();
//   can_intf.setBaudRate(baud_rate);
//   can_intf.setMaxMB(16);
//   can_intf.enableFIFO();
//   can_intf.enableFIFOInterrupt();
//   can_intf.onReceive(onCanMessage);
//   return true;
// }

// // GENERAL
// void ocw_init(uint8_t n_drives_, uint32_t baud_rate_) {

//   /* SET GLOBAL VARS */
//   // Save arguments
//   baud_rate = baud_rate_;
//   n_drives = n_drives_;

//   // Instantiate ODrive objects
//   odrives = new ODriveCAN*[n_drives];
//   for (uint8_t i = 0; i < n_drives; ++i) {
//     if (i==0) {
//       odrives[i] = &odrv0;
//     } else {
//       odrives[i] = new ODriveCAN(wrap_can_intf(can_intf), i); // i = node ID
//     }
//   }

//   /* SETUP COMMS*/
//   Serial.begin(115200);

//   // Wait for up to 3 seconds for the serial port to be opened on the PC side.
//   // If no PC connects, continue anyway.
//   for (int i = 0; i < 30 && !Serial; ++i) {
//     delay(100);
//   }
//   delay(200);


//   Serial.println("Starting ODriveCAN demo");

//   // Register callbacks for the heartbeat and encoder feedback messages
//   odrv0.onFeedback(onFeedback, &odrv0_user_data);
//   odrv0.onStatus(onHeartbeat, &odrv0_user_data);

//   // Configure and initialize the CAN bus interface. This function depends on
//   // your hardware and the CAN stack that you're using.
//   if (!setupCan()) {
//     Serial.println("CAN failed to initialize: reset required");
//     while (true); // spin indefinitely
//   }

//   Serial.println("Waiting for ODrive...");
//   while (!odrv0_user_data.received_heartbeat) {
//     pumpEvents(can_intf);
//   }

//   Serial.println("found ODrive");

//   // request bus voltage and current (1sec timeout)
//   Serial.println("attempting to read bus voltage and current");
//   Get_Bus_Voltage_Current_msg_t vbus;
//   if (!odrv0.request(vbus, 1000)) {
//     Serial.println("vbus request failed!");
//     while (true); // spin indefinitely
//   }

//   Serial.print("DC voltage [V]: ");
//   Serial.println(vbus.Bus_Voltage);
//   Serial.print("DC current [A]: ");
//   Serial.println(vbus.Bus_Current);

//   Serial.println("Enabling closed loop control...");
//   while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
//     odrv0.clearErrors();
//     delay(1);
//     odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

//     // Pump events for 150ms. This delay is needed for two reasons;
//     // 1. If there is an error condition, such as missing DC power, the ODrive might
//     //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
//     //    on the first heartbeat response, so we want to receive at least two
//     //    heartbeats (100ms default interval).
//     // 2. If the bus is congested, the setState command won't get through
//     //    immediately but can be delayed.
//     for (int i = 0; i < 15; ++i) {
//       delay(10);
//       pumpEvents(can_intf);
//     }
//   }

//   Serial.println("ODrive running!");
// }

// void ocw_basic_sinusoid() {
//   pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
//                         // Note that on MCP2515-based platforms, this will delay for a fixed 10ms.
//                         //
//                         // This has been found to reduce the number of dropped messages, however it can be removed
//                         // for applications requiring loop times over 100Hz.

//   float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

//   float t = 0.001 * millis();
  
//   float phase = t * (TWO_PI / SINE_PERIOD);

//   odrv0.setPosition(
//     sin(phase), // position
//     cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
//   );

//   // print position and velocity for Serial Plotter
//   if (odrv0_user_data.received_feedback) {
//     Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
//     odrv0_user_data.received_feedback = false;
//     Serial.print("odrv0-pos:");
//     Serial.print(feedback.Pos_Estimate);
//     Serial.print(",");
//     Serial.print("odrv0-vel:");
//     Serial.println(feedback.Vel_Estimate);
//   }
// }


#include <OdriveCANWrapper.hpp>
#include <Arduino.h>

/* MAIN */
void setup() {
    initialize_singleODCW();
}

void loop() {
    simplesine_singleODCW();
}
