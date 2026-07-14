#include "OdriveCANWrapper.hpp"
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // hack to prevent teensy compile error

/* COMMON STRUCTS */
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
  Get_Iq_msg_t last_currents;
  bool received_currents = false;
  Get_Torques_msg_t last_torques;
  bool received_torques = false;
};

/* FUNCTION PROTOTPYES */
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);
void onCanMessage(const CanMsg& msg);
bool setupCan();

/* GLOBAL VARS */
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
static ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
static ODriveCAN** odrives; // Make sure all ODriveCAN instances are accounted for here
static ODriveUserData odrv0_user_data; // Keep some application-specific user data for every ODrive.
static uint8_t n_drives = 1;
static uint32_t baud_rate = DEFAULT_BAUD;

/* COMMON CALLBACKS */
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

void onCurrents(Get_Iq_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_currents = msg;
  odrv_user_data->received_currents = true;
}

void onTorques(Get_Torques_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_torques = msg;
  odrv_user_data->received_torques = true;
}

void onCanMessage(const CanMsg& msg) {
  for (uint8_t i = 0; i < n_drives; ++i) {
    ODriveCAN* odrive = odrives[i];
    onReceive(msg, *odrive);
  }
}

/* ARBITRARY PARAMETERS */
// Write a float parameter (e.g. axis0.controller.config.vel_integrator_limit)
bool writeParamFloat(uint8_t node_id, uint16_t endpoint_id, float value) {
    CAN_message_t msg;
    msg.id = (node_id << 5) | CMD_RXSDO;
    msg.len = 8;
    msg.buf[0] = OPCODE_WRITE;
    msg.buf[1] = endpoint_id & 0xFF;
    msg.buf[2] = (endpoint_id >> 8) & 0xFF;
    msg.buf[3] = 0; // reserved
    memcpy(&msg.buf[4], &value, sizeof(value));
    return can_intf.write(msg);
}

// Send a read request; the reply must be caught in your onCanMessage handler
bool requestParamRead(uint8_t node_id, uint16_t endpoint_id) {
    CAN_message_t msg;
    msg.id = (node_id << 5) | CMD_RXSDO;
    msg.len = 4;
    msg.buf[0] = OPCODE_READ;
    msg.buf[1] = endpoint_id & 0xFF;
    msg.buf[2] = (endpoint_id >> 8) & 0xFF;
    msg.buf[3] = 0;
    return can_intf.write(msg);
}


/* INIT FUNCS */
// CAN
bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(baud_rate);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

// GENERAL
bool started = false;
void initialize_singleODCW(uint8_t n_drives_, uint32_t baud_rate_) {

  /* SET GLOBAL VARS */
  // Save arguments
  baud_rate = baud_rate_;
  n_drives = n_drives_;

  // Instantiate ODrive objects
  odrives = new ODriveCAN*[n_drives];
  for (uint8_t i = 0; i < n_drives; ++i) {
    if (i==0) {
      odrives[i] = &odrv0;
    } else {
      odrives[i] = new ODriveCAN(wrap_can_intf(can_intf), i); // i = node ID
    }
  }

  /* SETUP COMMS*/
  Serial.begin(115200);
  Serial.println("Started Teensy...");

  Serial.println("~~~~~~~~~~Starting ODriveCAN~~~~~~~~~~");

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  // odrv0.onFeedback(onFeedback, &odrv0_user_data); // Experiments showed we don't want to rely on Odrive for telemetry
  // odrv0.onCurrents(onCurrents, &odrv0_user_data);
  // odrv0.onTorques(onTorques, &odrv0_user_data); 

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
  }

  Serial.println("found ODrive");

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1000)) {
    Serial.println("vbus request failed!");
    while (true); // spin indefinitely
  }

  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  odrv0.setAbsolutePosition(HOMED_STARTING_POS_ESTIMATE); // Zero starting position

  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("~~~~~~~~~~ODrive running!~~~~~~~~~~");

  // Wait for startup msg from PC. Expect 0x01
  while (!started) {
    if (Serial.available() > 0) {
      uint8_t byte_in = Serial.read();

      if (byte_in == 0x01) {
        started = true;

        // Hack - to make sure computer sees
        Serial.write(0x10);
      }
    }
  }
}

/* TELEMETRY*/

void poll_telemetry() {
  // Experimentation showed that relying on Odrive autosending didn't work
  Get_Encoder_Estimates_msg_t feedback;
  if (odrv0.request(feedback, 10)) {
    odrv0_user_data.last_feedback = feedback;
    odrv0_user_data.received_feedback = true;
  }
  Get_Iq_msg_t currents;
  if (odrv0.request(currents, 10)) {
    odrv0_user_data.last_currents = currents;
    odrv0_user_data.received_currents = true;
  }
  Get_Torques_msg_t torques;
  if (odrv0.request(torques, 10)) {
    odrv0_user_data.last_torques = torques;
    odrv0_user_data.received_torques = true;
  }
}


void sendTelemetry_singleODCW(float cmd) {
  poll_telemetry();

  // Get most recent telemetry
  // ZOH if nothing
  uint32_t t_ms = millis();
  static float cur_Pos_Estimate = 0.0;
  static float cur_Vel_Estimate = 0.0;
  static float cur_Iq_Setpoint = 0.0;
  static float cur_Iq_Measured = 0.0;
  static float cur_Tau_Target = 0.0;
  static float cur_Tau_Estimate = 0.0;
  // Flip ODrive frame of reference sign (except for currents, which are already reverse)
  if (odrv0_user_data.received_feedback) {
    cur_Pos_Estimate = -1*odrv0_user_data.last_feedback.Pos_Estimate;
    cur_Vel_Estimate = -1*odrv0_user_data.last_feedback.Vel_Estimate;
    odrv0_user_data.received_feedback = false;
  }
  if (odrv0_user_data.received_currents) {
    cur_Iq_Setpoint = odrv0_user_data.last_currents.Iq_Setpoint;
    cur_Iq_Measured = odrv0_user_data.last_currents.Iq_Measured;
    odrv0_user_data.received_currents = false;
  }
  if (odrv0_user_data.received_torques) {
    cur_Tau_Target = -1*odrv0_user_data.last_torques.Torque_Target;
    cur_Tau_Estimate = -1*odrv0_user_data.last_torques.Torque_Estimate;
    odrv0_user_data.received_torques = false;
  }

  // Build packet
  uint8_t packet[SINGLE_TEL_PKCT_SIZE];

  packet[0] = 0xAA;  // Start bits
  packet[1] = 0x55;

  memcpy(&packet[2], &t_ms, 4);  // Telemetry
  memcpy(&packet[6], &cur_Pos_Estimate, 4);
  memcpy(&packet[10], &cur_Vel_Estimate, 4);
  memcpy(&packet[14], &cur_Iq_Setpoint, 4);
  memcpy(&packet[18], &cur_Iq_Measured, 4);
  memcpy(&packet[22], &cur_Tau_Target, 4);
  memcpy(&packet[26], &cur_Tau_Estimate, 4);
  memcpy(&packet[30], &cmd, 4);  // Units can vary

  uint8_t checksum = 0; // Simple checksum = sum of previous bytes modulo 256
  for (int i = 0; i < (SINGLE_TEL_PKCT_SIZE-1); i++) checksum += packet[i];
  packet[(SINGLE_TEL_PKCT_SIZE-1)] = checksum;

  Serial.write(packet, SINGLE_TEL_PKCT_SIZE);  // Send packet

}

/* SAFE SHUTDOWN */

void stop_motor_single() {
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);

    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }
}

void start_motor_single() {
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }
}

// Called cyclically to stop control if logger signals end
bool check_for_shutdown_msg() {
  static String incoming = "";
  bool status_ok = true;

  // Check for "stop"
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      incoming.trim();  // strips \r and any whitespace
      if (incoming == "stop") {
        // handle stop condition
        status_ok = false;
        stop_motor_single();
      }
      incoming = "";  // reset for next run. Probably doesn't really matter
    } else {
      incoming += c;
    }
  }

  return status_ok;
}

/* BASIC TRAJECTORIES, SINGLE MOTOR */

bool simplesine_pos_singleODCW() {

  // Set control mode on first call
  static bool is_first = true;
  if (is_first) {
    odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    is_first = false;
  }

  float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

  float t = 0.001 * millis();  
  float phase = t * (TWO_PI / SINE_PERIOD);

  odrv0.setPosition(
    sin(phase), // position
    cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
  );

  // print telemtry
  sendTelemetry_singleODCW(sin(phase));

  // Always check for shutdown
  return check_for_shutdown_msg();
}

// Basic torque sinewave
bool simplesine_tau_singleODCW() {

  // Set control mode on first call
  static bool is_first = true;
  if (is_first) {
    odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    is_first = false;
  }

  float AMP = 0.1;  // [N-m]
  float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

  float t = 0.001 * millis();
  float phase = t * (TWO_PI / SINE_PERIOD);

  odrv0.setTorque(AMP*sin(phase));

  // print telemtry
  sendTelemetry_singleODCW(AMP*sin(phase));

  // Always check for shutdown
  return check_for_shutdown_msg();
}

void perform_linear_torque_sid(OdriveLinearTorqueSID cur_lt_sid) {

  // Initialize
  odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  float t0 = 0.001*millis(); // [s]
  uint32_t cycle_n = 0;
  bool is_rising = true;
  uint32_t t_prev = micros();
  bool status_ok = true;  // Used to allow stop in logger

  while ((cycle_n < cur_lt_sid.cycles) && (status_ok)) {
    float tau_setpoint = 0.0;  // [N-m]
    float phase_t = 0.001*millis() - t0;  // [s]

    // Enforce 1 kHz cycle
    uint32_t now = micros();
    if ( (now - t_prev) > 1000) {
      if (is_rising) {
        tau_setpoint = -(cur_lt_sid.amp/cur_lt_sid.ramp_t)*phase_t;
        odrv0.setTorque(tau_setpoint);

        if (phase_t > cur_lt_sid.ramp_t) {
          // End of rise. Begin fall
          odrv0.setTorque(0.0);
          stop_motor_single(); // Necessary, otherwise cogging comp keeps it moving
          is_rising = false;
          t0 = 0.001*millis(); 
        }

      } else {

        if (phase_t > cur_lt_sid.fall_t) {
          // End of fall, move to next cycle
          cycle_n++;

          odrv0.setTorque(0.0);
          start_motor_single();
          is_rising = true;
          t0 = 0.001*millis(); 
        }
      }

      // Per 1 kHz actions
      sendTelemetry_singleODCW(tau_setpoint);
      status_ok = check_for_shutdown_msg();
      t_prev = now;
    }
  }

  // At the end, shutdown
  stop_motor_single();
}

// Sinusoidal torque sysid
#define PAUSE_TIME 2.5 // [s]
bool perform_sinusoidal_torque_sid(OdriveSinusoidTorqueSID cur_st_sid) {

  // Initialize
  start_motor_single();
  odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  float t0 = 0.001*millis(); // [s]
  uint32_t cycle_n = 0;
  bool is_cycling = true;
  uint32_t t_prev = micros();
  bool status_ok = true;  // Used to allow stop in logger
  bool pause_on = true;

  while ((pause_on) && (status_ok)) {
    float tau_setpoint = 0.0;  // [N-m]
    float phase_t = 0.001*millis() - t0;  // [s]

    // Enforce 1 kHz cycle
    uint32_t now = micros();
    if ( (now - t_prev) > 1000) {
      if (is_cycling) {

        // Compute sinusoidal torque
        for (uint8_t i = 0; i < cur_st_sid.n_components; i++) {
          tau_setpoint += cur_st_sid.amps[i]*sin(2*M_PI*cur_st_sid.freqs[i]*phase_t);
        }

        // Check for cycle, using lead frequency
        float cycle_fraction = cur_st_sid.freqs[0]*phase_t;  // No. cycles, with decimals
        if ((cycle_fraction - float(cycle_n)) > 1.0) {
          cycle_n++; // If fraction > 1.0 above cycle_n, a cycle has occurred
        }

        // Check for end
        if (cycle_n > cur_st_sid.cycles) {
          odrv0.setTorque(0.0);
          stop_motor_single(); // Necessary, otherwise cogging comp keeps it moving
          is_cycling = false;
          t0 = 0.001*millis(); 
        } else {
          odrv0.setTorque(tau_setpoint);
        }

      } else {
        if (phase_t > PAUSE_TIME) {
          // Cause break and return
          pause_on = false; 
        }
      }

      // Per 1 kHz actions
      sendTelemetry_singleODCW(tau_setpoint);
      status_ok = check_for_shutdown_msg();
      t_prev = now;
    }
  }

  // At the end, ensure shutdown
  stop_motor_single();

  return status_ok;
}