#include <Encoder.h> // Include necessary libraries
#include <IntervalTimer.h>
#include "AccelKF.hpp"

#define KF_PACKET 19

#define TRANSMIT_PERIOD 1000 // [us]. 1000 is 1 KHz
#define BAUD_RATE 115200

#define DEG_PER_PULSE 360.0/2400 // Website says 600 P.R?
#define RAD_PER_PULSE (2.0 * 3.1415926535)/2400
// But maybe theres a gear ratio b/c empirically it was 2400
// Also this is (360 deg or 2pi rad/1 rev) * (1 rev / 2400 pulse)
int enc1Pin = 2;
int enc2Pin = 3;
// float current_rad;

Encoder furataEncoder(enc1Pin, enc2Pin);
IntervalTimer transmitTimer;

volatile bool sendRad = false;

// ----- Low Pass Filter stuff -----
#define CUTOFF_FREQ 4.0f // Hz 
#define DELTA_T 0.001f // [1/s]
const long inv_dt = 1000.0f; // [1/s]
float alpha;
volatile long current_speed = 0;
volatile float current_speed_f = 0.0;
volatile float filtered_speed_f = 0.0;
volatile long speed_to_transmit = 0;

// ---------- EKF parameters ----------
float dt = 0.001;  // sample time in seconds

// Process noise covariance
float Q[3][3] = {
  {1e-6, 0, 0},
  {0, 1e-2, 0},
  {0, 0, 10.0}
};

// Measurement noise (encoder)
const float R = 5.8e-7;

// Init EKF state 
float xi[3] = {0.0, 0.0, 0.0};       // [theta, theta_dot, theta_ddot]
float Pi[3][3] = { {0.01, 0.0, 0.0}, {0.0, 0.01, 0.0}, {0.0, 0.0, 0.01} };  // covariance

AccelKF thisKF = AccelKF(xi[0], xi[1], xi[2], R, Q, dt, Pi);


/* START CORE */

void setup() {
  // Init Kalman
  alpha = expf(-2.0f * PI * CUTOFF_FREQ * DELTA_T);

  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  
  // Wait until Python signals ready
  while (true) {
    if (Serial.available()) {
      byte b = Serial.read();
      if (b == 0x01) {  // Python startup byte
        break;
      }
    }
  }
  
  furataEncoder.write(0); // Init to 0

  // Trigger transmission with a timer
  transmitTimer.begin(encoderISR, TRANSMIT_PERIOD);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (sendRad) {

    // *** STRING TRANSMISSION ***
    // current_rad = current_pulses*RAD_PER_PULSE;
    // Serial.println(current_rad);

    // *** BYTE TRANSMISSION ***
    long current_pulses = furataEncoder.read();
    // sendPacket(current_pulses);
    sendKalman(current_pulses);
    sendRad = false;
  }
}

// *** Transmitting bytes ***
void sendPacket(long current_pulses) {

  static long last_pulses = furataEncoder.read();
  uint8_t packet[11];  // 2 start bytes + 4 pos + 4 velo + 1 checksum

  // Velocity estimation
  current_speed = (current_pulses - last_pulses)*inv_dt;
  filtered_speed_f = alpha*filtered_speed_f + (1.0-alpha)*current_speed;  // Will cast to float
  speed_to_transmit = long(filtered_speed_f);

  // ** Build Packet ** // 

  // Start bytes
  packet[0] = 0xAA;
  packet[1] = 0x55;

  // Copy 32-bit integer into packet (little-endian)
  memcpy(&packet[2], &current_pulses, 4);
  memcpy(&packet[6], (const void*) &speed_to_transmit, 4);

  // Simple checksum = sum of previous 6 bytes modulo 256
  uint8_t checksum = 0;
  for (int i = 0; i < 10; i++) checksum += packet[i];
  packet[10] = checksum;

  // Reset previous
  last_pulses = current_pulses;

  // Transmit
  Serial.write(packet, 11);
}

// *** ISR *** //
void encoderISR(void) {
  sendRad = true; // Only read and set flag, don't transmit in ISR
}

// *** KALMAN STUFF *** //
float counts_to_rads(long current_pulses) {
  return (float) (RAD_PER_PULSE*current_pulses);
}

// Transmitting Kalman
void sendKalman(long current_pulses) {

  static long last_pulses = furataEncoder.read();
  float theta_meas = counts_to_rads(current_pulses);
  uint8_t packet[KF_PACKET];  // 2 start bytes + 4 pos + 4 velo + 4 kpos + 4 kvelo +1 checksum

  // Velocity estimation
  current_speed = (current_pulses - last_pulses)*inv_dt;
  filtered_speed_f = alpha*filtered_speed_f + (1.0-alpha)*current_speed;  // Will cast to float
  speed_to_transmit = long(filtered_speed_f);

  // Kalman filter
  thisKF.update(theta_meas);

  // ** Build Packet ** // 

  // Start bytes
  packet[0] = 0xAA;
  packet[1] = 0x55;

  // Copy 32-bit integer into packet (little-endian)
  float pos = thisKF.getPosition();
  float vel = thisKF.getVelocity();
  float acc = thisKF.getAcceleration();
  memcpy(&packet[2], (const void*) &speed_to_transmit, 4);
  memcpy(&packet[6], &pos, 4);
  memcpy(&packet[10], &vel, 4);
  memcpy(&packet[14], &acc, 4);

  // Simple checksum = sum of previous bytes modulo 256
  uint8_t checksum = 0;
  for (int i = 0; i < 18; i++) checksum += packet[i];
  packet[18] = checksum;

  // Reset previous
  last_pulses = current_pulses;

  // Transmit
  Serial.write(packet, KF_PACKET);
}
