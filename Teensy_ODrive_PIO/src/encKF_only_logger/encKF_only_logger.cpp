#include <Encoder.h> // Include necessary libraries
#include <IntervalTimer.h>
#include <Arduino.h>
#include <AccelKF.hpp>

// Function declarations
void sendPacket(long current_pulses);
void encoderISR(void);
float counts_to_rads(long current_pulses);
void sendKalman(long current_pulses);

// Constants
#define KF_PACKET 47

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
bool started = false;

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
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);

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
  
  furataEncoder.write(0); // Init to 0

  // Trigger transmission with a timer
  transmitTimer.begin(encoderISR, TRANSMIT_PERIOD);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (sendRad) {
    // *** BYTE TRANSMISSION ***
    long current_pulses = furataEncoder.read();
    sendKalman(current_pulses);
    sendRad = false;
  }
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

  float theta_meas = counts_to_rads(current_pulses);
  uint8_t packet[KF_PACKET];  // 2 start bytes + 4 pos + 4 velo + 4 kpos + 4 kvelo +1 checksum

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
  memset(&packet[2], 0, KF_PACKET-2-12);
  memcpy(&packet[(KF_PACKET-1-12)], &pos, 4);
  memcpy(&packet[(KF_PACKET-1-8)], &vel, 4);
  memcpy(&packet[(KF_PACKET-1-4)], &acc, 4);

  // Simple checksum = sum of previous bytes modulo 256
  uint8_t checksum = 0;
  for (int i = 0; i < (KF_PACKET-1); i++) checksum += packet[i];
  packet[KF_PACKET-1] = checksum;

  // Transmit
  Serial.write(packet, KF_PACKET);
}
