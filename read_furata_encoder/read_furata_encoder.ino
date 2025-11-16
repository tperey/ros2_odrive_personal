#include <Encoder.h> // Include necessary libraries
#include <TimerOne.h>

#define TRANSMIT_PERIOD 1000 // [us]
// 1 ms period, or 1 kHz tranmission
#define BAUD_RATE 115200

#define DEG_PER_PULSE 360.0/2400 // Website says 600 P.R?
#define RAD_PER_PULSE (2.0 * 3.1415926535)/2400
// But maybe theres a gear ratio b/c empirically it was 2400
// Also this is (360 deg or 2pi rad/1 rev) * (1 rev / 2400 pulse)
int enc1Pin = 2;
int enc2Pin = 3;
long current_pulses;
float current_deg;

Encoder furataEncoder(enc1Pin, enc2Pin);

volatile bool sendDeg = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  furataEncoder.write(0); // Init to 0

  // Trigger transmission with a timer
  Timer1.initialize(TRANSMIT_PERIOD);
  Timer1.attachInterrupt(encoderISR);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (sendDeg) {
    current_deg = current_pulses*DEG_PER_PULSE;
    Serial.println(current_deg);

    sendDeg = false;
  }
}

// *** ISR *** //
void encoderISR(void) {
  current_pulses = furataEncoder.read();
  sendDeg = true; // Only read and set flag, don't transmit in ISR
}
