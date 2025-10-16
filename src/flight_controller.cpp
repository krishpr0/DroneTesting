#include "flight_controller.h"

void setup() {
  Serial.begin(115200);
  Serial1.begin(SBUS_BAUD_RATE, SERIAL_8N1, SBUS_RX_PIN, -1); // RX only
  sbus_rx.Begin();
  initializeESCs();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED off (active low)
}

void loop() {
  readRCInputs();
  determineArmStatus();
  updateLED();
  writeToESCs();
  sendTelemetry();
  delay(10);
}

void initializeESCs() {
  esc1.attach(ESC1_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc2.attach(ESC2_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc3.attach(ESC3_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc4.attach(ESC4_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc1.writeMicroseconds(MIN_THROTTLE);
  esc2.writeMicroseconds(MIN_THROTTLE);
  esc3.writeMicroseconds(MIN_THROTTLE);
  esc4.writeMicroseconds(MIN_THROTTLE);
}

void readRCInputs() {
  bfs::SbusData data;
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    throttleRC = data.ch[0]; // Assuming channel 0 is throttle
    rollRC = data.ch[1];     // Assuming channel 1 is roll
    pitchRC = data.ch[2];    // Assuming channel 2 is pitch
    yawRC = data.ch[3];      // Assuming channel 3 is yaw
    isFailsafe = data.failsafe || data.lost_frame > 0; // Set failsafe if lost frames or failsafe triggered
    throttleRC = constrain(throttleRC, 172, 1811);
    rollRC = constrain(rollRC, 172, 1811);
    pitchRC = constrain(pitchRC, 172, 1811);
    yawRC = constrain(yawRC, 172, 1811);
  } else {
    throttleRC = 172; // Failsafe minimum
    rollRC = 991;
    pitchRC = 991;
    yawRC = 991;
    isFailsafe = true;
  }
}

void determineArmStatus() {
  if (isFailsafe) {
    isArmed = false;
  } else if (throttleRC < DISARM_THRESHOLD && yawRC > ARM_THRESHOLD) {
    isArmed = true;
  } else if (throttleRC < DISARM_THRESHOLD && yawRC < DISARM_THRESHOLD) {
    isArmed = false;
  }
}

void writeToESCs() {
  if (isArmed && !isFailsafe) {
    int throttle = map(throttleRC, 172, 1811, MIN_THROTTLE, MAX_THROTTLE);
    esc1.writeMicroseconds(throttle);
    esc2.writeMicroseconds(throttle);
    esc3.writeMicroseconds(throttle);
    esc4.writeMicroseconds(throttle);
  } else {
    esc1.writeMicroseconds(MIN_THROTTLE);
    esc2.writeMicroseconds(MIN_THROTTLE);
    esc3.writeMicroseconds(MIN_THROTTLE);
    esc4.writeMicroseconds(MIN_THROTTLE);
  }
}

void updateLED() {
  digitalWrite(LED_PIN, isArmed && !isFailsafe ? LOW : HIGH); // LED on when armed and no failsafe
}

void sendTelemetry() {
  Serial.print("Throttle: ");
  Serial.print(throttleRC);
  Serial.print(" | Armed: ");
  Serial.print(isArmed ? "Yes" : "No");
  Serial.print(" | Failsafe: ");
  Serial.println(isFailsafe ? "Yes" : "No");
}