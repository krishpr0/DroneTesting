#include "flight_controller.h"

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  initializeI2C();
  initializeSensors();
  initializeRC();
  initializeESCs();
  initializePIDs();
  initializeFilter();
}

void loop() {
  readRCInputs();
  readSensors();
  readGPS();
  determineArmStatus();
  if (!isArmed) {
    disarmMotors();
    return;
  }
  determineMode();
  setHomeIfNeeded();
  updateSetpoints();
  computePIDs();
  applyDFiltering();
  mixMotors();
  writeToESCs();
  handleMSP();
  sendMSPTelemetry();
  delay(LOOP_DELAY_MS);
}

void initializeI2C() {
  Wire.begin(I2C_SDA, I2C_SCL);
}

void initializeSensors() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1);
  }
  mpu.setAccelerometerRange(MPU_ACCEL_RANGE);
  mpu.setGyroRange(MPU_GYRO_RANGE);
  mpu.setFilterBandwidth(MPU_FILTER_BANDWIDTH);

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found");
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_OFF, Adafruit_BMP280::STANDBY_MS_1000);

  if (!mag.begin()) {
    Serial.println("HMC5883L not found");
    while (1);
  }

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
}

void initializeRC() {
  Serial1.begin(SBUS_BAUD, SERIAL_8N1, SBUS_RX_PIN, -1);
  sbus_rx.Begin();
}

void initializeESCs() {
  esc1.attach(ESC1_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc2.attach(ESC2_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc3.attach(ESC3_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc4.attach(ESC4_PIN, MIN_THROTTLE, MAX_THROTTLE);
  disarmMotors();
}

void initializePIDs() {
  rollPID = PID(&rollInput, &rollOutput, &rollSetpoint, KP_ROLL, KI_ROLL, KD_ROLL, DIRECT);
  pitchPID = PID(&pitchInput, &pitchOutput, &pitchSetpoint, KP_PITCH, KI_PITCH, KD_PITCH, DIRECT);
  yawPID = PID(&yawInput, &yawOutput, &yawSetpoint, KP_YAW, KI_YAW, KD_YAW, DIRECT);
  altPID = PID(&altInput, &altOutput, &altSetpoint, KP_ALT, KI_ALT, KD_ALT, DIRECT);
  posLatPID = PID(&latInput, &latOutput, &latSetpoint, KP_POS, KI_POS, KD_POS, DIRECT);
  posLonPID = PID(&lonInput, &lonOutput, &lonSetpoint, KP_POS, KI_POS, KD_POS, DIRECT);

  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);
  altPID.SetMode(AUTOMATIC);
  posLatPID.SetMode(AUTOMATIC);
  posLonPID.SetMode(AUTOMATIC);

  rollPID.SetOutputLimits(-500, 500);
  pitchPID.SetOutputLimits(-500, 500);
  yawPID.SetOutputLimits(-500, 500);
  altPID.SetOutputLimits(-500, 500);
  posLatPID.SetOutputLimits(-500, 500);
  posLonPID.SetOutputLimits(-500, 500);
}

void initializeFilter() {
  filter.begin(FILTER_SAMPLE_RATE);
}

void readRCInputs() {
  bfs::SbusData data;
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    throttleRC = data.ch[THROTTLE_CH];
    rollRC = data.ch[ROLL_CH];
    pitchRC = data.ch[PITCH_CH];
    yawRC = data.ch[YAW_CH];
    aux1 = data.ch[AUX1_CH];
    aux2 = data.ch[AUX2_CH];
    isFailsafe = data.failsafe || data.lost_frame > 0;
  } else {
    throttleRC = RC_MIN;
    rollRC = RC_NEUTRAL;
    pitchRC = RC_NEUTRAL;
    yawRC = RC_NEUTRAL;
    aux1 = RC_MIN;
    aux2 = RC_MIN;
    isFailsafe = true;
  }
}

void determineArmStatus() {
  if (isFailsafe) {
    isArmed = false;
    return;
  }
  if (throttleRC < IDLE_THROTTLE_THRESHOLD && aux1 > ARM_THRESHOLD) {
    isArmed = true;
  } else if (throttleRC < IDLE_THROTTLE_THRESHOLD && aux1 < DISARM_THRESHOLD) {
    isArmed = false;
  }
}

void determineMode() {
  if (aux2 < 1000) currentMode = NORMAL;
  else if (aux2 < 1500) currentMode = HOVER;
  else currentMode = RTH;
}

void readSensors() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sensors_event_t event;
  mag.getEvent(&event);
  filter.update(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, event.magnetic.x, event.magnetic.y, event.magnetic.z);
  rollInput = filter.getRoll();
  pitchInput = filter.getPitch();
  yawInput = filter.getYaw();

  altInput = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
}

void readGPS() {
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read())) {
      if (gps.location.isValid()) {
        latInput = gps.location.lat();
        lonInput = gps.location.lng();
      }
    }
  }
}

void setHomeIfNeeded() {
  if (!homeSet && gps.location.isValid() && gps.satellites.value() >= GPS_MIN_SATS) {
    homeLat = latInput;
    homeLon = lonInput;
    homeSet = true;
  }
}

void updateSetpoints() {
  rollSetpoint = map(rollRC, RC_MIN, RC_MAX, -30.0, 30.0); // Degrees
  pitchSetpoint = map(pitchRC, RC_MIN, RC_MAX, -30.0, 30.0); // Degrees
  yawSetpoint = yawRC; // Raw for rate control
  altSetpoint = (currentMode == RTH) ? RTH_ALTITUDE : map(throttleRC, RC_MIN, RC_MAX, 0.0, 100.0);
  if (currentMode == HOVER || currentMode == RTH) {
    latSetpoint = homeLat;
    lonSetpoint = homeLon;
  } else {
    latSetpoint = latInput;
    lonSetpoint = lonInput;
  }
}

void computePIDs() {
  rollPID.Compute();
  pitchPID.Compute();
  yawPID.Compute();
  altPID.Compute();
  posLatPID.Compute();
  posLonPID.Compute();
}

void applyDFiltering() {
  rollOutput = prevRollD * D_FILTER_ALPHA + rollOutput * (1 - D_FILTER_ALPHA);
  pitchOutput = prevPitchD * D_FILTER_ALPHA + pitchOutput * (1 - D_FILTER_ALPHA);
  yawOutput = prevYawD * D_FILTER_ALPHA + yawOutput * (1 - D_FILTER_ALPHA);
  altOutput = prevAltD * D_FILTER_ALPHA + altOutput * (1 - D_FILTER_ALPHA);
  latOutput = prevLatD * D_FILTER_ALPHA + latOutput * (1 - D_FILTER_ALPHA);
  lonOutput = prevLonD * D_FILTER_ALPHA + lonOutput * (1 - D_FILTER_ALPHA);

  prevRollD = rollOutput;
  prevPitchD = pitchOutput;
  prevYawD = yawOutput;
  prevAltD = altOutput;
  prevLatD = latOutput;
  prevLonD = lonOutput;
}

void mixMotors() {
  int throttle = map(throttleRC, RC_MIN, RC_MAX, MIN_THROTTLE, MAX_THROTTLE);
  motor1 = throttle + rollOutput - pitchOutput + yawOutput;
  motor2 = throttle - rollOutput - pitchOutput - yawOutput;
  motor3 = throttle + rollOutput + pitchOutput - yawOutput;
  motor4 = throttle - rollOutput + pitchOutput + yawOutput;
  motor1 = constrain(motor1, MIN_THROTTLE, MAX_THROTTLE);
  motor2 = constrain(motor2, MIN_THROTTLE, MAX_THROTTLE);
  motor3 = constrain(motor3, MIN_THROTTLE, MAX_THROTTLE);
  motor4 = constrain(motor4, MIN_THROTTLE, MAX_THROTTLE);
}

void writeToESCs() {
  if (isArmed && !isFailsafe) {
    esc1.writeMicroseconds(motor1);
    esc2.writeMicroseconds(motor2);
    esc3.writeMicroseconds(motor3);
    esc4.writeMicroseconds(motor4);
  } else {
    esc1.writeMicroseconds(MIN_THROTTLE);
    esc2.writeMicroseconds(MIN_THROTTLE);
    esc3.writeMicroseconds(MIN_THROTTLE);
    esc4.writeMicroseconds(MIN_THROTTLE);
  }
}

void disarmMotors() {
  esc1.writeMicroseconds(MIN_THROTTLE);
  esc2.writeMicroseconds(MIN_THROTTLE);
  esc3.writeMicroseconds(MIN_THROTTLE);
  esc4.writeMicroseconds(MIN_THROTTLE);
  isArmed = false;
}

void handleMSP() {
  if (Serial.available() >= 6) {
    if (Serial.read() != 0x58) return;
    uint8_t dir = Serial.read();
    uint8_t size = Serial.read();
    uint8_t cmd = Serial.read();
    if (Serial.available() < size + 1) return;
    uint8_t data[size];
    Serial.readBytes(data, size);
    uint8_t crc = Serial.read();
    uint8_t calc_crc = 0x58 ^ dir ^ size ^ cmd;
    for (int i = 0; i < size; i++) calc_crc ^= data[i];
    if (calc_crc != crc) return;

    if (dir == 0) {
      uint8_t resp[32];
      uint8_t resp_size = 0;
      switch (cmd) {
        case 30: { // MSP_ATTITUDE
          resp_size = 6;
          int16_t roll = (int16_t)(rollInput * 10);
          int16_t pitch = (int16_t)(pitchInput * 10);
          int16_t yaw = (int16_t)(yawInput * 10);
          memcpy(resp, &roll, 2);
          memcpy(resp + 2, &pitch, 2);
          memcpy(resp + 4, &yaw, 2);
          break;
        }
        case 101: { // MSP_STATUS
          resp_size = 2;
          resp[0] = isArmed ? 1 : 0;
          resp[1] = (uint8_t)currentMode;
          break;
        }
      }
      if (resp_size > 0) {
        uint8_t resp_crc = 0x58 ^ 1 ^ resp_size ^ cmd;
        for (int i = 0; i < resp_size; i++) resp_crc ^= resp[i];
        Serial.write(0x58); Serial.write(1); Serial.write(resp_size); Serial.write(cmd);
        Serial.write(resp, resp_size); Serial.write(resp_crc);
      }
    }
  }
}

void sendMSPTelemetry() {
  if (millis() - lastMspBroadcast < MSP_BROADCAST_INTERVAL) return;
  uint8_t buf[32];
  buf[0] = 0x58; buf[1] = 1; buf[2] = 6; buf[3] = 30;
  int16_t roll = (int16_t)(rollInput * 10);
  int16_t pitch = (int16_t)(pitchInput * 10);
  int16_t yaw = (int16_t)(yawInput * 10);
  memcpy(buf + 4, &roll, 2);
  memcpy(buf + 6, &pitch, 2);
  memcpy(buf + 8, &yaw, 2);
  uint8_t crc = 0x58 ^ 1 ^ 6 ^ 30;
  for (int i = 4; i < 10; i++) crc ^= buf[i];
  buf[10] = crc;
  Serial.write(buf, 11);

  buf[2] = 2; buf[3] = 101;
  buf[4] = isArmed ? 1 : 0;
  buf[5] = (uint8_t)currentMode;
  crc = 0x58 ^ 1 ^ 2 ^ 101 ^ buf[4] ^ buf[5];
  buf[6] = crc;
  Serial.write(buf, 7);

  lastMspBroadcast = millis();
}