#include "flight_controller.h"

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
Servo esc1, esc2, esc3, esc4;
bfs::SbusRx sbus_rx(&Serial1, SBUS_RX_PIN, -1, false, false);
Madgwick filter;

double rollInput = 0, pitchInput = 0, yawInput = 0;
double rollSetpoint = 0, pitchSetpoint = 0, yawSetpoint = 0;
double rollOutput = 0, pitchOutput = 0, yawOutput = 0;
double altInput = 0, altSetpoint = 0, altOutput = 0;
double latInput = 0, lonInput = 0, latSetpoint = 0, lonSetpoint = 0, latOutput = 0, lonOutput = 0;

PID rollPID(&rollInput, &rollOutput, &rollSetpoint, KP_ROLL, KI_ROLL, KD_ROLL, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, KP_PITCH, KI_PITCH, KD_PITCH, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, KP_YAW, KI_YAW, KD_YAW, DIRECT);
PID altPID(&altInput, &altOutput, &altSetpoint, KP_ALT, KI_ALT, KD_ALT, DIRECT);
PID posLatPID(&latInput, &latOutput, &latSetpoint, KP_POS, KI_POS, KD_POS, DIRECT);
PID posLonPID(&lonInput, &lonOutput, &lonSetpoint, KP_POS, KI_POS, KD_POS, DIRECT);

double homeLat = 0, homeLon = 0;
bool homeSet = false, isArmed = false;
FlightMode currentMode = NORMAL;
double prevRollD = 0, prevPitchD = 0, prevYawD = 0, prevAltD = 0, prevLatD = 0, prevLonD = 0;
int motor1 = MIN_THROTTLE, motor2 = MIN_THROTTLE, motor3 = MIN_THROTTLE, motor4 = MIN_THROTTLE;
bool isFailsafe = false;
int shakeCounter = 0;
unsigned long lastRcUpdate = 0, lastMspBroadcast = 0;
int throttleRC = 0, rollRC = 0, pitchRC = 0, yawRC = 0, aux1 = 0, aux2 = 0;