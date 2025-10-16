#include "flight_controller.h"

Servo esc1, esc2, esc3, esc4;
bfs::SbusRx sbus_rx(&Serial1, SBUS_RX_PIN, -1, false, false);

int throttleRC, rollRC, pitchRC, yawRC;
bool isArmed = false;
bool isFailsafe = false; // New variable for failsafe state