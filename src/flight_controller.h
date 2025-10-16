#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "sbus.h"
#include "config.h"

extern Servo esc1, esc2, esc3, esc4;
extern bfs::SbusRx sbus_rx;

extern int throttleRC, rollRC, pitchRC, yawRC;
extern bool isArmed;
extern bool isFailsafe;

void setup();
void loop();
void initializeESCs();
void readRCInputs();
void determineArmStatus();
void writeToESCs();
void updateLED();
void sendTelemetry();

#endif