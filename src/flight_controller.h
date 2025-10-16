#ifdef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <SBUS.h>

extern Servo esc1, esc2, esc3, esc4;
extern SBUS sbus;

extern int throttleRC, rollRC, pitchRC, yawRC;
extern bool isArmed;

void setup();
void loop();
void initializeESCs();
void readRCInputs();
void determineArmStatus();
void writeToESCs();

#endif