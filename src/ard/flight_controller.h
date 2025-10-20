#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_HMC5883_U.h>
#include <TinyGPS++.h>
#include <PID_v1.h>
#include <ESP32Servo.h>
#include <sbus.h>
#include <MadgwickAHRS.h>
#include <EEPROM.h>
#include "config.h"

extern Adafruit_MPU6050 mpu;
extern Adafruit_BMP280 bmp;
extern Adafruit_HMC5883_Unified mag;
extern TinyGPSPlus gps;
extern HardwareSerial GPSSerial;
extern Servo esc1, esc2, esc3, esc4;
extern bfs::SbusRx sbus_rx;
extern Madgwick filter;

extern double rollInput, pitchInput, yawInput;
extern double rollSetpoint, pitchSetpoint, yawSetpoint;
extern double rollOutput, pitchOutput, yawOutput;
extern double altInput, altSetpoint, altOutput;
extern double latInput, lonInput, latSetpoint, lonSetpoint, latOutput, lonOutput;

extern PID rollPID, pitchPID, yawPID, altPID, posLatPID, posLonPID;

extern double homeLat, homeLon;
extern bool homeSet, isArmed;
extern FlightMode currentMode;
extern double prevRollD, prevPitchD, prevYawD, prevAltD, prevLatD, prevLonD;
extern int motor1, motor2, motor3, motor4;
extern bool isFailsafe;
extern int shakeCounter;
extern unsigned long lastRcUpdate, lastMspBroadcast;
extern int throttleRC, rollRC, pitchRC, yawRC, aux1, aux2;

void setup();
void loop();
void initializeI2C();
void initializeSensors();
void initializeRC();
void initializeESCs();
void initializePIDs();
void initializeFilter();
void readRCInputs();
void determineArmStatus();
void determineMode();
void readSensors();
void readGPS();
void setHomeIfNeeded();
void updateSetpoints();
void computePIDs();
void applyDFiltering();
void mixMotors();
void writeToESCs();
void disarmMotors();
void handleMSP();
void sendMSPTelemetry();

#endif