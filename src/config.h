#ifndef CONFIG_H
#define CONFIG_H

// ESC Pins
#define ESC1_PIN 13
#define ESC2_PIN 12
#define ESC3_PIN 14
#define ESC4_PIN 27
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000

// SBUS Settings
#define SBUS_RX_PIN 16
#define SBUS_BAUD_RATE 100000

// General
#define ARM_THRESHOLD 1700
#define DISARM_THRESHOLD 1300

// LED
#define LED_PIN 2  // Built-in LED on ESP32 (active low)

#endif