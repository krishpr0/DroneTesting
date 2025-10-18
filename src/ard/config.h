#ifndef CONFIG_H
#define CONFIG_H

// I2C Pins
#define I2C_SDA 21
#define I2C_SCL 22

// MPU6050 (Gyro/Accel)
#define MPU_ACCEL_RANGE MPU6050_RANGE_8_G
#define MPU_GYRO_RANGE MPU6050_RANGE_500_DEG
#define MPU_FILTER_BANDWIDTH MPU6050_BAND_21_HZ

// HMC5883L (Magnetometer)
#define MAG_ADDRESS 0x1E

// BMP280 (Barometer)
#define BMP_MODE MODE_NORMAL
#define BMP_TEMP_OVERSAMPLING SAMPLING_X2
#define BMP_PRESSURE_OVERSAMPLING SAMPLING_X16
#define BMP_FILTER FILTER_OFF
#define BMP_STANDBY_MS STANDBY_MS_1000
#define SEA_LEVEL_PRESSURE_HPA 1013.25

// GPS (NEO-6M)
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// PPM/SBUS
#define RC_PROTOCOL SBUS
#define SBUS_RX_PIN 4
#define SBUS_BAUD 100000

// ESCs
#define ESC1_PIN 5   // Front left (X config)
#define ESC2_PIN 18  // Front right
#define ESC3_PIN 19  // Rear right
#define ESC4_PIN 23  // Rear left
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000

// RC Channels
#define THROTTLE_CH 0
#define ROLL_CH 1
#define PITCH_CH 2
#define YAW_CH 3
#define AUX1_CH 4  // Arm switch
#define AUX2_CH 5  // Mode switch

// Thresholds
#define ARM_THRESHOLD 1700
#define DISARM_THRESHOLD 1300
#define IDLE_THROTTLE 1050
#define IDLE_THROTTLE_THRESHOLD 1100
#define RC_MIN 172
#define RC_MAX 1811
#define RC_NEUTRAL 991

// Flight Modes
enum FlightMode { NORMAL, HOVER, RTH };

// Safety
#define GPS_MIN_SATS 6
#define ALT_HOLD_TOL 1.0
#define POS_HOLD_TOL 5.0
#define SHAKE_THRESHOLD 20.0
#define SHAKE_COUNT 10

// PID Defaults
#define KP_ROLL 4.0
#define KI_ROLL 0.1
#define KD_ROLL 18.0
#define KP_PITCH 4.0
#define KI_PITCH 0.1
#define KD_PITCH 18.0
#define KP_YAW 4.0
#define KI_YAW 0.1
#define KD_YAW 18.0
#define KP_ALT 2.0
#define KI_ALT 0.05
#define KD_ALT 10.0
#define KP_POS 1.0
#define KI_POS 0.01
#define KD_POS 5.0
#define D_FILTER_ALPHA 0.95

// RTH
#define RTH_ALTITUDE 50.0
#define RTH_SPEED 10.0

// Filter
#define FILTER_SAMPLE_RATE 100.0

// MSP
#define MSP_BROADCAST_INTERVAL 100

// Loop
#define LOOP_DELAY_MS 10

#endif