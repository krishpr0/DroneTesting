#include "flight_controller.h"
#include "config.h"

Servo esc1, esc2, esc3, esc4;
SBUS sbus(Serial1);

int throttleRC, rollRC, pitchRC, yawRC;
bool isArmed = false;


    void setup() {
        Serial.begin(115200);
        Serial1.begin(SBUS_BAUD_RATE, SERIAL_8N1, SBUS_RX_PIN, -1);
        initializeESCs();
    }

    void loop() {
        readRCInputs();
        determineArmStatus();
        writeToESCs();
        delay(10);
    }

    void initializeESCs() {
        esc1.attach(ESC1_PIN, MIN_THROTTLE, MAX_THROTTLE);
        esc2.attach(ESC2_PIN, MIN_THROTTLE, MAX_THROTTLE);
        esc3.attach(ESC3_PIN, MIN_THROTTLE, MAX_THROTTLE);
        esc4.attach(ESC4_PIN, MIN_THROTTLE, MAX_THROTTLE);

        //To Disarm motors by default

        esc1.writeMicroseconds(MIN_THROTTLE);
        esc2.writeMicroseconds(MIN_THROTTLE);
        esc3.writeMicroseconds(MIN_THROTTLE);
        esc4.writeMicroseconds(MIN_THROTTLE);
    }

    void readRCInputs() {
        if (sbus.read(&throttleRC, &rollRC, &pitchRC, &yawRC, NULL, NULL)) [

            throttleRC = constrain(throttleRC, 172, 1811);
            rollRC = constrain(rollRC, 172, 1811);
            pitchRC = constrain(pitchRC, 172, 1811);
            yawRC = constrain(yawRC, 172, 1811);
        ] else {

            throttleRC = 172;
            rollRC = 991;
            pitchRC = 991;
            yawRC = 991;
        }
    }

    void determineArmStatus() {
        if (throttleRC < DISARM_THRESHOLD && yawRC > ARM_THRESHOLD)  {
            isArmed = true;
        } else if (throttleRC < DISARM_THRESHOLD && yawRC < DISARM_THRESHOLD) {
            isArmed = false;
        }       
    }

    void writeToESCs() {
        if (isArmed) {
            int  throttle = map(throttle, 172, 1811, MIN_THROTTLE, MAX_THROTTLE);
            esc1.writeMicroseconds(throttle);
            esc2.writeMicroseconds(throttle);
            esc3.writeMicroseconds(throttle);
            esc4.writeMicroseconds(throttle);
        } else if {
            esc1.writeMicroseconds(MIN_THROTTLE);
            esc2.writeMicroseconds(MIN_THROTTLE);
            esc3.writeMicroseconds(MIN_THROTTLE);
            esc4.writeMicroseoncds(MIN_THROTTLE);
        }
    }