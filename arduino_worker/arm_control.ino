#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN  102 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  512 // this is the 'maximum' pulse length count (out of 4096)


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Move Arm
void armMove(int device, int angle) {
    pwm.setPWM(device, 0, angleToPulse(angle));
}


// Reset arm angle to init state
void armReset() {
    armMove(0, 90);
    armMove(1, 90);
    armMove(2, 90);
    armMove(3, 90);
}


// Map angle of 0 to 180 to Servo min and Servo max
int angleToPulse(int ang) {
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    // sprintf(tmp, "Angle %d Pulse %d", ang, pulse);
    // myPrint("debug", tmp);
    return pulse;
}

void armSetup() {
    pwm.begin();
    pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
    armReset();
}
