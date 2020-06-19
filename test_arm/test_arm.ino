/*
This file is to test if your arm works
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN 102  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 512  // this is the 'maximum' pulse length count (out of 4096)


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int angles[360];


void setup() {
    Serial.begin(9600);
    Serial.println("Servo test!");
    pwm.begin();
    pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
    delay(1000);
    Serial.println("Init");
    for(int i=0; i<4; i++)
        pwm.setPWM(i, 0, angleToPulse(90));
}


void loop() {
    // init angle of all arms is 90
    Serial.println("Start");
    for(int i=0; i<4; i++) {
        int angle = 90;
        for(; angle<181 && (i!=1 || angle<135); ++angle) {
            delay(10);
            pwm.setPWM(i, 0, angleToPulse(angle));
        }
        for(; angle>0; --angle) {
            delay(10);
            pwm.setPWM(i, 0, angleToPulse(angle));
        }
        for(; angle<91; ++angle) {
            delay(10);
            pwm.setPWM(i, 0, angleToPulse(angle));
        }
    }
    Serial.println("OK");
    delay(1000);
}


int angleToPulse(int ang) {
    // map angle of 0 to 180 to Servo min and Servo max 
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    Serial.print("Angle: ");
    Serial.print(ang);
    Serial.print("Pulse: ");
    Serial.println(pulse);
    return pulse;
}
