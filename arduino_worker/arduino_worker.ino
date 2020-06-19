#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN  102 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  512 // this is the 'maximum' pulse length count (out of 4096)


// our servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// variable for reading input
char input_str[100];
int input_num = 0;
int input_byte = 0;
int input_sep = 0;
char tmp[100];


template<class T>
void myPrint(char* log_type, T c) {
    // log print function
    Serial.print('[');
    Serial.print(log_type);
    Serial.print("] ");
    Serial.println(c);
}


void setup() {
    Serial.begin(9600);
    myPrint("sys", "Start");
    pwm.begin();
    pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

    arm_reset();
}


void loop() {
    // read command
    readInput();
    // and do nothing
    delay(10);
}


void readInput() {
    if (Serial.available() > 0) {
        // read one char
        input_byte = Serial.read();
        input_str[input_num] = input_byte;
        // myPrint("debug", input_str[input_num]);

        // text
        if (isGraph(input_str[input_num])) {
            input_str[++input_num] = '\0';
        } else
            return;

        // stop
        if(input_str[input_num - 1] == ';') {
            run_command();
            // echo when fininshed
            myPrint("echo", input_str);
            input_num = input_sep = 0;
            return;
        }

        // seprate char
        if(input_str[input_num - 1] == ',') {
            input_sep = input_num - 1;
        }
    }
}


void run_command() {
    // wheel:
    // e.g. `w0,30`; `w1,-120;`
    // TODO

    // arm: device, angle;
    // e.g. `a0,120;` `a1,150;`
    if(input_sep > 0 && input_str[0] == 'a') {
        int dev = input_str[1] - '0';
        int ang = atoi(input_str + (input_sep + 1));
        sprintf(tmp, "Arm %d move to %d", dev, ang);
        myPrint("arm", tmp);
        pwm.setPWM(dev, 0, angleToPulse(ang));
    }

    // reset
    // e.g. r0, r1
    if(input_num >= 2 && input_str[0] == 'r' && input_str[1] == '0') {
        arm_reset();
        return;
    }
}


void arm_reset() {
    // init angle of arm
    pwm.setPWM(0, 0, angleToPulse(90));
    pwm.setPWM(1, 0, angleToPulse(90));
    pwm.setPWM(2, 0, angleToPulse(90));
    pwm.setPWM(3, 0, angleToPulse(90));
    delay(1000);
}


int angleToPulse(int ang) {
    // map angle of 0 to 180 to Servo min and Servo max 
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    sprintf(tmp, "Angle %d Pulse %d", ang, pulse);
    myPrint("debug", tmp);
    return pulse;
}
