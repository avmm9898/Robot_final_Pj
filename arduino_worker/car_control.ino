#include<TimerOne.h>
#define PI 3.14

// Constants for Interrupt Pins
// Change values if not using Arduino Uno
const byte MOTOR_A = 3;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 2;  // Motor 1 Interrupt Pin - INT 0 - Left Motor

// Constant for steps in disk
const float stepcount = 360.00;  // 20 Slots in disk, change if different

// Constant for wheel diameter
const float wheeldiameter = 32.50; // Wheel diameter in millimeters, change if different

// Integers for pulse counters
volatile unsigned int counterA = 0;
volatile unsigned int counterB = 0;

// Motor A
const int enA = 11;
const int in1 = 13;
const int in2 = 12;

// Motor B
const int enB = 10;
const int in3 = 8;
const int in4 = 7;

// Interrupt Service Routines
// Motor A pulse count ISR
// increment Motor A counter value
void ISR_countA() { counterA++; }

// Motor B pulse count ISR
// increment Motor B counter value
void ISR_countB() { counterB++; }

// Speed of wheel
float speedA = 0;
float speedB = 0;


// Function to convert from centimeters to steps
int CMtoSteps(float cm) {
    int result;  // Final calculation result
    float circumference = (wheeldiameter * PI) / 10; // Calculate wheel circumference in cm
    float cm_step = circumference / stepcount;  // CM per Step

    float f_result = cm / cm_step;  // Calculate result as a float
    result = (int) f_result; // Convert to an integer (note this is NOT rounded)

    return result;  // End and return result
}


// set Direction
void setDirection(char mode) {
    // Forward
    if (mode == 'F') {
        // Set Motor A forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        // Set Motor B forward
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }
    // Reverse
    else if (mode == 'B') {
        // Set Motor A reverse
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);

        // Set Motor B reverse
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }
    // Right
    else if (mode == 'L') {
        // Set Motor A reverse
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);

        // Set Motor B forward
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }
    // Left
    else if (mode == 'R') {
        // Set Motor A forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        // Set Motor B reverse
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }
}


// set wheel speed
void carSpeed(char wheel, int mspeed, char mode) {
    setDirection(mode);
    if (wheel == 'A')
        analogWrite(enA, mspeed);
    else if (wheel == 'B')
        analogWrite(enB, mspeed);
}


// TimerOne ISR
void ISR_timerone()
{
    // Stop the timer
    Timer1.detachInterrupt();

    // calculate speed mm/s and reset
    speedA = counterA * wheeldiameter * PI / stepcount;
    counterA = 0;
    speedB = counterB * wheeldiameter * PI / stepcount;
    counterB = 0;

    // Enable the timer
    Timer1.attachInterrupt(ISR_timerone);
}


// get speed in mm/s
float getSpeed(char wheel) {
    if (wheel == 'A')
        return speedA;
    else if (wheel == 'B')
        return speedB;
    else
        return -999;
}


// Function to Move
void carMove(int cm, int mspeed, char mode) {
    counterA = 0;  //  reset counter A to zero
    counterB = 0;  //  reset counter B to zero
    int steps = CMtoSteps(cm);  // cm to encoder steps

    setDirection(mode);

    // Go forward until step value is reached
    while (steps > counterA && steps > counterB) {
        if (steps > counterA) {
            analogWrite(enA, mspeed);
        } else {
            analogWrite(enA, 0);
        }
        if (steps > counterB) {
            analogWrite(enB, mspeed);
        } else {
            analogWrite(enB, 0);
        }
    }

    // Stop when done
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    counterA = 0;  // reset counter A to zero
    counterB = 0;  // reset counter B to zero
}


void carSetup() {
    carReset();
    attachInterrupt(digitalPinToInterrupt(MOTOR_A), ISR_countA, RISING);  // Increase counter A when speed sensor pin goes High
    attachInterrupt(digitalPinToInterrupt(MOTOR_B), ISR_countB, RISING);  // Increase counter B when speed sensor pin goes High
    // BUG: https://www.pjrc.com/teensy/td_libs_TimerOne.html
    // Timer1 used pin 9, 10 where is overlap with motor B
    // TODO Change pin10
    // Timer1.initialize(500000);  // set timer for 0.5sec
    // Timer1.attachInterrupt(ISR_timerone);  // Enable the timer
}


// reset car speed
void carReset() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enA, 0);
    analogWrite(enB, 0);
}

