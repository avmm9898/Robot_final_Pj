// 記得先上傳到Arduino再run python檔
// 此程式會控制pca個馬達輪流旋轉，並用python的監控視窗輸入speed, distance, 正反轉(例如:100,100,0) 車子會開始行走

// Constants for Interrupt Pins
// Change values if not using Arduino Uno
String str;
int cmd[3] = {0};   //speed   distance    direction
int i;


const byte MOTOR_A = 3;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 2;  // Motor 1 Interrupt Pin - INT 0 - Left Motor

// Constant for steps in disk
const float stepcount = 360.00;  // 20 Slots in disk, change if different

// Constant for wheel diameter
const float wheeldiameter = 32.50; // Wheel diameter in millimeters, change if different

// Integers for pulse counters
volatile int counter_A = 0;
volatile int counter_B = 0;


// Motor A

int enA = 11;
int in1 = 13;
int in2 = 12;

// Motor B

int enB = 10;
int in3 = 8;
int in4 = 7;

//servo
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
// Watch video V1 to understand the two lines below: http://youtu.be/y8X9X10Tn1k
#define SERVOMIN  102 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  512 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;



int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
  Serial.print("Angle: "); Serial.print(ang);
  Serial.print(" pulse: "); Serial.println(pulse);
  return pulse;
}

// Interrupt Service Routines

// Motor A pulse count ISR
void ISR_countA()
{
  counter_A++;  // increment Motor A counter value
}

// Motor B pulse count ISR
void ISR_countB()
{
  counter_B++;  // increment Motor B counter value
}

// Function to convert from centimeters to steps
int CMtoSteps(float cm) {

  int result;  // Final calculation result
  float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;  // CM per Step

  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)

  return result;  // End and return result

}

// Function to Move Forward
void MoveForward(int steps, int mspeed)
{
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set Motor B forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Go forward until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }
    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
  }

  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

}

// Function to Move in Reverse
void MoveReverse(int steps, int mspeed)
{
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Set Motor B reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Go in reverse until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }
    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
  }

  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

}

// Function to Spin Right
void SpinRight(int steps, int mspeed)
{
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Set Motor B forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Go until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }
    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
  }

  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

}

// Function to Spin Left
void SpinLeft(int steps, int mspeed)
{
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set Motor B reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Go until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }
    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
  }

  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

}

void setup()
{
  // Attach the Interrupts to their ISR's
  attachInterrupt(digitalPinToInterrupt (MOTOR_A), ISR_countA, RISING);  // Increase counter A when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR_B), ISR_countB, RISING);  // Increase counter B when speed sensor pin goes High

  // Test Motor Movement  - Experiment with your own sequences here

  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
}


void loop()
{
  if (Serial.available()) {
    str = Serial.readStringUntil('\n');
    char *delim = ",";
    char *pch;
    char buf[100];
    i = 0;
    str.toCharArray(buf, sizeof(buf));
    pch = strtok(buf, delim);
    while (pch != NULL) {
      cmd[i] = atof(pch);
      pch = strtok (NULL, delim);
      i++;
    }
    //forward
    if (cmd[2] == 0) {
      MoveForward(CMtoSteps(cmd[1]), cmd[0]);
      delay(1000);  // Wait one second
    }
    else if (cmd[2] == 1) {
      MoveReverse(CMtoSteps(cmd[1]), cmd[0]);
      delay(1000);  // Wait one second
    }
  }
   for (int i = 0; i < 4; i++)
  {
    for ( int angle = 0; angle < 181; angle += 10) {
      delay(50);
      pwm.setPWM(i, 0, angleToPulse(angle) );
    }

  }
  delay(500);// wait for 0.5 second
}
