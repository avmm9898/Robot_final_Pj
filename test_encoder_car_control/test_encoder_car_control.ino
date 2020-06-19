//此程式會利用encoder回傳值來控制車子行走的距離
//此程式中包含直走、倒車、左右轉之function，可自行加入loop裡試試
 
// Constants for Interrupt Pins
// Change values if not using Arduino Uno
 
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
 
// Interrupt Service Routines
 
// Motor A pulse count ISR
// increment Motor A counter value 
void ISR_countA() { counter_A++; }
 
// Motor B pulse count ISR
// increment Motor B counter value
void ISR_countB() { counter_B++; }
 

// Function to convert from centimeters to steps
int CMtoSteps(float cm) {
    int result;  // Final calculation result
    float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
    float cm_step = circumference / stepcount;  // CM per Step

    float f_result = cm / cm_step;  // Calculate result as a float
    result = (int) f_result; // Convert to an integer (note this is NOT rounded)

    return result;  // End and return result
}
 

// Function to Move
void Move(int steps, int mspeed, char mode) {
    counter_A = 0;  //  reset counter A to zero
    counter_B = 0;  //  reset counter B to zero

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
    counter_A = 0;  // reset counter A to zero
    counter_B = 0;  // reset counter B to zero 
}
 

void setup() {
    // Attach the Interrupts to their ISR's
    attachInterrupt(digitalPinToInterrupt(MOTOR_A), ISR_countA, RISING);  // Increase counter A when speed sensor pin goes High
    attachInterrupt(digitalPinToInterrupt(MOTOR_B), ISR_countB, RISING);  // Increase counter B when speed sensor pin goes High
}
 

void loop()
{
    // Test Motor Movement  - Experiment with your own sequences here  
    Move(CMtoSteps(150), 250, 'F');  // Forward 25.4 cm at 250 speed
    delay(1000);  // Wait one second
    Move(CMtoSteps(150), 200, 'B');  // Reverse 25.4 cm at 200 speed
    delay(1000);  // Wait one second
    Move(CMtoSteps(150), 150, 'R');  // Right 25.4 cm at 150 speed
    delay(1000);  // Wait one second
    Move(CMtoSteps(150), 100, 'L');  // Left 25.4 cm at 100 speed
    delay(1000);  // Wait one second
}
