//此程式會利用encoder回傳值來計算車子的速度
//可利用序列埠監控視窗觀察車子的轉速


#include "TimerOne.h"
 
// Constants for Interrupt Pins
const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0
const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1
 
// Integers for pulse counters
unsigned int counter1 = 0;
unsigned int counter2 = 0;

// Motor A
int enA = 11;
int in1 = 13;
int in2 = 12;
 
// Motor B 
int enB = 10;
int in3 = 8;
int in4 = 7;
 
// Float for number of slots in encoder disk
float diskslots = 360;  // encoder slot = 360
 
// Interrupt Service Routines 
// Motor 1 pulse count ISR
void ISR_count1()  
{
  counter1++;  // increment Motor 1 counter value
} 
 
// Motor 2 pulse count ISR
void ISR_count2()  
{
  counter2++;  // increment Motor 2 counter value
} 
 
// TimerOne ISR
void ISR_timerone()
{
  Timer1.detachInterrupt();  // Stop the timer
  Serial.print("Motor Speed 1: "); 
  float rotation1 = (counter1 / diskslots) * 60.00;  // calculate RPM for Motor 1
  Serial.print(rotation1);  
  Serial.print(" RPM - "); 
  counter1 = 0;  //  reset counter to zero
  Serial.print("Motor Speed 2: "); 
  float rotation2 = (counter2 / diskslots) * 60.00;  // calculate RPM for Motor 2
  Serial.print(rotation2);  
  Serial.println(" RPM"); 
  counter2 = 0;  //  reset counter to zero
  Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
}

void forward() {
  analogWrite(enA, 150);
  analogWrite(enB, 150);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

}
void setup() 
{
  Serial.begin(9600);
  
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);  // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING);  // Increase counter 2 when speed sensor pin goes High
  Timer1.attachInterrupt( ISR_timerone ); // Enable the timer
} 
 
void loop()
{
forward();
delay(3000);
}
