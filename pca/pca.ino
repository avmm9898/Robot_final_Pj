
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  102 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  512 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
}


void loop() {

for(int i=0; i<4; i++)
  {
    for( int angle =0; angle<181; angle +=10){
      delay(50);
        pwm.setPWM(i, 0, angleToPulse(angle) );     
    }
 
  } 
  delay(500);// wait for 0.5 second
 
}


int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}
