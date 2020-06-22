# 2020 Final Project of Robotics

## Target
Insert the object into hole using robot(wheel and arm) with RGBD camera.


## Some utility
* `arduino_worker`
    An arduino script that allow to communicate with PC
* `arduino_connector.py`
    A python script that allow to communicate with arduino.
    The document of how to send command to arduino written here.


## Requirement
* Arduino(Uno)
    * `PWM library`
    * `arduino-cli lib install "Adafruit PWM Servo Driver Library"`
    * `TimerOne`
    * `arduino-cli lib install "TimerOne"`

* Python3 and it's package
    * Python3.7+
    * numpy
    * pyserial

* Realsense
`https://github.com/IntelRealSense/librealsense`


## Unknown script
* InputControlDistance
* encoderSpeedDetection


## Test
* test each joint of the arms
`test_arm/test_arm.ino`

* test the arms repeatedly
`test_arm_repetitive.py`

* test the car with encoding
`test_encoder_car_control/test_encoder_car_control.ino`

## Data
* Our google drive
https://drive.google.com/drive/u/1/folders/18e6d5P7vW5tq1IALxg0aSmPtt-QSELH9

## Document
* Our Proposal
https://docs.google.com/document/d/18njmWKULvyB6Nja7IZb4NpPqIBjUhZPCYzez5XdNj9Y/edit

* Our Progress 2
https://github.com/avmm9898/Robot_final_Pj/blob/master/doc/Progress2.pdf

* Our Final
Unfinished yet.

## To do list
* integrate playYOLO and realsense image, including acquire depth from x,y info.
