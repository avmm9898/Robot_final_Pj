# 2020 Final Project of Robotics

## Target
Insert the object into hole using robot(wheel and arm) with RGBD camera.


## Files
* `arduino_worker`
    An arduino script that allow to communicate with PC
* `arduino_connector.py`
    A python script that allow to communicate with arduino.
    The document of how to send command to arduino written here.
* `realsense_basic.py`
    Camera Object for reading colored image and depth image
* `arm_inverse_kinematic.py`
    Inverse kinematic
* `arm_move_with_visual.py`
    Read image and move the arm
* `car_move_with_visual.py`
    Read image and move the car
* `car_move_with_visual.py`
    Read image and move the car
* `/data`
    Put non-code data here
* `/doc`
    Put documents here
* `main.py`
    The main function
* `playYOLO.py`
    Yolo related codes


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

* YOLOv3 辨識模型，包含平台tiny、洞
  https://drive.google.com/drive/folders/1iq4aOWEoC3pH7jgbg-wTGPG5zBOkf2vy?usp=sharing
  
  

## State:

1. YOLO偵測平台目標物，回傳XYZ，車子朝目標逼近
2. 當行駛至70cm內，精準度增加，可以偵測深度(深度值700以內還算精準)
3. 距離平台目標50cmcm左右，切換至洞偵測模型，開始辨識洞
4. 辨識到3個洞，回傳3組XYZ
5. 手臂根據XYZ移動
