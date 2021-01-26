# 2020 Final Project of Robotics

## Target
Insert the object into hole using robot(wheel and arm) with RGBD camera.

## Result
Final
[![](https://github.com/avmm9898/Robot_final_Pj/blob/master/data/demo_all.png)](https://www.youtube.com/watch?v=BlEOE2fy_Zc)

Approach
[![](https://github.com/avmm9898/Robot_final_Pj/blob/master/data/demo_approach.png)](https://www.youtube.com/watch?v=7tt9smP7aGs)

Insert
[![](https://github.com/avmm9898/Robot_final_Pj/blob/master/data/demo_insert.png)](https://www.youtube.com/watch?v=eL94G9sQi4A)


## Files
* `arduino_worker`
    An arduino script that allow to communicate with PC
* `arduino_connector.py`
    A python script that allow to communicate with arduino.
    The document of how to send command to arduino written here.
* `control.py`
    Some utility
* `playYOLO.py`
    Yolo related codes, using openCV-dnn module to load darknet and detect object.
* `realsense_basic.py`
    Camera Object for reading colored image and depth image
* `arm_inverse_kinematic.py`
    Calculate inverse kinematic
* `arm_move.py`
    1. Moving arm by xyz position or by specific angle
    2. Calculate the transform from camera
* `arm_move_with_visual.py`
    Detect object from image and move the arm
* `car_move_with_visual.py`
    Read image and move the car
* `main.py`
    The main function that move the car to platform and move the arm to it
* `config.py`
    The config file. e.g. serial port
* `/data`
    Put non-code data here
* `/doc`
    Put documents here


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
* opencv-python
    
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
* Proposal
  https://docs.google.com/document/d/18njmWKULvyB6Nja7IZb4NpPqIBjUhZPCYzez5XdNj9Y/edit

* Progress 2
  https://github.com/avmm9898/Robot_final_Pj/blob/master/doc/Progress2.pdf

* Final Report
  https://docs.google.com/document/d/1U4hTdkWk3_W5jxNzboUAq03IZFFIPNzS-lzf1Dpz3Pk/edit?usp=sharing

* YOLOv3 weights，包含平台tiny、洞
  https://drive.google.com/drive/folders/1iq4aOWEoC3pH7jgbg-wTGPG5zBOkf2vy?usp=sharing

## State:
1. YOLO偵測平台目標物，回傳XYZ，車子朝目標逼近
2. 當行駛至70cm內，精準度增加，可以偵測深度(深度值700以內還算精準)
3. 距離平台目標50cmcm左右，切換至洞偵測模型，開始辨識洞
4. 辨識到3個洞，回傳3組XYZ
5. 手臂根據XYZ移動
