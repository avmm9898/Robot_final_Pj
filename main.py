# main function
import time
import cv2
import numpy as np
import threading

from realsense_basic import run, Camera
from arduino_connector import setArduinoCar, setArduinoArm
from car_move_with_visual import detectAndGo
from arm_move_with_visual import onetime_run, getXYZ
from functools import partial
from playYOLO import YOLO


# move arm away
setArduinoArm(0, 0)
time.sleep(1)
setArduinoArm(1, 0)
time.sleep(1)

platform_net = YOLO("platform_tiny", 0.4)
hole_net=YOLO("yolov3_hole",0.8)
# move car
run(partial(detectAndGo, platform_net.detect))
run(partial(MinorApproach, hole_net.detect))


# move arm
onetime_run(getXYZ)
